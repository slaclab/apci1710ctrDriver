/* apci1710ctr.c */

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
  #include <linux/autoconf.h>
#else
  #include <generated/autoconf.h>
#endif

#include <linux/spinlock.h>
#include <linux/ioctl.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/pci.h>    // struct pci_dev
#include <linux/cdev.h>   // struct cdev
#include <linux/poll.h>   // poll_table
#include <linux/mutex.h>  // struct mutex
#include <asm/io.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
  #include <asm/system.h>
#endif
#include <asm/uaccess.h>
#include <asm/bitops.h>
#include <linux/sched.h>

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/circ_buf.h>

#include "apci1710.h"
#include "apci1710-kapi.h"

#include "apci1710ctr.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("SLAC");
MODULE_DESCRIPTION("APCI-1710-CTR");

static int major = 0;               /* default to dynamic major */
module_param(major, int, 0);
MODULE_PARM_DESC(major, "Major device number");

static int hysteresis = 1;
module_param(hysteresis, int, 0);
MODULE_PARM_DESC(hysteresis, "Hysteresis mode (1=on, 0=off)");

EXPORT_NO_SYMBOLS;

#define NUM_CTR_CHANNELS  4

#define DEVNAME  "apci1710ctr"

static const char modulename[] = DEVNAME;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
static struct class * apci1710ctr_class = NULL;
#endif

static struct pci_dev *_pdev;

#ifdef INTENABLE_PROC
static int apci1710_intEnable_all(void);
static int apci1710_intDisable_all(void);
#endif /* INTENABLE_PROC */
static int apci1710_intEnable(int);
static int apci1710_intDisable(int);

/*
 * ring buffer
 *
 * The buffer's full or empty state can be resolved from the read and write pointers.
 * When the ptrs are equal, the buffer is empty.
 * When the read ptr is one greater than the write ptr, the buffer is full.
 * The capacity of the buffer is (_ringSize - 1) elements.
 */

static unsigned _ringSize = APCI1710CTR_DEFAULT_RINGSIZE;

typedef struct {
  unsigned int      channelIndex;   /* index */
  struct pci_dev *  pdev;           /* vendor driver */

  /* statistics */
  atomic_t interruptCount;
  atomic_t overflowCount;
  atomic_t frameCount;

  /* input buffer */
  counterBuf_t * ringBuf;
  struct circ_buf ring;

  /* lock */
  struct mutex lock;

  /* read queue */
  wait_queue_head_t inq;

  /* char device */
  struct cdev cdev;
  struct device *dev;

} counter_channel_t;

/* statically allocate */
static counter_channel_t counter_channel[NUM_CTR_CHANNELS];

/* proc */

#define CTR_PROC_DIRNAME0 "driver/apci1710ctr0"
#define CTR_PROC_DIRNAME1 "driver/apci1710ctr1"
#define CTR_PROC_DIRNAME2 "driver/apci1710ctr2"
#define CTR_PROC_DIRNAME3 "driver/apci1710ctr3"

struct proc_dir_entry *proc_parent[NUM_CTR_CHANNELS];

void apci1710ctr_proc_create(void);
void apci1710ctr_proc_remove(void);

/* atomic */
#include <asm/atomic.h>

/* ring buffer methods */
static unsigned int ringbufLevel(counter_channel_t *pchan);
static bool ringbufPushLocked(counter_channel_t *pchan, int32_t counter);
static bool ringbufPop(counter_channel_t *pchan, int32_t *counter, uint16_t *frameCount, uint16_t *flags, uint32_t *timestamp);
void ringbufReset(counter_channel_t *pchan);

static int counterModuleInit(void)
{
  int ii;
  for (ii = 0; ii < NUM_CTR_CHANNELS; ii++) {
    counter_channel[ii].pdev = _pdev;
    counter_channel[ii].channelIndex = ii;
    atomic_set(&counter_channel[ii].interruptCount, 0);
    atomic_set(&counter_channel[ii].overflowCount, 0);
    atomic_set(&counter_channel[ii].frameCount, 0);

    /* allocate ring buffer */
    counter_channel[ii].ringBuf = kmalloc(_ringSize * sizeof(counterBuf_t), GFP_KERNEL);

    /* clear ring buffer */
    counter_channel[ii].ring.head = counter_channel[ii].ring.tail = 0;
  }
  return 0;
}

static int counterModuleFini(void)
{
  /* free ring buffers */

  int ii;
  for (ii = NUM_CTR_CHANNELS - 1; ii >= 0; ii--) {
    if (!IS_ERR(counter_channel[ii].ringBuf)) {
      kfree(counter_channel[ii].ringBuf);
    }
  }
  return 0;
}

static int overflowCountGet(counter_channel_t *pchan)
{
  return atomic_read(&pchan->overflowCount);
}

static void overflowCountClear(counter_channel_t *pchan)
{
  atomic_set(&pchan->overflowCount, 0);
}

static void overflowCountIncrement(counter_channel_t *pchan)
{
  atomic_inc(&pchan->overflowCount);
}

static int frameCountGet(counter_channel_t *pchan)
{
  return atomic_read(&pchan->frameCount);
}

static void frameCountIncrement(counter_channel_t *pchan)
{
  atomic_inc(&pchan->frameCount);
}

static void frameCountClear(counter_channel_t *pchan)
{
  atomic_set(&pchan->frameCount, 0);
}

static int interruptCountGet(counter_channel_t *pchan)
{
  return atomic_read(&pchan->interruptCount);
}

static void interruptCountIncrement(counter_channel_t *pchan)
{
  atomic_inc(&pchan->interruptCount);
}

static void interruptCountClear(counter_channel_t *pchan)
{
  atomic_set(&pchan->interruptCount, 0);
}

static void apci1710_interrupt (struct pci_dev * pdev)
{
  uint8_t   mm;
  uint32_t  im;
  int32_t   latch;
  counter_channel_t *pchan;

  if (_pdev) {
    (void) i_APCI1710_TestInterrupt(_pdev, &mm, &im, (uint32_t *)&latch);
    if (mm < NUM_CTR_CHANNELS) {
      pchan = counter_channel + mm;
      interruptCountIncrement(pchan);

      /* callback already holds spinlock */
      ringbufPushLocked(pchan, latch);

    } else {
      printk("%s: %s: Error: chan=%u\n", modulename, __FUNCTION__, mm);
    }
  }
}

/*
 * apci1710_softReset - reset counter channel
 *
 * Disable interrupts, reset ring buffer, and clear statistics
 * for one counter channel.
 */
static int apci1710_softReset (counter_channel_t *pchan)
{
  if (pchan == NULL) {
    printk("%s: %s: pchan is NULL\n", modulename, __FUNCTION__);
  } else if (pchan->pdev == NULL) {
    printk("%s: %s: pchan->pdev is NULL\n", modulename, __FUNCTION__);
  } else {

    apci1710_intDisable(pchan->channelIndex);

    frameCountClear(pchan);
    overflowCountClear(pchan);
    interruptCountClear(pchan);

    ringbufReset(pchan);
  }

  return 0;
}

#ifdef INTENABLE_PROC
static int apci1710_intEnable_all(void)
{
  int err6 = 1;
  unsigned long irqstate;
  int moduleNumber;

  if (_pdev) {
    apci1710_lock(_pdev, &irqstate);

    /* Set the interrupt routine */
    err6 = i_APCI1710_SetBoardIntRoutine (_pdev, apci1710_interrupt);
    if (!err6) {
      /* Enable the latch interrupt for ALL modules */
      for (moduleNumber = 0; moduleNumber < NUM_CTR_CHANNELS; moduleNumber++) {
        (void) i_APCI1710_EnableLatchInterrupt(_pdev, moduleNumber);
      }
    }

    apci1710_unlock(_pdev, irqstate);
  }
  return err6;
}
#endif /* INTENABLE_PROC */

static int apci1710_intEnable(int moduleNumber)
{
  int err6 = 1;
  unsigned long irqstate;

  if (_pdev && (moduleNumber >= 0) && (moduleNumber < NUM_CTR_CHANNELS)) {
    apci1710_lock(_pdev, &irqstate);

    /* Set the interrupt routine */
    err6 = i_APCI1710_SetBoardIntRoutine (_pdev, apci1710_interrupt);
    if (!err6) {
      /* Enable the latch interrupt */
      (void) i_APCI1710_EnableLatchInterrupt(_pdev, moduleNumber);
    }

    apci1710_unlock(_pdev, irqstate);
  }
  return err6;
}

#ifdef INTENABLE_PROC
static int apci1710_intDisable_all(void)
{
  unsigned long irqstate;
  int moduleNumber;

  if (_pdev) {
    apci1710_lock(_pdev, &irqstate);

    /* Disable the latch interrupt for ALL modules */
    for (moduleNumber = 0; moduleNumber < NUM_CTR_CHANNELS; moduleNumber++) {
      (void) i_APCI1710_DisableLatchInterrupt(_pdev, moduleNumber);
    }

    apci1710_unlock(_pdev, irqstate);
  }
  return 0;
}
#endif /* INTENABLE_PROC */

static int apci1710_intDisable(int moduleNumber)
{
  unsigned long irqstate;

  if (_pdev && (moduleNumber >= 0) && (moduleNumber < NUM_CTR_CHANNELS)) {
    apci1710_lock(_pdev, &irqstate);

    /* Disable the latch interrupt */
    (void) i_APCI1710_DisableLatchInterrupt(_pdev, moduleNumber);

    apci1710_unlock(_pdev, irqstate);
  }
  return 0;
}

static int slac_inc_counter_kernel (void)
{
  int err1 = 0, err2 = 0, err8 = 0, err9 = 0;
  unsigned long irqstate;
  bool initFailed = false;
  int moduleNumber;

  uint8_t b_CounterRange = APCI1710_32BIT_COUNTER;        // Selection form counter range.
  uint8_t b_FirstCounterModus = APCI1710_QUADRUPLE_MODE;  // First counter operating mode.
  uint8_t b_FirstCounterOption;                           // First counter option.

  if (!_pdev) {
    printk("%s: %s: _pdev is not set\n", modulename, __FUNCTION__);
    return 0;
  }

  if (hysteresis) {
    printk("%s: hysteresis mode is ENABLED\n", modulename);
    b_FirstCounterOption = APCI1710_HYSTERESIS_ON;
  } else {
    printk("%s: hysteresis mode is DISABLED\n", modulename);
    b_FirstCounterOption = APCI1710_HYSTERESIS_OFF;
  }

  for (moduleNumber = 0; moduleNumber < NUM_CTR_CHANNELS; moduleNumber++) {
    /* Lock the function to avoid parallel configurations */
    apci1710_lock(_pdev, &irqstate);

    /* Initialise the incremental counter */
    err1 = i_APCI1710_InitCounter (_pdev,
                        moduleNumber,
                        b_CounterRange,
                        b_FirstCounterModus,
                        b_FirstCounterOption,
                        b_FirstCounterModus,
                        b_FirstCounterOption);

    if (!err1) {
      uint32_t dump;
      err2 = i_APCI1710_SetDigitalChlOff(_pdev, moduleNumber);
      err8 = i_APCI1710_Write32BitCounterValue(_pdev, moduleNumber, 0);
      /* read back counter in order to update latch value */
      err9 = i_APCI1710_Read32BitCounterValue(_pdev, moduleNumber, &dump);
    }

    /* Unlock the function so that other applications can call it */
    apci1710_unlock(_pdev, irqstate);

    if (err1) {
      printk ("i_APCI1710_InitCounter(%d) failed (%d)\n", moduleNumber, err1);
      initFailed = true;
    }

    if (err2) {
      initFailed = true;
      printk ("i_APCI1710_SetDigitalChlOff failed (%d)\n", err2);
    }

    if (err8) {
      initFailed = true;
      printk ("i_APCI1710_Write32BitCounterValue failed (%d)\n", err8);
    }

    if (err9) {
      initFailed = true;
      printk ("i_APCI1710_Read32BitCounterValue failed (%d)\n", err9);
    }

    if (!initFailed) {
      printk ("%s: Initialization of module %d successful\n", __FUNCTION__, moduleNumber);
    }
  }

  return 0;
}

/* ===== /proc =================================================== */

static int counter_proc_show(struct seq_file *m, void *v) {
  uint32_t value;
  int err1 = 1;
  unsigned long irqstate;
  counter_channel_t *pchan = (counter_channel_t *)m->private;

  if ((pchan == NULL) || (pchan->channelIndex > NUM_CTR_CHANNELS)) {
    printk("%s: %s: private data error\n", modulename, __FUNCTION__);
    return -EFAULT;
  }

  apci1710_lock(_pdev, &irqstate);
  err1 = i_APCI1710_Read32BitCounterValue(_pdev, pchan->channelIndex, &value);
  apci1710_unlock(_pdev, irqstate);

  if (err1) {
    value = 0;  /* default to 0 in case of error */
    if (err1 == 3) {
      printk("%s: %s: Counter %u not initialized\n", modulename, __FUNCTION__, pchan->channelIndex);
    } else {
      printk("%s: %s: Counter %u error %d\n", modulename, __FUNCTION__, pchan->channelIndex, err1);
    }
  }
  seq_printf(m, "%d\n", value);
  return 0;
}

static int counter_proc_open_channel0(struct inode *inode, struct  file *file) {
  return single_open(file, counter_proc_show, &counter_channel[0]);
}

static int counter_proc_open_channel1(struct inode *inode, struct  file *file) {
  return single_open(file, counter_proc_show, &counter_channel[1]);
}

static int counter_proc_open_channel2(struct inode *inode, struct  file *file) {
  return single_open(file, counter_proc_show, &counter_channel[2]);
}

static int counter_proc_open_channel3(struct inode *inode, struct  file *file) {
  return single_open(file, counter_proc_show, &counter_channel[3]);
}

static ssize_t counter_proc_write(struct file *file, const char __user *buf,  size_t count, loff_t *ppos) {
  int val;
  int err2 = 1;
  struct seq_file *ss = (struct seq_file *)file->private_data;
  counter_channel_t *pchan = NULL;

  if (ss) {
    pchan = (counter_channel_t *)ss->private;
  }

  if ((pchan == NULL) || (pchan->channelIndex > NUM_CTR_CHANNELS)) {
    printk("%s: %s: private data error\n", modulename, __FUNCTION__);
    return -EFAULT;
  }

  if (kstrtoint_from_user(buf, count, 0, &val)) {
    return -EFAULT;
  }
  if (_pdev) {
    err2 = i_APCI1710_Write32BitCounterValue(_pdev, pchan->channelIndex, (uint32_t) val);

    if (err2 == 3) {
      printk("%s: %s: Counter %u not initialized\n", modulename, __FUNCTION__, pchan->channelIndex);
    } else if (err2) {
      printk("%s: %s: Counter %u error %d\n", modulename, __FUNCTION__, pchan->channelIndex, err2);
    }
  }
  return count;
}

static const struct file_operations counter_proc_fops_channel0 = {
  .owner = THIS_MODULE,
  .open = counter_proc_open_channel0,
  .read = seq_read,
  .write = counter_proc_write,
  .llseek = seq_lseek,
  .release = single_release,
};

static const struct file_operations counter_proc_fops_channel1 = {
  .owner = THIS_MODULE,
  .open = counter_proc_open_channel1,
  .read = seq_read,
  .write = counter_proc_write,
  .llseek = seq_lseek,
  .release = single_release,
};

static const struct file_operations counter_proc_fops_channel2 = {
  .owner = THIS_MODULE,
  .open = counter_proc_open_channel2,
  .read = seq_read,
  .write = counter_proc_write,
  .llseek = seq_lseek,
  .release = single_release,
};

static const struct file_operations counter_proc_fops_channel3 = {
  .owner = THIS_MODULE,
  .open = counter_proc_open_channel3,
  .read = seq_read,
  .write = counter_proc_write,
  .llseek = seq_lseek,
  .release = single_release,
};

static int digout_proc_show(struct seq_file *m, void *v) {
  seq_printf(m, "Usage:\n"
                "echo {0|1} > digout\n"
                "example: echo 1 > digout\n"
                "sets value of digital output\n");
  return 0;
}

static int digout_proc_open_channel0(struct inode *inode, struct  file *file) {
  return single_open(file, digout_proc_show, &counter_channel[0]);
}

static int digout_proc_open_channel1(struct inode *inode, struct  file *file) {
  return single_open(file, digout_proc_show, &counter_channel[1]);
}

static int digout_proc_open_channel2(struct inode *inode, struct  file *file) {
  return single_open(file, digout_proc_show, &counter_channel[2]);
}

static int digout_proc_open_channel3(struct inode *inode, struct  file *file) {
  return single_open(file, digout_proc_show, &counter_channel[3]);
}

static ssize_t digout_proc_write(struct file *file, const char __user *buf,  size_t count, loff_t *ppos) {
  unsigned int val;
  int err3 = 1;
  struct seq_file *ss = (struct seq_file *)file->private_data;
  counter_channel_t *pchan = NULL;
  unsigned long irqstate;

  if (ss) {
    pchan = (counter_channel_t *)ss->private;
  }

  if ((pchan == NULL) || (pchan->channelIndex > NUM_CTR_CHANNELS)) {
    printk("%s: %s: private data error\n", modulename, __FUNCTION__);
    return -EFAULT;
  }

  if (kstrtouint_from_user(buf, count, 0, &val)) {
    return -EFAULT;
  }
  if (val > 1) {
    return -EINVAL;
  }
  if (_pdev) {
    apci1710_lock(_pdev, &irqstate);
    if (val) {
      err3 = i_APCI1710_SetDigitalChlOn(_pdev, pchan->channelIndex);
    } else {
      err3 = i_APCI1710_SetDigitalChlOff(_pdev, pchan->channelIndex);
    }
    apci1710_unlock(_pdev, irqstate);

    /* error check */
    if (err3 == 3) {
      printk("%s: %s: Counter %u not initialized\n", modulename, __FUNCTION__, pchan->channelIndex);
    } else if (err3) {
      printk("%s: %s: Counter %u error %d\n", modulename, __FUNCTION__, pchan->channelIndex, err3);
    }
  }
  return count;
}

static const struct file_operations digout_proc_fops_channel0 = {
  .owner = THIS_MODULE,
  .open = digout_proc_open_channel0,
  .read = seq_read,
  .write = digout_proc_write,
  .llseek = seq_lseek,
  .release = single_release,
};

static const struct file_operations digout_proc_fops_channel1 = {
  .owner = THIS_MODULE,
  .open = digout_proc_open_channel1,
  .read = seq_read,
  .write = digout_proc_write,
  .llseek = seq_lseek,
  .release = single_release,
};

static const struct file_operations digout_proc_fops_channel2 = {
  .owner = THIS_MODULE,
  .open = digout_proc_open_channel2,
  .read = seq_read,
  .write = digout_proc_write,
  .llseek = seq_lseek,
  .release = single_release,
};

static const struct file_operations digout_proc_fops_channel3 = {
  .owner = THIS_MODULE,
  .open = digout_proc_open_channel3,
  .read = seq_read,
  .write = digout_proc_write,
  .llseek = seq_lseek,
  .release = single_release,
};

#ifdef INTENABLE_PROC

static ssize_t intEnable_proc_read(struct file *file, char __user *buf, size_t size, loff_t *ppos) {
  static const char message[] = "Usage:\n"
                                "echo {0|1} > intEnable\n"
                                "example: echo 1 > intEnable\n"
                                "enable (1) or disable (0) counter interrupts\n";
  return simple_read_from_buffer(buf, size, ppos, message, sizeof(message));
}

static ssize_t intEnable_proc_write(struct file *file, const char __user *buf,  size_t count, loff_t *ppos) {
  unsigned int val;
  if (kstrtouint_from_user(buf, count, 0, &val)) {
    return -EFAULT;
  }
  if (val > 1) {
    return -EINVAL;
  }
  if (val) {
    apci1710_intEnable_all();
  } else {
    apci1710_intDisable_all();
  }

  return count;
}

static const struct file_operations intEnable_proc_fops = {
  .owner = THIS_MODULE,
  .read = intEnable_proc_read,
  .write = intEnable_proc_write,
  .llseek = default_llseek,
};

#endif /* INTENABLE_PROC */

#ifdef SOFTRESET_PROC

static ssize_t softReset_proc_read(struct file *file, char __user *buf, size_t size, loff_t *ppos) {
  static const char message[] = "Usage:\n"
                                "echo 1 > softReset\n"
                                "do soft reset\n";
  return simple_read_from_buffer(buf, size, ppos, message, sizeof(message));
}

static ssize_t softReset_proc_write(struct file *file, const char __user *buf,  size_t count, loff_t *ppos) {
  unsigned int val;
  int moduleNumber;

  if (kstrtouint_from_user(buf, count, 0, &val)) {
    return -EFAULT;
  }
  if (val != 1) {
    return -EINVAL;
  }

  for (moduleNumber = 0; moduleNumber < NUM_CTR_CHANNELS; moduleNumber++) {
    (void) apci1710_softReset(counter_channel + moduleNumber);
  }

  return count;
}

static const struct file_operations softReset_proc_fops = {
  .owner = THIS_MODULE,
  .read = softReset_proc_read,
  .write = softReset_proc_write,
  .llseek = default_llseek,
};

#endif /* SOFTRESET_PROC */

static int status_proc_show(struct seq_file *m, void *v) {
  unsigned long irqstate;
  uint8_t status1;
  uint32_t value2;
  int err1, err2;
  counter_channel_t *pchan = (counter_channel_t *)m->private;

  if ((pchan == NULL) || (pchan->channelIndex > NUM_CTR_CHANNELS)) {
    printk("%s: %s: private data error\n", modulename, __FUNCTION__);
    return -EFAULT;
  }

  if (_pdev) {
    /* Lock the function to avoid parallel configurations */
    apci1710_lock(_pdev, &irqstate);

    err1 = i_APCI1710_ReadLatchRegisterStatus(_pdev, pchan->channelIndex, 0, &status1);
    err2 = i_APCI1710_ReadLatchRegisterValue(_pdev, pchan->channelIndex, 0, &value2);

    /* Unlock the function so that other applications can call it */
    apci1710_unlock(_pdev, irqstate);

    if (err1 == 3) {
      seq_printf(m, "Counter not initialized\n");
      return 0;
    }

    if (!err1 && !err2) {
      seq_printf(m, "ReadLatchRegisterStatus: 0x%08x (%u)\n", status1, status1);
      seq_printf(m, "ReadLatchRegisterValue:  %d (%u)\n", value2,  value2);
    } else {
      seq_printf(m, "ReadLatchRegisterStatus: 0x%08x (%u) (err=%d)\n", status1, status1, err1);
      seq_printf(m, "ReadLatchRegisterValue:  %d (%u) (err=%d)\n", value2,  value2, err2);
    }
    seq_printf(m, "Interrupt count: %d\n", interruptCountGet(counter_channel + pchan->channelIndex));
    seq_printf(m, "Frame count:     %d\n", frameCountGet(counter_channel + pchan->channelIndex));
    seq_printf(m, "Overflow count:  %d\n", overflowCountGet(counter_channel + pchan->channelIndex));
    seq_printf(m, "Buffer level:    %u / %u\n", ringbufLevel(counter_channel + pchan->channelIndex), _ringSize - 1);
  } else {
    seq_printf(m, "lookupboard_by_index(0) failed\n");
  }
  return 0;
}

static int status_proc_open_channel0(struct inode *inode, struct  file *file) {
  return single_open(file, status_proc_show, &counter_channel[0]);
}

static int status_proc_open_channel1(struct inode *inode, struct  file *file) {
  return single_open(file, status_proc_show, &counter_channel[1]);
}

static int status_proc_open_channel2(struct inode *inode, struct  file *file) {
  return single_open(file, status_proc_show, &counter_channel[2]);
}

static int status_proc_open_channel3(struct inode *inode, struct  file *file) {
  return single_open(file, status_proc_show, &counter_channel[3]);
}

static const struct file_operations status_proc_fops_channel0 = {
  .owner = THIS_MODULE,
  .open = status_proc_open_channel0,
  .read = seq_read,
  .llseek = seq_lseek,
  .release = single_release,
};

static const struct file_operations status_proc_fops_channel1 = {
  .owner = THIS_MODULE,
  .open = status_proc_open_channel1,
  .read = seq_read,
  .llseek = seq_lseek,
  .release = single_release,
};

static const struct file_operations status_proc_fops_channel2 = {
  .owner = THIS_MODULE,
  .open = status_proc_open_channel2,
  .read = seq_read,
  .llseek = seq_lseek,
  .release = single_release,
};

static const struct file_operations status_proc_fops_channel3 = {
  .owner = THIS_MODULE,
  .open = status_proc_open_channel3,
  .read = seq_read,
  .llseek = seq_lseek,
  .release = single_release,
};

/* ===== /proc === ^^^ =========================================== */

static int counter_dev_open(struct inode *ii, struct file *filp)
{
  counter_channel_t *dev = container_of(ii->i_cdev, counter_channel_t, cdev);

  mutex_lock(&dev->lock);     /* LOCK */

  apci1710_softReset(dev);    /* disable interrupts, reset ring buffer, clear stats */

  mutex_unlock(&dev->lock);   /* UNLOCK */

  filp->private_data = dev;   /* for other methods */

  return 0;
}

static int counter_dev_close(struct inode *ii, struct file *filp)
{
  return 0;
}

/*
 *  typedef struct counterBuf {
 *    int             counter;
 *    unsigned int    timestamp;
 *    unsigned short  frameCount;
 *    unsigned short  flags;
 *  } counterBuf_t;
 */

static ssize_t counter_dev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
  counter_channel_t *pchan = filp->private_data;
  counterBuf_t tmpbuf;

  mutex_lock(&pchan->lock);             /* LOCK */

  while (pchan->ring.head == pchan->ring.tail) {    /* buffer is empty */
    /* nothing to read */
    mutex_unlock(&pchan->lock);         /* UNLOCK */
    if (filp->f_flags & O_NONBLOCK) {
      return -EAGAIN;
    }

    /* read: going to sleep */
    if (wait_event_interruptible(pchan->inq, pchan->ring.head != pchan->ring.tail)) {
      return -ERESTARTSYS;
    }

    /* otherwise loop, but first reacquire the lock */
    mutex_lock(&pchan->lock);           /* LOCK */
  }

  /* ok, data is there, return something */
  ringbufPop(pchan, &tmpbuf.counter, &tmpbuf.frameCount, &tmpbuf.flags, &tmpbuf.timestamp);
  count = min(count, sizeof(tmpbuf));
  if (copy_to_user(buf, &tmpbuf, count)) {
    mutex_unlock(&pchan->lock);         /* UNLOCK */
    printk(KERN_WARNING "apci1710ctr: read(ctr=%u) = -EFAULT\n", pchan->channelIndex);
    return -EFAULT;
  }

  mutex_unlock(&pchan->lock);           /* UNLOCK */

  return count;
}

static unsigned int counter_dev_poll(struct file *filp, poll_table *wait)
{
  counter_channel_t *pchan = filp->private_data;
  unsigned int mask = 0;

  mutex_lock(&pchan->lock);             /* LOCK */

  poll_wait(filp, &pchan->inq, wait);

  if (pchan->ring.head != pchan->ring.tail) {
    /* buffer is not empty */
    mask |= POLLIN | POLLRDNORM;        /* readable */
  }

  mutex_unlock(&pchan->lock);           /* UNLOCK */

  return mask;
}

static long counter_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
  counter_channel_t *pchan = filp->private_data;
  long rv = 0;

  switch (cmd) {
    case APCI1710CTR_IOCRESET:
      apci1710_softReset(pchan);        /* disable interrupts, reset ring buffer, clear stats */
      break;

    case APCI1710CTR_IOCINTENABLE:
      apci1710_intEnable(pchan->channelIndex);    /* enable interrupts */
      break;

    case APCI1710CTR_IOCINTDISABLE:
      apci1710_intDisable(pchan->channelIndex);   /* disable interrupts */
      break;

    default:
      rv = -EINVAL;
      break;
  }

  return rv;
}

static struct file_operations counter_fops = {
  .owner = THIS_MODULE,
  .open = counter_dev_open,
  .release = counter_dev_close,
  .read = counter_dev_read,
  .poll = counter_dev_poll,
  .unlocked_ioctl = counter_dev_ioctl
};

/** Called when module loads. */
static int __init apci1710ctr_init(void)
{
  dev_t devid;
  int rc;
  int board_index = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
  int minor;
#endif

  printk("%s: looking for board %u\n", modulename,board_index);
  _pdev = apci1710_lookup_board_by_index(0);
  if (!_pdev) {
    printk("%s: board %u not found\n", modulename, board_index);
    return -ENODEV;
  }

  printk("%s: calling counterModuleInit()\n", modulename);
  counterModuleInit();

  apci1710ctr_proc_create();

  /* allocate device numbers */
  if (major) {
    /* nonzero major number was set by module parameter */
    devid = MKDEV(major, 0);
    rc = register_chrdev_region(devid, NUM_CTR_CHANNELS, DEVNAME);
  } else {
    /* major number allocated dynamically */
    rc = alloc_chrdev_region(&devid, 0, NUM_CTR_CHANNELS, DEVNAME);
    major = MAJOR(devid);
  }
  if (rc < 0) {
    printk(DEVNAME ":allocating device numbers failed\n");
  } else {
    printk(DEVNAME ": major = %d\n", major);
  }

  /* initialize a read queue and mutex for each counter channel */
  for (minor = 0; minor < NUM_CTR_CHANNELS; minor++) {
    init_waitqueue_head(&counter_channel[minor].inq);
    mutex_init(&counter_channel[minor].lock);
  }

  /* create a char device for each counter channel */
  apci1710ctr_class = class_create (THIS_MODULE, DEVNAME);
  if (IS_ERR(apci1710ctr_class)) {
    printk (KERN_WARNING "%s: class_create() error\n", DEVNAME );
  } else {
    for (minor = 0; minor < NUM_CTR_CHANNELS; minor++) {
      cdev_init(&counter_channel[minor].cdev, &counter_fops);
      counter_channel[minor].cdev.owner = THIS_MODULE;
      if (cdev_add(&counter_channel[minor].cdev, MKDEV(major, minor), 1) == -1) {
        printk (KERN_WARNING "%s_%d: cdev_add() error\n", DEVNAME, minor);
      } else {
        counter_channel[minor].dev = device_create(apci1710ctr_class, NULL, MKDEV(major, minor), NULL, "%s_%d", DEVNAME, minor);
        if (IS_ERR(counter_channel[minor].dev)) {
          printk (KERN_WARNING "%s_%d: device_create() error\n", DEVNAME, minor);
        }
      }
    }
  }

  /* initial configuration of hardware */
  slac_inc_counter_kernel();

  return 0;
}
//-------------------------------------------------------------------

/** Called when module is unloaded. */
static void __exit apci1710ctr_exit(void)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
  int minor = 0;
  if (apci1710ctr_class && !IS_ERR(apci1710ctr_class)) {
    for (minor = NUM_CTR_CHANNELS - 1; minor >= 0; minor--) {
      device_destroy(apci1710ctr_class, MKDEV(major, minor));
      cdev_del(&counter_channel[minor].cdev);
    }
    class_destroy(apci1710ctr_class);
  }
#endif

  /* free device numbers */
  unregister_chrdev_region(MKDEV(major,0), NUM_CTR_CHANNELS);

  apci1710ctr_proc_remove();

  printk("%s: calling counterModuleFini()\n", modulename);
  counterModuleFini();
}
//------------------------------------------------------------------------------

void apci1710ctr_proc_create(void)
{
  printk("%s: calling proc_create()\n", modulename);

  proc_parent[0] = proc_mkdir(CTR_PROC_DIRNAME0, NULL);
  proc_parent[1] = proc_mkdir(CTR_PROC_DIRNAME1, NULL);
  proc_parent[2] = proc_mkdir(CTR_PROC_DIRNAME2, NULL);
  proc_parent[3] = proc_mkdir(CTR_PROC_DIRNAME3, NULL);

  proc_create(CTR_PROC_DIRNAME0 "/status", 0, NULL, &status_proc_fops_channel0);
  proc_create(CTR_PROC_DIRNAME1 "/status", 0, NULL, &status_proc_fops_channel1);
  proc_create(CTR_PROC_DIRNAME2 "/status", 0, NULL, &status_proc_fops_channel2);
  proc_create(CTR_PROC_DIRNAME3 "/status", 0, NULL, &status_proc_fops_channel3);

  proc_create(CTR_PROC_DIRNAME0 "/counter", 0, NULL, &counter_proc_fops_channel0);
  proc_create(CTR_PROC_DIRNAME1 "/counter", 0, NULL, &counter_proc_fops_channel1);
  proc_create(CTR_PROC_DIRNAME2 "/counter", 0, NULL, &counter_proc_fops_channel2);
  proc_create(CTR_PROC_DIRNAME3 "/counter", 0, NULL, &counter_proc_fops_channel3);

  proc_create(CTR_PROC_DIRNAME0 "/digout", 0, NULL, &digout_proc_fops_channel0);
  proc_create(CTR_PROC_DIRNAME1 "/digout", 0, NULL, &digout_proc_fops_channel1);
  proc_create(CTR_PROC_DIRNAME2 "/digout", 0, NULL, &digout_proc_fops_channel2);
  proc_create(CTR_PROC_DIRNAME3 "/digout", 0, NULL, &digout_proc_fops_channel3);

#ifdef INTENABLE_PROC
  proc_create(CTR_PROC_DIRNAME0 "/intEnable", 0, NULL, &intEnable_proc_fops);
#endif /* INTENABLE_PROC */
#ifdef SOFTRESET_PROC
  proc_create(CTR_PROC_DIRNAME0 "/softReset", 0, NULL, &softReset_proc_fops);
#endif /* SOFTRESET_PROC */
}

void apci1710ctr_proc_remove(void)
{
  printk("%s: calling proc_remove()\n", modulename);
  if (proc_parent[0]) {
    remove_proc_entry(CTR_PROC_DIRNAME0 "/status", NULL);
    remove_proc_entry(CTR_PROC_DIRNAME0 "/counter", NULL);
    remove_proc_entry(CTR_PROC_DIRNAME0 "/digout", NULL);
    proc_remove(proc_parent[0]);
  }
  if (proc_parent[1]) {
    remove_proc_entry(CTR_PROC_DIRNAME1 "/status", NULL);
    remove_proc_entry(CTR_PROC_DIRNAME1 "/counter", NULL);
    remove_proc_entry(CTR_PROC_DIRNAME1 "/digout", NULL);
    proc_remove(proc_parent[1]);
  }
  if (proc_parent[2]) {
    remove_proc_entry(CTR_PROC_DIRNAME2 "/status", NULL);
    remove_proc_entry(CTR_PROC_DIRNAME2 "/counter", NULL);
    remove_proc_entry(CTR_PROC_DIRNAME2 "/digout", NULL);
    proc_remove(proc_parent[2]);
  }
  if (proc_parent[3]) {
    remove_proc_entry(CTR_PROC_DIRNAME3 "/status", NULL);
    remove_proc_entry(CTR_PROC_DIRNAME3 "/counter", NULL);
    remove_proc_entry(CTR_PROC_DIRNAME3 "/digout", NULL);
    proc_remove(proc_parent[3]);
  }
}

/*
 * ringbufLevel -
 *
 * Count items in the buffer.
 * This routine must be called with the device lock NOT held.
 */
static unsigned int ringbufLevel(counter_channel_t *pchan)
{
  unsigned long irqstate;
  unsigned int rv;

  apci1710_lock(pchan->pdev, &irqstate);
  rv = CIRC_CNT(pchan->ring.head, pchan->ring.tail, _ringSize);
  apci1710_unlock(pchan->pdev, irqstate);

  return rv;
}

/*
 * ringbufPushLocked -
 *
 * Update write index after setting element in place.
 * This routine must be called with the device lock HELD.
 */
static bool ringbufPushLocked(counter_channel_t *pchan, int32_t counter)
{
  int tmpIndex, frameCountTmp;
  bool  rv;

  if (! CIRC_SPACE(pchan->ring.head, pchan->ring.tail, _ringSize)) {

    /* buffer full! update the overflow counter */
    overflowCountIncrement(pchan);
    rv = false;

  } else {

    tmpIndex = (pchan->ring.head + 1) % _ringSize;

    /* get and update the frame counter (does this belong here?) */
    frameCountTmp = frameCountGet(pchan);
    frameCountIncrement(pchan);

    /* update the element */
    pchan->ringBuf[tmpIndex].counter = counter;
    pchan->ringBuf[tmpIndex].frameCount = (uint16_t)frameCountTmp;

    /* update the head index */
    pchan->ring.head = tmpIndex;

    /* wake up any waiters */
    wake_up_interruptible(&pchan->inq);

    rv = true;
  }
  return rv;
}

/*
 * ringbufPop -
 *
 * Update read index after retrieving the element.
 * This routine must be called with the device lock NOT held.
 */
static bool ringbufPop(counter_channel_t *pchan, int32_t *counter, uint16_t *frameCount, uint16_t *flags, uint32_t *timestamp)
{
  int   tmpIndex;
  unsigned long irqstate;
  bool  rv = false;

  apci1710_lock(pchan->pdev, &irqstate);

  if (pchan->ring.head != pchan->ring.tail) {
    /* copy data */
    tmpIndex = (pchan->ring.tail + 1) % _ringSize;
    *counter = pchan->ringBuf[tmpIndex].counter;
    *frameCount = pchan->ringBuf[tmpIndex].frameCount;

    /* update read index */
    pchan->ring.tail = tmpIndex;
    rv = true;
  }
  apci1710_unlock(pchan->pdev, irqstate);

  return rv;
}

/*
 * ringbufReset -
 *
 * This routine must be called with the device lock NOT held.
 */
void ringbufReset(counter_channel_t *pchan)
{
  unsigned long irqstate;

  apci1710_lock(pchan->pdev, &irqstate);
  pchan->ring.head = pchan->ring.tail = 0;
  apci1710_unlock(pchan->pdev, irqstate);
}

module_exit(apci1710ctr_exit);
module_init(apci1710ctr_init);
