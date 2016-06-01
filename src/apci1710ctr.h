/* apci1710ctr.h */

#ifndef __INC_apci1710ctr
#define __INC_apci1710ctr

#include <linux/ioctl.h>

#include "apci1710ctr_ioctl.h"
#include "apci1710ctr_buf.h"

#define APCI1710CTR_MODE_DEFAULT    4

#define APCI1710CTR_HYSTERESIS_DEFAULT  1

#define APCI1710CTR_FILTER_OFF      0
#define APCI1710CTR_FILTER_MIN      1
#define APCI1710CTR_FILTER_MAX      15

/* input filter 9 = 500ns */
#define APCI1710CTR_FILTER_DEFAULT  9

#endif
