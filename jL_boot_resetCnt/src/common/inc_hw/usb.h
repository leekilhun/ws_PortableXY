/*
 * usb.h
 *
 *  Created on: Jul 10, 2021
 *      Author: gns2l
 */

#ifndef SRC_COMMON_INC_HW_USB_H_
#define SRC_COMMON_INC_HW_USB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw_def.h"


#ifdef _USE_HW_USB


typedef enum
{
  USB_NON_MODE,
  USB_CDC_MODE,
  USB_MSC_MODE
} UsbMode;



bool usbInit(void);
bool usbBegin(UsbMode usb_mode);
void usbDeInit(void);
bool usbIsOpen(void);
bool usbIsConnect(void);

UsbMode usbGetMode(void);


#endif

#ifdef __cplusplus
}
#endif


#endif /* SRC_COMMON_INC_HW_USB_H_ */
