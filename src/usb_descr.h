/*
 * usb_descr.h
 *
 *  Created on: Nov 18, 2020
 *      Author: bertold
 */

#ifndef USB_DESCR_H_
#define USB_DESCR_H_

#define USB_RTYPE_DIR_MASK                  0x80U
#define USB_RTYPE_DIR_HOST2DEV              0x00U
#define USB_RTYPE_DIR_DEV2HOST              0x80U
#define USB_RTYPE_TYPE_MASK                 0x60U
#define USB_RTYPE_TYPE_STD                  0x00U
#define USB_RTYPE_TYPE_CLASS                0x20U
#define USB_RTYPE_TYPE_VENDOR               0x40U
#define USB_RTYPE_TYPE_RESERVED             0x60U
#define USB_RTYPE_RECIPIENT_MASK            0x1FU
#define USB_RTYPE_RECIPIENT_DEVICE          0x00U
#define USB_RTYPE_RECIPIENT_INTERFACE       0x01U
#define USB_RTYPE_RECIPIENT_ENDPOINT        0x02U
#define USB_RTYPE_RECIPIENT_OTHER           0x03U

#define USB_REQ_GET_STATUS                  0U
#define USB_REQ_CLEAR_FEATURE               1U
#define USB_REQ_SET_FEATURE                 3U
#define USB_REQ_SET_ADDRESS                 5U
#define USB_REQ_GET_DESCRIPTOR              6U
#define USB_REQ_SET_DESCRIPTOR              7U
#define USB_REQ_GET_CONFIGURATION           8U
#define USB_REQ_SET_CONFIGURATION           9U
#define USB_REQ_GET_INTERFACE               10U
#define USB_REQ_SET_INTERFACE               11U
#define USB_REQ_SYNCH_FRAME                 12U

#define USB_DESCRIPTOR_DEVICE               1U
#define USB_DESCRIPTOR_CONFIGURATION        2U
#define USB_DESCRIPTOR_STRING               3U
#define USB_DESCRIPTOR_INTERFACE            4U
#define USB_DESCRIPTOR_ENDPOINT             5U
#define USB_DESCRIPTOR_DEVICE_QUALIFIER     6U
#define USB_DESCRIPTOR_OTHER_SPEED_CFG      7U
#define USB_DESCRIPTOR_INTERFACE_POWER      8U
#define USB_DESCRIPTOR_INTERFACE_ASSOCIATION 11U

#define USB_FEATURE_ENDPOINT_HALT           0U
#define USB_FEATURE_DEVICE_REMOTE_WAKEUP    1U
#define USB_FEATURE_TEST_MODE               2U


/**
 * @brief   Helper macro for index values into descriptor strings.
 */
#define USB_DESC_INDEX(i) ((uint8_t)(i))

/**
 * @brief   Helper macro for byte values into descriptor strings.
 */
#define USB_DESC_BYTE(b) ((uint8_t)(b))

/**
 * @brief   Helper macro for word values into descriptor strings.
 */
#define USB_DESC_WORD(w)                                                    \
  (uint8_t)((w) & 255U),                                                    \
  (uint8_t)(((w) >> 8) & 255U)

/**
 * @brief   Helper macro for BCD values into descriptor strings.
 */
#define USB_DESC_BCD(bcd)                                                   \
  (uint8_t)((bcd) & 255U),                                                  \
  (uint8_t)(((bcd) >> 8) & 255)

/*
 * @define  Device Descriptor size.
 */
#define USB_DESC_DEVICE_SIZE                18U

/**
 * @brief   Device Descriptor helper macro.
 */
#define USB_DESC_DEVICE(bcdUSB, bDeviceClass, bDeviceSubClass,              \
                        bDeviceProtocol, bMaxPacketSize, idVendor,          \
                        idProduct, bcdDevice, iManufacturer,                \
                        iProduct, iSerialNumber, bNumConfigurations)        \
  USB_DESC_BYTE(USB_DESC_DEVICE_SIZE),                                      \
  USB_DESC_BYTE(USB_DESCRIPTOR_DEVICE),                                     \
  USB_DESC_BCD(bcdUSB),                                                     \
  USB_DESC_BYTE(bDeviceClass),                                              \
  USB_DESC_BYTE(bDeviceSubClass),                                           \
  USB_DESC_BYTE(bDeviceProtocol),                                           \
  USB_DESC_BYTE(bMaxPacketSize),                                            \
  USB_DESC_WORD(idVendor),                                                  \
  USB_DESC_WORD(idProduct),                                                 \
  USB_DESC_BCD(bcdDevice),                                                  \
  USB_DESC_INDEX(iManufacturer),                                            \
  USB_DESC_INDEX(iProduct),                                                 \
  USB_DESC_INDEX(iSerialNumber),                                            \
  USB_DESC_BYTE(bNumConfigurations)

/**
 * @brief   Configuration Descriptor size.
 */
#define USB_DESC_CONFIGURATION_SIZE         9U

/**
 * @brief   Configuration Descriptor helper macro.
 */
#define USB_DESC_CONFIGURATION(wTotalLength, bNumInterfaces,                \
                               bConfigurationValue, iConfiguration,         \
                               bmAttributes, bMaxPower)                     \
  USB_DESC_BYTE(USB_DESC_CONFIGURATION_SIZE),                               \
  USB_DESC_BYTE(USB_DESCRIPTOR_CONFIGURATION),                              \
  USB_DESC_WORD(wTotalLength),                                              \
  USB_DESC_BYTE(bNumInterfaces),                                            \
  USB_DESC_BYTE(bConfigurationValue),                                       \
  USB_DESC_INDEX(iConfiguration),                                           \
  USB_DESC_BYTE(bmAttributes),                                              \
  USB_DESC_BYTE(bMaxPower)

/**
 * @brief   Interface Descriptor size.
 */
#define USB_DESC_INTERFACE_SIZE             9U

/**
 * @brief   Interface Descriptor helper macro.
 */
#define USB_DESC_INTERFACE(bInterfaceNumber, bAlternateSetting,             \
                           bNumEndpoints, bInterfaceClass,                  \
                           bInterfaceSubClass, bInterfaceProtocol,          \
                           iInterface)                                      \
  USB_DESC_BYTE(USB_DESC_INTERFACE_SIZE),                                   \
  USB_DESC_BYTE(USB_DESCRIPTOR_INTERFACE),                                  \
  USB_DESC_BYTE(bInterfaceNumber),                                          \
  USB_DESC_BYTE(bAlternateSetting),                                         \
  USB_DESC_BYTE(bNumEndpoints),                                             \
  USB_DESC_BYTE(bInterfaceClass),                                           \
  USB_DESC_BYTE(bInterfaceSubClass),                                        \
  USB_DESC_BYTE(bInterfaceProtocol),                                        \
  USB_DESC_INDEX(iInterface)

/**
 * @brief   Interface Association Descriptor size.
 */
#define USB_DESC_INTERFACE_ASSOCIATION_SIZE 8U

/**
 * @brief   Interface Association Descriptor helper macro.
 */
#define USB_DESC_INTERFACE_ASSOCIATION(bFirstInterface,                     \
                           bInterfaceCount, bFunctionClass,                 \
                           bFunctionSubClass, bFunctionProcotol,            \
                           iInterface)                                      \
  USB_DESC_BYTE(USB_DESC_INTERFACE_ASSOCIATION_SIZE),                       \
  USB_DESC_BYTE(USB_DESCRIPTOR_INTERFACE_ASSOCIATION),                      \
  USB_DESC_BYTE(bFirstInterface),                                           \
  USB_DESC_BYTE(bInterfaceCount),                                           \
  USB_DESC_BYTE(bFunctionClass),                                            \
  USB_DESC_BYTE(bFunctionSubClass),                                         \
  USB_DESC_BYTE(bFunctionProcotol),                                         \
  USB_DESC_INDEX(iInterface)

/**
 * @brief   Endpoint Descriptor size.
 */
#define USB_DESC_ENDPOINT_SIZE              7U

/**
 * @brief   Endpoint Descriptor helper macro.
 */
#define USB_DESC_ENDPOINT(bEndpointAddress, bmAttributes, wMaxPacketSize,   \
                          bInterval)                                        \
  USB_DESC_BYTE(USB_DESC_ENDPOINT_SIZE),                                    \
  USB_DESC_BYTE(USB_DESCRIPTOR_ENDPOINT),                                   \
  USB_DESC_BYTE(bEndpointAddress),                                          \
  USB_DESC_BYTE(bmAttributes),                                              \
  USB_DESC_WORD(wMaxPacketSize),                                            \
  USB_DESC_BYTE(bInterval)
/** @} */



#endif /* USB_DESCR_H_ */
