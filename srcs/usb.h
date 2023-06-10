#ifndef __USB_DESCRIPTOR_H__
#define __USB_DESCRIPTOR_H__

#include <stdint.h>

// `bmRequestType` bit definitions.
#define USB_REQUEST_TYPE_TYPE_MASK 0x60u
#define USB_REQUEST_TYPE_TYPE_STANDARD 0x00u
#define USB_REQUEST_TYPE_TYPE_CLASS 0x20u
#define USB_REQUEST_TYPE_TYPE_VENDOR 0x40u

#define USB_REQUEST_TYPE_RECIPIENT_MASK 0x1fu
#define USB_REQUEST_TYPE_RECIPIENT_DEVICE 0x00u
#define USB_REQUEST_TYPE_RECIPIENT_INTERFACE 0x01u
#define USB_REQUEST_TYPE_RECIPIENT_ENDPOINT 0x02u

#define USB_REQUEST_TYPE_DIRECTION_MASK 0x80u
#define USB_DIRECTION_OUT 0x00u
#define USB_DIRECTION_IN 0x80u

#define USB_TRANSFER_TYPE_CONTROL 0x0
#define USB_TRANSFER_TYPE_ISOCHRONOUS 0x1
#define USB_TRANSFER_TYPE_BULK 0x2
#define USB_TRANSFER_TYPE_INTERRUPT 0x3
#define USB_TRANSFER_TYPE_BITS 0x3

// Descriptor types.
#define USB_DESCRIPTOR_TYPE_DEVICE 0x01
#define USB_DESCRIPTOR_TYPE_CONFIG 0x02
#define USB_DESCRIPTOR_TYPE_STRING 0x03
#define USB_DESCRIPTOR_TYPE_INTERFACE 0x04
#define USB_DESCRIPTOR_TYPE_ENDPOINT 0x05
#define USB_DESCRIPTOR_TYPE_HID 0x21
#define USB_DESCRIPTOR_TYPE_REPORT 0x22

#define USB_REQUEST_GET_STATUS 0x0
#define USB_REQUEST_CLEAR_FEATURE 0x01
#define USB_REQUEST_SET_FEATURE 0x03
#define USB_REQUEST_SET_ADDRESS 0x05
#define USB_REQUEST_GET_DESCRIPTOR 0x06
#define USB_REQUEST_SET_DESCRIPTOR 0x07
#define USB_REQUEST_GET_CONFIGURATION 0x08
#define USB_REQUEST_SET_CONFIGURATION 0x09
#define USB_REQUEST_GET_INTERFACE 0x0a
#define USB_REQUEST_SET_INTERFACE 0x0b
#define USB_REQUEST_SYNC_FRAME 0x0c

#define USB_REQUEST_MSC_GET_MAX_LUN 0xfe
#define USB_REQUEST_MSC_RESET 0xff

#define USB_FEATURE_ENDPOINT_HALT 0x00
#define USB_FEATURE_DEVICE_REMOTE_WAKEUP 0x01
#define USB_FEATURE_TEST_MODE 0x02

// Do not align those structures so the size could be used as length.
//
// I personally prefer snake case, but those names are from USB specification.

struct usb_setup_packet {
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
} __attribute__((packed));

struct usb_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
} __attribute__((packed));

struct usb_device_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t bcdUSB;
	uint8_t bDeviceClass;
	uint8_t bDeviceSubClass;
	uint8_t bDeviceProtocol;
	uint8_t bMaxPacketSize0;
	uint16_t idVendor;
	uint16_t idProduct;
	uint16_t bcdDevice;
	uint8_t iManufacturer;
	uint8_t iProduct;
	uint8_t iSerialNumber;
	uint8_t bNumConfigurations;
} __attribute__((packed));

struct usb_configuration_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t wTotalLength;
	uint8_t bNumInterfaces;
	uint8_t bConfigurationValue;
	uint8_t iConfiguration;
	uint8_t bmAttributes;
	uint8_t bMaxPower;
} __attribute__((packed));

struct usb_interface_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bInterfaceNumber;
	uint8_t bAlternateSetting;
	uint8_t bNumEndpoints;
	uint8_t bInterfaceClass;
	uint8_t bInterfaceSubClass;
	uint8_t bInterfaceProtocol;
	uint8_t iInterface;
} __attribute__((packed));

struct usb_endpoint_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t bInterval;
} __attribute__((packed));

struct usb_endpoint_descriptor_long {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t bInterval;
	uint8_t bRefresh;
	uint8_t bSyncAddr;
} __attribute__((packed));

struct usb_string_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	// We only support 1 language.
	uint16_t wLANGID;
} __attribute__((packed));

struct usb_hid_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t bcdHID;
	uint8_t bCountryCode;
	uint8_t bNumDescriptors;
	// We only use Report Descriptor so this is enough.
	uint8_t bDescriptorType1;
	uint16_t wDescriptorLength1;
} __attribute__((packed));

#endif
