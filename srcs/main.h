#ifndef __PICO_BUTTON_H__
#define __PICO_BUTTON_H__

#include <stdint.h>
#include <stdbool.h>

#include "usb.h"

// Ideally I should use `malloc()` to alloc different numbers of endpoints for
// interfaces, but I am afraid I cannot do this on embedded devices. So if you
// need more, enlarge this max value and control the actually number of
// endpoints in interface structure.
#define MAX_ENDPOINTS_PER_INTERFACE 1
// Handle interfaces is easier, because we only have 1 interfaces array.
#define N_INTERFACES 1
#define N_DESCRIPTOR_STRINGS 3

struct usb_endpoint;
struct usb_interface;
struct usb_device;

typedef void (*usb_endpoint_complete_callback)(struct usb_endpoint *endpoint,
					       struct usb_device *device,
					       uint8_t *buf, uint16_t len);

// Struct in which we keep the endpoint data.
struct usb_endpoint {
	const struct usb_endpoint_descriptor *descriptor;
	usb_endpoint_complete_callback on_complete;

	// Pointers to endpoint + buffer control registers in the USB DPSRAM.
	volatile uint32_t *endpoint_control;
	volatile uint32_t *buffer_control;
	volatile uint8_t *data_buffer;

	// Toggle after each packet (unless replying to a SETUP).
	uint8_t next_pid;
};

struct usb_interface {
	struct usb_interface_descriptor *descriptor;
	struct usb_hid_descriptor *hid_descriptor;
	uint8_t *report_descriptor;
	int n_endpoints;
	struct usb_endpoint *endpoints[MAX_ENDPOINTS_PER_INTERFACE];
};

// Each device has 1 configuation, and can have many interfaces. Each interface
// can have many endpoints.
struct usb_device {
	struct usb_device_descriptor *descriptor;
	struct usb_configuration_descriptor *configuration_descriptor;
	struct usb_string_descriptor *string_descriptor;
	char *descriptor_strings[N_DESCRIPTOR_STRINGS];
	// EP0 does not belong to interfaces.
	struct usb_endpoint *ep0_in;
	struct usb_endpoint *ep0_out;
	struct usb_interface *interfaces[N_INTERFACES];
	uint8_t address;
	bool should_set_address;
	bool could_remote_wakeup;
	bool configured;
	bool suspended;
};

struct app {
	struct usb_device *device;
	uint32_t last_button_events;
	// Used to debounce.
	uint32_t last_button_time;
};

#endif
