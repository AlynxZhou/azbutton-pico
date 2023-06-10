#include <stdlib.h>
#include <string.h>

// USB register definitions from pico-sdk.
#include "hardware/regs/usb.h"
// USB hardware struct definitions from pico-sdk.
#include "hardware/structs/usb.h"
// For functions to enable interrupt.
#include "hardware/irq.h"
// For functions to reset the USB controller.
#include "hardware/resets.h"
// For GPIO.
#include "hardware/gpio.h"

// For time related functions from pico-sdk.
#include "pico/stdlib.h"

#include "main.h"

#define EP0_IN_ADDRESS (USB_DIRECTION_IN | 0)
#define EP0_OUT_ADDRESS (USB_DIRECTION_OUT | 0)
#define EP1_IN_ADDRESS (USB_DIRECTION_IN | 1)

// Because now I implemented multi-packet transfer, it should be OK to change
// this to a smaller size, but if this is too small, it might be too slow to
// reply setup (too much data to send and transfer is blocked), so just use
// 32 or 64.
#define PACKET_SIZE 64

#define usb_hw_set hw_set_alias(usb_hw)
#define usb_hw_clear hw_clear_alias(usb_hw)

// `0x65` is Application, typically AT-101 Keyboard ends here.
#define HID_KEYBOARD_KEYS 0x66

#define HID_KEYBOARD_MODIFIER_NONE 0x00

#define HID_KEYBOARD_INDEX_MODIFIER 0
#define HID_KEYBOARD_INDEX_KEYS 2

// USB HID protocol says 6 keys in an event is the requirement for BIOS
// keyboard support, though OS could support more keys via modifying the report
// descriptor.
#define HID_KEYBOARD_MAX_KEYS 6
#define HID_KEYBOARD_EVENT_SIZE (2 + HID_KEYBOARD_MAX_KEYS)

#define HID_KEYBOARD_RESERVED 0x00
#define HID_KEYBOARD_ERROR_ROLL_OVER 0x01

#define HID_KEYBOARD_SCANCODE_ENTER 0x28

#define BUTTON_PIN 2
#define LED_PIN 1

#define DEBOUNCE_TIME 50

// We have to declare a global variable here, because we cannot pass arguments
// to the interrupt handler.
static struct app *this = NULL;

static inline uint32_t get_boot_ms(void)
{
	return to_ms_since_boot(get_absolute_time());
}

void led_on(void)
{
	gpio_put(LED_PIN, 1);
}

void led_off(void)
{
	gpio_put(LED_PIN, 0);
}

void led_init(void)
{
	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);
	// Turn off the LED initially.
	led_off();
}

// When plugging/unplugging device, the D+/D- state are unstable and may
// accidentally trigger suspend or resume (don't ask me why I am not a circuit
// engineer), so we just skip those before we are properly configured.

void usb_handle_device_suspend(struct usb_device *device)
{
	if (!device->configured)
		return;

	if (device->suspended)
		return;

	// Turn off the LED if device suspends.
	led_off();
	device->suspended = true;
}

void usb_handle_device_resume(struct usb_device *device)
{
	if (!device->configured)
		return;

	if (!device->suspended)
		return;

	// Turn on the LED if device resumes.
	led_on();
	device->suspended = false;
}

void usb_remote_wakeup(struct usb_device *device)
{
	if (!device->configured)
		return;

	if (!device->could_remote_wakeup)
		return;

	usb_hw_set->sie_ctrl = USB_SIE_CTRL_RESUME_BITS;

	// Resume itself.
	usb_handle_device_resume(device);
}

// void usb_handle_device_connect_disconnect(struct usb_device *device)
// {
// 	// If host disconnected, exit.
// 	if (!(usb_hw->sie_status & USB_SIE_STATUS_CONNECTED_BITS))
// 		exit(0);
// }

void usb_handle_bus_reset(struct usb_device *device)
{
	device->address = 0;
	device->should_set_address = false;
	device->configured = false;
	device->suspended = false;
	led_off();
	// Don't forget to reset the actual address.
	usb_hw->dev_addr_ctrl = 0;
}

static inline bool usb_endpoint_is_in(struct usb_endpoint *endpoint)
{
	return endpoint->descriptor->bEndpointAddress & USB_DIRECTION_IN;
}

void usb_endpoint_handle_packet_next(struct usb_endpoint *endpoint)
{
	uint32_t remaining_length =
		endpoint->user_buffer_length - endpoint->transferred_length;
	uint16_t length =
		MIN(remaining_length, endpoint->descriptor->wMaxPacketSize);

	// Tell the USB controller our desired length for this packet, but we
	// may not send so much data actually, so we don't update transferred
	// length here.
	//
	// The control flags use the higher byte, and length uses the lower
	// byte, so they won't conflict.
	uint32_t value = length | USB_BUF_CTRL_AVAIL;

	if (usb_endpoint_is_in(endpoint)) {
		if (endpoint->user_buffer != NULL) {
			memcpy((void *)endpoint->data_buffer,
			       endpoint->user_buffer +
				       endpoint->transferred_length,
			       length);
		}
		value |= USB_BUF_CTRL_FULL;
	}

	if (endpoint->next_pid == 1) {
		value |= USB_BUF_CTRL_DATA1_PID;
		endpoint->next_pid = 0;
	} else {
		value |= USB_BUF_CTRL_DATA0_PID;
		endpoint->next_pid = 1;
	}

	if (endpoint->transferred_length + length >=
	    endpoint->user_buffer_length)
		value |= USB_BUF_CTRL_LAST;

	// Set the actuall buffer control register to let the USB controller
	// start to work.
	*endpoint->buffer_control = value & ~USB_BUF_CTRL_AVAIL;
	// According to the datasheet, we have to wait before set available bit
	// to prevent from concurrent accessing.
	__asm volatile("b 1f\n"
		       "1: b 1f\n"
		       "1: b 1f\n"
		       "1: b 1f\n"
		       "1: b 1f\n"
		       "1: b 1f\n"
		       "1:\n"
		       :
		       :
		       : "memory");
	*endpoint->buffer_control = value;
}

void usb_endpoint_handle_packet_done(struct usb_endpoint *endpoint,
				     struct usb_device *device)
{
	uint32_t buffer_control = *endpoint->buffer_control;
	// Get the actual length of this packet. We will use this to
	// update transferred length.
	uint16_t length = buffer_control & USB_BUF_CTRL_LEN_MASK;

	if (!usb_endpoint_is_in(endpoint))
		if (endpoint->user_buffer != NULL)
			memcpy(endpoint->user_buffer +
				       endpoint->transferred_length,
			       (void *)endpoint->data_buffer, length);

	endpoint->transferred_length += length;

	// Comparing transferred length and user buffer length is not enough,
	// when reading from host, we may use a larger user buffer that cannot
	// be totally filled by host. If the actual length of this packet is
	// smaller than `wMaxPacketSize`, we know there is no more packet.
	if (endpoint->transferred_length >= endpoint->user_buffer_length ||
	    length < endpoint->descriptor->wMaxPacketSize) {
		// Reset state after callback, so you can handle such things in
		// callback.
		endpoint->on_complete(endpoint, device);
		endpoint->busy = false;
		endpoint->user_buffer = NULL;
		endpoint->user_buffer_length = 0;
		endpoint->transferred_length = 0;
	} else {
		usb_endpoint_handle_packet_next(endpoint);
	}
}

static inline uint32_t
usb_endpoint_address_to_bit(struct usb_endpoint *endpoint)
{
	// Lower 3 bits are endpoint number, and 7th bit is direction.
	//
	// The result is defined in <https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2040/hardware_structs/include/hardware/structs/usb.h#L213-L247>.
	return (endpoint->descriptor->bEndpointAddress & 0x7) * 2 +
	       (usb_endpoint_is_in(endpoint) ? 0 : 1);
}

void usb_handle_buffer_status(struct usb_device *device)
{
	// Each bit represents an endpoint.
	uint32_t buffers = usb_hw->buf_status;

	// Check EP0 first.
	if (buffers & (1 << usb_endpoint_address_to_bit(device->ep0_in)))
		usb_endpoint_handle_packet_done(device->ep0_in, device);
	if (buffers & (1 << usb_endpoint_address_to_bit(device->ep0_out)))
		usb_endpoint_handle_packet_done(device->ep0_out, device);

	// Check endpoints belongs to interfaces.
	for (int i = 0; i < N_INTERFACES; ++i) {
		struct usb_interface *interface = device->interfaces[i];
		for (int j = 0; j < interface->n_endpoints; ++j) {
			struct usb_endpoint *endpoint = interface->endpoints[j];
			if (buffers &
			    (1 << usb_endpoint_address_to_bit(endpoint))) {
				usb_endpoint_handle_packet_done(endpoint,
								device);
			}
		}
	}
}

// Actual transfer is done by the USB controller.
//
// IN transfer:
//   1. Fill endpoint's data packet buffer with data we need to send in user's
//      data buffer.
//   2. Tell the USB controller this endpoint is ready.
//   3. The USB controller will send data to host from endpoint's data packet
//      buffer.
//
// OUT transfer:
//   1. Tell the USB controller this endpoint is ready.
//   2. The USB controller will receive data from host to endpoint's data packet
//      buffer.
//   3. Fill user's data buffer with data just received in endpoint's data
//      packet buffer.
int usb_endpoint_start_transfer(struct usb_endpoint *endpoint, uint8_t *buffer,
				uint32_t length)
{
	// Caller should prepare a queue and only start new transfer in complete
	// callback.
	if (endpoint->busy)
		return -1;

	endpoint->busy = true;
	endpoint->user_buffer = buffer;
	endpoint->user_buffer_length = length;
	endpoint->transferred_length = 0;

	usb_endpoint_handle_packet_next(endpoint);

	return 0;
}

void usb_set_address(struct usb_device *device,
		     volatile struct usb_setup_packet *packet)
{
	// According to the standard, we are not expected to change address
	// immediately, we must to finish STATUS stage of Control Transfer from
	// address 0 first, so we delay it to EP0 in's complete callback.
	device->address = packet->wValue & 0xff;
	device->should_set_address = true;
}

void usb_set_configuration(struct usb_device *device,
			   volatile struct usb_setup_packet *packet)
{
	device->configured = true;
	led_on();
}

void usb_set_feature(struct usb_device *device,
		     volatile struct usb_setup_packet *packet)
{
	uint8_t recipient = packet->bmRequestType &
			    USB_REQUEST_TYPE_RECIPIENT_MASK;
	uint16_t wValue = packet->wValue;
	uint16_t wIndex = packet->wIndex;
	if (recipient == USB_REQUEST_TYPE_RECIPIENT_DEVICE) {
		// This is the only device feature that can be clear.
		if (wValue == USB_FEATURE_DEVICE_REMOTE_WAKEUP) {
			device->could_remote_wakeup = true;
		} else if (wValue == USB_FEATURE_TEST_MODE) {
			// Too complicated, I just don't implement it.
		}
	} else if (recipient == USB_REQUEST_TYPE_RECIPIENT_ENDPOINT) {
		if (wValue == USB_FEATURE_ENDPOINT_HALT) {
			// Not implemented.
			// TODO: First implement STALL/NAK, which should be done
			// by writing registers to controll the USB controller.
		}
	}
}

void usb_clear_feature(struct usb_device *device,
		       volatile struct usb_setup_packet *packet)
{
	uint8_t recipient = packet->bmRequestType &
			    USB_REQUEST_TYPE_RECIPIENT_MASK;
	uint16_t wValue = packet->wValue;
	uint16_t wIndex = packet->wIndex;
	if (recipient == USB_REQUEST_TYPE_RECIPIENT_DEVICE) {
		// This is the only device feature that can be clear.
		if (wValue == USB_FEATURE_DEVICE_REMOTE_WAKEUP) {
			device->could_remote_wakeup = false;
		}
	} else if (recipient == USB_REQUEST_TYPE_RECIPIENT_ENDPOINT) {
		if (wValue == USB_FEATURE_ENDPOINT_HALT) {
			// Not implemented.
			// TODO: First implement STALL/NAK, which should be done
			// by writing registers to controll the USB controller.
		}
	}
}

void usb_get_device_descriptor(struct usb_device *device,
			       volatile struct usb_setup_packet *packet)
{
	usb_endpoint_start_transfer(
		device->ep0_in, (uint8_t *)device->descriptor,
		MIN(device->descriptor->bLength, packet->wLength));
}

void usb_get_configuration_descriptor(struct usb_device *device,
				      volatile struct usb_setup_packet *packet)
{
	uint8_t *start = malloc(device->configuration_descriptor->wTotalLength);
	if (start == NULL)
		return;

	uint8_t *buffer = start;

	memcpy(buffer, device->configuration_descriptor,
	       device->configuration_descriptor->bLength);
	buffer += device->configuration_descriptor->bLength;

	// According to USB specification, we should also provide interface
	// descriptors, HID descriptors and endpoint descriptors along with
	// configuration descriptors.

	for (int i = 0; i < N_INTERFACES; ++i) {
		struct usb_interface *interface = device->interfaces[i];
		// Interface descriptor.
		memcpy(buffer, interface->descriptor,
		       interface->descriptor->bLength);
		buffer += interface->descriptor->bLength;
		// HID descriptor.
		memcpy(buffer, interface->hid_descriptor,
		       interface->hid_descriptor->bLength);
		buffer += interface->hid_descriptor->bLength;
		for (int j = 0; j < interface->n_endpoints; ++j) {
			struct usb_endpoint *endpoint = interface->endpoints[j];
			// Endpoint descriptor.
			memcpy(buffer, endpoint->descriptor,
			       endpoint->descriptor->bLength);
			buffer += endpoint->descriptor->bLength;
		}
	}

	usb_endpoint_start_transfer(
		device->ep0_in, start,
		MIN(device->configuration_descriptor->wTotalLength,
		    packet->wLength));
	free(start);
}

void usb_get_string_descriptor(struct usb_device *device,
			       volatile struct usb_setup_packet *packet)
{
	// This is different from `wIndex`, but we only supports 1 language, so
	// `wIndex` is useless for us.
	uint8_t index = packet->wValue & 0xff;

	if (index == 0) {
		// String Descriptor Zero.
		usb_endpoint_start_transfer(
			device->ep0_in, (uint8_t *)device->string_descriptor,
			MIN(device->string_descriptor->bLength,
			    packet->wLength));
	} else {
		// Yes, USB specification says index starts from 1.
		char *string = device->descriptor_strings[index - 1];
		uint8_t bLength = 2 + strlen(string) * 2;
		uint8_t *start = malloc(bLength);
		if (start == NULL)
			return;
		uint8_t *buffer = start;
		uint8_t bDescriptorType = USB_DESCRIPTOR_TYPE_STRING;

		*buffer++ = bLength;
		*buffer++ = bDescriptorType;

		// For alpha and number, Unicode is just ASCII in lower byte and
		// 0 in higher byte.
		uint16_t *bString = (uint16_t *)buffer;
		uint8_t c;
		do {
			c = *string++;
			*bString++ = c;
		} while (c != '\0');

		usb_endpoint_start_transfer(device->ep0_in, start,
					    MIN(bLength, packet->wLength));
		free(start);
	}
}

void usb_get_report_descriptor(struct usb_device *device,
			       volatile struct usb_setup_packet *packet)
{
	// In HID, `wIndex` is interface number.
	uint16_t wIndex = packet->wIndex;

	for (int i = 0; i < N_INTERFACES; ++i) {
		struct usb_interface *interface = device->interfaces[i];
		if (interface->descriptor->bInterfaceNumber == wIndex) {
			usb_endpoint_start_transfer(
				device->ep0_in, interface->report_descriptor,
				MIN(interface->hid_descriptor
					    ->wDescriptorLength1,
				    packet->wLength));
			break;
		}
	}
}

void usb_get_status(struct usb_device *device,
		    volatile struct usb_setup_packet *packet)
{
	uint8_t recipient = packet->bmRequestType &
			    USB_REQUEST_TYPE_RECIPIENT_MASK;
	// If you need the actual interface/endpoint number, read the `wIndex`
	// field, I just don't need it here.
	//
	// The `wLength` should always be 2, if not, stop buying devices from
	// that vendor.
	if (recipient == USB_REQUEST_TYPE_RECIPIENT_DEVICE) {
		// Well, the actual values are the same with `bmAttributes` in
		// configuration descriptor, but the positions are not.
		//
		// This device is bus-powered so only check remote wakeup.
		uint8_t status[2] = { 0x00, device->could_remote_wakeup ?
						    0x02 :
						    0x00 };
		usb_endpoint_start_transfer(device->ep0_in, status,
					    MIN(sizeof(status),
						packet->wLength));
	} else if (recipient == USB_REQUEST_TYPE_RECIPIENT_INTERFACE) {
		// All bits are reserved for interface.
		uint8_t status[2] = { 0x00, 0x00 };
		usb_endpoint_start_transfer(device->ep0_in, status,
					    MIN(sizeof(status),
						packet->wLength));
	} else if (recipient == USB_REQUEST_TYPE_RECIPIENT_ENDPOINT) {
		// We just never halt.
		uint8_t status[2] = { 0x00, 0x00 };
		usb_endpoint_start_transfer(device->ep0_in, status,
					    MIN(sizeof(status),
						packet->wLength));
	} else {
		// Why are you falling here? When I write this program those
		// values are reserved, are you still a human?
	}
}

void usb_handle_setup_request(struct usb_device *device)
{
	volatile struct usb_setup_packet *packet =
		(struct usb_setup_packet *)&usb_dpram->setup_packet;
	// The 7th bit is direction.
	uint8_t direction = packet->bmRequestType &
			    USB_REQUEST_TYPE_DIRECTION_MASK;
	uint8_t request = packet->bRequest;

	// See Control Transfer in <https://www.usbmadesimple.co.uk/ums_3.htm>.
	//
	// Control Transfer has 3 stage: SETUP, DATA and STATUS.
	//
	// We already finished SETUP stage by receiving the setup packet.

	// Then we are in DATA stage, this is optional because most OUT
	// transfers has no data. But no matter it is IN or OUT, it always
	// starts from DATA1.
	if (direction == USB_DIRECTION_OUT) {
		device->ep0_out->next_pid = 1;
		switch (request) {
		case USB_REQUEST_SET_ADDRESS:
			usb_set_address(device, packet);
			break;
		case USB_REQUEST_SET_CONFIGURATION:
			usb_set_configuration(device, packet);
			break;
		case USB_REQUEST_SET_FEATURE:
			usb_set_feature(device, packet);
			break;
		case USB_REQUEST_CLEAR_FEATURE:
			usb_clear_feature(device, packet);
			break;
		default:
			break;
		}
	} else if (direction == USB_DIRECTION_IN) {
		device->ep0_in->next_pid = 1;
		if (request == USB_REQUEST_GET_DESCRIPTOR) {
			uint16_t descriptor_type = packet->wValue >> 8;
			switch (descriptor_type) {
			case USB_DESCRIPTOR_TYPE_DEVICE:
				usb_get_device_descriptor(device, packet);
				break;
			case USB_DESCRIPTOR_TYPE_CONFIG:
				usb_get_configuration_descriptor(device,
								 packet);
				break;
			case USB_DESCRIPTOR_TYPE_STRING:
				usb_get_string_descriptor(device, packet);
				break;
			case USB_DESCRIPTOR_TYPE_REPORT:
				usb_get_report_descriptor(device, packet);
				break;
			default:
				break;
			}
		} else if (request == USB_REQUEST_GET_STATUS) {
			// I try to dump debug stream from Linux kernel, it
			// seems you need to answer a `GET_STATUS` request after
			// resume, otherwise Linux kernel will return a IO error
			// and disconnect/reconnect the device.
			usb_get_status(device, packet);
		}
	}

	// After DATA stage, the last one is STATUS stage, we need to transfer a
	// zero-length DATA1 packet from the other direction.
	if (direction == USB_DIRECTION_OUT) {
		device->ep0_in->next_pid = 1;
		usb_endpoint_start_transfer(device->ep0_in, NULL, 0);
	} else if (direction == USB_DIRECTION_IN) {
		device->ep0_out->next_pid = 1;
		usb_endpoint_start_transfer(device->ep0_out, NULL, 0);
	}
}

void usb_on_events(void)
{
	struct usb_device *device = this->device;
	uint32_t status = usb_hw->ints;

	if (status & USB_INTS_SETUP_REQ_BITS) {
		usb_handle_setup_request(device);
		usb_hw_clear->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
	}

	if (status & USB_INTS_BUFF_STATUS_BITS) {
		usb_handle_buffer_status(device);
		// This is cleared by clearing all bits in `buf_status`.
		usb_hw_clear->buf_status = usb_hw->buf_status;
	}

	if (status & USB_INTS_BUS_RESET_BITS) {
		usb_handle_bus_reset(device);
		usb_hw_clear->sie_status = USB_SIE_STATUS_BUS_RESET_BITS;
	}

	// See <https://github.com/hathach/tinyusb/blob/86c416d4c0fb38432460b3e11b08b9de76941bf5/src/portable/raspberrypi/rp2040/dcd_rp2040.c#L310-L328>
	// and <https://github.com/hathach/tinyusb/blob/86c416d4c0fb38432460b3e11b08b9de76941bf5/src/portable/raspberrypi/rp2040/dcd_rp2040.c#L348-L355>.
	//
	// We should use connect and disconnect interrupts to turn off LED on
	// host disconnecting (system power off), but it seems that we need to
	// connect a VBUS detect circuit before using this, and in the example
	// it just force VBUS detect by overriding its bit to make it always
	// behave like connected to a host, so we cannot use this.
	//
	// But we will get suspend interrupts when host disconnecting in this
	// case, and turn off LED on suspend is a reasonable behavior, so we
	// handle this in suspend interrupts.

	// if (status & USB_INTS_DEV_CONN_DIS_BITS) {
	// 	usb_handle_device_connect_disconnect(device);
	// 	usb_hw_clear->sie_status = USB_SIE_STATUS_CONNECTED_BITS;
	// }

	if (status & USB_INTS_DEV_SUSPEND_BITS) {
		usb_handle_device_suspend(device);
		usb_hw_clear->sie_status = USB_SIE_STATUS_SUSPENDED_BITS;
	}

	if (status & USB_INTS_DEV_RESUME_FROM_HOST_BITS) {
		usb_handle_device_resume(device);
		usb_hw_clear->sie_status = USB_SIE_STATUS_RESUME_BITS;
	}
}

static inline uint32_t usb_buffer_offset(volatile uint8_t *buffer)
{
	return (uint32_t)buffer ^ (uint32_t)usb_dpram;
}

void usb_init_endpoint(struct usb_endpoint *endpoint)
{
	// Get the data buffer as an offset of the USB controller's DPRAM.
	uint32_t dpram_offset = usb_buffer_offset(endpoint->data_buffer);
	uint32_t reg = EP_CTRL_ENABLE_BITS | EP_CTRL_INTERRUPT_PER_BUFFER |
		       (endpoint->descriptor->bmAttributes
			<< EP_CTRL_BUFFER_TYPE_LSB) |
		       dpram_offset;
	*endpoint->endpoint_control = reg;
}

void usb_init_interface(struct usb_interface *interface)
{
	for (int i = 0; i < interface->n_endpoints; ++i)
		usb_init_endpoint(interface->endpoints[i]);
}

void usb_init(struct usb_device *device)
{
	// See <https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#hardware_resets>.
	reset_block(RESETS_RESET_USBCTRL_BITS);
	unreset_block_wait(RESETS_RESET_USBCTRL_BITS);

	// Clear any previous state in dpram just in case.
	memset(usb_dpram, 0, sizeof(*usb_dpram));

	// Mux the controller to the onboard usb PHY.
	usb_hw->muxing = USB_USB_MUXING_TO_PHY_BITS |
			 USB_USB_MUXING_SOFTCON_BITS;

	// We need to connect a VBUS detect circuit to make VBUS detect work,
	// but we don't have it, so just override the bit, then the device just
	// thinks it is plugged into a host.
	//
	// This prevents device connect and disconnect interrupts, we will get
	// suspend interrupts instead.
	usb_hw->pwr = USB_USB_PWR_VBUS_DETECT_BITS |
		      USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;

	// Enable the USB controller in device mode.
	usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS;

	// Enable an interrupt per EP0 transaction.
	usb_hw->sie_ctrl = USB_SIE_CTRL_EP0_INT_1BUF_BITS;

	// Enable interrupts for when a buffer is done, when the bus is reset,
	// when a setup packet is received, when device suspends, and when
	// device resumes.
	usb_hw->inte = USB_INTS_BUFF_STATUS_BITS | USB_INTS_BUS_RESET_BITS |
		       USB_INTS_SETUP_REQ_BITS | USB_INTS_DEV_SUSPEND_BITS |
		       USB_INTS_DEV_RESUME_FROM_HOST_BITS;

	for (int i = 0; i < N_INTERFACES; ++i)
		usb_init_interface(device->interfaces[i]);

	// Present full speed device by enabling pull up on DP.
	usb_hw_set->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;

	// Enable USB interrupt at processor.
	irq_set_enabled(USBCTRL_IRQ, true);
	// Set interrupt handler for USB.
	irq_set_exclusive_handler(USBCTRL_IRQ, usb_on_events);
}

void ep0_in_on_complete(struct usb_endpoint *endpoint,
			struct usb_device *device)
{
	// Set address when we finished the STATUS stage from address 0.
	if (device->should_set_address) {
		device->should_set_address = false;
		usb_hw->dev_addr_ctrl = device->address;
	}
}

void ep0_out_on_complete(struct usb_endpoint *endpoint,
			 struct usb_device *device)
{
}

void ep1_in_on_complete(struct usb_endpoint *endpoint,
			struct usb_device *device)
{
}

// Don't send event if USB is not configured.

void button_press(struct usb_device *device)
{
	if (!device->configured)
		return;

	uint8_t buffer[HID_KEYBOARD_EVENT_SIZE];

	buffer[HID_KEYBOARD_INDEX_MODIFIER] = HID_KEYBOARD_MODIFIER_NONE;
	buffer[1] = HID_KEYBOARD_RESERVED;
	memset(&buffer[HID_KEYBOARD_INDEX_KEYS], 0, HID_KEYBOARD_MAX_KEYS);

	// If you want other keys, modify here.
	buffer[HID_KEYBOARD_INDEX_KEYS] = HID_KEYBOARD_SCANCODE_ENTER;

	// We already know which endpoint is for keyboard.
	usb_endpoint_start_transfer(device->interfaces[0]->endpoints[0], buffer,
				    sizeof(buffer));
}

void button_release(struct usb_device *device)
{
	if (!device->configured)
		return;

	uint8_t buffer[HID_KEYBOARD_EVENT_SIZE];

	buffer[HID_KEYBOARD_INDEX_MODIFIER] = HID_KEYBOARD_MODIFIER_NONE;
	buffer[1] = HID_KEYBOARD_RESERVED;
	// Release is just remove a scancode from previous event.
	memset(&buffer[HID_KEYBOARD_INDEX_KEYS], 0, HID_KEYBOARD_MAX_KEYS);

	// We already know which endpoint is for keyboard.
	usb_endpoint_start_transfer(device->interfaces[0]->endpoints[0], buffer,
				    sizeof(buffer));
}

void button_on_events(uint gpio, uint32_t events)
{
	struct usb_device *device = this->device;

	// It looks like Pico has Schmitt triggers and by default enabling it,
	// so maybe this software debouncing is not needed, but keeping it is
	// harmless.
	if (get_boot_ms() - this->last_button_time > DEBOUNCE_TIME) {
		// Recommend to keep the position of this line to be precise.
		this->last_button_time = get_boot_ms();

		// Pico triggers GPIO events as a timer! So I need to manually
		// ignore repeated events.
		if (events != this->last_button_events) {
			this->last_button_events = events;

			// Because this is a HID keyboard, if it is suspended,
			// it requires a remote wakeup on button events.
			if (device->suspended)
				usb_remote_wakeup(device);

			if (events & GPIO_IRQ_LEVEL_HIGH)
				button_press(device);
			else if (events & GPIO_IRQ_LEVEL_LOW)
				button_release(device);
		}
	}
}

void button_init(void)
{
	gpio_pull_down(BUTTON_PIN);
	gpio_set_irq_enabled_with_callback(
		BUTTON_PIN, GPIO_IRQ_LEVEL_HIGH | GPIO_IRQ_LEVEL_LOW, true,
		&button_on_events);
}

int main(void)
{
	// You could grab those values via `lsusb -vv -d VID:PID`.
	struct usb_endpoint_descriptor ep0_in_descriptor = {
		.bLength = sizeof(ep0_in_descriptor),
		.bDescriptorType = USB_DESCRIPTOR_TYPE_ENDPOINT,
		// EP number 0, OUT from host (rx to device).
		.bEndpointAddress = EP0_IN_ADDRESS,
		.bmAttributes = USB_TRANSFER_TYPE_CONTROL,
		.wMaxPacketSize = PACKET_SIZE,
		.bInterval = 0
	};
	struct usb_endpoint ep0_in = { .descriptor = &ep0_in_descriptor,
				       .on_complete = &ep0_in_on_complete,
				       .endpoint_control = NULL,
				       .buffer_control =
					       &usb_dpram->ep_buf_ctrl[0].in,
				       .data_buffer = usb_dpram->ep0_buf_a,
				       .user_buffer = NULL,
				       .user_buffer_length = 0,
				       .transferred_length = 0,
				       .busy = false,
				       .next_pid = 0 };
	struct usb_endpoint_descriptor ep0_out_descriptor = {
		.bLength = sizeof(ep0_out_descriptor),
		.bDescriptorType = USB_DESCRIPTOR_TYPE_ENDPOINT,
		// EP number 0, OUT from host (rx to device).
		.bEndpointAddress = EP0_OUT_ADDRESS,
		.bmAttributes = USB_TRANSFER_TYPE_CONTROL,
		.wMaxPacketSize = PACKET_SIZE,
		.bInterval = 0
	};
	struct usb_endpoint ep0_out = { .descriptor = &ep0_out_descriptor,
					.on_complete = &ep0_out_on_complete,
					.endpoint_control = NULL,
					.buffer_control =
						&usb_dpram->ep_buf_ctrl[0].out,
					.data_buffer = usb_dpram->ep0_buf_b,
					.user_buffer = NULL,
					.user_buffer_length = 0,
					.transferred_length = 0,
					.busy = false,
					.next_pid = 0 };
	struct usb_endpoint_descriptor ep1_in_descriptor = {
		.bLength = sizeof(ep1_in_descriptor),
		.bDescriptorType = USB_DESCRIPTOR_TYPE_ENDPOINT,
		// EP number 1, IN from host (tx from device).
		.bEndpointAddress = EP1_IN_ADDRESS,
		// Keyboard's transfer type is interrupt.
		.bmAttributes = USB_TRANSFER_TYPE_INTERRUPT,
		// On my mouse and keyboard, they use a smaller value 8, but I
		// don't know how to do multi-packet transfer, so just use 64.
		.wMaxPacketSize = PACKET_SIZE,
		// This has different formats for full-speed and high-speed.
		.bInterval = 1
	};
	struct usb_endpoint ep1_in = {
		.descriptor = &ep1_in_descriptor,
		.on_complete = &ep1_in_on_complete,
		// EP1 starts at offset 0 for endpoint control.
		.endpoint_control = &usb_dpram->ep_ctrl[0].in,
		.buffer_control = &usb_dpram->ep_buf_ctrl[1].in,
		// First free EPX buffer.
		// If I change `wMaxPacketSize`, should I also change this?
		.data_buffer = &usb_dpram->epx_data[0 * PACKET_SIZE],
		.user_buffer = NULL,
		.user_buffer_length = 0,
		.transferred_length = 0,
		.busy = false,
		.next_pid = 0
	};
	// The specification is available here:
	// <https://www.usb.org/sites/default/files/hid1_11.pdf>
	//
	// In particular, read:
	//  - 6.2.2 Report Descriptor
	//  - Appendix B.1 Protocol 1 (Keyboard)
	//  - Appendix C: Keyboard Implementation
	//
	// Normally a basic HID keyboard uses 8 bytes:
	//     Modifier Reserved Key Key Key Key Key Key
	//
	// You can dump your device's report descriptor with:
	//
	//     sudo usbhid-dump -m vid:pid -e descriptor
	//
	// (change vid:pid to your device's vendor ID and product ID).
	uint8_t report_descriptor[] = {
		// Usage Page (Generic Desktop)
		0x05, 0x01,
		// Usage (Keyboard)
		0x09, 0x06,

		// Collection (Application)
		0xA1, 0x01,

		// Usage Page (Key Codes)
		0x05, 0x07,
		// Usage Minimum (224)
		0x19, 0xE0,
		// Usage Maximum (231)
		0x29, 0xE7,
		// Logical Minimum (0)
		0x15, 0x00,
		// Logical Maximum (1)
		0x25, 0x01,
		// Report Size (1)
		0x75, 0x01,
		// Report Count (8)
		0x95, 0x08,
		// Input (Data, Variable, Absolute): Modifier byte
		0x81, 0x02,

		// Report Size (8)
		0x75, 0x08,
		// Report Count (1)
		0x95, 0x01,
		// Input (Constant): Reserved byte
		0x81, 0x01,

		// Usage Page (LEDs)
		0x05, 0x08,
		// Usage Minimum (1)
		0x19, 0x01,
		// Usage Maximum (5)
		0x29, 0x05,
		// Report Size (1)
		0x75, 0x01,
		// Report Count (5)
		0x95, 0x05,
		// Output (Data, Variable, Absolute): LED report
		0x91, 0x02,

		// Report Size (3)
		0x75, 0x03,
		// Report Count (1)
		0x95, 0x01,
		// Output (Constant): LED report padding
		0x91, 0x01,

		// Usage Page (Key Codes)
		0x05, 0x07,
		// Usage Minimum (0)
		0x19, 0x00,
		// Usage Maximum (101)
		0x29, HID_KEYBOARD_KEYS - 1,
		// Logical Minimum (0)
		0x15, 0x00,
		// Logical Maximum(101)
		0x25, HID_KEYBOARD_KEYS - 1,
		// Report Size (8)
		0x75, 0x08,
		// Report Count (6)
		0x95, HID_KEYBOARD_MAX_KEYS,
		// Input (Data, Array): Keys
		0x81, 0x00,

		// End Collection
		0xC0
	};
	struct usb_hid_descriptor hid_descriptor = {
		.bLength = sizeof(hid_descriptor),
		.bDescriptorType = USB_DESCRIPTOR_TYPE_HID,
		.bcdHID = 0x0111,
		.bCountryCode = 0,
		.bNumDescriptors = 1,
		.bDescriptorType1 = USB_DESCRIPTOR_TYPE_REPORT,
		.wDescriptorLength1 = sizeof(report_descriptor)
	};
	struct usb_interface_descriptor if0_descriptor = {
		.bLength = sizeof(if0_descriptor),
		.bDescriptorType = USB_DESCRIPTOR_TYPE_INTERFACE,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		// Interface has 1 endpoint except EP0.
		.bNumEndpoints = 1,
		// HID.
		.bInterfaceClass = 0x03,
		// Keyboard could be boot interface.
		.bInterfaceSubClass = 0x01,
		// 1 is for keyboard, this is only valid for boot interface.
		.bInterfaceProtocol = 1,
		.iInterface = 0
	};
	struct usb_interface if0 = { .descriptor = &if0_descriptor,
				     .hid_descriptor = &hid_descriptor,
				     .report_descriptor = report_descriptor,
				     // Keyboard only needs 1 IN endpoint.
				     .n_endpoints = 1,
				     .endpoints = { &ep1_in } };
	struct usb_string_descriptor string_descriptor = {
		.bLength = sizeof(string_descriptor),
		.bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
		// English (United States).
		.wLANGID = 0x0409
	};
	struct usb_configuration_descriptor configuration_descriptor = {
		.bLength = sizeof(configuration_descriptor),
		.bDescriptorType = USB_DESCRIPTOR_TYPE_CONFIG,
		// To simplify I just manually calculate here, if you add more
		// interfaces, don't forget update this.
		.wTotalLength =
			(sizeof(configuration_descriptor) +
			 sizeof(if0_descriptor) + sizeof(hid_descriptor) +
			 sizeof(ep1_in_descriptor)),
		.bNumInterfaces = 1,
		// Configuration 1.
		.bConfigurationValue = 1,
		// No string.
		.iConfiguration = 0,
		// I grab those values from my keyboard.
		// Attributes: bus powered, remote wakeup.
		.bmAttributes = 0xa0,
		// 100 mA.
		.bMaxPower = 50
	};
	struct usb_device_descriptor descriptor = {
		.bLength = sizeof(descriptor),
		.bDescriptorType = USB_DESCRIPTOR_TYPE_DEVICE,
		// USB 1.1 device.
		.bcdUSB = 0x0110,
		// Specified in interface descriptor.
		.bDeviceClass = 0,
		// No subclass.
		.bDeviceSubClass = 0,
		// No protocol.
		.bDeviceProtocol = 0,
		// Max packet size for ep0.
		.bMaxPacketSize0 = PACKET_SIZE,
		// Your vendor id.
		.idVendor = 0xa3a7,
		// Your product ID.
		.idProduct = 0x0001,
		// No device revision number,
		.bcdDevice = 0x0000,
		// Manufacturer string index.
		.iManufacturer = 1,
		// Product string index.
		.iProduct = 2,
		// No serial number.
		.iSerialNumber = 3,
		// One configuration.
		.bNumConfigurations = 1
	};
	struct usb_device device = {
		.descriptor = &descriptor,
		.configuration_descriptor = &configuration_descriptor,
		.string_descriptor = &string_descriptor,
		.descriptor_strings = {
			// Vendor.
			"Alynx Zhou",
			// Product.
			"AZButton Pico",
			// Serial Number.
			// Actually it is a string, so just put anything I want.
			// You'd better change this if you are flashing
			// different chips, otherwise they are the same to one
			// host.
			"Strelizia"
		},
		.ep0_in = &ep0_in,
		.ep0_out = &ep0_out,
		.interfaces = {&if0},
		.address = 0,
		.should_set_address = false,
		.could_remote_wakeup = true,
		.configured = false,
		.suspended = false
	};
	struct app app = { .device = &device,
			   .last_button_events = 0,
			   .last_button_time = get_boot_ms() };

	this = &app;

	led_init();
	usb_init(this->device);
	button_init();

	// Start main loop.
	while (true)
		tight_loop_contents();

	return 0;
}
