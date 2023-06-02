AZButton Pico
=============

A pico-based, single-button USB HID keyboard firmware.
------------------------------------------------------

# Usage

Before building this project, you need to prepare [pico-sdk](https://github.com/raspberrypi/pico-sdk/). You may need the `develop` branch if you are using GCC 13. And [this article](https://wellys.com/posts/rp2040_c_linux/) could be a good tutorial about how to setup it on Linux.

```
$ git clone https://github.com/AlynxZhou/azbutton-pico.git
$ cd azbutton-pico
$ mkdir build && cd build
$ PICO_SDK_PATH=/WHERE/YOU/CLONE/PICO/SDK cmake ..
$ PICO_SDK_PATH=/WHERE/YOU/CLONE/PICO/SDK make
```

Take out your pico, connect a LED to GPIO 1 and GND with a resistor, and connect a button to GPIO 2 and 3.3V, then press and hold BOOTSEL button on your pico and connect it with you computer, mount it to somewhere.

```
$ cp lib/azbutton-pico.uf2 /WHERE/YOU/MOUNT/PICO
```

It should re-connect to your computer, then you'll see it as a USB HID keyboard, press the button will send Enter.

# Modification

## Scancode

Search the code for `HID_KEYBOARD_SCANCODE_ENTER`, and replace them with other scancodes you define.

NOTE: If you want to use it as a modifier key (Ctrl, Alt or others), it's not the same, because HID uses different bytes for normal key and modifier key. You could edit `button_press()` and `button_release()` to cancel the normal key and change the modifier key, but I suggest to keep using Enter on it, and remap scancode on your system, for example, [use udev rules to remap keys](https://wiki.archlinux.org/title/Map_scancodes_to_keycodes#Using_udev).

## GPIO PIN

Search and change the value of `BUTTON_PIN` and `LED_PIN`.

## USB Vendor/Product ID and String

IDs are defined in device descriptor, and strings are defined in `descriptor_strings`.
