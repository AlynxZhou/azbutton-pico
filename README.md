AZButton Pico
=============

A pico-based, single-button USB HID keyboard firmware.
------------------------------------------------------

# Usage

Before building this project, you need to prepare [pico-sdk](https://github.com/raspberrypi/pico-sdk/). You may need the `develop` branch if you are using a very new GCC. And [this article](https://wellys.com/posts/rp2040_c_linux/) could be a good tutorial about how to setup it on Linux. On macOS, you need to install `gcc-arm-embedded` for cross compiling.

```
$ git clone https://github.com/AlynxZhou/azbutton-pico.git
$ cd azbutton-pico
$ mkdir build && cd build
$ PICO_SDK_PATH=/WHERE/YOU/CLONE/PICO/SDK cmake -DUSB_SERIAL=CHANGETHISEACHBOARD ..
$ PICO_SDK_PATH=/WHERE/YOU/CLONE/PICO/SDK make
```

If you are not using the original Pico from Raspberry Pi, you may need to define your board model, for example, I am using a WaveShare RP2040-Zero, then I use the following CMake command:

```
$ PICO_SDK_PATH=/WHERE/YOU/CLONE/PICO/SDK cmake -DPICO_BOARD=waveshare_rp2040_zero -DUSB_SERIAL=CHANGETHISEACHBOARD ..
```

List of board models could be find in [this directory of pico-sdk](https://github.com/raspberrypi/pico-sdk/tree/master/src/boards/include/boards).

**NOTE**: If you happen to have the same WaveShare RP2040-Zero board as mine, you may got [this bug](https://github.com/raspberrypi/pico-sdk/issues/1304) which prevents it from working after unplug and replug the USB cable, I already submit [a PR to fix the bug](https://github.com/raspberrypi/pico-sdk/pull/1421).

Take out your pico, connect a LED to GPIO 1 and GND with a resistor, and connect a button to GPIO 2 and 3.3V, then press and hold BOOTSEL button on your pico and connect it with you computer, mount it to somewhere.

```
$ cp lib/azbutton-pico.uf2 /WHERE/YOU/MOUNT/PICO
```

It should re-connect to your computer, then you'll see it as a USB HID keyboard, press the button will send F23.

# Modification

## Scancode

Search the code for `HID_KEYBOARD_SCANCODE_KEY`, and replace them with other scancodes you define.

NOTE: If you want to use it as a modifier key (Ctrl, Alt, Shift or GUI), it's not the same, because HID uses different bytes for normal key and modifier key. You could edit `button_press()` and `button_release()` to empty the normal key array and change the modifier key byte, but I suggest to keep using Compose (Application) key in code, and remap it on your system, for example, [use udev rules to remap keys](https://wiki.archlinux.org/title/Map_scancodes_to_keycodes#Using_udev). Some keys won't work on macOS, for example F21~F23.

## GPIO PIN

Search and change the value of `BUTTON_PIN` and `LED_PIN`.

## USB Vendor/Product ID and String

IDs are defined in device descriptor, and strings are defined in `descriptor_strings`.

## USB Serial

You should use different USB serial for different boards, otherwise your system may treat them as the same one, you could set USB serial by changing `-DUSB_SERIAL=CHANGETHISEACHBOARD` argument of CMake.
