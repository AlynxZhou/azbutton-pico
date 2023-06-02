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

Take out your pico, connect a LED to GPIO 7 and GND with a resistor, and connect a button to GPIO 3 and 3.3V, then press and hold BOOTSEL button on your pico and connect it with you computer, mount it to somewhere.

```
$ cp lib/azbutton-pico.uf2 /WHERE/YOU/MOUNT/PICO
```

It should re-connect to your computer, then you'll see it as a USB HID keyboard, press the button will send Enter. You could modify the code to use other key's scancode. You can also change Vendor/Product ID and string in code.
