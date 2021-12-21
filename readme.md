# Mini Bridge

Mini Bridge is a USB to serial adaptor for mobile devices (includes iPhone and iPad).

## Structure
``` text
.
├── _build
│   └── stm32f4
├── compile_commands.json
├── generate_compile_commands.sh
├── inc
├── lib
│   ├── CMSIS_5
│   ├── cmsis_device_f4
│   ├── FreeRTOS-Kernel
│   ├── lwip
│   ├── stm32f4xx_hal_driver
│   └── tinyusb
├── Makefile
├── readme.md
├── src
│   ├── arch
│   ├── asset
│   ├── FreeRTOSConfig
│   ├── freertos_hook.c
│   ├── lwipopts.h
│   ├── main.c
│   ├── main.h
│   ├── net.c
│   ├── net.h
│   ├── ssi_example.cxx
│   ├── tusb_config.h
│   └── usb_descriptors.c
├── stm32f4
│   ├── board.c
│   ├── board.h
│   ├── board_mcu.h
│   ├── board.mk
│   ├── STM32F401VCTx_FLASH.ld
│   ├── stm32f4xx_hal_conf.h
│   └── utility.c
└── web
    ├── fs
    ├── fsdata.c
    └── makefsdata
```

Sources go in [src/](src/), board drivers go in [stm32f4/](stm32f4/), sources of web interface go in [web/](web/), third party libraries go in [lib/](lib/).

## Building

This project is designed to build with make. Make will determine submodules to initialise at the build time, so you won't need to initialise submodules. 

The `arm-none-eabi-*` tools can be downloaded from [gnu-toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads).

Example:

``` bash
> git clone https://github.com/shuanglengyunji/mini-bridge.git
> cd mini-bridge
> make
```

## Installation

Use the [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) to flash the firmware in the bootload mode. 

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License
[MIT](https://choosealicense.com/licenses/mit/)
