# Mini Bridge

Mini Bridge is a USB to serial adaptor for mobile devices (includes iPhone and iPad).
## Setting up the Build Environment (Linux/Ubuntu)

### Install `arm-none-eabi-*` toolchain
Download `arm-none-eabi-*` tools from [gnu-toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads), extract files, and include `gcc-arm-none-eabi-xxx/bin/` to `$PATH` variable in ubuntu by adding the following line to ` ~/.bashrc`.
``` bash
export PATH="/absolute-path-to-gcc-arm-none-eabi-xxx/bin:$PATH"
```
### Install `jq`
``` bash
> sudo apt install jq
```
## Build the code 

This project is designed to build with `make` command. You won't need to initialise submodules because make will initialise required submodules for you. 

``` bash
> git clone https://github.com/shuanglengyunji/mini-bridge.git
> cd mini-bridge
> make
```

## Installation

Use the [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) to flash the firmware in the bootloader mode. 

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License
[MIT](https://choosealicense.com/licenses/mit/)
