#! /bin/sh


docker run --rm -it -v $PWD:/module/ --device /dev/ttyUSB0 --privileged -P bitcraze/aideck-nina /bin/bash -c \
    "/openocd-esp32/bin/openocd -f interface/ftdi/olimex-arm-usb-ocd-h.cfg -f board/esp-wroom-32.cfg \
    -c 'program_esp32 build/bootloader/bootloader.bin 0x1000 verify' \
    -c 'program_esp32 build/module.bin 0x10000 verify' \
    -c 'program_esp32 build/partitions_singleapp.bin 0x8000 verify \
    reset exit'"


# python /esp-mdf/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 
# 0x1000 /module/build/bootloader/bootloader.bin 
# 0x10000 /module/build/module.bin 
# 0x8000 /module/build/partitions_singleapp.bin
