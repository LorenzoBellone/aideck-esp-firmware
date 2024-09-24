docker run --rm -it -v $PWD:/module/ --device /dev/ttyUSB0 --privileged -P nina_esp_mdf /bin/bash -c "cd /module; source /esp-mdf/export.sh; ls /;make clean ;make menuconfig; make all"
