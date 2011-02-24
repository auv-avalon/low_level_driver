#!/bin/bash

scp crumb_firmware.hex root@avalon:/tmp
ssh root@avalon avrdude -V -cavrisp2 -patmega128 -Pusb -Uflash:w:/tmp/crumb_firmware.hex -F
ssh root@avalon rm /tmp/crumb_firmware.hex
