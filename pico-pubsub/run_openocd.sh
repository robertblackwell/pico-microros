#!/bin/bash

${PICO_OPENOCD_PATH}/src/openocd -s ${PICO_OPENOCD_PATH}/tcl/ -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000"
