if [ "$1" == "f" ]; then    # flash
  ~/.platformio/packages/tool-openocd/bin/openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000; init; reset halt; rp2040.core1 arp_reset assert 0; rp2040.core0 arp_reset assert 0; reset halt; program build/*.elf verify; reset; exit"
  exit
elif [ "$1" == "u" ]; then  # upload through USB
  cp build/*.uf2 /Volumes/RPI-RP2
  exit
elif [ "$1" == "s" ]; then  # serve
  ~/.platformio/packages/tool-openocd/bin/openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000; init"
  exit
elif [ "$1" == "r" ]; then  # serve, reset
  ~/.platformio/packages/tool-openocd/bin/openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000; init; reset"
  exit
elif [ "$1" == "s" ]; then  # serve, stop
  ~/.platformio/packages/tool-openocd/bin/openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000; init; halt"
  exit
elif [ "$1" == "m" ]; then  # monitor serial
  screen /dev/tty.usbmodem* 115200
  exit
fi

if ! [ -d "build" ]; then
  mkdir build
fi
cd build
PICO_TOOLCHAIN_PATH=~/.platformio/packages/toolchain-gccarmnoneeabi/bin cmake .. -DCMAKE_BUILD_TYPE=${TYPE:-Debug}
make

