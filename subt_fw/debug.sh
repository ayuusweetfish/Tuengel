cat >build/gdbinit <<EOF
define hook-quit
  set confirm off
end
define hook-run
  set confirm off
end
define hookpost-run
  set confirm on
end
set pagination off
target extended-remote localhost:3333

b my_trap_line
commands
  silent
  printf "%s\n", my_buf
  c
end
EOF

~/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-gdb build/*.elf -x build/gdbinit
rm build/gdbinit
