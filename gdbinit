set remotelogfile gdb_log.txt
target remote 127.0.0.1:3333
set remote hardware-watchpoint-limit 2
mon reset halt
flushregs
thb *0x40007d54
tb app_main
# c
