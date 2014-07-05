#!/bin/sh

#VALGRIND="valgrind --leak-check=full --log-file=valgrind.txt -v"

OPTIONS="-a0x0230ad -s -rtt.rrd power.log"

cd ../bin
rtl_fm -Alut -f433550000 -s200000 -r96000 -g10 2> /dev/null | ./efergy $OPTIONS 
 

