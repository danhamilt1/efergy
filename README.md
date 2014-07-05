# A decoder for the efergy meter protocol#
The decoder expects to be fed from an rtl_fm demodulator tuned to 433MHz (or
thereabouts). The rtl_fm could be using a dvb-t usb stick to provide the 
raw digitised samples.

### What is this repository for? ###
For anybody to copy and use the code.

* It was written for fun, so uses C++ just to play (no classes involved).
* It will output statistics of times between packets.
* It can log to an rrd database if required.

* Version 1.0

### How do I get set up? ###

* Compile the code
    * g++ -O3 -oefergy efergy.cpp -lpthread -lrrd
* No configuration required
* Dependicies
    * librrd
    * pthread
* Database configuration
    * There is a script rrdCreate.sh which gives an example rrd database
* How to run tests
    * efergy -h will return the command line options
    * rtl_fm -Alut -f433550000 -s200000 -r96000 -g10 2> /dev/null | ./efergy -a0x0230ad -s -rtt.rrd power.log
* Deployment instructions
    * Left to the user

### Who do I talk to? ###

* Neil Johnston