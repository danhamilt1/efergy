Repository cloned from original owner at: https://bitbucket.org/neil_johnston/efergy.git

# A decoder for the efergy elite meter protocol#
The efergy meter is a home electrical power meter. It uses a clamp meter on the mains input to the distribution board. The clamp measures the current and every 6seconds sends off the reading using FSK on 433MHz. Normally a small display picks this up and intergates up the power used to give you a display showing how much power (VA really) you are using.

* ####The meter i have is an npower elite 2.0R meter which appears to be a re-branded efergy elite.

We use a FSK/FM demodulator to decode the signal from the clamp meter to provide bits to this program. This allows us to see more real time data than that available from the normal display. 

We can see power every 6 seconds, so even a switch on/off of a light can be logged. We can produce nice graphs of what is happening.

The decoder expects to be fed from an rtl_fm demodulator tuned to 433MHz (or
thereabouts). The rtl_fm could be using a dvb-t usb stick to provide the 
raw digitised samples.

As reading every 6seconds is rather a lot of data the program will by default only log the maximum in the last 60 seconds. This cuts things down a lot and allows graphs to be produced with enough detail to catch things but not excessively.

### What is this repository for? ###
For anybody to copy and use the code.

* It was written for fun (?!), so uses C++ just to play (no classes involved).
* It will output statistics of times between packets.
* It can log to an rrd database if required.
* It will log every 60 seconds by default, the maximum power in the last interval is logged.
* It can print out all packets that pass the checksum in debug mode.
* The code contains a description of the algorithm for packet recovery.

### How do I get set up? ###

* Compile the code
    * g++ -O3 -oefergy efergy.cpp -lpthread -lrrd
* Configuration
    * No configuration required.
* Dependicies
    * librrd
    * pthread
    * rtl_fm for demodulated data
    * something to feed the rtl_fm with samples, dvb-t usb stick?
    * raspberry pi (or some other Linux box)
* Database configuration
    * There is a script rrdCreate.sh which gives an example rrd database.
    * The default logging is every 60 seconds.
* How to run tests
    * efergy -h will return the command line options.
    * rtl_fm -Alut -f433550000 -s200000 -r96000 -g10 2> /dev/null | ./efergy -a0x0230ad -s -rtt.rrd power.log
    * The -a0x0230ad is my meters address. Removing this will default to logging all packets that pass the checksum.
* Deployment instructions
    * Left to the user.

### TODO ###
* Need something to plot the data
    * gnuplot
    * highcharts
* Make the program directly interface to rtl_fm.
* Add in accumulating power logging. Currently running at around 98.5 receive of all Tx'd packets so error would be less than 1%. Get better than 99.5% if i include stats for only 1 or 2 missed packets.

### Links ###
The following were useful when creating this program.

* [Original link I followed](http://rtlsdr-dongle.blogspot.com.au/2013/11/finally-complete-working-prototype-of.html?) 
* [Blog post that covers some more points](http://goughlui.com/?p=5109)
* [Description of the packet protocol](http://electrohome.pbworks.com/w/page/34379858/Efergy-Elite-Wireless-Meter-Hack)
* [Installing a dvb-t / rtl_fm on a raspberry pi](http://zr6aic.blogspot.co.uk/2013/02/setting-up-my-raspberry-pi-as-sdr-server.html)
* [highcharts plotting of data](http://blog.tafkas.net/2012/10/03/gathering-and-charting-temperatures-using-rrdtool-and-highcharts/) - still todo

