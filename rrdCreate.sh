#!/bin/sh

createRrd()
{
        # create RRD databse to hold min,max,avg for hour,day,week,month,year
        name=$1

        # power every 60 seconds for 1 source
        # timeout is 600 seconds
	# min is zero max is 30000watts
        # rra's of hour, day, week month and year
        # min average and max values
        rrdtool create $name.rrd --start N --step 60 \
        DS:$name:GAUGE:600:0:30000 \
        RRA:MIN:0.5:1:60 \
        RRA:MIN:0.5:1:1440 \
        RRA:MIN:0.5:60:168 \
        RRA:MIN:0.5:60:720 \
        RRA:MIN:0.5:1440:365 \
        RRA:AVERAGE:0.5:1:60 \
        RRA:AVERAGE:0.5:1:1440 \
        RRA:AVERAGE:0.5:60:168 \
        RRA:AVERAGE:0.5:60:720 \
        RRA:AVERAGE:0.5:1440:365 \
        RRA:MAX:0.5:1:60 \
        RRA:MAX:0.5:1:1440 \
        RRA:MAX:0.5:60:168 \
        RRA:MAX:0.5:60:720 \
        RRA:MAX:0.5:1440:365
}

createRrd tt


