/*
 * efergy.cpp
 * 
 * Copyright 2014 neil johnston 
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */

/* A program to decode the efergy fm data and log the power measurements
 * to file.
 * 
 * The program takes in demodulated data from rtl_fm which is tuned to
 * 433MHz (approximately). The rtl_fm is using some form of DVB-T USB
 * stick as its digitiser.
 * An EFERGY monitor is providing an FSK signal containg AC power 
 * measurements from a clamp meter on the mains of the house.
 * 
 * This program will sync to the structure of the data, details given 
 * below, and output the resulting power measurement to file. 
 * 
 * The program is based on the work of 
 *      Nathaniel Elijah 
 *      http://rtlsdr-dongle.blogspot.com.au/2013/11/finally-complete-working-prototype-of.html?
 * 
 *      Gough (me@goughlui.com)
 *      http://goughlui.com/?p=5109
 * 
 *      joesito
 *      http://electrohome.pbworks.com/w/page/34379858/Efergy-Elite-Wireless-Meter-Hack
 * 
 * The rtl_fm command line with pipe to efergy is
 *      rtl_fm -f 433550000 -s200000 -r96000 -g19.7 2>/dev/null | efergy -a0x0230ad -s power.log
 * 
 * This rtl_fm command line may not be optimal for the signal as it is 
 * using defaults. The frequency may be off, sample rate and filtering
 * may not be the best, further work may optimise this. 
 * The centre frequency can be found by tuning either side with an AM 
 * detector listening for the best tone either side, or some form of
 * spectrum analyser.
 * 
 * The protocol of the data is described on the above web pages and my 
 * understanding is recounted here.
 * 
 * Protocol
 * ========
 * The data starts with a long sequence of ones
 * Each data bit is PWM encoded 
 *      short pulse for a zero 
 *      long pulse for a one
 *      pulses are at the end of each data bit period
 *          so negative edge ends the data bit
 *          you can probably work it the other way round though.        
 * There are 8 bytes per packet of data
 * The bytes [0,1,2] are an address of the clamp meter
 * The byte [3] is a control byte giving the update period
 *      This control byte may also contain battery status information
 * The bytes [4,5] are the current reading, big endian
 * The byte [6] is a scaling factor for the current.
 *      This a signed byte so scaling is multplication & division.
 * The byte [7] is a checksum.
 * 
 * Packets are transmitted every 6 seconds, other efergy meters support
 * different periods. This the value encoded into byte [3].
 * 
 * With the rtl_fm parameters above we expect the start pulse to be >=40
 * samples and the data bits to be 18-20 samples long with a zero pulse
 * approximately 6 samples and a one 14 samples.
 * 
 * Algorithm for protocol byte recovery
 * ====================================
 *  1. detect the start pulse
 *  2. look for a negative edge, this is the sync for the data packet
 *  3. count ones seen
 *  4. on negative edge compare number of ones to one/zero threshold
 *  5. store 0/1 in byte
 *  6. loop to 2. until we have 8bytes
 *  7. check checksum byte and process packet
 *  8. loop to 1.
 * 
 * This is not particularly robust to noise as it is working on edges. 
 * But it's a lot simpler than implmenting clock recovery.
 * 
 * Logging
 * =======
 * So that we can use rrd we want to make sure we have no missing data.
 * When we don't decode anything we still need to output a power.
 * To do this we create a thread with a logging write every n seconds.
 * We update the power level to be the maximum in the last n seconds.
 * We write the same value out until we have a good value to log.
 * 
 * Compile
 * =======
 *  g++ -O3 -oefergy efergy.cpp -lpthread -lrrd
 * 
 * Testing
 * =======
 * Tested with a recording of the output of rtl_fm.
 * 
 *  rtl_fm -Alut -f433550000 -s200000 -r96000 -g19.7 | tee efergy_fm.raw > efergy.raw
 * 
 * The -Alut saved 50% cpu on the pi from not using it, went down to 25% 
 * 
 * Wait until some data appears then we have a file with good test data.
 * File efergy.raw can then be used for regression testing. 
 * 
 * Run valgrind for memory leaks
 * valgrind --leak-check=full --log-file=valgrind.txt -q -v efergy tst < efergy_fm.raw
 * valgrind --leak-check=full --log-file=valgrind.txt --track-origins=yes -q -v ./efergy tst < efergy_fm.raw
 * 
 *   Linking in rrd will fail valgrind, use #define DONT_LINK_RRD and
 *   recompile to check for leaks without rrd
 * 
 * Notes
 * =====
 * Found that there are other signals interfering with the expected 
 * efergy packets. Could be the CH controller or neighbours. Lots
 * of packets with 8bytes at least of zeros following a 5byte ones.
 * 
 */

#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <string>
#include <cstring>
#include <cerrno>
#include <ctime>
#include <csignal>
#include <map>
#include <cmath>

#include <unistd.h>
#include <pthread.h>

// comment out to use valgrind without the leaks in rrd
#define USE_RRD

#ifdef USE_RRD
#include <rrd.h>   // rrd may require apt-get install librrd-dev 
#endif

#define LENGTH_PROTOCOL_BYTES (9)
#define MIN_SYNC_PULSE_SAMPLE_WIDTH (40)
#define MIN_ONE_PULSE_WIDTH (18)

#define MIN_ONE_BIT_SAMPLE_WIDTH (40)
#define SYNC_WORD (0x2d)
#define DATA_START_POINT (5)

#define DEFAULT_VOLTAGE (230.0)
#define DEFAULT_LOG_PERIOD (1)
#define DEFAULT_STAT_PACKETS (100)

// logging thread needs access to the power
// so mutex lock and global variable
pthread_mutex_t dataLock;
double _power=0;
// structure for passing mutliple parmaeters into thread at creation
struct threadParams
{
	unsigned int delay;
	FILE *output;
	std::string rrdFilename;
};

// Global for exit on signal
bool _exitNow=false;

// some stats on times between packets
typedef std::map<unsigned int, unsigned long long,
		std::less<unsigned int> > mapOfDelayCounts;

static void signalHandler(int signal)
{
	_exitNow=true;
}

bool checksum(unsigned char * bytes, int length)
{
	// checksum is last byte
	// sum bytes and test against equivelance
	bool passed = false;
	unsigned char checksum = 0;
	for(int i=0; i<(length-2); i++)
	{
		checksum += bytes[i];
	}
	if (checksum-1 == bytes[length-2])
	{
		passed = true;
	}
	return(passed);
}

double getPower(unsigned char * currentBytes, float voltage)
{
	// currentBytes[3], 0,1 are the current - rightmost bit = 32

	// scaling byte conversion

	double power = 0.0;
	double current = static_cast<double>(currentBytes[0]<<6)*256.0 + static_cast<double>(currentBytes[1]<<6);

	power = ((voltage * current)/1000.0);

	return(power);
}

bool checkAddress(unsigned char *addressBytes,
		unsigned char *address, int length)
{
	bool match=false;
	if(memcmp(addressBytes, address, length)==0)
	{
		match=true;
	}
	return(match);
}


bool getPacket(unsigned char *packet, int length, FILE *input) {
	// look for our packet in the demodulated data

	// lots of variables dealing with byte extraction
	// from the oversampled data being read in
	int bitCount = 0;          // count of bits for a byte
	int byteCount = 0;         // index into packet array
	unsigned char byte;        // for byte building from bits
	int revsCount = 0;		   // count looking for first one bit in revs


	bool gotPacket = false;
	bool gotSync = false;

	while (!feof(input) && !gotPacket) {
		short lastSample = 0;
		short sample;
		bitCount = 0;

		if (!gotSync) {
			sample = static_cast<short>(fgetc(input) | (fgetc(input) << 8)); // get the input sample, little endian 16bits


			// Check if the sample is a 1 or 0
			if (sample >= 0) {
				revsCount++;
			} else {
				revsCount = 0;
			}

			if (revsCount >= MIN_ONE_BIT_SAMPLE_WIDTH) { // Check if we have minimum samples for a 1
				for (int i = 0; i < MIN_ONE_BIT_SAMPLE_WIDTH/2; ++i) { // We are at the end of the bit so offset by half a bit width to find center of next bit
					sample = static_cast<short>(fgetc(input)
							| (fgetc(input) << 8));
				}
				bool revsFlipFlop = true;
				// This may fail to pick up some packets as we see some with long 1 streak after ramp up
				while (!feof(input)) { // Look for end of revs with flip flop until we get 2 the same
					lastSample = sample;
					if (sample >= 0) {
						if (revsFlipFlop) {
							break;
						}
					} else {
						if (!revsFlipFlop) {
							break;
						}
					}

					revsFlipFlop = !revsFlipFlop;

					for (int i = 0; i < MIN_ONE_BIT_SAMPLE_WIDTH; ++i) { // Now step a full bit width to get center of next bit
						sample = static_cast<short>(fgetc(input)
								| (fgetc(input) << 8));
					}
				}
				bitCount = 1;
				byte = 0;

				byte = (byte << 1) | (lastSample >= 0 ? 1 : 0);
				while (!feof(input) && bitCount != 8) { // Look for end of revs
					for (int i = 0; i < MIN_ONE_BIT_SAMPLE_WIDTH; ++i) {
						sample = static_cast<short>(fgetc(input)
								| (fgetc(input) << 8));
					}
					int bit = sample >= 0 ? 1 : 0;
					byte = (byte << 1) | bit;
					bitCount++;
				}

				if (byte == SYNC_WORD) { // Establish whether the sync word is correct
					gotSync = true;
				}
			}
		} else { // start to build the packet
			gotSync = false;
			byteCount = 0;
			revsCount = 0;
			packet[byteCount++] = byte;

			while (!feof(input) && byteCount <= length) {
				bitCount = 0;

				while (!feof(input) && bitCount != 8) {
					for (int i = 0; i < MIN_ONE_BIT_SAMPLE_WIDTH; ++i) {
						sample = static_cast<short>(fgetc(input)
								| (fgetc(input) << 8));
					}
					int bit = sample >= 0 ? 1 : 0;
					//printf("%d", bit);
					byte = (byte << 1) | bit;
					bitCount++;
				}
				packet[byteCount++] = byte;
				byte = 0;
			}

			gotPacket = true;
		}

	} // while(!feof(input) && !gotPacket)

	return (gotPacket);
}

std::string getDateTime()
{
	// return a date time string
	// output is compatible with a standard rrd database input format
	// time will be in UTC so we don't have to worry about DST changes
	//
	// 2013-10-12 20:25:02
	//
	time_t now=time(0);
	struct tm *timeNow;
	timeNow=gmtime(&now);
	char buffer[80]={0};
	strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", timeNow);
	if(buffer[79]!=0)
	{
		fprintf(stderr, "Buffer overrun in strftime()\n");
		exit(1);
	}
	std::string dateTime=buffer;
	return(dateTime);
}

void logLatest(double power)
{
	FILE *latest=fopen("latest.txt", "w");
	if(latest)
	{
		std::string timeNow=getDateTime();
		fprintf(latest, "%s, %.0f\n", timeNow.c_str(), power);
		fclose(latest);
	}
}


void* logData(void *arg)
{
	// thread to log powers to file
	// arg is logging threadParams
	// logs to file every delay seconds
	// resets the global _power to zero every time it logs to file

	struct threadParams *params=static_cast<struct threadParams *>(arg);
	double lastPower=0;
	double power=0;
	bool rrdLogging=false;
	char *rrdArgs[3];
	char *rrdCommand=0;
	char *rrdFile=0;

	if(params->rrdFilename.size() > 0)
	{
		rrdLogging=true;
		rrdCommand=new char[50];
		rrdFile=new char[params->rrdFilename.size()+5];
		if(rrdCommand && rrdFile)
		{
			snprintf(rrdCommand, 49, "update");
			snprintf(rrdFile, params->rrdFilename.size()+1, "%s",
					params->rrdFilename.c_str());
			rrdArgs[0]=rrdCommand;
			rrdArgs[1]=rrdFile;
		}
		else
		{
			fprintf(stderr, "Failed on rrd arg creation\n");
			exit(1);
		}
	}

	while(!_exitNow)
	{

		// sync logging to the minute
		while ( (time(0) % 60) && !_exitNow )
		{
			sleep(1);
		}

		// lock access to the global _power
		pthread_mutex_lock(&dataLock);
		power=_power;
		_power=0;
		pthread_mutex_unlock(&dataLock);

		bool estimated=false;

		if(power == 0)
		{
			power=lastPower;
			estimated=true;
		}

		// logging to output file
		std::string timeNow=getDateTime();
		fprintf(params->output, "%s %.0f %c\n", timeNow.c_str(),
				power, (estimated?'e':' '));
		fflush(params->output);

		// logging to rrd
		if(rrdLogging)
		{
			char tmp[100]={0};
			snprintf(tmp, 99, "N:%.0f", power);
			rrdArgs[2]=tmp;

			//fprintf(stdout, "rrd %s %s %s\n", rrdArgs[0], rrdArgs[1], rrdArgs[2]);

#ifdef USE_RRD
			if(rrd_update(3, rrdArgs) == -1)
			{
				fprintf(stderr, "Error, rrd failed, %s\n",
						rrd_get_error());
				rrd_clear_error();
			}
#endif
		}

		// wait for next logging time, but allow quick exit
		int delay=(60*params->delay)-10;
		while(!_exitNow && delay--)
		{
			sleep(1);
		}

		lastPower=power;
	}
	fprintf(stderr, "Logging thread exit\n");

	delete [] rrdCommand;
	delete [] rrdFile;

	return NULL;
}

void outputStats(unsigned long long totalPackets,
		unsigned long long passedPackets, unsigned long long ourPackets,
		mapOfDelayCounts statsGood)
{
	FILE *statsF=fopen("stats.txt", "w");
	if (statsF)
	{
		fprintf(statsF, "Total packets: %llu\n", totalPackets);
		fprintf(statsF, "passed cksum : %llu\n", passedPackets);
		fprintf(statsF, "passed addr  : %llu\n", ourPackets);
		fprintf(statsF, "Offsets, passed address packets\n");
		std::map<unsigned int, unsigned long long>::const_iterator stat;
		for(stat=statsGood.begin(); stat!=statsGood.end(); stat++)
		{
			double pc=(100*static_cast<double>(stat->second))/ourPackets;
			fprintf(statsF, "\t%u sec, %llu, %.2f%%\n",
					stat->first, stat->second, pc);
		}
		fclose(statsF);
	}
	return;
}

void accumulatePower(double power)
{
	static double _totalPower=0.0;
	static time_t _lastTime=time(0);

	time_t now=time(0);

	// number of seconds between last and current
	double diff=difftime(now, _lastTime);
	diff=floor( (fabs(diff)+3) /6);

	// totaling in kw/hr
	_totalPower+=(power/600000)*diff;
	fprintf(stdout, "TOTAL: %.3f %.0f %.1f\n", _totalPower, power, diff);
	_lastTime=now;
	return;
}

void printHelp(char *programName)
{
	fprintf(stderr, "Usage: %s [-aAdhlrsv] logFile\n", programName);
	fprintf(stderr, "\n");
	fprintf(stderr, "Efergy meter decoder, requires rtl_fm as input\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "-a x  : Address x for filtering, eg 0x123456\n");
	fprintf(stderr, "-A    : All meter addresses used\n");
	fprintf(stderr, "-d    : Debug, prints all cksum passed packets\n");
	fprintf(stderr, "-D    : Debug, print all packets\n");
	fprintf(stderr, "-h    : This help\n");
	fprintf(stderr, "-l    : Log period in minutes, default %d\n",
			DEFAULT_LOG_PERIOD);
	fprintf(stderr, "-r x  : enable rrd logging to database file x\n");
	fprintf(stderr, "-s    : Stats every %d packets to stats.txt\n",
			DEFAULT_STAT_PACKETS);
	fprintf(stderr, "-v x  : Voltage to use, default %0.fv\n",
			DEFAULT_VOLTAGE);
	fprintf(stderr, "\n");
	return;
}

int main(int argc, char **argv)
{
	// we must have a short of 16bits
	assert(sizeof(short)==2);

	// signal handlers
	_exitNow=false;
	(void)signal(SIGINT, signalHandler);
	(void)signal(SIGTERM, signalHandler);

	// command line options
	bool debug=false;
	bool debugAll=false;
	bool ignoreAddress=false;
	bool statsOutput=false;
	std::string addressString;
	float voltage=DEFAULT_VOLTAGE;
	unsigned int logPeriod=DEFAULT_LOG_PERIOD;
	std::string rrdFilename="";

	// parse command line parameters
	opterr = 0;
	int command;
	while ((command = getopt (argc, argv, "a:AdDhl:r:sv:")) != -1)
	{
		switch (command)
		{
		case 'h':
			printHelp(argv[0]);
			exit(0);
			break;
		case 'd':
			debug=true;
			fprintf(stderr, "Debug to stdout enabled\n");
			break;
		case 'D':
			debugAll=true;
			fprintf(stderr, "Debug of all packets to stdout enabled\n");
			break;
		case 'A':
			ignoreAddress=true;
			fprintf(stderr, "Ignore of efergy address bytes enabled\n");
			break;
		case 'a':
		{
			addressString=optarg;
			break;
		}
		case 'l':
		{
			if(sscanf(optarg, "%u", &logPeriod)!=1)
			{
				fprintf(stderr, "Failed, can't convert '%s' from -l option to minutes\n", optarg);
				printHelp(argv[0]);
				exit(1);
			}
			else
			{
				fprintf(stderr, "Using %dminutes as log period\n", logPeriod);
			}
			break;
		}
		case 'r':
		{
			rrdFilename=optarg;
			break;
		}
		case 's':
		{
			statsOutput=true;
			break;
		}
		case 'v':
		{
			if(sscanf(optarg, "%f", &voltage)!=1)
			{
				fprintf(stderr, "Failed, can't convert '%s' from -v option to voltage\n", optarg);
				printHelp(argv[0]);
				exit(1);
			}
			else
			{
				fprintf(stderr, "Using %.0fvolts for power calculations\n", voltage);
			}
			break;
		}
		case '?':
		{
			if(optopt=='a')
				fprintf(stderr, "Failed, '-a' requires argument, eg -a0xab1234\n\n");
			if(optopt=='l')
				fprintf(stderr, "Failed, '-l' requires argument, eg -l10\n\n");
			if(optopt=='r')
				fprintf(stderr, "Failed, '-r' requires argument, eg -rpowers.rrd\n\n");
			if(optopt=='v')
				fprintf(stderr, "Failed, '-v' requires argument, eg -v240\n\n");
			printHelp(argv[0]);
			exit(1);
			break;
		}
		default:
			break;
		}
	}

	// get the output log filename
	if((argc-optind) != 1)
	{
		fprintf(stderr, "Failed, missing the log filename\n\n");
		printHelp(argv[0]);
		exit(1);
	}

	std::string filename;
	filename.append(argv[optind]);
	FILE *output=fopen(filename.c_str(), "a"); // append mode
	if(!output)
	{
		fprintf(stderr, "Failed, can't open log file '%s', %s",
				filename.c_str(), strerror(errno));
		exit(1);
	}
	else
	{
		fprintf(stderr, "Logging to '%s'\n", filename.c_str());
	}

	// rrd logging
	if(rrdFilename.size()>0)
	{
		// check file exists
		if(access(rrdFilename.c_str(), F_OK | R_OK | W_OK) == 0)
		{
			fprintf(stderr, "Logging to rrd file '%s'\n",
					rrdFilename.c_str());
		}
		else
		{
			fprintf(stderr, "Failed, can't open rrd file '%s', %s\n",
					rrdFilename.c_str(), strerror(errno));
			exit(1);
		}
	}

	// address filtering
	unsigned char address[3]={0};
	if(addressString.size()==0)
	{
		ignoreAddress=true;
		fprintf(stderr, "Warning, no address (-a option), ignoring addresses\n");
	}
	else
	{
		// always hate doing this bit
		// assuming the string is in the format "0x123456"
		int tmp[3];
		if(sscanf(addressString.c_str(), "0x%02x%02x%02x",
				&tmp[0], &tmp[1], &tmp[2]) == 3)
		{
			address[0]=tmp[0]&0xff;
			address[1]=tmp[1]&0xff;
			address[2]=tmp[2]&0xff;
			fprintf(stderr, "Using address '%02x%02x%02x' for filtering\n",
					address[0], address[1], address[2]);
		}
		else
		{
			fprintf(stderr, "Failed to parse address from '%s'\n",
					addressString.c_str());
			printHelp(argv[0]);
			exit(1);
		}
	}

	// create a thread to perform the logging
	int ptherr;
	pthread_t loggingTid=0;
	struct threadParams params;
	params.delay=logPeriod;
	params.output=output;
	params.rrdFilename=rrdFilename;
	ptherr=pthread_create(&loggingTid, NULL, &logData, &params);
	if(ptherr != 0)
	{
		fprintf(stderr, "Failed, can't create logging thread, %s\n",
				strerror(ptherr));
		exit(1);
	}
	else
	{
		fprintf(stderr, "created logging thread, logging every %u minute%c\n",
				logPeriod, (logPeriod>1)?'s':' ');
	}

	// packet holds extracted packet data protocol bytes
	unsigned char packet[LENGTH_PROTOCOL_BYTES];
	unsigned long long totalPackets=0;
	unsigned long long ourPackets=0;
	unsigned long long passedPackets=0;
	time_t lastPacketTime=time(0);
	mapOfDelayCounts statsGood;
	FILE *file;
	file = stdin;//fopen("/home/daniel/Desktop/git/efergy/rtl_fm_output.raw", "rb");//


	// the core of the program, loop until input ends
	// reading from stdin, if there is nothing coming in we will hang
	fprintf(stdout, "Reading from stdin, ctrl-d to close file\n");
	while (!_exitNow &&
			!feof(file) &&
			getPacket(packet, LENGTH_PROTOCOL_BYTES, file)
	)
	{
		totalPackets++;
		if((totalPackets%DEFAULT_STAT_PACKETS) == 0 )
		{
			outputStats(totalPackets, passedPackets, ourPackets, statsGood);
		}

		if(debugAll)
		{
			fprintf(stdout, "Packet: ");
			for(int b=0; b<LENGTH_PROTOCOL_BYTES; b++)
				fprintf(stdout, "%02x ", packet[b]);
			fprintf(stdout, "\n");
		}

		double power=0.0;
		bool passed=checksum(packet, LENGTH_PROTOCOL_BYTES);
		if(passed)
		{
			passedPackets++;
			if(ignoreAddress || checkAddress(&packet[1], address, sizeof(address)))
			{
				ourPackets++;

				if(statsOutput)
				{
					// record times between good packets
					time_t timeNow=time(0);
					statsGood[(timeNow-lastPacketTime)]++;
					lastPacketTime=timeNow;
				}

				// extract the power
				power = getPower(&packet[LENGTH_PROTOCOL_BYTES-DATA_START_POINT], voltage);

				// log latest to a file
				logLatest(power);

				// push the data to the logging thread
				// we record the maximum power in the logging interval
				// _power will be zero if it has been logged already
				pthread_mutex_lock(&dataLock);
				if(power>_power)
				{
					_power=power;
				}
				pthread_mutex_unlock(&dataLock);

				accumulatePower(power);
			}

			if(debug)
			{
				fprintf(stdout, "%.0f ", power);
				for(int i=0; i<LENGTH_PROTOCOL_BYTES; i++)
				{
					fprintf(stdout, "%02x", packet[i]);
				}
				fprintf(stdout, " %s\n", passed?"P":"F");
			}
		} // if(passed)
	}

	// clean up and exit
	_exitNow=true;
	if(loggingTid)
	{
		pthread_join(loggingTid, 0);
	}
	fclose(output);

	// stats on packets
	if(statsOutput)
	{
		outputStats(totalPackets, passedPackets, ourPackets, statsGood);
	}

	return(0);
}


