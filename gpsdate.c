/*
 * This file is part of gpsdate.
 *
 * Copyright (C) 2014 Adam Heinrich <adam@adamh.cz>
 *
 * Gpsdate is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Gpsdate is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with gpsdate.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <signal.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>
#include <sys/time.h>
#include <math.h>

#include "nmea.h"
#include "serial_port.h"

#define DEFAULT_BAUDRATE	9600
#define DEFAULT_TIMEOUT		10
#define POLL_INTERVAL_MS	1000
/* the correcti is 1024 weeks : 1024*7*24*3600 */
#define ROLLOVER_CORRECTION	619315200
/* adjtime() slews the clock at only ~0.5 ms per second (1 s takes ~33 min),
   so larger offsets are stepped with settimeofday() instead: */
#define MAX_SLEW_USEC		1000000LL

#define TIME_FMT		"%04d-%02d-%02d %02d:%02d:%02d.%03ld"

static volatile bool read_gps = true;
static bool date_changed;

static bool opt_correct_rollover = false;
static bool opt_dry_run = false;

static const char *help_text =
"Sets time from a GPS receiver connected to a serial port as a local time.\n\n"
"Options:\n"
"  -b <baudrate>    Sets baud rate. Only a limited set of baud rates {2400,\n"
"                   4800, ..., 921600} is supported (Default %d baud).\n"
"  -t,-d <timeout>  Sets the maximum timeout in seconds or 0 for no timeout\n"
"                   (Default %d seconds)\n"
"  -h               Displays this help.\n"
"  -r               Correct GPS week number rollover\n"
"  -n               Dry run , Get the GPS time but don\'t apply it\n";


/* Subtract the ‘struct timeval’ values X and Y,
   storing the result in RESULT.
   Return 1 if the difference is negative, otherwise 0. */

int timeval_subtract (struct timeval *result, struct timeval *x, struct timeval *y)
{
  /* Perform the carry for the later subtraction by updating y. */
  if (x->tv_usec < y->tv_usec) {
    long int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
    y->tv_usec -= 1000000 * nsec;
    y->tv_sec += nsec;
  }
  if (x->tv_usec - y->tv_usec > 1000000) {
    long int nsec = (x->tv_usec - y->tv_usec) / 1000000;
    y->tv_usec += 1000000 * nsec;
    y->tv_sec -= nsec;
  }

  /* Compute the time remaining to wait.
     tv_usec is certainly positive. */
  result->tv_sec = x->tv_sec - y->tv_sec;
  result->tv_usec = x->tv_usec - y->tv_usec;

  /* Return 1 if result is negative. */
  return (x->tv_sec < y->tv_sec);
}

/* Converts an optional ".ddd..." suffix to microseconds (digits beyond 6 are
   ignored). Returns 0 when there is no fraction, -1 when it is malformed. */
static long parse_fraction_usec(const char *s)
{
	long usec = 0;
	long scale = 100000;

	if (*s == '\0')
		return 0;
	if (*s++ != '.')
		return -1;

	for (; *s; s++) {
		if (!isdigit((unsigned char)*s))
			return -1;
		usec += (*s - '0') * scale;
		scale /= 10;
	}

	return usec;
}

static void print_help(bool full_help, const char *program_name)
{
	if (full_help) {
		printf("Usage: %s [options] port\n", program_name);
		printf(help_text, DEFAULT_BAUDRATE, DEFAULT_TIMEOUT);
	} else {
		fprintf(stderr, "Usage: %s [-bt] port\n", program_name);
		fprintf(stderr, "Run `%s -h` for help.\n", program_name);
	}
}

static void process_message(const char *msgid, const char **data)
{
	size_t length = 0;
	time_t rollover_correction = (time_t)0;
	struct	timeval sys_timev, 
			gps_timev, 
			delta_timev;

	if ( opt_correct_rollover == true ) rollover_correction = (time_t)ROLLOVER_CORRECTION;

	while (data[length])
		length++;

	/* Accept RMC from any talker (GPRMC, GNRMC, GLRMC, ...). Field count
	   is 11 before NMEA 2.3, 12 with mode (2.3), 13 with nav status (4.1): */
	if (strlen(msgid) == 5 && strcmp(msgid + 2, "RMC") == 0 &&
	    length >= 11 && length <= 13) {

		/* Only parse time when there is a fix: */
		if (strcmp(data[1], "A") != 0)
			return;

		/* Parse UTC time (hhmmss[.sss]): */
		int day, month, year, hours, minutes, seconds;
		int consumed = 0;
		long usec;
		const char *tfmt = "%02d%02d%02d";

		if (sscanf(data[0], "%02d%02d%02d%n", &hours, &minutes, &seconds,
			   &consumed) != 3 || consumed != 6)
			return;

		usec = parse_fraction_usec(data[0] + consumed);
		if (usec < 0)
			return;

		if (sscanf(data[8], tfmt, &day, &month, &year) != 3)
			return;

		year += 2000;

		if ((day >= 1 && day <= 31) &&
		    (month >= 1 && month <= 12) &&
		    (year >= 2000) &&
		    (hours >= 0 && hours <= 23) &&
		    (minutes >= 0 && minutes <= 59) &&
		    (seconds >= 0 && seconds <= 59)) {

			/* Get current time, print it and modify: */
			
			gettimeofday(&sys_timev, 0);

			struct tm *t = gmtime(&sys_timev.tv_sec);

			printf("Local time was: " TIME_FMT " (%s)\n",
			       (t->tm_year + 1900), (t->tm_mon + 1), t->tm_mday,
			       t->tm_hour, t->tm_min, t->tm_sec,
			       (long)sys_timev.tv_usec / 1000, t->tm_zone);
			printf("GPS   time  is: " TIME_FMT " (%s)\n",
			       year, month, day, hours, minutes, seconds,
			       usec / 1000, t->tm_zone);


			t->tm_year = year - 1900;
			t->tm_mon = month - 1;
			t->tm_mday = day;
			t->tm_hour = hours;
			t->tm_min = minutes;
			t->tm_sec = seconds;

			/* Set new system time: */
			gps_timev.tv_sec = timegm(t);

			gps_timev.tv_sec+=rollover_correction;
			gps_timev.tv_usec = usec;

			if ( opt_correct_rollover == true )  {

				struct tm *new_t = gmtime(&gps_timev.tv_sec);
				printf("1024 weeks Corrected time is: " TIME_FMT " (%s)\n",
                        	       (new_t->tm_year + 1900), (new_t->tm_mon + 1), new_t->tm_mday,
	                               new_t->tm_hour, new_t->tm_min, new_t->tm_sec,
				       usec / 1000, new_t->tm_zone);
			}
			if ( opt_dry_run == false ) {
				timeval_subtract(&delta_timev,&gps_timev,&sys_timev);
				long long delta_usec = (long long)delta_timev.tv_sec * 1000000LL +
						       delta_timev.tv_usec;
				if (llabs(delta_usec) < MAX_SLEW_USEC) {
					printf("Adjtime (slewing by %+.3f s) : ",
					       (double)delta_usec / 1e6);
					if (adjtime(&delta_timev,NULL) == 0 )  {
                                                printf("Successfully updated local time.\n");
                                                date_changed = true;
                                        } else {
                                                fprintf(stderr, "Couldn't set local time. "
                                                        "Do you have root privileges?\n");
                                        }

						
				}
				else
				{
					printf("settime : ");
					if (settimeofday(&gps_timev, NULL) == 0) {
						printf("Successfully updated local time.\n");
						date_changed = true;
					} else {
						fprintf(stderr, "Couldn't set local time. "
							"Do you have root privileges?\n");
					}	
				}
			}
			else	
			{
				 printf("Dry run : Time no set\n");
			}

			read_gps = false;
		}
	}
}

static void signal_handler(int signo)
{
	read_gps = false;
}

int main(int argc, char **argv)
{
	const char *port_name = NULL;
	int timeout = DEFAULT_TIMEOUT;
	int baudrate = DEFAULT_BAUDRATE;

	/* Handle options: */
	int c;
	while ((c = getopt(argc, argv, "hrnb:d:t:")) != -1) {
		switch (c) {
		case 'b':
			sscanf(optarg, "%d", &baudrate);
			break;
		case 't':
		case 'd': /* For backward compatibility ("delay"): */
			sscanf(optarg, "%d", &timeout);
			break;
		case 'r':
			opt_correct_rollover = true;
			break;
                case 'n':
                        opt_dry_run = true;
                        break;
		case 'h':
			print_help(true, argv[0]);
			return 0;
		default:
			print_help(false, argv[0]);
			return 1;
		}
	}

	/* Open port: */
	if (optind < argc) {
		port_name = argv[optind];
	} else {
		print_help(false, argv[0]);
		return 1;
	}

	int fd = serial_port_open(port_name, baudrate);
	if (fd == -1) {
		fprintf(stderr, "Can't open port %s at %d baud\n",
			port_name, baudrate);
		return 1;
	}

	signal(SIGINT, &signal_handler);
	signal(SIGTERM, &signal_handler);

	time_t start_time, curr_time;
	time(&start_time);

	/* Read everything available as soon as it arrives, so no backlog
	   builds up at high update rates and the parsed time is fresh: */
	char buffer[4096];
	struct pollfd pfd = { .fd = fd, .events = POLLIN };

	while (read_gps) {
		time(&curr_time);
		if (timeout > 0 && (curr_time - start_time) >= timeout) {
			printf("No valid time information from GPS in the last "
			       "%d seconds.\n", timeout);
			break;
		}

		/* Wake up at least once a second to check the timeout: */
		int ready = poll(&pfd, 1, POLL_INTERVAL_MS);
		if (ready < 0 && errno != EINTR) {
			perror("poll");
			break;
		}
		if (ready <= 0)
			continue;

		ssize_t nread = read(fd, buffer, sizeof(buffer));
		if (nread > 0) {
			nmea_parse(buffer, (size_t)nread, &process_message);
		} else if (nread == 0 || (errno != EAGAIN && errno != EINTR)) {
			fprintf(stderr, "Lost connection to %s\n", port_name);
			break;
		}
	}

	close(fd);

	return date_changed ? 0 : 2;
}
