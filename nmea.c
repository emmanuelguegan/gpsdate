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

#include "nmea.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define ARRAY_SIZE(x)	(sizeof(x)/sizeof(*(x)))

static char data[79 + 3 + 1]; /* '$' + 79 characters + CR + LF + '\0' */
static size_t data_length;

static const char *tokens[79 + 1]; /* 79 plus one always empty */
static size_t tokens_length;

static int checksum;
static bool checksum_process;

/* Discard input until the next '$' (at startup, after a sentence ends, or on overflow): */
static bool wait_for_start = true;

static void reset_parser(void)
{
	data[0] = '\0';
	data_length = 0;

	tokens[0] = data;
	tokens_length = 0;

	checksum = 0;
	checksum_process = false;

	wait_for_start = false;
}

void nmea_parse(const char *buffer, size_t buffer_size,
		void (*callback)(const char *msgid, const char **data))
{
	int n;
	unsigned int val;

	if (!buffer || !callback)
		return;

	/* A for loop (not while) so that `continue` still advances to the next
	   byte; skipping the increment would loop forever on the same byte: */
	for (; buffer_size > 0 && *buffer; buffer++, buffer_size--) {
		if (wait_for_start && *buffer != '$')
			continue;

		switch (*buffer) {
			case '$': /* NMEA sentence start */
				reset_parser();
				checksum_process = true;
				break;
			case 10:
			case 13: /* NMEA sentence end (CR/LF) */
				if (data_length == 0 || tokens_length == 0) {
					reset_parser();
					break;
				}

				data[data_length++] = '\0';
				n = sscanf(tokens[tokens_length], "%x", &val);

				if (n == 1 && checksum == val) {
					/* First token is MSGID,
					   last  token is checksum: */
					tokens[tokens_length] = NULL;
					callback(tokens[0], &tokens[1]);
				}

				reset_parser();
				wait_for_start = true;
				break;
			case '*': /* Checksum delimiter (fall through) */
				checksum_process = false;
			case ',': /* Data delimiter */
				data[data_length++] = '\0';

				/* Buffer full: the sentence is too long to be valid
				   NMEA, so drop it without calling the callback. All
				   input is ignored until the next '$', which resets
				   the parser and starts a new sentence: */
				if (data_length >= ARRAY_SIZE(data) ||
				    tokens_length + 1 >= ARRAY_SIZE(tokens)) {
					wait_for_start = true;
					break;
				}

				tokens[++tokens_length] = &data[data_length];
				break;
			default:
				data[data_length++] = *buffer;
				/* Buffer full: drop the sentence, as above: */
				if (data_length >= ARRAY_SIZE(data))
					wait_for_start = true;
				break;
		}

		if (checksum_process && *buffer != '$') {
			checksum ^= *buffer;
		}
	}
}
