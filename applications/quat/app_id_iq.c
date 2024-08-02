/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "app.h"
#include "ch.h"
#include "hal.h"

// Some useful includes
#include "mc_interface.h"
#include "utils_math.h"
#include "encoder/encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 1024);

// Private functions
static void get_id_iq(int argc, const char **argv);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile float myId;
static volatile float myIq;

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {
	stop_now = false;
	chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa),
			NORMALPRIO, my_thread, NULL);

	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
			"set_id_iq",
			"Set both intensities, Id and Iq",
			"[f f]",
			get_id_iq);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	terminal_unregister_callback(get_id_iq);
	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

void app_custom_configure(app_configuration *conf) {
	(void)conf;
}

static THD_FUNCTION(my_thread, arg) {
	(void)arg;

	chRegSetThreadName("App SET ID IQ");

	is_running = true;

	for(;;) {
		// Check if it is time to stop.
		if (stop_now) {
			is_running = false;
			return;
		}

		timeout_reset(); // Reset timeout if everything is OK.
		mc_interface_set_current_id_iq(myId, myIq);
		// Run your logic here. A lot of functionality is available in mc_interface.h.

		chThdSleepMilliseconds(50);
	}
}

static void get_id_iq(int argc, const char **argv) {
	if (argc == 3) {
		sscanf(argv[1], "%f", &myId);
		sscanf(argv[2], "%f", &myIq);
		commands_printf("\nId = %.2f A \tIq = %.2f", (double) myId, (double) myIq);
		myId *= -1;
	} else {
		commands_printf("This command requires one argument.\n");
	}
}
