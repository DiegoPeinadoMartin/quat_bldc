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

#include "buffer.h"

#include <math.h>
#include <string.h>
#include <stdio.h>


#define NBYTESFRAME	17
#define MAXOLDMSG 		4

typedef struct {
	uint8_t send_buffer[NBYTESFRAME*(MAXOLDMSG+1)];
	int32_t ind;
	uint8_t npendientes;
}quat_can_frame_data_t;

static quat_can_frame_data_t can_msgs;

// Threads
static THD_FUNCTION(quat_send_data_thread, arg);
static THD_WORKING_AREA(quat_send_data_thread_wa, 512);

// Private variables
static volatile bool quat_send_data_stop_now = true;
static volatile bool quat_send_data_is_running = false;

extern volatile app_configuration *AppConf;

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void quat_send_data_start(void) {
	quat_send_data_stop_now = false;
	chThdCreateStatic(quat_send_data_thread_wa, sizeof(quat_send_data_thread_wa),
			NORMALPRIO, quat_send_data_thread, NULL);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void quat_send_data_stop(void) {
	quat_send_data_stop_now = true;
	while (quat_send_data_is_running) {
		chThdSleepMilliseconds(1);
	}
}

void quat_send_data_configure(void) {
	can_msgs.ind = 0;
	can_msgs.npendientes = 0;
}

void quat_set_can_msg(uint8_t est1, uint8_t est2, float w1, float w2, float w3, float w2ref1, float w2ref2, float current, float voltage, float factor, bool pendiente){
	can_msgs.ind = 0;
	uint8_t origen = 0;
	uint8_t *mibuffer;
	uint8_t estado = (est2 << 4) | (est1 & 0xF);

	if (pendiente){
		if (can_msgs.npendientes < MAXOLDMSG)
			can_msgs.npendientes++;
		origen = can_msgs.npendientes*NBYTESFRAME;
	}
	can_msgs.send_buffer[origen] = estado;
	can_msgs.ind++;
	mibuffer = &(can_msgs.send_buffer[origen]);
	buffer_append_float16(mibuffer, w1, 1e2, &can_msgs.ind);
	buffer_append_float16(mibuffer, w2, 1e2, &can_msgs.ind);
	buffer_append_float16(mibuffer, w3, 1e2, &can_msgs.ind);
	buffer_append_float16(mibuffer, w2ref1, 1e2, &can_msgs.ind);
	buffer_append_float16(mibuffer, w2ref2, 1e2, &can_msgs.ind);
	buffer_append_float16(mibuffer, current, 1e2, &can_msgs.ind);
	buffer_append_float16(mibuffer, voltage, 1e2, &can_msgs.ind);
	buffer_append_float16(mibuffer, factor, 1e2, &can_msgs.ind);
}

void quat_send_can_msg(void){
	uint8_t *mibuffer;
	if (can_msgs.npendientes){
		mibuffer = &can_msgs.send_buffer[can_msgs.npendientes*NBYTESFRAME];
		comm_can_transmit_sid(AppConf->controller_id, mibuffer, 5);
		comm_can_transmit_sid(AppConf->controller_id, mibuffer+5, 6);
		comm_can_transmit_sid(AppConf->controller_id, mibuffer+11, 6);
		can_msgs.npendientes--;
	} else {
		mibuffer = &can_msgs.send_buffer[0];
		comm_can_transmit_sid(AppConf->controller_id, mibuffer, 5);
		comm_can_transmit_sid(AppConf->controller_id, mibuffer+5, 6);
		comm_can_transmit_sid(AppConf->controller_id, mibuffer+11, 6);
	}
}

static THD_FUNCTION(quat_send_data_thread, arg) {
	(void)arg;
	chRegSetThreadName("QUAT Data Send");
	quat_send_data_is_running = true;
	for(;;) {
		// Check if it is time to stop.
		if (quat_send_data_stop_now) {
			quat_send_data_is_running = false;
			return;
		}
		quat_send_can_msg();
		timeout_reset(); // Reset timeout if everything is OK.
		chThdSleepMilliseconds(50);
	}
}
