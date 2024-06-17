/*
 * quat_cadence.c
 *
 *  Created on: Mar 14, 2024
 *      Author: bicho
 */

#include "app.h"
#include "ch.h"
#include "hal.h"

#include "utils.h"
#include "commands.h"

#include "quat_cadence.h"
#include "app_quatBike.h"

#define TIME_MAX_SYSTIME   ((systime_t)-1)


//  EXTERNAL VARIABLES *****************

extern volatile app_configuration *AppConf;
extern volatile t_ebike_model myBike;

// *****************************************

#define HW_QUAD_SENSOR1_PORT									HW_UART_P_TX_PORT
#define HW_QUAD_SENSOR1_PIN										HW_UART_P_TX_PIN
#define HW_QUAD_SENSOR2_PORT									HW_UART_P_RX_PORT
#define HW_QUAD_SENSOR2_PIN										HW_UART_P_RX_PIN
#define HW_QUAD_WHEEL_SENSOR_PORT  					HW_ICU_GPIO
#define HW_QUAD_WHEEL_SENSOR_PIN  						HW_ICU_PIN

#define MAX_WHEEL_PERIOD													3

static THD_FUNCTION(quat_cadence_process_thread, arg);
static THD_WORKING_AREA(quat_cadence_process_thread_wa, 512);

static volatile bool quat_cadence_thread_is_running = false;
static volatile bool quat_cadence_thread_stop_now = true;

static volatile float max_pulse_period = 0.0;
static volatile float min_cadence_period = 0.0;
static volatile float direction_conf = 0.0;
volatile float cadence_rpm = 0;
volatile float wheel_rpm = 0;

const int8_t QEM[] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0}; // Quadrature Encoder Matrix
int8_t direction_qem;
uint8_t new_state;
static uint8_t old_state = 0;
static systime_t old_timestamp = 0;
static float timeinterval = 0;
static systime_t old_timestamp_wheel = 0;
static float inactivity_time = 0;
static float period_filtered = 0;
float period_cadence = 0;
volatile float period_wheel = 1e20;
//static int32_t correct_direction_counter = 0;

 volatile uint8_t PAS1_level = 0;
 volatile uint8_t PAS2_level = 0;
static volatile uint8_t WSPEED_LEVEL = 0;
static volatile uint8_t WSPEED_LEVEL_old = 0;

void quat_cadence_start(void){
	quat_cadence_thread_stop_now = false;
	if (!quat_cadence_thread_is_running){
		chThdCreateStatic(quat_cadence_process_thread_wa, sizeof(quat_cadence_process_thread_wa), NORMALPRIO, quat_cadence_process_thread, NULL);
		quat_cadence_thread_is_running = true;
	}
	commands_printf("START QUAT Cadence");
}

void quat_cadence_stop(void){
	quat_cadence_thread_stop_now = true;
	while (quat_cadence_thread_is_running) {
		chThdSleepMilliseconds(1);
	}
	commands_printf("STOP QUAT Cadence");
}

void quat_cadence_configure(void){
	// configuración cadencia
		max_pulse_period = 1.0 / ((AppConf->app_pas_conf.pedal_rpm_start / 60.0) * AppConf->app_pas_conf.magnets) * 1.2;
		min_cadence_period = 1.0 / ((AppConf->app_pas_conf.pedal_rpm_end * 3.0 / 60.0));
		(AppConf->app_pas_conf.invert_pedal_direction) ? (direction_conf = -1.0) : (direction_conf = 1.0);

		commands_printf("Max pulse period = %f", (double) max_pulse_period);
		commands_printf("Min cadence period = %f", (double) min_cadence_period);
		commands_printf("direction_conf = %f", (double) direction_conf);
}

static THD_FUNCTION(quat_cadence_process_thread, arg) {
	(void)arg;
	chRegSetThreadName("QUAT Cadence");
	palSetPadMode(HW_QUAD_SENSOR1_PORT, HW_QUAD_SENSOR1_PIN, PAL_MODE_INPUT_PULLUP); // sensor de cadencia
	palSetPadMode(HW_QUAD_SENSOR2_PORT, HW_QUAD_SENSOR2_PIN, PAL_MODE_INPUT_PULLUP); // sensor de cadencia
	palSetPadMode(HW_QUAD_WHEEL_SENSOR_PORT, HW_QUAD_WHEEL_SENSOR_PIN, PAL_MODE_INPUT_PULLUP); // sensor velocidad de la rueda

	for(;;) {
		//period_cadence = 0;
		//cadence_rpm = 0;

		systime_t sleep_time = CH_CFG_ST_FREQUENCY / AppConf->app_pas_conf.update_rate_hz;
		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 1;
		}
		chThdSleep(sleep_time);

		if (quat_cadence_thread_stop_now) {
			quat_cadence_thread_is_running = false;
			return;
		}

		PAS1_level = palReadPad(HW_QUAD_SENSOR1_PORT, HW_QUAD_SENSOR1_PIN);
		PAS2_level = palReadPad(HW_QUAD_SENSOR2_PORT, HW_QUAD_SENSOR2_PIN);
		WSPEED_LEVEL = palReadPad(HW_QUAD_WHEEL_SENSOR_PORT, HW_QUAD_WHEEL_SENSOR_PIN);

		old_state = new_state;
		new_state = PAS2_level * 2 + PAS1_level;
		direction_qem = (float) QEM[old_state * 4 + new_state];

		const systime_t timestamp = chVTGetSystemTimeX(); //  /(float)CH_CFG_ST_FREQUENCY;


		if (WSPEED_LEVEL && !WSPEED_LEVEL_old){
			period_wheel = (timestamp - old_timestamp_wheel)  / (float)CH_CFG_ST_FREQUENCY;
			old_timestamp_wheel = timestamp;
		}
		if (period_wheel > MAX_WHEEL_PERIOD) {
			wheel_rpm = 0;
		} else {
			wheel_rpm = 60.0/period_wheel;
		}

		if (direction_qem == 2) continue;
		if( new_state == 3 && direction_qem != 0) {
			inactivity_time = 0.0;
			timeinterval = timestamp - old_timestamp;
			if (timestamp < old_timestamp) {
				timeinterval += TIME_MAX_SYSTIME;
			}
			period_cadence = (timestamp - old_timestamp) / (float)CH_CFG_ST_FREQUENCY * (float)AppConf->app_pas_conf.magnets;
			old_timestamp = timestamp;
			UTILS_LP_MOVING_AVG_APPROX(period_filtered, period_cadence, myBike.myStats.Ncad);
			if(period_filtered < min_cadence_period) { //can't be that short, abort
				continue;
			}
			cadence_rpm = 60.0 / period_filtered;
			cadence_rpm *= (direction_conf * (float)direction_qem);
		}	else {
			inactivity_time += 1.0 / (float)AppConf->app_pas_conf.update_rate_hz;
			// if no pedal activity, set RPM as zero
			if(inactivity_time > max_pulse_period) {
				cadence_rpm = 0.0;
			}
		}
		WSPEED_LEVEL_old = WSPEED_LEVEL;
	}
}
