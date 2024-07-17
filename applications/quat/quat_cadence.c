/*
 * quat_cadence.c
 *
 *  Created on: Mar 14, 2024
 *      Author: bicho
 */

#include "app.h"
#include "ch.h"
#include "hal.h"

#include "i2c_bb.h"
#include "utils.h"
#include "commands.h"
#include "terminal.h"


#include "quat_cadence.h"
#include "app_quatBike.h"

#define TIME_MAX_SYSTIME   ((systime_t)-1)


//  EXTERNAL VARIABLES *****************

extern volatile app_configuration *AppConf;
extern volatile t_ebike_model myBike;

// *****************************************

typedef struct {
	int16_t xh;
	int16_t xl;
	int16_t yh;
	int16_t yl;
	uint8_t addr;
	uint8_t mode;
	uint8_t rate;
	uint8_t range;
	uint8_t oversampling;
} t_magnetometer;

// Public variables

volatile float cadence_rpm;
volatile float wheel_rpm;
volatile float period_wheel;
volatile uint8_t PAS1_level;
volatile uint8_t PAS2_level;
volatile uint8_t WSPEED_LEVEL;


// Private variables
static THD_FUNCTION(quat_cadence_process_thread, arg);
static THD_WORKING_AREA(quat_cadence_process_thread_wa, 512);

static void getMagnetometer(int argc, const char **argv);

static volatile bool quat_cadence_thread_is_running = false;
static volatile bool quat_cadence_thread_stop_now = true;
static unsigned char rx_buf[6];
static unsigned char tx_buf[2];

static i2c_bb_state i2ccompass;
static t_magnetometer compass;

int16_t valors[3];
bool readOk = false;

// Private functions
void quat_reset_magnetometer(void);

void quat_cadence_start(void){
	quat_cadence_thread_stop_now = false;
	if (!quat_cadence_thread_is_running){
		chThdCreateStatic(quat_cadence_process_thread_wa, sizeof(quat_cadence_process_thread_wa), NORMALPRIO, quat_cadence_process_thread, NULL);
		quat_cadence_thread_is_running = true;
	}
	commands_printf("START QUAT Cadence");
	terminal_register_command_callback(	"quat_mag", "Output Magnetometer Data", "[] ", 	getMagnetometer);
}

void quat_cadence_stop(void){
	terminal_unregister_callback(getMagnetometer);
	quat_cadence_thread_stop_now = true;
	while (quat_cadence_thread_is_running) {
		chThdSleepMilliseconds(1);
	}
	commands_printf("STOP QUAT Cadence");
}

void quat_cadence_configure(void){
	// configuración cadencia
	i2ccompass.sda_gpio = HW_I2C_SDA_PORT;
	i2ccompass.sda_pin = HW_I2C_SDA_PIN;
	i2ccompass.scl_gpio = HW_I2C_SCL_PORT;
	i2ccompass.scl_pin = HW_I2C_SCL_PIN;
	i2ccompass.rate = I2C_BB_RATE_400K;
	i2c_bb_init(&i2ccompass);

	 compass.addr = QMC5883L_ADDR;
	 compass.oversampling = QMC5883L_CONFIG_OS64;
	 compass.range = QMC5883L_CONFIG_8GAUSS;
	 compass.rate = QMC5883L_CONFIG_100HZ;
	 compass.mode = QMC5883L_CONFIG_CONT;
	 quat_reset_magnetometer();
}

static int  quat_ready_magnetometer(void){
	uint8_t rxb[2];
	uint8_t txb[2];

	txb[0] = QMC5883L_STATUS;
	bool res = i2c_bb_tx_rx(&i2ccompass, compass.addr, txb, 1, rxb, 1);

	if (res) {
		return rxb[0] & QMC5883L_STATUS_DRDY;
	} else {
		return 0;
	}
}

static int quat_read_magnetometer(int16_t* valores){
	tx_buf[0] = QMC5883L_X_LSB;
	bool res = i2c_bb_tx_rx(&i2ccompass, compass.addr, tx_buf, 1, rx_buf, 6);
	if (!res) {
		return 0;
	}
	// Magnetometer values
	for (int i = 0; i < 3; i++) {
		valores[i] = (  (int16_t) rx_buf[2 * i]  |  ( (int16_t)  rx_buf[2 * i + 1] << 8) );
	}
	return 1;
}

void quat_reset_magnetometer(void){
	bool res;
	i2c_bb_restore_bus(&i2ccompass);
	tx_buf[0] = QMC5883L_RESET;
	tx_buf[1] = 0x01;
	res = i2c_bb_tx_rx(&i2ccompass, compass.addr, tx_buf, 2, rx_buf, 0); //write_register(addr,QMC5883L_RESET,0x01);
	commands_printf("\nRESET result %s", res ? "true": "false");
	tx_buf[0] = QMC5883L_CONFIG;
	tx_buf[1] = compass.oversampling|compass.range|compass.rate|compass.mode;
	res = i2c_bb_tx_rx(&i2ccompass, compass.addr, tx_buf, 2, rx_buf, 0); //write_register(addr,QMC5883L_CONFIG,oversampling|range|rate|mode);
	commands_printf("\nCONFIG result %s", res ? "true": "false");
}

static THD_FUNCTION(quat_cadence_process_thread, arg) {
	(void)arg;
	chRegSetThreadName("QUAT Cadence");

	quat_cadence_configure();
	int16_t raw_mag_tmp[3];
	commands_printf("\nRESULTADO DE READY %s", quat_ready_magnetometer() ? "True": "False");
	commands_printf("\nRESULTADO DE READ DATA %s", quat_read_magnetometer(raw_mag_tmp) ? "True": "False");

	for(;;) {

		// CONTROL THREAD FREQUENCY
		systime_t sleep_time = CH_CFG_ST_FREQUENCY / AppConf->app_pas_conf.update_rate_hz;
		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 1;
		}
		chThdSleep(sleep_time);

		// CONTROL IF THREAD HAS TO DIE
		if (quat_cadence_thread_stop_now) {
			quat_cadence_thread_is_running = false;
			return;
		}
		// NEW TIME STAMP
//		 const systime_t timestamp = chVTGetSystemTimeX(); //  /(float)CH_CFG_ST_FREQUENCY;
		 if (quat_ready_magnetometer()) {
			 if (quat_read_magnetometer(valors)){
				 readOk = true;
			 } else {
				 readOk = false;
			 }
		 }
	}
}

static void getMagnetometer(int argc, const char **argv) {
	(void) argv;
	if (argc == 1) {
		if (readOk){
			commands_printf("\nMX= %d\t MY = %d\t MZ = %d", valors[0], valors[1], valors[2]);
		} else {
			commands_printf("\nERROR LECTURA MAGNETÓMETRO");
		}
	}
}
