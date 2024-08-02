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
#include <stdio.h>

#include "quat_cadence.h"
#include "app_quatBike.h"

#define TIME_MAX_SYSTIME   ((systime_t)-1)


//  EXTERNAL VARIABLES *****************

extern app_configuration *AppConf;
//extern t_ebike_model myBike;

// *****************************************

typedef struct {
	int16_t xh;
	int16_t xl;
	int16_t yh;
	int16_t yl;
	int16_t zh;
	int16_t zl;
	int16_t xOffset;
	int16_t yOffset;
	int16_t zOffset;
	uint8_t addr;
	uint8_t mode;
	uint8_t rate;
	uint8_t range;
	uint8_t oversampling;
	bool calibrating;
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
static void calibrateMagnetometer(int argc, const char **argv);
static void resetMag(int argc, const char **argv);

static int quat_read_norm_mag(void);

static volatile bool quat_cadence_thread_is_running = false;
static volatile bool quat_cadence_thread_stop_now = true;
static unsigned char rx_i2cBuffer[6];
static unsigned char tx_i2cBuffer[2];

static i2c_bb_state i2ccompass;
static t_magnetometer compass;

int16_t valors[3];
float myUT[3];
int16_t myHeading;
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
	terminal_register_command_callback(	"quat_resetmag", "Reset Magnetometer Data", "[] ", 	resetMag);
	terminal_register_command_callback("quat_calibrate", "Calibrate magnetometer", "[0/1 0/1", calibrateMagnetometer);
}

void quat_cadence_stop(void){
	terminal_unregister_callback(getMagnetometer);
	terminal_unregister_callback(resetMag);
	terminal_unregister_callback(calibrateMagnetometer);
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
	i2ccompass.rate = I2C_BB_RATE_100K;
	i2c_bb_init(&i2ccompass);

	compass.addr = QMC5883L_ADDR;
	compass.oversampling = QMC5883L_CONFIG_OS256;
	compass.range = QMC5883L_CONFIG_2GAUSS;
	compass.rate = QMC5883L_CONFIG_100HZ;
	compass.mode = QMC5883L_CONFIG_CONT;
	commands_printf("\nIniciado, no reseteado");
	quat_reset_magnetometer();
	commands_printf("\nReseteado");

	compass.xl = compass.xh = compass.yl = compass.yh = compass.zh = compass.zl = 0;
	compass.xOffset = AppConf->imu_conf.rot_roll*1000;
	compass.yOffset = AppConf->imu_conf.rot_pitch*1000;
	compass.zOffset = AppConf->imu_conf.rot_yaw*1000;
	commands_printf("\nInicializado Compass limits");
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

static int quat_read_raw_mag(int16_t* valores){
	tx_i2cBuffer[0] = QMC5883L_X_LSB;
	bool res = i2c_bb_tx_rx(&i2ccompass, compass.addr, tx_i2cBuffer, 1, rx_i2cBuffer, 6);
	if (!res) {
		return 0;
	}
	// Magnetometer values
	for (int i = 0; i < 3; i++) {
		uint16_t a = rx_i2cBuffer[2*i];
		uint16_t b = rx_i2cBuffer[2*i+1] << 8;
		valores[i] = (int16_t)  a | b;
	}
	return 1;
}

static int quat_conv_magnetometer_utesla(void){
	for (int i = 0; i < 3; i++) {
		myUT[i] =  (float)valors[i]/32767.0*200.0;
	}
	return 1;
}

static int quat_read_norm_mag(void){
	int16_t tmp_mag[3];
	if(!quat_read_raw_mag(tmp_mag)) return 0;

	valors[0] = tmp_mag[0] - compass.xOffset;
	valors[1] = tmp_mag[1] - compass.yOffset;
	valors[2] = tmp_mag[2] - compass.zOffset;
	return 1;
}

static int quat_calibrate_mag(void){
	int16_t tmp_mag[3];
	if(!quat_read_raw_mag(tmp_mag)) return 0;
	if (tmp_mag[0]<compass.xl) compass.xl = tmp_mag[0];
	if (tmp_mag[0]>compass.xh) compass.xh = tmp_mag[0];
	if (tmp_mag[1]<compass.yl) compass.yl = tmp_mag[1];
	if (tmp_mag[1]>compass.yh) compass.yh = tmp_mag[1];
	if (tmp_mag[2]<compass.zl) compass.zl = tmp_mag[2];
	if (tmp_mag[2]>compass.zh) compass.zh = tmp_mag[2];
	if ( (compass.xl >= compass.xh) || ((compass.yl >= compass.yh) | (compass.zl >= compass.zh)) )  return 0;
	compass.xOffset = (compass.xl+compass.xh)/2;
	compass.yOffset = (compass.yl+compass.yh)/2;
	compass.zOffset = (compass.zl+compass.zh)/2;

	valors[0] = tmp_mag[0] - compass.xOffset;
	valors[1] = tmp_mag[1] - compass.yOffset;
	valors[2] = tmp_mag[2] - compass.zOffset;
	return 1;
}

static void quat_get_heading(void){
		float heading = (float) 180.0* (float) atan2((double) valors[0], (double) -valors[1])/M_PI;
		if(heading<=0) heading += 360.0;
		myHeading = (int16_t) heading;
}

void quat_reset_magnetometer(void){
	bool res;
	i2c_bb_restore_bus(&i2ccompass);
	tx_i2cBuffer[0] = QMC5883L_RESET;
	tx_i2cBuffer[1] = 0x01;
	res = i2c_bb_tx_rx(&i2ccompass, compass.addr, tx_i2cBuffer, 2, rx_i2cBuffer, 0); //write_register(addr,QMC5883L_RESET,0x01);
	commands_printf("\nRESET result %s", res ? "true": "false");
	tx_i2cBuffer[0] = QMC5883L_CONFIG;
	tx_i2cBuffer[1] = compass.oversampling|compass.range|compass.rate|compass.mode;
	res = i2c_bb_tx_rx(&i2ccompass, compass.addr, tx_i2cBuffer, 2, rx_i2cBuffer, 0); //write_register(addr,QMC5883L_CONFIG,oversampling|range|rate|mode);
	commands_printf("\nCONFIG result %s", res ? "true": "false");
}

static THD_FUNCTION(quat_cadence_process_thread, arg) {
	(void)arg;
	chRegSetThreadName("QUAT Cadence");
	chThdSleep((systime_t) 30000);

	int16_t raw_mag_tmp[3];
	commands_printf("\nRESULTADO DE READY %s", quat_ready_magnetometer() ? "True": "False");
	commands_printf("\nRESULTADO DE READ DATA %s", quat_read_raw_mag(raw_mag_tmp) ? "True": "False");

	for(;;) {

		uint16_t cuentas = 0;
		// CONTROL THREAD FREQUENCY
		// systime_t sleep_time = CH_CFG_ST_FREQUENCY / AppConf->app_pas_conf.update_rate_hz;
		systime_t sleep_time = CH_CFG_ST_FREQUENCY/300.0;
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

		 if (quat_ready_magnetometer()) {
			 cuentas = 0;
			 if (compass.calibrating) {
				 if(quat_calibrate_mag()){
					 quat_get_heading();
					 quat_conv_magnetometer_utesla();
					 readOk = true;
				 } else {
					 quat_reset_magnetometer();
					 readOk = false;
				 }
			 } else {
				 if (quat_read_norm_mag() ){
					 quat_get_heading();
					 quat_conv_magnetometer_utesla();
					 readOk = true;
				 } else {
					 quat_reset_magnetometer();
					 readOk = false;
				 }
			 }
		 } else {
			 cuentas++;
			 if (cuentas >= 100){
				 readOk = false;
				 cuentas = 0;
			 }
		 }
	}
}

static void getMagnetometer(int argc, const char **argv) {
	(void) argv;
	if (argc == 1) {
		if (readOk){
			commands_printf("\nMX= %d\t MY = %d\t MZ = %d", valors[0], valors[1], valors[2]);
			commands_printf("\nB= %f uT \t BY = %f uT\t BZ = %f uT", (double) myUT[0], (double) myUT[1], (double) myUT[2]);
			commands_printf("\nXL = %d\t XH = %d\t YL = %d\t YH = %d\t ZL = %d\t ZH = %d", compass.xl, compass.xh, compass.yl, compass.yh, compass.zl, compass.zh);
			commands_printf("\nOffset X = %d\t Offset Y = %d\t Offset Z = %d", compass.xOffset, compass.yOffset, compass.zOffset);
			commands_printf("\n***********************************************");
			commands_printf("\nHEADING = %d", myHeading);
			commands_printf("\n***********************************************");
			commands_printf("\nCalibrating %s", (compass.calibrating) ? "true": "false");

		} else {
			commands_printf("\nERROR LECTURA MAGNETÓMETRO");
		}
	}
}
static void resetMag(int argc, const char **argv){
	(void) argv;
	if (argc == 1) {
		compass.addr = QMC5883L_ADDR;
		compass.oversampling = QMC5883L_CONFIG_OS256;
		compass.range = QMC5883L_CONFIG_2GAUSS;
		compass.rate = QMC5883L_CONFIG_100HZ;
		compass.mode = QMC5883L_CONFIG_CONT;
		commands_printf("\nIniciado, no reseteado");
		quat_reset_magnetometer();
		commands_printf("\nReseteado");

		if (compass.calibrating) {
			compass.xl = 32700;
			compass.yl = 32700;
			compass.zl =  32700;
			compass.xh = -32700;
			compass.yh = -32700;
			compass.zh = -32700;
			compass.xOffset = compass.yOffset = compass.zOffset = 0;
		} else {
			compass.xl = compass.xh = compass.yl = compass.yh = compass.zh = compass.zl = 0;
			compass.xOffset = AppConf->imu_conf.rot_roll*1000;
			compass.yOffset = AppConf->imu_conf.rot_pitch*1000;
			compass.zOffset = AppConf->imu_conf.rot_yaw*1000;
		}
		commands_printf("\nInicializado Compass limits");
	}
}

void save_mag_calibration(void){
	AppConf->imu_conf.rot_roll = compass.xOffset/1000.0;
	AppConf->imu_conf.rot_pitch = compass.yOffset/1000.0;
	AppConf->imu_conf.rot_yaw = compass.zOffset/1000.0;
	conf_general_store_app_configuration(AppConf);
}

static void calibrateMagnetometer(int argc, const char **argv){
	if (argc == 3) {
		int setCalibration = 0;
	    int saveCalibration = 0;
	   sscanf(argv[1], "%d", &setCalibration);
	   sscanf(argv[2], "%d", &saveCalibration);
	    if (saveCalibration>0 && compass.calibrating)
	    	save_mag_calibration();

	    if (setCalibration == 0)
	    	compass.calibrating = false;
	    else
	    	compass.calibrating = true;
	} else {
	    commands_printf("\n\r Uso quat_calibrate setCalibration, saveCalibration");
	  }
}
