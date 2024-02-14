/*
 * app_quatBike.c
 *
 *  Created on: Feb 6, 2024
 *      Author: bicho
 */

// Basic includes: app, chibios, and STM32Hall
#include "app.h"
#include "ch.h"
#include "hal.h"

// Some useful includes
#include "mc_interface.h"
#include "utils.h"
#include "encoder/encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "app_quatBike.h"

// Threads POINTERS
static THD_FUNCTION(quat_thread, arg);
static THD_WORKING_AREA(quat_thread_wa, 1024);

static THD_FUNCTION(quat_display_process_thread, arg);
static THD_WORKING_AREA(quat_display_process_thread_wa, 2048);

static THD_FUNCTION(quat_cadence_process_thread, arg);
static THD_WORKING_AREA(quat_cadence_process_thread_wa, 2048);

// START AND STOP VARIABLES
static volatile bool Quat_stop_now = true;
static volatile bool Quat_is_running = false;

static volatile bool quat_display_thread_is_running = false;
static volatile bool quat_display_thread_stop_now = true;
static volatile bool quat_display_uart_is_running = false;

static volatile bool quat_cadence_thread_is_running = false;
static volatile bool quat_cadence_thread_stop_now = true;

// ************************************ COMMUNICATIONS DEFINES AND VARIABLES
#define QUAT_DISPLAY_BAUD	9600
#define MAX_TXBUFF 13
#define MAX_RXBUFFSETTINGS 15
#define MAX_RXBUFFRUNNING  10
#define QUAT_TX_BUFFER_SIZE 52
#define QUAT_RX_BUFFER_SIZE 60
#define CMD_HEAD			0x3A
#define CMD_ADDRESS			0x1A
#define CMD_SETTINGS		0x53
#define CMD_RUNNING			0x52
#define CMD_END1			0x0D
#define CMD_END2			0x0A
#define CHECKSUM_START_SETTINGS	11
#define CHECKSUM_START_RUNNNING	6
#define DATA_LENGTH_SETTINGS	7
#define DATA_LENGTH_RUNNNING	2
#define DATA_LENGTH_ANSWER		5

typedef struct {
	unsigned int rd_ptr;
	unsigned int wr_ptr;
	unsigned char data[QUAT_RX_BUFFER_SIZE];
	unsigned char tx[QUAT_TX_BUFFER_SIZE];
} quat_serial_buffer_t;

const unsigned char password[64] = {
		137, 159, 134, 249, 88, 11, 250, 61, 33, 150,
		3, 193, 118, 141, 209, 94, 226, 68, 146, 158,
		145, 127, 216, 62, 116, 230, 101, 211, 251, 54,
		229, 247, 20, 222, 59, 63, 35, 252, 142, 238,
		23, 197, 84, 77, 147, 173, 210, 57, 142, 223,
		157, 97, 36, 160, 229, 237, 75, 80, 37, 113,
		154, 88, 23, 120
};

static SerialConfig uart_cfg = { QUAT_DISPLAY_BAUD, 0, USART_CR2_LINEN, 0};
static quat_serial_buffer_t serial_buffer;


// ************************************ CADENCE VARIABLES

static volatile float max_pulse_period = 0.0;
static volatile float min_cadence_period = 0.0;
static volatile float direction_conf = 0.0;
static volatile float cadence_rpm = 0;

const int8_t QEM[] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0}; // Quadrature Encoder Matrix
int8_t direction_qem;
uint8_t new_state;
static uint8_t old_state = 0;
static float old_timestamp = 0;
static float inactivity_time = 0;
static float period_filtered = 0;
float period_cadence = 0;
//static int32_t correct_direction_counter = 0;

static volatile uint8_t PAS1_level = 0;
static volatile uint8_t PAS2_level = 0;

// ************************************ CADENCE VARIABLES

// ************************************ MAIN APP VARIABLES

static volatile app_configuration *AppConf;
static const volatile mc_configuration *mc_conf;

static bool sendGraphs = false;
float samp = 0.0;
uint16_t periodo;

typedef enum {
	STOP = 0,
	CLOSING_GAP,
	STABLE,
	SLIPING
} t_motor_state;


typedef struct {
	float pedal_rpm;
	float motor_rpm;
	float motor_erpm;
	float chainring_rpm;
} t_bicycloidal_system;

typedef struct {
	uint8_t motor_periodH;
	uint8_t motor_periodL;
	uint8_t motor_intensity;
	uint8_t statusBatt;
	uint8_t statusMot;
	uint8_t progDisplay;
} t_display_data;

typedef struct {
	float bike_velocity;
	float bike_intensity;
	float assistance_program;
	float motor_reference_erpm;
} t_ebike_variables;

typedef struct{
	float WheelRadius;
	float Rtransmision;
	float K;
	float I;
} t_ebike_conf;

typedef struct {
	t_bicycloidal_system myBicycloidal;  // Variables del bicicloidal
	t_display_data myDisplayData; 	// Variables enviadas al display
	t_ebike_variables myVariables;  // estado de la bicicleta/motor
	t_ebike_conf myConf;  // configuración de la ebike
} t_ebike_model;

static t_ebike_model myBike;


static float AP[] = {0, 0.25, 0.50, 0.75, 1.0, 1.5};

void recalculaEstado(void);
static void setGraphOn(int argc, const char **argv);
static void getProgram(int argc, const char **argv);
void sendGraphs_experiment(void);


// ************************************* UTILITY ***************************************************
/*!

 * * \brief Utility function to convert from kph to erpm.
 */
float get_erpm_from_kph(float kmph){
  return (kmph/(3.6*mc_conf->si_wheel_diameter*M_PI)*(30.0*mc_conf->si_motor_poles*mc_conf->si_gear_ratio));
}
/*!
 * \brief Utility function to convert from erpm to kph..
 */
float get_kph_from_erpm(float miErpm){
  return (miErpm*(3.6*mc_conf->si_wheel_diameter*M_PI)/(30.0*mc_conf->si_motor_poles*mc_conf->si_gear_ratio));
}
/*!
 * \brief Utility function to convert from RPM to km/h.
 */
float get_kph_from_rpm(float miRpm){
  return (miRpm*(3.6*mc_conf->si_wheel_diameter*M_PI)/(30.0*mc_conf->si_gear_ratio));
}
// ************************************* UTILITY ***************************************************

// ************************************* CADENCE ***************************************************

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

// ************************************* CADENCE ***************************************************


// ************************************* DISPLAY ***************************************************
void quat_display_serial_start(void) {
	quat_display_thread_stop_now = false;
	if (!quat_display_thread_is_running) {
	  chThdCreateStatic(quat_display_process_thread_wa, sizeof(quat_display_process_thread_wa), NORMALPRIO, quat_display_process_thread, NULL);
	  quat_display_thread_is_running = true;
	}
	serial_buffer.rd_ptr = 0;
	serial_buffer.wr_ptr = 0;

	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
		  PAL_STM32_OSPEED_HIGHEST |
		  PAL_STM32_PUDR_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
		  PAL_STM32_OSPEED_HIGHEST |
		  PAL_STM32_PUDR_PULLUP);

	sdStart(&HW_UART_DEV, &uart_cfg);
	quat_display_uart_is_running = true;
	commands_printf("START QUAT SerialDisp");
}

void quat_display_serial_stop(void){
	quat_display_thread_stop_now = true;
	while (quat_display_thread_is_running) {
		chThdSleepMilliseconds(1);
	}
	commands_printf("STOP QUAT SerialDisp");
}

static uint16_t quat_display_serial_checksum(uint8_t *buf, uint8_t len) {
  uint16_t sum = 0;
  for(int i = 0; i<len; i++) {
      sum += buf[i];
  }
  return sum;
}

static void quat_display_serial_send_packet(unsigned char *data, unsigned int len) {
	if (quat_display_uart_is_running) {
		sdWrite(&HW_UART_DEV, data, len);
	}
}

static void quat_display_serial_byte_process(unsigned char byte) {
	serial_buffer.data[serial_buffer.wr_ptr++] = byte;
	uint8_t rd_ptr;
	uint16_t checksum_income;
	uint16_t checksum_outcome;
	uint8_t checksum_addr;
	uint8_t checksum_start = 0;
	uint8_t cmd_received;

	rd_ptr = serial_buffer.rd_ptr;	//read pointer to try at different start addresses
	// process with at least 3 bytes available to read
	while( (serial_buffer.wr_ptr - rd_ptr ) > 2) {
		if(serial_buffer.data[rd_ptr] == CMD_HEAD && serial_buffer.data[rd_ptr+1] == CMD_ADDRESS) {
			// start bytes found
			cmd_received = serial_buffer.data[rd_ptr+2];
			if ( cmd_received == CMD_SETTINGS){
				if( (serial_buffer.wr_ptr - rd_ptr) < MAX_RXBUFFSETTINGS)
					return;
				checksum_start = CHECKSUM_START_SETTINGS;

			} else if ( cmd_received  == CMD_RUNNING){
				if( (serial_buffer.wr_ptr - rd_ptr) < MAX_RXBUFFRUNNING)
					return;
				checksum_start = CHECKSUM_START_RUNNNING;
			}
			if ( !(serial_buffer.data[rd_ptr+checksum_start+2]==CMD_END1 &&
					 serial_buffer.data[rd_ptr+checksum_start+3]==CMD_END2)
			){
				// ha ido algo mal porque tengo todos los bytes del frame pero no coincide
				// el final del frame. Elimino la cabecera y sigo buscando un frame entero
				// quizás podría eliminar toda la longitud del frame, pero entonces igual me
				// cargo un frame entero. Esta solución puede ser más lenta, pero quizás más
				// segura
				serial_buffer.rd_ptr += 2;
				return;
			}
			checksum_addr = rd_ptr + checksum_start;
			if(checksum_addr <= serial_buffer.wr_ptr) {	//check the checksum has been received
				checksum_income = serial_buffer.data[checksum_addr]+ (serial_buffer.data[checksum_addr+1] <<8);
				// check sum
				if( checksum_income == quat_display_serial_checksum(serial_buffer.data + rd_ptr + 1, checksum_start-1) ) {
					// paquete correcto, obtener datos del display
					myBike.myDisplayData.progDisplay = serial_buffer.data[rd_ptr+4];
					// enviar paquete al display con info de la ebike
					serial_buffer.tx[0] = CMD_HEAD;
					serial_buffer.tx[1] = CMD_ADDRESS;
					serial_buffer.tx[2] = cmd_received;
					serial_buffer.tx[3] = DATA_LENGTH_ANSWER;
					serial_buffer.tx[4] = myBike.myDisplayData.statusBatt;
					serial_buffer.tx[5] = myBike.myDisplayData.motor_intensity;
					serial_buffer.tx[6] = myBike.myDisplayData.motor_periodH;
					if (cmd_received == CMD_SETTINGS){
						serial_buffer.tx[7] = password[serial_buffer.data[rd_ptr + 9]];
					} else {
						serial_buffer.tx[7] = myBike.myDisplayData.motor_periodL;
					}
					serial_buffer.tx[8] = myBike.myDisplayData.statusMot;
					checksum_outcome = quat_display_serial_checksum(serial_buffer.tx+1, DATA_LENGTH_ANSWER+2);
					serial_buffer.tx[9] = checksum_outcome & 0xFF;
					serial_buffer.tx[10] = (checksum_outcome >> 8) & 0xFF;
					serial_buffer.tx[11] = CMD_END1;
					serial_buffer.tx[12] = CMD_END2;
					quat_display_serial_send_packet(serial_buffer.tx, MAX_TXBUFF);
					if (cmd_received == CMD_SETTINGS)
						serial_buffer.rd_ptr = rd_ptr + MAX_RXBUFFSETTINGS-1;	//mark bytes as read
					else
						serial_buffer.rd_ptr = rd_ptr + MAX_RXBUFFRUNNING-1;	//mark bytes as read
				}
			}
		}
		rd_ptr++;
	}
	if(serial_buffer.rd_ptr > 0) {
		memmove(serial_buffer.data, serial_buffer.data + serial_buffer.rd_ptr, QUAT_RX_BUFFER_SIZE - serial_buffer.rd_ptr);
		 serial_buffer.wr_ptr -= serial_buffer.rd_ptr;
		 serial_buffer.rd_ptr = 0;
	}
	if(serial_buffer.wr_ptr == (QUAT_RX_BUFFER_SIZE - 1) ) {
		//shift buffer to the left discarding the oldest byte
		memmove(serial_buffer.data,serial_buffer.data + 1, QUAT_RX_BUFFER_SIZE - 1);
		serial_buffer.wr_ptr -= 1;
	}
}

static void quat_display_serial_check_rx(void){
	bool rx = true;
	while (rx) {
		rx = false;
	    if (quat_display_uart_is_running) {
	    	msg_t res = sdGetTimeout(&HW_UART_DEV, TIME_INFINITE);
	    	if (res != MSG_TIMEOUT) {
	    		quat_display_serial_byte_process(res);
	    		rx = true;
	    	}
	    }
	}
}

// ************************************* DISPLAY ***************************************************

// ************************************* MAIN APP EBIKE ***************************************************

void app_custom_start(void) {
	Quat_stop_now = false;
	if (!Quat_is_running){
		chThdCreateStatic(quat_thread_wa, sizeof(quat_thread_wa), NORMALPRIO, quat_thread, NULL);
		commands_printf("START QuatApp");
	}

	terminal_register_command_callback(
		"quat_pa",
		"Output PA",
		"[] ",
		getProgram);

	terminal_register_command_callback(
		"quat_graph",
		"Output real time values to the experiments graph",
		"[On(1)/Off(0)] ",
		setGraphOn);
}

void app_custom_stop(void) {
	Quat_stop_now = true;
	while (Quat_is_running) {
		chThdSleepMilliseconds(1);
	}
	commands_printf("STOP QuatApp");
	quat_display_serial_stop();
	quat_cadence_stop();
	terminal_unregister_callback(getProgram);
	terminal_unregister_callback(setGraphOn);
}

void app_custom_configure(app_configuration *conf) {
	AppConf = conf;
	mc_conf = mc_interface_get_configuration();

	// configuración cadencia
	max_pulse_period = 1.0 / ((AppConf->app_pas_conf.pedal_rpm_start / 60.0) * AppConf->app_pas_conf.magnets) * 1.2;
	min_cadence_period = 1.0 / ((AppConf->app_pas_conf.pedal_rpm_end * 3.0 / 60.0));
	(AppConf->app_pas_conf.invert_pedal_direction) ? (direction_conf = -1.0) : (direction_conf = 1.0);

	commands_printf("Max pulse period = %f", (double) max_pulse_period);
	commands_printf("Min cadence period = %f", (double) min_cadence_period);
	commands_printf("direction_conf = %f", (double) direction_conf);

	// configuración para pruebas
	myBike.myConf.K = (1.0 - 1.0/58.0)*(1.0 - 1.0/51.0);
	myBike.myConf.I = myBike.myConf.K / ( 1.0 - myBike.myConf.K);

	myBike.myConf.WheelRadius = mc_conf->si_wheel_diameter/2.0;
	myBike.myConf.Rtransmision = mc_conf->si_gear_ratio;
	myBike.myBicycloidal.chainring_rpm = 00;
	myBike.myVariables.bike_velocity = RPM2RADPS_f(myBike.myBicycloidal.chainring_rpm * myBike.myConf.Rtransmision) * myBike.myConf.WheelRadius;
	periodo = 60000/(myBike.myBicycloidal.chainring_rpm * myBike.myConf.Rtransmision);
	myBike.myDisplayData.motor_periodH = (periodo >> 8) & 0xFF;
	myBike.myDisplayData.motor_periodL = periodo & 0xFF;
}

void app_init_graphs(void){
	commands_init_plot("Motor RPM", "Motor Ref");
	commands_plot_add_graph("Motor ERPM");
	commands_plot_add_graph("Reference ERPM");
	commands_plot_add_graph("Pedal Cadence");
	commands_plot_add_graph("ChainRing Cadence");
}

void recalculaEstado(void){
	myBike.myBicycloidal.motor_erpm = mc_interface_get_rpm();
	myBike.myBicycloidal.motor_erpm = (myBike.myBicycloidal.motor_erpm > 0.01) ? myBike.myBicycloidal.motor_erpm: 0.0;

	myBike.myBicycloidal.motor_rpm = myBike.myBicycloidal.motor_erpm / (mc_conf->si_motor_poles/2.0);
	myBike.myBicycloidal.chainring_rpm = cadence_rpm;

	myBike.myBicycloidal.pedal_rpm = (myBike.myBicycloidal.chainring_rpm - (1.0-myBike.myConf.K)*myBike.myBicycloidal.motor_rpm)/myBike.myConf.K;
	//myBike.myState.bike_velocity = RPM2RADPS_f(myBike.myBicycloidal.chainring_rpm * myBike.myConf.Rtransmision) * myBike.myConf.WheelRadius;
	myBike.myVariables.bike_velocity = get_kph_from_rpm(myBike.myBicycloidal.chainring_rpm);
	if (myBike.myBicycloidal.chainring_rpm>0.01){
		periodo = 60000/(myBike.myBicycloidal.chainring_rpm * myBike.myConf.Rtransmision);
		periodo = (periodo > 3500) ? 3500: periodo;
	} else periodo = 3500;
	myBike.myDisplayData.motor_periodH = (periodo >> 8) & 0xFF;
	myBike.myDisplayData.motor_periodL = periodo & 0xFF;
	myBike.myVariables.assistance_program = AP[(uint8_t) myBike.myDisplayData.progDisplay/51];
	myBike.myVariables.motor_reference_erpm = myBike.myConf.I*myBike.myVariables.assistance_program*myBike.myBicycloidal.pedal_rpm*mc_conf->si_motor_poles/2.0;
}

// ************************************* MAIN APP EBIKE ***************************************************



// ************************************* THREADS ***************************************************

static THD_FUNCTION(quat_display_process_thread, arg) {
  (void)arg;
  chRegSetThreadName("QUAT SerialDisp");
  chThdSleepMilliseconds(500);
  for(;;) {
    chThdSleepMilliseconds(100);
    quat_display_serial_check_rx();
    timeout_reset();
  }
}


static THD_FUNCTION(quat_cadence_process_thread, arg) {
	(void)arg;
	chRegSetThreadName("QUAT Cadence");
	palSetPadMode(HW_QUAD_SENSOR1_PORT, HW_QUAD_SENSOR1_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_QUAD_SENSOR2_PORT, HW_QUAD_SENSOR2_PIN, PAL_MODE_INPUT_PULLUP);

	systime_t sleep_time2 = CH_CFG_ST_FREQUENCY / AppConf->app_pas_conf.update_rate_hz;
	commands_printf("sleep time = %u", sleep_time2);
	for(;;) {

		//period_cadence = 0;
		//cadence_rpm = 0;

		systime_t sleep_time = CH_CFG_ST_FREQUENCY / AppConf->app_pas_conf.update_rate_hz;
		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 1;
		}
		chThdSleep(sleep_time);
		PAS1_level = palReadPad(HW_QUAD_SENSOR1_PORT, HW_QUAD_SENSOR1_PIN);
		PAS2_level = palReadPad(HW_QUAD_SENSOR2_PORT, HW_QUAD_SENSOR2_PIN);

		old_state = new_state;
		new_state = PAS2_level * 2 + PAS1_level;
		direction_qem = (float) QEM[old_state * 4 + new_state];

		const float timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;

		if (direction_qem == 2) continue;
		if( new_state == 3 && direction_qem != 0) {
			period_cadence = (timestamp - old_timestamp) * (float)AppConf->app_pas_conf.magnets;
			old_timestamp = timestamp;
			UTILS_LP_FAST(period_filtered, period_cadence, 0.1);
			if(period_filtered < min_cadence_period) { //can't be that short, abort
				continue;
			}
			cadence_rpm = 60.0 / period_filtered;
			cadence_rpm *= (direction_conf * (float)direction_qem);
			inactivity_time = 0.0;
		}	else {
			inactivity_time += 1.0 / (float)AppConf->app_pas_conf.update_rate_hz;
			// if no pedal activity, set RPM as zero
			if(inactivity_time > max_pulse_period) {
				cadence_rpm = 0.0;
			}
		}
	}
}

static THD_FUNCTION(quat_thread, arg) {
	(void)arg;

	chRegSetThreadName("QUAT EBike");
	Quat_is_running = true;

	quat_display_serial_start();
	quat_cadence_start();
	app_init_graphs();

	for(;;) {
		systime_t sleep_time = CH_CFG_ST_FREQUENCY / AppConf->app_adc_conf.update_rate_hz;
		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 1;
		}
		chThdSleep(sleep_time);
		timeout_reset();
		if (Quat_stop_now) {
			Quat_is_running = false;
			return;
		}
		recalculaEstado();
		mc_interface_set_pid_speed(myBike.myVariables.motor_reference_erpm);

		if (sendGraphs) sendGraphs_experiment();
		timeout_reset();
	}
}


// ************************************* MAIN THREAD EBIKE ***************************************************


//  ************************************* TERMINAL CALLBACKS

static void setGraphOn(int argc, const char **argv) {
	if (argc == 2) {
		int graphOn = 0;
		sscanf(argv[1], "%d", &graphOn);
		if (graphOn == 0) {
			sendGraphs = false;
		} else{
			sendGraphs = true;
			samp = 0.0;
		}
	} else {
		commands_printf("\n\r Uso quat_graph 1/0");
  }
}

static void getProgram(int argc, const char **argv) {
	(void) argv;
	if (argc == 1) {
		commands_printf("Received data = %u", myBike.myDisplayData.progDisplay);
		commands_printf("Periodo H= %x", myBike.myDisplayData.motor_periodH);
		commands_printf("Periodo L= %x", myBike.myDisplayData.motor_periodL);
		commands_printf("ChainRing = %f", (double) myBike.myBicycloidal.chainring_rpm);
		commands_printf("Vel = %f", (double) myBike.myVariables.bike_velocity);
		commands_printf("periodo = %u", periodo);

		commands_printf("W1 = %f RPM", (double) myBike.myBicycloidal.pedal_rpm);
		commands_printf("W2 = %f RPM", (double) myBike.myBicycloidal.motor_rpm);
		commands_printf("W3 = %f RPM", (double) myBike.myBicycloidal.chainring_rpm);

		commands_printf("W2ref = %f ERPM", (double) myBike.myVariables.motor_reference_erpm);

		commands_printf("Modo Display %u", myBike.myDisplayData.progDisplay);
		commands_printf("Nivel Asistencia %f", (double) myBike.myVariables.assistance_program);

		commands_printf("Periodo cadencia = %f", (double) period_cadence);
		commands_printf("Periodo filtrado = %f", (double) period_filtered);

		commands_printf("new_state %d", new_state);
		commands_printf("old_state %d", old_state);
		commands_printf("direction qem %f", (double) direction_qem);

	} else {
		commands_printf("\n\r Uso quat_graph 1/0");
  }
}

void sendGraphs_experiment(void){
	commands_plot_set_graph(0);
	commands_send_plot_points(samp, myBike.myBicycloidal.motor_erpm);
	commands_plot_set_graph(1);
	commands_send_plot_points(samp, myBike.myVariables.motor_reference_erpm);
	commands_plot_set_graph(2);
	commands_send_plot_points(samp, myBike.myBicycloidal.pedal_rpm);
	commands_plot_set_graph(3);
	commands_send_plot_points(samp, myBike.myBicycloidal.chainring_rpm);
	samp++;
}
