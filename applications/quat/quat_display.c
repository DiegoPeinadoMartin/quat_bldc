/*
 * quat_display.c
 *
 *  Created on: Mar 14, 2024
 *      Author: bicho
 */

#include "app.h"
#include "ch.h"
#include "hal.h"

#include "utils.h"
#include "commands.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "quat_display.h"
#include "app_quatBike.h"

//  EXTERNAL VARIABLES *****************

extern volatile t_ebike_model myBike;

// *****************************************

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


static THD_FUNCTION(quat_display_process_thread, arg);
static THD_WORKING_AREA(quat_display_process_thread_wa, 2048);

static volatile bool quat_display_thread_is_running = false;
static volatile bool quat_display_thread_stop_now = true;
static volatile bool quat_display_uart_is_running = false;

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

static THD_FUNCTION(quat_display_process_thread, arg) {
  (void)arg;
  chRegSetThreadName("QUAT SerialDisp");
  chThdSleepMilliseconds(500);
  for(;;) {
    chThdSleepMilliseconds(100);
    quat_display_serial_check_rx();
//    timeout_reset();
  }
}
