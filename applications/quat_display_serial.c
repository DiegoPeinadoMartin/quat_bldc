/*
 * quat_display_serial.c
 *
 *  Created on: Feb 7, 2024
 *      Author: bicho
 */

#include "conf_general.h"
#include "hw.h"
#include "quat_display_serial.h"
#include "app.h"
#include "ch.h"
#include "hal.h"
#include "packet.h"
#include "commands.h"
#include "mc_interface.h"
#include "utils.h"
#include <math.h>
#include <string.h>
#include "comm_can.h"
#include "datatypes.h"

#define MAX_TXBUFF 13
#define MAX_RXBUFFSETTINGS 15
#define MAX_RXBUFFRUNNING  10

#define MOOEVO_TX_BUFFER_SIZE 52
#define MOOEVO_RX_BUFFER_SIZE 60

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
	unsigned char data[MOOEVO_RX_BUFFER_SIZE];
	unsigned char tx[MOOEVO_TX_BUFFER_SIZE];
} quat_serial_buffer_t;


static volatile bool mooevo_display_thread_is_running = false;
static volatile bool mooevo_display_uart_is_running = false;

/* UART driver configuration structure */
static SerialConfig uart_cfg = {
                                MOOEVO_DISPLAY_BAUD,
                                0,
                                USART_CR2_LINEN,
                                0
};

static mooevo_serial_buffer_t serial_buffer;

// Threads
static THD_WORKING_AREA(mooevo_display_process_thread_wa, 1024);
static THD_FUNCTION(mooevo_display_process_thread, arg);

static uint8_t mooevo_checksum(uint8_t *buf, uint8_t len);
static void mooevo_serial_send_packet(unsigned char *data, unsigned int len);
static void mooevo_serial_display_byte_process(unsigned char byte);
static void mooevo_serial_display_check_rx(void);

void mooevo_display_serial_start(void) {
  estado_vehiculo = get_estado_vehiculo();
  if (!mooevo_display_thread_is_running) {
      chThdCreateStatic(mooevo_display_process_thread_wa, sizeof(mooevo_display_process_thread_wa),
              NORMALPRIO, mooevo_display_process_thread, NULL);
      mooevo_display_thread_is_running = true;
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
  mooevo_display_uart_is_running = true;
}

static uint8_t mooevo_checksum(uint8_t *buf, uint8_t len) {
  uint8_t sum = 0;
  for(int i = 0; i<len; i++) {
      sum += buf[i];
  }
  return sum;
}

static void mooevo_serial_send_packet(unsigned char *data, unsigned int len) {
	if (mooevo_display_uart_is_running) {
		sdWrite(&HW_UART_DEV, data, len);
	}
}

static void mooevo_serial_display_byte_process(unsigned char byte) {
	//append new byte to the buffer.
	serial_buffer.data[serial_buffer.wr_ptr] = byte;
	serial_buffer.wr_ptr++;

	uint8_t rd_ptr = serial_buffer.rd_ptr;	//read pointer to try at different start addresses
	// process with at least 2 bytes available to read
	while( (serial_buffer.wr_ptr - rd_ptr ) > 1) {
      if(serial_buffer.data[rd_ptr] == CMD_HEAD) {
          // start byte found
        if( (serial_buffer.wr_ptr - rd_ptr) < MAX_RXBUFF)
          return;
        uint8_t checksum_addr;
        checksum_addr = rd_ptr + MAX_RXBUFF - 1;
        if(checksum_addr <= serial_buffer.wr_ptr) {	//check the checksum has been received
          // check sum
          if( serial_buffer.data[checksum_addr] == mooevo_checksum(serial_buffer.data + rd_ptr + 1, 3) ) {
            estado_vehiculo->modo = (serial_buffer.data[rd_ptr+1] >> 4) & 0xFF;
            estado_vehiculo->reversa = (serial_buffer.data[rd_ptr+3] & 0x01);
            serial_buffer.rd_ptr = rd_ptr + 4;	//mark bytes as read
            // enviar paquete al display con info de estado_Vehiculo
            serial_buffer.tx[0] = CMD_HEAD;
            serial_buffer.tx[1] = 0x00; // error de momento no env�o errores
            serial_buffer.tx[2] = estado_vehiculo->velocidad & 0xFF;
            serial_buffer.tx[3] = estado_vehiculo->velocidad >> 8;
            serial_buffer.tx[4] = 100;
            serial_buffer.tx[5] = estado_vehiculo->intensidad*68.0/120.0;
            serial_buffer.tx[6] = mooevo_checksum(serial_buffer.tx+1, MAX_TXBUFF-2);
            mooevo_serial_send_packet(serial_buffer.tx, MAX_TXBUFF);
          }
        }
      }
      rd_ptr++;
	}
	if(serial_buffer.rd_ptr > 0) {
		memmove(serial_buffer.data, serial_buffer.data + serial_buffer.rd_ptr, MOOEVO_RX_BUFFER_SIZE - serial_buffer.rd_ptr);
		serial_buffer.wr_ptr -= serial_buffer.rd_ptr;
		serial_buffer.rd_ptr = 0;
	}
	if(serial_buffer.wr_ptr == (MOOEVO_RX_BUFFER_SIZE - 1) ) {
		//shift buffer to the left discarding the oldest byte
		memmove(serial_buffer.data,serial_buffer.data + 1, MOOEVO_RX_BUFFER_SIZE - 1);
		serial_buffer.wr_ptr -= 1;
	}
}

static void mooevo_serial_display_check_rx(void){
  bool rx = true;
  while (rx) {
    rx = false;
    if (mooevo_display_uart_is_running) {
      msg_t res = sdGetTimeout(&HW_UART_DEV, TIME_INFINITE);
      if (res != MSG_TIMEOUT) {
        mooevo_serial_display_byte_process(res);
        rx = true;
      }
    }
  }
}

static THD_FUNCTION(mooevo_display_process_thread, arg) {
  (void)arg;
  chRegSetThreadName("Mooevo serial display");

  chThdSleepMilliseconds(500);
  for(;;) {
    chThdSleepMilliseconds(100);
    mooevo_serial_display_check_rx();
  }
}
