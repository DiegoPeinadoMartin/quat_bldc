/*
 * app_quatBike.h
 *
 *  Created on: Feb 6, 2024
 *      Author: bicho
 */

#ifndef APPLICATIONS_APP_QUATBIKE_H_
#define APPLICATIONS_APP_QUATBIKE_H_

#define HW_QUAD_SENSOR1_PORT	HW_UART_P_TX_PORT
#define HW_QUAD_SENSOR1_PIN		HW_UART_P_TX_PIN
#define HW_QUAD_SENSOR2_PORT	HW_UART_P_RX_PORT
#define HW_QUAD_SENSOR2_PIN		HW_UART_P_RX_PIN


typedef struct {
	float pedal_rpm;
	float motor_rpm;
	float chainring_rpm;
} t_bicycloidal_state;


#endif /* APPLICATIONS_APP_QUATBIKE_H_ */
