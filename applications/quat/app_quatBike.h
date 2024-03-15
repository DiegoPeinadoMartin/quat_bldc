/*
 * app_quatBike.h
 *
 *  Created on: Feb 6, 2024
 *      Author: bicho
 */

#ifndef APPLICATIONS_APP_QUATBIKE_H_
#define APPLICATIONS_APP_QUATBIKE_H_

typedef enum {
	STOP = 0,
	STABLE,
	SLIPING,
	RECOVERING,
	JUMPING
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
	float assistance_program_OLD;
	float effective_assistance_program;
	float assistance_program_Factor;
	float motor_reference_erpm;
	float motor_reference_rpm;
	float motor_torque;
	bool cambioAP;
} t_ebike_variables;

typedef struct{
	float WheelRadius;
	float Rtransmision;
	uint8_t npolepairs;
	float K;
	float I;
	float Motor_rev_max;
	bool  modo_moto;
} t_ebike_conf;

typedef struct {
	float motor_reference_rpm_filtered;
	float motor_rpm_filtered;
	float motor_reference_rpm_derivate;
	float motor_rpm_derivate;
	float n[6];
	uint8_t Ncad;
	uint8_t Nomeg;
	float fuzzyDerivative;
	float fuzzyOmega;
}t_stats_estimators;

typedef struct {
	t_motor_state myMotorState; // Estado del motor
	t_motor_state myMotorState_OLD; // Estado del motor ANTERIOR
	t_bicycloidal_system myBicycloidal;  // Variables del bicicloidal
	t_display_data myDisplayData; 	// Variables enviadas al display
	t_ebike_variables myVariables;  // Variables de estado de la bicicleta/motor
	t_ebike_conf myConf;  // configuración de la ebike
	t_stats_estimators myStats; // estadisticos para determinar estado
} t_ebike_model;


#endif /* APPLICATIONS_APP_QUATBIKE_H_ */
