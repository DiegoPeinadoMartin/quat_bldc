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
#include "quat_cadence.h"
#include "quat_display.h"


//  EXTERNAL VARIABLES *****************

extern volatile float cadence_rpm;
extern volatile float period_wheel;
extern volatile uint8_t PAS1_level;
extern volatile uint8_t PAS2_level;

// ***************************************
// SHARED VARIABLES ********************

volatile app_configuration *AppConf;
volatile t_ebike_model myBike;
// ***************************************

// Threads POINTERS
static THD_FUNCTION(quat_thread, arg);
static THD_WORKING_AREA(quat_thread_wa, 1024);

// START AND STOP VARIABLES
static volatile bool Quat_stop_now = true;
static volatile bool Quat_is_running = false;

// ************************************ MAIN APP VARIABLES

#define CONST_AP		79.5774715459

static const volatile mc_configuration *mc_conf;

static uint8_t sendGraphs = false;
float samp = 0.0;
uint16_t periodo;
static float h;
static float AP_ramp;

static float AP[] = {0, 0.20, 0.40, 0.50, 0.65, 0.8};
static float NR[] = {0, 500, 1000, 1500, 2000, 2500};


void actualizaVariables(void);
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
  return (  3.6*(miRpm * M_PI/30.)*(0.5*mc_conf->si_wheel_diameter*mc_conf->si_gear_ratio) );
}
// ************************************* UTILITY ***************************************************




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

	quat_cadence_configure();

	myBike.myMotorState = STOP;

	myBike.myBicycloidal.chainring_rpm = 0;
	myBike.myBicycloidal.motor_erpm;
	myBike.myBicycloidal.motor_rpm;
	myBike.myBicycloidal.pedal_rpm = 0;

	myBike.myDisplayData.motor_intensity = 0;
	myBike.myDisplayData.motor_periodH = (3500 >> 8) & 0xFF;
	myBike.myDisplayData.motor_periodL = 3500 & 0xFF;
	myBike.myDisplayData.progDisplay = 0;
	myBike.myDisplayData.statusBatt = 0;
	myBike.myDisplayData.statusMot = 0;

	myBike.myConf.WheelRadius = mc_interface_get_configuration()->si_wheel_diameter/2.0;
	myBike.myConf.Rtransmision = mc_interface_get_configuration()->si_gear_ratio;
	myBike.myConf.npolepairs = mc_interface_get_configuration()->si_motor_poles/2.0;
	myBike.myConf.K = (1.0 - 1.0/58.0)*(1.0 - 1.0/51.0);
	myBike.myConf.I = myBike.myConf.K / ( 1.0 - myBike.myConf.K);
	myBike.myConf.Motor_rev_max = AppConf->app_balance_conf.hertz;
	//myBike.myConf.modo_moto = (AppConf->app_balance_conf.ki > 0.0) ? true: false;
	myBike.myConf.modo_moto = false;

	commands_printf("Wheel Radius Conf = %f",  (double) ( myBike.myConf.WheelRadius ));
	commands_printf("Transmission Ratio Conf = %f", (double) ( myBike.myConf.Rtransmision  ));
	commands_printf("Motor number of Pole Pairs = %u", (myBike.myConf.npolepairs) );

	myBike.myVariables.assistance_program = 0.0;
	myBike.myVariables.assistance_program_OLD = 0.0;
	myBike.myVariables.effective_assistance_program = 0.0;
	myBike.myVariables.assistance_program_Factor = 1.0;
	myBike.myVariables.bike_intensity = 0;
	myBike.myVariables.bike_velocity = 0;
	myBike.myVariables.cambioAP = false;
	myBike.myVariables.motor_reference_erpm = 0;
	myBike.myVariables.motor_reference_rpm = 0;
	myBike.myVariables.motor_torque = 0;

	myBike.myStats.motor_reference_rpm_filtered = 0;
	myBike.myStats.motor_rpm_filtered = 0;
	myBike.myStats.motor_reference_rpm_derivate = 0;
	myBike.myStats.motor_rpm_derivate = 0;
	myBike.myStats.n[0] = 0;
	myBike.myStats.n[1] = 0;
	myBike.myStats.n[2] = 0;
	myBike.myStats.n[3] = 0;
	myBike.myStats.n[4] = 0;
	myBike.myStats.n[5] = 0;
	myBike.myStats.Ncad = AppConf->app_balance_conf.kp;
	myBike.myStats.Nomeg = AppConf->app_balance_conf.ki;
	myBike.myStats.fuzzyDerivative = AppConf->app_balance_conf.kd;
	myBike.myStats.fuzzyOmega = AppConf->app_balance_conf.kp2;
}

void app_init_graphs(void){
	commands_init_plot("Motor RPM", "Motor Ref");
	commands_plot_add_graph("Motor ERPM");
	commands_plot_add_graph("Reference ERPM");
	commands_plot_add_graph("Pedal Cadence");
	commands_plot_add_graph("ChainRing Cadence");
}

void actualizaVariables(void){

	static systime_t last_time = 0;

	// ****************** A actualizar velocidad del MOTOR  **************************************
	myBike.myBicycloidal.motor_erpm = mc_interface_get_rpm();
	myBike.myBicycloidal.motor_erpm = (fabsf( myBike.myBicycloidal.motor_erpm) > 5.0*myBike.myConf.npolepairs ) ? myBike.myBicycloidal.motor_erpm: 0.0;
	myBike.myBicycloidal.motor_rpm = myBike.myBicycloidal.motor_erpm / (myBike.myConf.npolepairs);

	// ****************** B actualizar velocidad PLATO **************************************
	myBike.myBicycloidal.chainring_rpm = cadence_rpm;

	// ****************** C actualizar velocidad PEDAL  **************************************
	myBike.myBicycloidal.pedal_rpm = (myBike.myBicycloidal.chainring_rpm - (1.0-myBike.myConf.K)*myBike.myBicycloidal.motor_rpm)/myBike.myConf.K;
	myBike.myBicycloidal.pedal_rpm = (fabsf( myBike.myBicycloidal.pedal_rpm) > 6.0) ? myBike.myBicycloidal.pedal_rpm: 0.0;

	// ****************** D actualizar velocidad BIKE **************************************
	myBike.myVariables.bike_velocity = 2000*M_PI/ period_wheel * myBike.myConf.WheelRadius*3.6;

	// ****************** D actualizar display VELOCIDAD BIKE **************************************
	if (myBike.myBicycloidal.chainring_rpm>0.5){
		periodo = period_wheel;
		periodo = (periodo > 3500) ? 3500: periodo;
	} else periodo = 3500;
	myBike.myDisplayData.motor_periodH = (periodo >> 8) & 0xFF;
	myBike.myDisplayData.motor_periodL = periodo & 0xFF;

	myBike.myVariables.assistance_program = AP[(uint8_t) myBike.myDisplayData.progDisplay/51];
	if (myBike.myVariables.assistance_program != myBike.myVariables.assistance_program_OLD) {
		myBike.myVariables.cambioAP = true;
	} else {
		myBike.myVariables.cambioAP = false;
	}

	if (myBike.myBicycloidal.chainring_rpm > 0.0) {
		myBike.myConf.Rtransmision = myBike.myVariables.bike_velocity / myBike.myConf.WheelRadius / myBike.myBicycloidal.chainring_rpm;
	}

	/***************************************
	 * Calculo los valores filtrados. El de la velocdad del motor con el valor recien leido. el de la velocidad de referencia, con el calculado en el ciclo anterior.
	 * Tengo un desfase de un ciclo, pero no creo que sea importante. Si no, se complica mucho ya que para saber cómo calcular el nuevo valor de referencia
	 * habría que hacer la transición. Habria que hacer una especie de leap-frog.
	 */

	UTILS_LP_MOVING_AVG_APPROX(myBike.myStats.motor_rpm_filtered, myBike.myBicycloidal.motor_rpm, myBike.myStats.Nomeg);
	UTILS_LP_MOVING_AVG_APPROX(myBike.myStats.motor_reference_rpm_filtered, myBike.myVariables.motor_reference_rpm, myBike.myStats.Nomeg);
	 h = (float)ST2S(chVTTimeElapsedSinceX(last_time));
	myBike.myStats.n[0] = myBike.myStats.n[1];
	myBike.myStats.n[1] = myBike.myStats.n[2];
	myBike.myStats.n[2] = myBike.myStats.motor_rpm_filtered;
	myBike.myStats.n[3] = myBike.myStats.n[4];
	myBike.myStats.n[4] = myBike.myStats.n[5];
	myBike.myStats.n[5] = myBike.myStats.motor_reference_rpm_filtered;
	myBike.myStats.motor_rpm_derivate = (myBike.myStats.n[0] + 2* myBike.myStats.n[2] - 4*myBike.myStats.n[1])/(2*h);
	myBike.myStats.motor_reference_rpm_derivate = (myBike.myStats.n[3] + 2* myBike.myStats.n[5] - 4*myBike.myStats.n[4])/(2*h);

	last_time = chVTGetSystemTimeX();
}

void transicionEstado(void){

	switch(myBike.myMotorState){
	case STOP:
		if (myBike.myBicycloidal.pedal_rpm>0) {
			myBike.myMotorState = STABLE;
			myBike.myMotorState_OLD = STOP;
		}
		break;
	case STABLE:
		if (myBike.myBicycloidal.pedal_rpm == 0) {
			myBike.myMotorState = STOP;
			myBike.myMotorState_OLD = STABLE;
			break;
		}
		if ( (myBike.myStats.motor_reference_rpm_derivate > myBike.myStats.motor_rpm_derivate+myBike.myStats.fuzzyDerivative) &&
				(myBike.myVariables.motor_reference_rpm > myBike.myBicycloidal.motor_rpm + myBike.myStats.fuzzyOmega)  ) {
			myBike.myMotorState = SLIPING;
			myBike.myMotorState_OLD = STABLE;
			break;
		}
		if ( myBike.myVariables.cambioAP){
			myBike.myMotorState = JUMPING;
			myBike.myMotorState_OLD = STABLE;
			break;
		}
		break;
	case SLIPING:
		if ( (myBike.myStats.motor_reference_rpm_derivate <= myBike.myStats.motor_rpm_derivate) ||
						(myBike.myVariables.motor_reference_rpm <= myBike.myBicycloidal.motor_rpm)  ) {
			myBike.myMotorState = STABLE;
			myBike.myMotorState_OLD = SLIPING;
			break;
		}
		if ( myBike.myVariables.cambioAP){
			myBike.myMotorState = JUMPING;
			myBike.myMotorState_OLD = SLIPING;
			break;
		}
		break;
	}

}

void accionVehiculo(void){

	/* ******************************************************************************************************
	 * La velocidad de referencia se hallará multiplicando el effective_assistance_program x assistance_program_Factor
	 * El effective_assistance_program limita por que la velocidad pedida sea mayor que la máxima del motor, o porque
	 * se llegue a la velocidad máxima.
	 * Esto es indpendiente a que se limite por estar deslizando, o porque se esté en un transitorio desde un resbalamiento, o
	 * también desde un cambio de programa
	 ***********************************************************************************************************/

	myBike.myVariables.effective_assistance_program = fminf(myBike.myVariables.assistance_program, (myBike.myConf.Motor_rev_max) / ( myBike.myConf.I * myBike.myBicycloidal.pedal_rpm));
	// *** EN CUANTO SE MIDA BIEN LA VELOCDAD DE LA BIKE, DESCOMENTAR PARA TENER LÍMITADO LA VELOCDAD MÁXIMA
	//myBike.myVariables.effective_assistance_program = fminf(myBike.myVariables.assistance_program, CONST_AP / (myBike.myConf.K * myBike.myConf.Rtransmision * myBike.myConf.WheelRadius * myBike.myBicycloidal.pedal_rpm));

	switch (myBike.myMotorState) {
	case STOP:
		myBike.myStats.motor_reference_rpm_derivate = 0;
		myBike.myStats.motor_rpm_derivate = 0;
		myBike.myStats.motor_reference_rpm_filtered = 0;
		myBike.myStats.motor_rpm_filtered = 0;
		for(int i = 0; i <= 5; i++)
			myBike.myStats.n[i] = 0;
		break;
	case STABLE:

		break;
	case SLIPING:
		myBike.myVariables.assistance_program_Factor =  myBike.myBicycloidal.motor_rpm / ( myBike.myConf.I * myBike.myBicycloidal.pedal_rpm * myBike.myVariables.effective_assistance_program );
		break;
	case RECOVERING:
		// Apply ramping

		AP_ramp = myBike.myVariables.assistance_program_Factor;
		float ramp_time = AppConf->app_adc_conf.ramp_time_pos;
		if (ramp_time > 0.01) {
			const float ramp_step = (float)ST2MS(chVTTimeElapsedSinceX(last_time)) / (ramp_time * 1000.0);
			utils_step_towards(&AP_ramp, 1.0 , ramp_step);
			myBike.myVariables.assistance_program_Factor = AP_ramp;
		}
		break;
	case JUMPING:

		break;
	}
	myBike.myVariables.motor_reference_rpm = myBike.myConf.I*myBike.myVariables.effective_assistance_program*myBike.myVariables.assistance_program_Factor *myBike.myBicycloidal.pedal_rpm;
	myBike.myVariables.motor_reference_erpm = myBike.myVariables.motor_reference_rpm * (myBike.myConf.npolepairs);
	myBike.myVariables.motor_torque = 0.75 * mc_conf->si_motor_poles * mc_interface_get_tot_current_filtered() * mc_conf->foc_motor_flux_linkage * 0.001;

}

void setOldValues(void) {
	myBike.myVariables.assistance_program_OLD = myBike.myVariables.assistance_program;
}

// ************************************* MAIN APP EBIKE ***************************************************



// ************************************* THREADS ***************************************************



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
		actualizaVariables();

		//transicionEstado();

		mc_interface_set_pid_speed(myBike.myVariables.motor_reference_erpm);

		setOldValues();
		if (sendGraphs > 0) sendGraphs_experiment();
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
			sendGraphs = 0;
		} else if (graphOn == 1){
			sendGraphs = 1;
			commands_init_plot("time", "RPM");
			commands_plot_add_graph("Motor RPM");
			commands_plot_add_graph("Reference RPM");
			commands_plot_add_graph("Pedal Cadence RPM");
			commands_plot_add_graph("ChainRing Cadence RPM");
		} else if (graphOn == 2) {
			sendGraphs = 2;
			commands_init_plot("time", "Current/adim");
			commands_plot_add_graph("Inst current");
			commands_plot_add_graph("Max current");
			commands_plot_add_graph("Avg current");
			commands_plot_add_graph("Effect. AP");
		} else {
			sendGraphs = 3;
			commands_init_plot("time", "N.m/Watts");
			commands_plot_add_graph("Motor Torque");
			commands_plot_add_graph("Avg Power");
			commands_plot_add_graph("Max Power");
		}
		samp = 0.0;
	} else {
		commands_printf("\n\r Uso quat_graph 3/2/1/0");
  }
}



void sendGraphs_experiment(void){
	if (sendGraphs == 1) {
		commands_plot_set_graph(0);
		commands_send_plot_points(samp, myBike.myBicycloidal.motor_rpm);
		commands_plot_set_graph(1);
		commands_send_plot_points(samp, myBike.myVariables.motor_reference_rpm);
		commands_plot_set_graph(2);
		commands_send_plot_points(samp, myBike.myBicycloidal.pedal_rpm);
		commands_plot_set_graph(3);
		commands_send_plot_points(samp, myBike.myBicycloidal.chainring_rpm);
	} else if (sendGraphs == 2){
		commands_plot_set_graph(0);
		commands_send_plot_points(samp, myBike.myVariables.bike_intensity);
		commands_plot_set_graph(1);
		commands_send_plot_points(samp, mc_interface_stat_current_max());
		commands_plot_set_graph(2);
		commands_send_plot_points(samp, mc_interface_stat_current_avg());
		commands_plot_set_graph(3);
		commands_send_plot_points(samp, myBike.myVariables.effective_assistance_program);
	} else {
		commands_plot_set_graph(0);
		commands_send_plot_points(samp, PAS1_level);
		commands_plot_set_graph(1);
		commands_send_plot_points(samp, PAS2_level);
//		commands_send_plot_points(samp, myBike.myVariables.motor_torque);
//		commands_plot_set_graph(1);
//		commands_send_plot_points(samp, mc_interface_stat_power_avg());
//		commands_plot_set_graph(2);
//		commands_send_plot_points(samp, mc_interface_stat_power_max());
	}
	samp++;
}

static void getProgram(int argc, const char **argv) {
	(void) argv;
	if (argc == 1) {
		commands_printf("Received data = %u", myBike.myDisplayData.progDisplay);
		commands_printf("Periodo H= %x", myBike.myDisplayData.motor_periodH);
		commands_printf("Periodo L= %x", myBike.myDisplayData.motor_periodL);
		commands_printf("ChainRing = %f", (double) cadence_rpm);
		commands_printf("Vel = %f", (double) myBike.myVariables.bike_velocity);
		commands_printf("periodo = %u", periodo);
		commands_printf("*******************************************************************************");
		commands_printf("Wheel Radius = %f", (double) myBike.myConf.WheelRadius);
		commands_printf("Transmission Ratio = %f", (double) myBike.myConf.Rtransmision);
		commands_printf("Wheel Radius Conf = %f",  (double) (mc_conf->si_wheel_diameter/2.0));
		commands_printf("Transmission Ratio Conf = %f", (double) mc_conf->si_gear_ratio);

		commands_printf("*******************************************************************************");
		commands_printf("W1 = %f RPM", (double) myBike.myBicycloidal.pedal_rpm);
		commands_printf("W2 = %f RPM", (double) myBike.myBicycloidal.motor_rpm);
		commands_printf("W3 = %f RPM", (double) myBike.myBicycloidal.chainring_rpm);

		commands_printf("W2ref = %f ERPM", (double) myBike.myVariables.motor_reference_erpm);
		commands_printf("*******************************************************************************");
		commands_printf("Modo Display %u", myBike.myDisplayData.progDisplay);
		commands_printf("Nivel Asistencia %f", (double) myBike.myVariables.assistance_program);
		commands_printf("Nivel Asistencia Efectiva %f", (double) myBike.myVariables.effective_assistance_program);

		commands_printf("Torque %f", (double) myBike.myVariables.motor_torque);

		commands_printf("*******************************************************************************");

		commands_printf("Rev max motor %f", (double) myBike.myConf.Motor_rev_max);
		commands_printf("Moto motor = %d", (double) myBike.myConf.modo_moto);

//		commands_printf("Periodo cadencia = %f", (double) period_cadence);
//		commands_printf("Periodo filtrado = %f", (double) period_filtered);
//
//		commands_printf("new_state %d", new_state);
//		commands_printf("old_state %d", old_state);
//		commands_printf("direction qem %f", (double) direction_qem);

	} else {
		commands_printf("\n\r Uso quat_pa");
  }
}
