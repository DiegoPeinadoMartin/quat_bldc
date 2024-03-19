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
static volatile LoopManagerType miLoop;

static uint8_t sendGraphs = false;
float samp = 0.0;
uint16_t periodo;
static float AP_ramp;

static float AP[] = {0, 0.20, 0.40, 0.50, 0.65, 0.8};
//static float NR[] = {0, 500, 1000, 1500, 2000, 2500};


// static void getLoopTimes(int argc, const char **argv);
void actualizaVariables(void);
static void setGraphOn(int argc, const char **argv);
static void getProgram(int argc, const char **argv);
void sendGraphs_experiment(void);

// ************************************* UTILITY ***************************************************
float get_erpm_from_kph(float kmph){
  return (kmph/(3.6*mc_conf->si_wheel_diameter*M_PI)*(30.0*mc_conf->si_motor_poles*mc_conf->si_gear_ratio));
}
float get_kph_from_erpm(float miErpm){
  return (miErpm*(3.6*mc_conf->si_wheel_diameter*M_PI)/(30.0*mc_conf->si_motor_poles*mc_conf->si_gear_ratio));
}
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
	terminal_unregister_callback(getProgram);
	terminal_unregister_callback(setGraphOn);
	Quat_stop_now = true;
	while (Quat_is_running) {
		chThdSleepMilliseconds(1);
	}
	commands_printf("STOP QuatApp");
	quat_display_serial_stop();
	quat_cadence_stop();
}

void app_custom_configure(app_configuration *conf) {
	AppConf = conf;
	mc_conf = mc_interface_get_configuration();

	quat_cadence_configure();

	/*
	 * INICIALIZACIÓN CONTROL TIEMPOS DE CICLO: miLoop
	 */
	miLoop.hertz = AppConf->app_balance_conf.hertz;
	miLoop.loop_time = US2ST((int)((1000.0/miLoop.hertz) * 1000.0));
	miLoop.current_time = 0;
	miLoop.last_time = 0;
	miLoop.diff_time = 0;
	miLoop.filtered_loop_overshoot = 0;
	miLoop.brake_timeout = 0;
	miLoop.loop_time_filter = AppConf->app_balance_conf.loop_time_filter;
	miLoop.filtered_loop_overshoot = 0;
	miLoop.filtered_diff_time = 0;
	if(miLoop.loop_time_filter > 0){
	    miLoop.loop_overshoot_alpha = 2*M_PI*((float)1/miLoop.hertz)*miLoop.loop_time_filter/(2*M_PI*((float)1/miLoop.hertz)*miLoop.loop_time_filter+1);
	 }
	miLoop.dt = 1.0/miLoop.hertz;
    // ****************************************************
	myBike.myMotorState = STOPPED;

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
	myBike.myConf.Motor_rev_max = AppConf->app_balance_conf.ki2;
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

void setLoopVariables(void){
	// set current time, last time, diff time, overshoot compensation, etc.
	miLoop.current_time = chVTGetSystemTimeX();
	  if(miLoop.last_time == 0){
		  miLoop.last_time = miLoop.current_time;
	  }
	  miLoop.diff_time = miLoop.current_time - miLoop.last_time;
	  miLoop.filtered_diff_time = 0.03 * miLoop.diff_time + 0.97 * miLoop.filtered_diff_time; // Purely a metric
	  miLoop.last_time = miLoop.current_time;
	  if(miLoop.loop_time_filter > 0){
		  miLoop.loop_overshoot = miLoop.diff_time - (miLoop.loop_time - roundf(miLoop.filtered_loop_overshoot));
		  miLoop.filtered_loop_overshoot = miLoop.loop_overshoot_alpha * miLoop.loop_overshoot + (1-miLoop.loop_overshoot_alpha)*miLoop.filtered_loop_overshoot;
	  }
	  miLoop.dt = ST2S(miLoop.diff_time);
}

void app_init_graphs(void){
	commands_init_plot("Motor RPM", "Motor Ref");
	commands_plot_add_graph("Motor ERPM");
	commands_plot_add_graph("Reference ERPM");
	commands_plot_add_graph("Pedal Cadence");
	commands_plot_add_graph("ChainRing Cadence");
}

void actualizaVariables(void){

	// ****************** A actualizar velocidad del MOTOR  **************************************
	myBike.myBicycloidal.motor_erpm = mc_interface_get_rpm();
	myBike.myBicycloidal.motor_erpm = (fabsf( myBike.myBicycloidal.motor_erpm) > 5.0*myBike.myConf.npolepairs ) ? myBike.myBicycloidal.motor_erpm: 0.0;
	myBike.myBicycloidal.motor_rpm = myBike.myBicycloidal.motor_erpm / (myBike.myConf.npolepairs);

	// ****************** B actualizar velocidad PLATO **************************************
	myBike.myBicycloidal.chainring_rpm = cadence_rpm;

	// ****************** C actualizar velocidad PEDAL  **************************************
	myBike.myBicycloidal.pedal_rpm = (myBike.myBicycloidal.chainring_rpm - (1.0-myBike.myConf.K)*myBike.myBicycloidal.motor_rpm)/myBike.myConf.K;
	myBike.myBicycloidal.pedal_rpm = (fabsf( myBike.myBicycloidal.pedal_rpm) > 10.0) ? myBike.myBicycloidal.pedal_rpm: 0.0;
	myBike.myBicycloidal.pedal_rpm = myBike.myBicycloidal.pedal_rpm < 0.0 ?  0.0 :myBike.myBicycloidal.pedal_rpm;

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

	// ******************* E actualizar el ratio efectivo de transmisión
	if (myBike.myBicycloidal.chainring_rpm > 0.0) {
		myBike.myConf.Rtransmision = myBike.myVariables.bike_velocity / myBike.myConf.WheelRadius / myBike.myBicycloidal.chainring_rpm;
	}

	/***************************************
	 * Calculo los valores filtrados. El de la velocdad del motor con el valor recien leido. el de la velocidad de referencia, con el calculado en el ciclo anterior.
	 * Tengo un desfase de un ciclo, pero no creo que sea importante. Si no, se complica mucho ya que para saber cómo calcular el nuevo valor de referencia
	 * habría que hacer la transición. Habria que hacer una especie de leap-frog.
	 */

	// el valor myBike.myBicycloidal.motor_rpm se acaba de actualizar
	UTILS_LP_MOVING_AVG_APPROX(myBike.myStats.motor_rpm_filtered, myBike.myBicycloidal.motor_rpm, myBike.myStats.Nomeg);
	// el valor myBike.myVariables.motor_reference_rpm está actualizado en el ciclo anterior
	UTILS_LP_MOVING_AVG_APPROX(myBike.myStats.motor_reference_rpm_filtered, myBike.myVariables.motor_reference_rpm, myBike.myStats.Nomeg);
	myBike.myStats.n[0] = myBike.myStats.n[1];
	myBike.myStats.n[1] = myBike.myStats.n[2];
	myBike.myStats.n[2] = myBike.myStats.motor_rpm_filtered;
	myBike.myStats.n[3] = myBike.myStats.n[4];
	myBike.myStats.n[4] = myBike.myStats.n[5];
	myBike.myStats.n[5] = myBike.myStats.motor_reference_rpm_filtered;
	//  Ajusto tres puntos (dos anteriores y el actual) a una parábola. El punto actual es x3 = 0, el anterior es x2 = -h, y el primero x1 = -2.h.
	// La derivada en x = 0, es igual a b (y = a.x² + b.x + c). El coeficiente b, tiene la expresión b  = ((y1-y3)-4.(y2-y3))/(2.h)

	myBike.myStats.motor_rpm_derivate = (myBike.myStats.n[0] + 2* myBike.myStats.n[2] - 4*myBike.myStats.n[1])/(2*miLoop.dt);
	myBike.myStats.motor_reference_rpm_derivate = (myBike.myStats.n[3] + 2* myBike.myStats.n[5] - 4*myBike.myStats.n[4])/(2*miLoop.dt);
}

void transicionEstado(void){
/*
 * En esta función se calculan las transiciones de estado
 */
	switch(myBike.myMotorState){
	case STOPPED:
		// si estaba parado y tengo pedaleo, paso a STABLE
		if (myBike.myBicycloidal.pedal_rpm>0) {
			myBike.myMotorState = STABLE;
			myBike.myMotorState_OLD = STOPPED;
		}
		break;
	case STABLE:
		// si estaba en STABLE, y me paro, paso a STOP
		if (myBike.myBicycloidal.pedal_rpm == 0) {
			myBike.myMotorState = STOPPED;
			myBike.myMotorState_OLD = STABLE;
			break;
		}
		// si la derivada de la referencia es mayor que la real + un término fuzzy, y además
		// la referencia es mayor que la real + otro término fuzzy, entonces estoy deslizando
		if ( (myBike.myStats.motor_reference_rpm_derivate > myBike.myStats.motor_rpm_derivate+myBike.myStats.fuzzyDerivative) &&
				(myBike.myVariables.motor_reference_rpm > myBike.myBicycloidal.motor_rpm + myBike.myStats.fuzzyOmega)  ) {
			myBike.myMotorState = SLIPPING;
			myBike.myMotorState_OLD = STABLE;
			break;
		}
		// si recibo un cambio de programa, paso a JUMPING
		if ( myBike.myVariables.cambioAP){
			myBike.myMotorState = JUMPING;
			myBike.myMotorState_OLD = STABLE;
			break;
		}
		break;
	case SLIPPING:
		// si estaba en SLIPPING, y me paro, paso a STOP
		if (myBike.myBicycloidal.pedal_rpm == 0) {
			myBike.myMotorState = STOPPED;
			myBike.myMotorState_OLD = SLIPPING;
			break;
		}
		// Si la pendiente de la referencia es menor que la real del motor, o la referencia es menor que la real, paso a RECOVERING
		if ( (myBike.myStats.motor_reference_rpm_derivate <= myBike.myStats.motor_rpm_derivate) ||
						(myBike.myVariables.motor_reference_rpm <= myBike.myBicycloidal.motor_rpm)  ) {
			myBike.myMotorState = RECOVERING;
			myBike.myMotorState_OLD = SLIPPING;
			break;
		}
		// si recibo un cambio de programa, paso a JUMPING (desde SLIPPING)
		if ( myBike.myVariables.cambioAP){
			myBike.myMotorState = JUMPING;
			myBike.myMotorState_OLD = SLIPPING;
			break;
		}
		break;
	case RECOVERING:
		// si estaba en RECOVERING, y me paro, paso a STOP
		if (myBike.myBicycloidal.pedal_rpm == 0) {
			myBike.myMotorState = STOPPED;
			myBike.myMotorState_OLD = RECOVERING;
			break;
		}
		// si la derivada de la referencia es mayor que la real + un término fuzzy, y además
		// la referencia es mayor que la real + otro término fuzzy, entonces estoy deslizando
		if ( (myBike.myStats.motor_reference_rpm_derivate > myBike.myStats.motor_rpm_derivate+myBike.myStats.fuzzyDerivative) &&
				(myBike.myVariables.motor_reference_rpm > myBike.myBicycloidal.motor_rpm + myBike.myStats.fuzzyOmega)  ) {
			myBike.myMotorState = SLIPPING;
			myBike.myMotorState_OLD = RECOVERING;
			break;
		}
		// si recibo un cambio de programa, paso a JUMPING
		if ( myBike.myVariables.cambioAP){
			myBike.myMotorState = JUMPING;
			myBike.myMotorState_OLD = RECOVERING;
			break;
		}
		if (myBike.myVariables.assistance_program_Factor > 0.98){
			myBike.myMotorState = STABLE;
			myBike.myMotorState_OLD = RECOVERING;
			break;
		}
		break;
	case JUMPING:
		// si estaba en JUMPING, y me paro, paso a STOP
		if (myBike.myBicycloidal.pedal_rpm == 0) {
			myBike.myMotorState = STOPPED;
			myBike.myMotorState_OLD = JUMPING;
			break;
		}
		// si la derivada de la referencia es mayor que la real + un término fuzzy, y además
		// la referencia es mayor que la real + otro término fuzzy, entonces estoy deslizando
		if ( (myBike.myStats.motor_reference_rpm_derivate > myBike.myStats.motor_rpm_derivate+myBike.myStats.fuzzyDerivative) &&
				(myBike.myVariables.motor_reference_rpm > myBike.myBicycloidal.motor_rpm + myBike.myStats.fuzzyOmega)  ) {
			myBike.myMotorState = SLIPPING;
			myBike.myMotorState_OLD = JUMPING;
			break;
		}
		// si recibo un cambio de programa, paso a JUMPING
		if ( myBike.myVariables.cambioAP){
			myBike.myMotorState = JUMPING;
			myBike.myMotorState_OLD = JUMPING;
			break;
		}
		if (myBike.myVariables.assistance_program_Factor > 0.98){
			myBike.myMotorState = STABLE;
			myBike.myMotorState_OLD = RECOVERING;
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
	float ramp_time_slipping;
	float ramp_time_jumping;
	float ramp_step_slipping;
	float ramp_step_jumping;

	switch (myBike.myMotorState) {
	case STOPPED:
		if (myBike.myMotorState != STOPPED) {
			myBike.myStats.motor_reference_rpm_derivate = 0;
			myBike.myStats.motor_rpm_derivate = 0;
			myBike.myStats.motor_reference_rpm_filtered = 0;
			myBike.myStats.motor_rpm_filtered = 0;
			for(int i = 0; i <= 5; i++)
				myBike.myStats.n[i] = 0;
		}
		break;
	case STABLE:
		if (myBike.myMotorState != STABLE) {
			myBike.myVariables.assistance_program_Factor = 1.0;
		}
		break;
	case SLIPPING:
		myBike.myVariables.assistance_program_Factor =  myBike.myBicycloidal.motor_rpm / ( myBike.myConf.I * myBike.myBicycloidal.pedal_rpm * myBike.myVariables.effective_assistance_program );
		break;
	case RECOVERING:
		// Apply ramping
		AP_ramp = myBike.myVariables.assistance_program_Factor;
		if (myBike.myMotorState != RECOVERING) {
			ramp_time_slipping = AppConf->app_adc_conf.ramp_time_pos+0.01;
			ramp_step_slipping = (float)ST2MS(miLoop.diff_time) / (ramp_time_slipping * 1000.0) *(1-AP_ramp);
			commands_printf("RAMP TIME SLIPPING = %f", (double) ramp_time_slipping);
			commands_printf("RAMP STEP SLIPPING = %f", (double) ramp_step_slipping);
		}
		utils_step_towards(&AP_ramp, 1.0 , ramp_step_slipping);
		myBike.myVariables.assistance_program_Factor = AP_ramp;

		break;
	case JUMPING:
		AP_ramp = myBike.myVariables.assistance_program_Factor;
		if (myBike.myVariables.cambioAP){
			ramp_time_jumping = AppConf->app_adc_conf.ramp_time_neg+0.01;
			AP_ramp = AP_ramp *  myBike.myVariables.assistance_program_OLD/myBike.myVariables.assistance_program;
			if (AP_ramp>= 1.0 ) AP_ramp = 1.0;
			ramp_step_jumping = (float)ST2MS(miLoop.diff_time) / (ramp_time_jumping * 1000.0)*(1-AP_ramp);
		}
		utils_step_towards(&AP_ramp, 1.0 , ramp_step_jumping);
		myBike.myVariables.assistance_program_Factor = AP_ramp;
		break;
	}
	myBike.myVariables.motor_reference_rpm = myBike.myConf.I*myBike.myVariables.effective_assistance_program*myBike.myVariables.assistance_program_Factor *myBike.myBicycloidal.pedal_rpm;
	myBike.myVariables.motor_reference_erpm = myBike.myVariables.motor_reference_rpm * (myBike.myConf.npolepairs);
	myBike.myVariables.motor_torque = 0.75 * mc_conf->si_motor_poles * mc_interface_get_tot_current_filtered() * mc_conf->foc_motor_flux_linkage * 0.001;

	mc_interface_set_pid_speed(myBike.myVariables.motor_reference_erpm);
}

void setOldValues(void) {
	myBike.myVariables.assistance_program_OLD = myBike.myVariables.assistance_program;
	myBike.myMotorState_OLD = myBike.myMotorState;
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

	while (!chThdShouldTerminateX()) {
		timeout_reset();
		setLoopVariables();
		timeout_reset();
		actualizaVariables();
		timeout_reset();
		transicionEstado();
		timeout_reset();
		accionVehiculo();


		setOldValues();
		if (sendGraphs > 0) sendGraphs_experiment();
		timeout_reset();
		chThdSleep(miLoop.loop_time - roundf(miLoop.filtered_loop_overshoot));
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
		} else if (graphOn == 3) {
			sendGraphs = 3;
			commands_init_plot("time", "N.m/Watts");
			commands_plot_add_graph("Motor Torque");
			commands_plot_add_graph("Avg Power");
			commands_plot_add_graph("Max Power");
		} else if (graphOn == 4) {
			sendGraphs = 4;
			commands_init_plot("time", "Filtered Angular Velocities");
			commands_plot_add_graph("Reference");
			commands_plot_add_graph("Actual");
		} else if (graphOn == 5) {
			sendGraphs = 5;
			commands_init_plot("time", "Filtered Angular Derivatives");
			commands_plot_add_graph("Reference");
			commands_plot_add_graph("Actual");
		} else if (graphOn == 6) {
			sendGraphs = 6;
			commands_init_plot("time", "ESTADOS");
			commands_plot_add_graph("NUEVO");
			commands_plot_add_graph("OLD");
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
	} else if (sendGraphs == 3){
		commands_plot_set_graph(0);
		commands_send_plot_points(samp, PAS1_level);
		commands_plot_set_graph(1);
		commands_send_plot_points(samp, PAS2_level);
//		commands_send_plot_points(samp, myBike.myVariables.motor_torque);
//		commands_plot_set_graph(1);
//		commands_send_plot_points(samp, mc_interface_stat_power_avg());
//		commands_plot_set_graph(2);
//		commands_send_plot_points(samp, mc_interface_stat_power_max());
	} else if (sendGraphs == 4){
		commands_plot_set_graph(0);
		commands_send_plot_points(samp, myBike.myStats.motor_reference_rpm_filtered);
		commands_plot_set_graph(1);
		commands_send_plot_points(samp, myBike.myStats.motor_rpm_filtered);
	} else if (sendGraphs == 5){
		commands_plot_set_graph(0);
		commands_send_plot_points(samp, myBike.myStats.motor_reference_rpm_derivate);
		commands_plot_set_graph(1);
		commands_send_plot_points(samp, myBike.myStats.motor_rpm_derivate);
	}  else if (sendGraphs == 6){
		commands_plot_set_graph(0);
		commands_send_plot_points(samp, myBike.myMotorState);
		commands_plot_set_graph(1);
		commands_send_plot_points(samp, myBike.myMotorState_OLD);
	} else {}
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
		commands_printf("Factor Asistencia %f", (double) myBike.myVariables.assistance_program_Factor);

		commands_printf("Torque %f", (double) myBike.myVariables.motor_torque);

		commands_printf("*******************************************************************************");

		commands_printf("Rev max motor %f", (double) myBike.myConf.Motor_rev_max);
		commands_printf("Moto motor = %d", (double) myBike.myConf.modo_moto);

		float diff = miLoop.diff_time;
		float sleep_time = miLoop.loop_time - roundf(miLoop.filtered_loop_overshoot);
		commands_printf("\n%f\t%f", (double)diff, (double)sleep_time);

		commands_printf("Estado %d", myBike.myMotorState);
		commands_printf("Estado OLD %d", myBike.myMotorState_OLD);

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
