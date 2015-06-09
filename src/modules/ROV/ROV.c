/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
 *   Authors: Eugen Solowjow <eugen.solowjow@gmail.com> &
 *   		  Axel Hackbarth <axel.hackbarth@gmail.com>
 *   http://www.tuhh.de/mum/forschung/forschungsgebiete-und-projekte/fluid-field-estimation-and-source-localization-by-dynamic-positioning-of-autonomous-underwater-sensing-vehicles.html
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ROV_mode.c
 * Application ROV
 */


#include <nuttx/config.h>
#include <drivers/drv_gpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/battery_status.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <sys/ioctl.h>

#include <nuttx/fs/nxffs.h>
#include <nuttx/fs/ioctl.h>
#include <arch/board/board.h>
#include "drivers/drv_pwm_output.h"

/* process-specific header files
#include "params.h" */

/* Prototypes */

/**
 * Daemon management function.
 */
__EXPORT int ROV_main(int argc, char *argv[]);



int ROV_main(int argc, char *argv[])
{
	/* Initialize actuator  struct and set it to zero*/
    struct actuator_controls_s actuators;
    memset(&actuators, 0, sizeof(actuators));

	/* Initialize armed struct and set ini values*/
    struct actuator_armed_s armed;
    armed.armed = true;
    armed.lockdown = false; /* lock down actuators if required, only in HIL */

	/* Advertise actuator and armed topics and publish ini values*/
    orb_advert_t armed_pub = orb_advertise(ORB_ID(actuator_armed), &armed);
    orb_advert_t actuator_pub_fd = orb_advertise(ORB_ID(actuator_controls_0), &actuators);


    /* handlers for sensor and attitude (EKF) subscriptions */
	int _v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
    int _bat_stat_sub = orb_subscribe(ORB_ID(battery_status));

    orb_set_interval(sensor_sub_fd, 1000);

    /* Initialize aux variables and abs time variable */
    int i ;
    int ret;
    char c_key;
    float rcvalue = 0.0f;
    hrt_abstime stime;


	//*******************data containers***********************************************************
	struct vehicle_attitude_s			_v_att;				/**< vehicle attitude */
	struct sensor_combined_s 			raw;				/**< sensor values */
	struct battery_status_s			    _bat_stat;			/**< battery status */


    /* Key pressed event */
    struct pollfd fds;
    		fds.fd = 0; /* stdin */
    		fds.events = POLLIN;

			// controller tuning parameters
    		float dgain = 0.05f;
			float roll_gain = 0.8f; // gain for roll compensator
			float roll_set  = 0.0f; // wanted roll
			float rollspeed_set  = 0.0f; // wanted rollspeed
			float roll_dominance = 5.0f; // gain for inverse influence on thrust and yaw when vehicle roll angle is not as desired
			float pitch_gain = 1.0f; // gain for uprighting moment (pitch)
			float pitch_set = 0.0f; // wanted pitch
			float pitchspeed_set = 0.0f; // wanted pitchspeed
			float pitch_dominance = roll_dominance; // gain for inverse influence on thrust and yaw when vehicle pitch angle is not as desired
			float yaw_gain = 0.0f; // yaw gain
			float yaw_set = 0.0f; // setpoint yaw rate
			float yawspeed_gain = 0.5f; // yaw rate gain
			float yawspeed_set = 0.0f; // setpoint yaw rate
			float yawspeed_max = 1.0f; // maximum yaw rate for autopilot
//			float yaw_dominance = 0; // gain for inverse influence on thrust and yaw when vehicle yaw angle is not as desired
			float thrust_set = 0.0f; // thrust setpoint
			float thrust_max = 0.15f; // max thrust for autopilot
			float pitch_depth_compensator = 0.0f; // pitch setpoint
			float actuator_limit = 0.4f; // actuator limit here instead of in mixer for adaptive change
			float depth_set = 0.4f;      // [ms]
			float autodepth_gain = 1.5f; // [rad/m]
			// status parameters
			bool autopilot = false;
			bool oldautopilot = false;
			bool autodepth = false;
			bool manual = true;

    /* Open PWM device driver*/
    int fd = open(&PWM_OUTPUT_DEVICE_PATH, 0);

    /* Print to console */
    printf("Start. Press g after ESCs ready. \n");

    /* Main loop */
    while (true) {
    	/* start main loop by publishing zero values to actuators to start ESCs */

    	/* Publish zero */
    	for (i=0; i<4; i++){
    		actuators.control[i] = rcvalue;
    		actuators.timestamp = hrt_absolute_time();
    		orb_publish(ORB_ID(actuator_armed), armed_pub, &armed);
    		orb_publish(ORB_ID(actuator_controls_0), actuator_pub_fd, &actuators);
    		}
    	usleep(10000);

    	/* Check whether key pressed */
    	ret = poll(&fds, 1, 0);
    	if (ret > 0) {		/* If key pressed event occured */
    		read(0, &c_key, 1);


    		if (c_key == 0x67) { // If "g" key pressed go into key control loop
    			warnx("Start\n");

				stime = hrt_absolute_time();

    			/* Keyboard control. Can be aborted with "c" key */
    			while (true) {

    				// set all actuators to zero each run, only if button is continuously pressed, an action is executed
//			    	for (unsigned k = 0; k < 4; k++) {
//			    		actuators.control[k] = 0.0f;
//			    	}

			    	usleep(20000);

    				ret = poll(&fds, 1, 0);
    				fflush(stdin);
    				    	if (ret > 0) {		// If key pressed event occured
    				    		read(0,&c_key,1);
    				    		fflush(stdin);

    				    		switch (c_key){
									case 0x63:         //c
										//User abort
										warnx("User abort\n");
										//Stop servos
										for (i=0; i<4; i++){
											actuators.control[i] = 0.0f;
										}
										actuators.timestamp = hrt_absolute_time();
										orb_publish(ORB_ID(actuator_armed), armed_pub, &armed);
										orb_publish(ORB_ID(actuator_controls_0), actuator_pub_fd, &actuators);

										usleep(10000);

										exit(0); //return
									break;
								    case 0x30:         // 0
										// Emergency stop
										actuators.control[0] = 0.0f;
										actuators.control[1] = 0.0f;
										actuators.control[2] = 0.0f;
										actuators.control[3] = 0.0f;
										autopilot = false;
									break;
									case 0x20:         // space
										// Emergency stop
										actuators.control[0] = 0.0f;
										actuators.control[1] = 0.0f;
										actuators.control[2] = 0.0f;
										actuators.control[3] = 0.0f;
										autopilot = false;
									break;

									// MANUAL MODE
					    			case 0x37:		// 7
					    				// roll left
										manual=true;actuators.control[0] = 0.0f;actuators.control[1] = 0.0f;actuators.control[2] = 0.0f;actuators.control[3] = 0.0f;
					    				actuators.control[0] = -1.0f;
					    		    break;
					    			case 0x39:		// 9
					    				// roll right
					    				manual=true;actuators.control[0] = 0.0f;actuators.control[1] = 0.0f;actuators.control[2] = 0.0f;actuators.control[3] = 0.0f;
					    				actuators.control[0] = +1.0f;
					    		    break;
									case 0x38:		// 8
										// pitch up
										manual=true;actuators.control[0] = 0.0f;actuators.control[1] = 0.0f;actuators.control[2] = 0.0f;actuators.control[3] = 0.0f;
										actuators.control[1] = +1.0f;
									break;
									case 0x35:		// 5
										// pitch down
										manual=true;actuators.control[0] = 0.0f;actuators.control[1] = 0.0f;actuators.control[2] = 0.0f;actuators.control[3] = 0.0f;
										actuators.control[1] = -1.0f;
									break;
									case 0x34:		// 4
										// turn left
										manual=true;actuators.control[0] = 0.0f;actuators.control[1] = 0.0f;actuators.control[2] = 0.0f;actuators.control[3] = 0.0f;
										actuators.control[2] = -1.0f;
									break;
									case 0x36:		// 6
										// turn right
										manual=true;actuators.control[0] = 0.0f;actuators.control[1] = 0.0f;actuators.control[2] = 0.0f;actuators.control[3] = 0.0f;
										actuators.control[2] = +1.0f;
									break;
									case 0x2B:         // +
										// Acc
										manual=true;actuators.control[0] = 0.0f;actuators.control[1] = 0.0f;actuators.control[2] = 0.0f;actuators.control[3] = 0.0f;
										actuators.control[3] = +1.0f;
									break;
								    case 0x31:         // 1
										// Rev. Acc
								    	manual=true;actuators.control[0] = 0.0f;actuators.control[1] = 0.0f;actuators.control[2] = 0.0f;actuators.control[3] = 0.0f;
										actuators.control[3] = -1.0f;
									break;
//									case 0x73:		// s
//									case 0x64:		// d
//					    			case 0x71:		// q
//								    case 0x66:      // f

									case 0x57:		// W
										// roll setpoint +.1
										roll_set = roll_set +.1f;
										printf("roll_set +.1f; = %8.4f",(double)roll_set);
									break;
									case 0x77:		// w
										// roll setpoint -.1
										roll_set = roll_set -.1f;
										printf("roll_set -.1f; = %8.4f",(double)roll_set);
									break;
					    			case 0x45:		// E
										// pitch setpoint +.1
										pitch_set = pitch_set +.1f;
										printf("pitch_set +.1f; = %8.4f",(double)pitch_set);
									break;
					    			case 0x65:		// e
										// pitch setpoint -.1
										pitch_set = pitch_set -.1f;
										printf("pitch_set -.1f; = %8.4f",(double)pitch_set);
									break;
					    			case 0x52:		// R
										// yaw setpoint +.1
										yaw_set = yaw_set +.1f;
										printf("yaw_set +.1f; = %8.4f",(double)yaw_set);
									break;
									case 0x72:      // r
										// yaw setpoint -.1
										yaw_set = yaw_set -.1f;
										printf("yaw_set -.1f; = %8.4f",(double)yaw_set);
									break;
					    			case 0x46:		// F
										// yaw gain +.1
										yaw_gain = yaw_gain +.1f;
										printf("yaw_gain +.1f; = %8.4f",(double)yaw_gain);
									break;
									case 0x66:      // f
										// yaw gain -.1
										yaw_gain = yaw_gain -.1f;
										printf("yaw_gain -.1f; = %8.4f",(double)yaw_gain);
									break;
					    			case 0x71:		// q
					    				oldautopilot = false;
									break;
					    			case 0x51:		// Q
					    				oldautopilot = true;
									break;

									// DEPTH CONTROL
								    case 0x61:		// a
										// autodepth controller
										autodepth = true;
										printf("autodepth activated, setpoint = %8.4f V",(double)depth_set);
									break;
								    case 0x79:		// y
										// autodepth controller off, pitch controller on
										autodepth = false;
										printf("autodepth deactivated, pitch control activated, setpoint = %8.4f",(double)pitch_depth_compensator);
									break;
									case 0x3C:     // >
										// depth setpoint -.1 or pitch setpoint +.1
										if (autodepth) {
											depth_set = depth_set - 0.1f;
											printf("depth_set -.1 = %8.4f",(double)depth_set);
										} else {
											pitch_depth_compensator = pitch_depth_compensator + 0.1f;
											printf("pitch_depth_compensator +.1 = %8.4f",(double)pitch_depth_compensator);
										}
									break;
									case 0x3E:     // <
										// depth setpoint +.1 or pitch setpoint -.1
										if (autodepth) {
											depth_set = depth_set + 0.1f;
											printf("depth_set +.1 = %8.4f",(double)depth_set);
										} else {
											pitch_depth_compensator = pitch_depth_compensator - 0.1f;
											printf("pitch_depth_compensator -.1 = %8.4f",(double)pitch_depth_compensator);
										}
									break;

								    case 0x2D:		// -
										// gyro & acc stabilized mode
										autopilot = true;
									break;
									//  keyboard control centered around j - neutral h,g left, k,l right, one/two row up forward, one row down backward
									// neutral position stabilizing
									case 0x6A:		// j
										thrust_set = 0.0f;
										yawspeed_set =  0.0f*yawspeed_max;
									break;
									case 0x68:		// h
										// slow left
										thrust_set = 0.0f;
										yawspeed_set = -0.4f*yawspeed_max;
									break;
									case 0x67:		// g
										// hard left
										thrust_set = 0.0f;
										yawspeed_set = -1.0f*yawspeed_max;
									break;
									case 0x6B:		// k
										// slow right
										thrust_set = 0.0f;
										yawspeed_set = 0.4f*yawspeed_max;
									break;
									case 0x6C:		// l
										// hard right
										thrust_set = 0.0f;
										yawspeed_set = 1.0f*yawspeed_max;
									break;
									case 0x75:		// u
										// forward
										thrust_set = thrust_max;
										yawspeed_set = 0.0f*yawspeed_max;
									break;
									case 0x7A:		// z
										// forward slow left
										thrust_set = thrust_max;
										yawspeed_set = -0.4f*yawspeed_max;
									break;
									case 0x74:		// t
										// forward hard left
										thrust_set = thrust_max;
										yawspeed_set = -1.0f*yawspeed_max;
									break;
									case 0x69:		// i
										// forward slow right
										thrust_set = thrust_max;
										yawspeed_set = 0.4f*yawspeed_max;
									break;
									case 0x6F:		// o
										// forward hard right
										thrust_set = thrust_max;
										yawspeed_set = 1.0f*yawspeed_max;
									break;
									case 0x6D:		// m
										// backward
										thrust_set = -thrust_max;
										yawspeed_set = 0.0f*yawspeed_max;
									break;
									case 0x6E:		// n
										// backward slow left
										thrust_set = -thrust_max;
										yawspeed_set = -0.4f*yawspeed_max;
									break;
									case 0x62:		// b
										// backward hard left
										thrust_set = -thrust_max;
										yawspeed_set = -1.0f*yawspeed_max;
									break;
									case 0x2C:		// ,
										// backward slow right
										thrust_set = -thrust_max;
										yawspeed_set = 0.4f*yawspeed_max;
									break;
									case 0x2E:		// .
										// backward hard right
										thrust_set = -thrust_max;
										yawspeed_set = 1.0f*yawspeed_max;
									break;


									// SET TUNING PARAMETERS //
									case 0x58:		// X
										// differential gain *1.1
										dgain = dgain * 1.1f;
										printf("dgain * 1.1f; = %8.4f",(double)dgain);
									break;
									case 0x78:		// x
										// differential gain /1.1
										dgain = dgain / 1.1f;
										printf("dgain / 1.1f; = %8.4f",(double)dgain);
									break;
									case 0x41:		// A
										// auto depth gain +.1
										autodepth_gain = autodepth_gain +.1f;
										printf("autodepth_gain +.1f; = %8.4f",(double)autodepth_gain);
									break;
									case 0x59:		// Y
										// auto depth gain -.1
										autodepth_gain = autodepth_gain -.1f;
										printf("autodepth_gain -.1f; = %8.4f",(double)autodepth_gain);
									break;
									case 0x55:     // U
										// thrust gain +.1
										thrust_max = fmin(1.0f,thrust_max + 0.05f);
										printf("thrust gain +.05 - thrust_max = %8.4f",(double)thrust_max);
									break;
									case 0x4D:     // M
										// thrust gain -.1
										thrust_max = fmax(0.0f,thrust_max - 0.05f);
										printf("thrust gain -.05 - thrust_max = %8.4f",(double)thrust_max);
									break;
									case 0x4C:     // L
										// yaw rate max *1.1
										yawspeed_max = yawspeed_max *1.1f;
										printf("yaw rate max *1.1 - yawspeed_max = %8.4f",(double)yawspeed_max);
									break;
									case 0x47:     // G
										// yaw rate max /1.1
										yawspeed_max = yawspeed_max /1.1f;
										printf("yaw rate max /1.1 - yawspeed_max = %8.4f",(double)yawspeed_max);
									break;
									case 0x4B:     // K
										// yaw rate gain +.1
										yawspeed_gain = yawspeed_gain + 0.1f;
										printf("yaw rate gain +.1 - yawspeed_gain = %8.4f",(double)yawspeed_gain);
									break;
									case 0x48:     // H
										// yaw rate gain /1.1
										yawspeed_gain = yawspeed_gain /1.1f;
										printf("yaw rate gain /1.1 - yawspeed_gain = %8.4f",(double)yawspeed_gain);
									break;
									case 0x5A:     // Z
										// pitch stabilizer gain *1.1
										pitch_gain = pitch_gain *1.1f;
										printf("pitch stabilizer gain *1.1 - pitch_gain = %8.4f",(double)pitch_gain);
									break;
									case 0x4E:     // N
										// pitch stabilizer gain /1.1
										pitch_gain = pitch_gain /1.1f;
										printf("pitch stabilizer gain /1.1 - pitch_gain = %8.4f",(double)pitch_gain);
									break;
									case 0x54:     // T
										// roll stabilizer gain *1.1
										roll_gain = roll_gain *1.1f;
										printf("roll stabilizer gain *1.1 - roll_gain = %8.4f",(double)roll_gain);
									break;
									case 0x42:     // B
										// roll stabilizer gain /1.1
										roll_gain = roll_gain /1.1f;
										printf("roll stabilizer gain /1.1 - roll_gain = %8.4f",(double)roll_gain);
									break;
									case 0x49:     // I
										// pitch stabilizer dominance *1.1
										pitch_dominance = pitch_dominance *1.1f;
										printf("pitch stabilizer dominance *1.1 - pitch_dominance = %8.4f",(double)pitch_dominance);
									break;
									case 0x3B:     // ;
										// pitch stabilizer dominance /1.1
										pitch_dominance = pitch_dominance /1.1f;
										printf("pitch stabilizer dominance /1.1 - pitch_dominance = %8.4f",(double)pitch_dominance);
									break;
									case 0x4F:     // O
										// roll stabilizer dominance *1.1
										roll_dominance = roll_dominance *1.1f;
										printf("roll stabilizer dominance *1.1 - roll_dominance = %8.4f",(double)roll_dominance);
									break;
									case 0x3A:     // :
										// roll stabilizer dominance /1.1
										roll_dominance = roll_dominance /1.1f;
										printf("roll stabilizer dominance /1.1 - roll_dominance = %8.4f",(double)roll_dominance);
									break;
									case 0x2A:     // *
										// actuator_limit +.1
										actuator_limit = actuator_limit + 0.1f;
										printf("actuator_limit +.1 = %8.4f",(double)actuator_limit);
									break;
									case 0x2F:     // /
										// actuator_limit -.1
										actuator_limit = actuator_limit - 0.1f;
										printf("actuator_limit -.1 = %8.4f",(double)actuator_limit);
									break;

								} // switch c_key
							} // if key pressed

							/* copy attitude topic which is produced by attitude estimator */
							orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
        				    /* copy sensors raw data into local buffer */
        				    orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
        				    /* copy battery status into local buffer */
        				    orb_copy(ORB_ID(battery_status), _bat_stat_sub, &_bat_stat);

    				    	if (manual) {
    				    		manual = false;
    				    	} else {
        				    	if (autopilot) {
        				    		if (oldautopilot) { // OLD CONTROLLER BASED ON RAW SENSOR MEASUREMENTS
										// gyro & acc controlled
										// roll moment when y-axis is not horizontal
										actuators.control[0] = roll_gain /10.0f * raw.accelerometer_m_s2[1];
										// yaw moment when omz not yawspeed_set and y and x axes are in horizontal plane
										actuators.control[2] = yawspeed_gain * (yawspeed_set - raw.gyro1_rad_s[2])
												/(1.0f+(float)fabs(pitch_dominance/10.0f*raw.accelerometer_m_s2[0])+(float)fabs(roll_dominance/10.0f*raw.accelerometer_m_s2[1]));
										// forward thrust when nose is directed horizontally
										actuators.control[3] = thrust_set
												/(1.0f+(float)fabs(pitch_dominance/10.0f*raw.accelerometer_m_s2[0])+(float)fabs(roll_dominance/10.0f*raw.accelerometer_m_s2[1]));
										// use autodepth or pitch compensator
										if (autodepth) {
											// pitch moment when x-axis is not horizontal, compensated with depth measurement and depth setpoint
											actuators.control[1] = pitch_gain * (-raw.accelerometer_m_s2[0]/10.0f) + autodepth_gain * ( raw.adc_voltage_v[6] - depth_set);
										} else {
											// pitch moment when x-axis is not horizontal, compensated with pitch setpoint
											actuators.control[1] = pitch_gain * (-raw.accelerometer_m_s2[0]/10.0f + pitch_depth_compensator);
										}
        				    		} else { // NEW CONTROLLER BASED ON EKF ESTIMATES
										// gyro & acc controlled
										// use autodepth or pitch compensator
										float thrust_sgn =  (thrust_set > 0) - (thrust_set < 0);
										if (autodepth) {
											// pitch set [rad] compensated with depth measurement and depth setpoint [m]
											pitch_set = thrust_sgn * autodepth_gain  * ( .3f * raw.adc_voltage_v[6] - depth_set);
										} else {
											// pitch moment when x-axis is not horizontal, compensated with pitch depth setpoint
//											pitch_set = thrust_sgn * pitch_depth_compensator;
										}
										// roll moment when y-axis is not horizontal
										float e_roll = roll_set - _v_att.roll;
										e_roll = (e_roll > (float)M_PI) ? (e_roll - 2.0f*(float)M_PI) : e_roll;
										e_roll = (e_roll <-(float)M_PI) ? (e_roll + 2.0f*(float)M_PI) : e_roll;
										// ersetzen mit e_roll = 2*(float)M_PI * (int)(e_roll/(float)M_PI)/2);
										float e_pitch = pitch_set - _v_att.pitch;
										e_pitch = (e_pitch > (float)M_PI) ? (e_pitch - 2.0f*(float)M_PI) : e_pitch;
										e_pitch = (e_pitch <-(float)M_PI) ? (e_pitch + 2.0f*(float)M_PI) : e_pitch;
										float e_yaw = yaw_set - _v_att.yaw;
										e_yaw = (e_yaw > (float)M_PI) ? (e_yaw - 2*(float)M_PI) : e_yaw;
										e_yaw = (e_yaw <-(float)M_PI) ? (e_yaw + 2*(float)M_PI) : e_yaw;
										// roll moment for roll angle compensation
										actuators.control[0] = roll_gain * e_roll + dgain * (rollspeed_set - _v_att.rollspeed);
										// pitch moment
										actuators.control[1] = pitch_gain * (e_pitch) + dgain * (pitchspeed_set - _v_att.pitchspeed);
										// yaw moment when yawspeed not yawspeed_set and roll and pitch are on setpoints
										actuators.control[2] = yaw_gain * (e_yaw) + yawspeed_gain * (yawspeed_set - _v_att.yawspeed)
												/(1.0f+roll_dominance * (float)fabs(e_roll)+pitch_dominance * (float)fabs(e_pitch));
										// forward thrust when nose is directed
										actuators.control[3] = thrust_set
												/(1.0f+roll_dominance * (float)fabs(e_roll)+pitch_dominance * (float)fabs(e_pitch));
        				    		} // oldautopilot
        				    	} // autopilot
    				    	} // manual



							/* sanity check and publish actuator outputs */
							if (isfinite(actuators.control[0]) &&
								isfinite(actuators.control[1]) &&
								isfinite(actuators.control[2]) &&
								isfinite(actuators.control[3])) {
								actuators.control[0] = fmax(fmin(actuators.control[0] * actuator_limit, actuator_limit), -actuator_limit);
								actuators.control[1] = fmax(fmin(actuators.control[1] * actuator_limit, actuator_limit), -actuator_limit);
								actuators.control[2] = fmax(fmin(actuators.control[2] * actuator_limit, actuator_limit), -actuator_limit);
								actuators.control[3] = fmax(fmin(actuators.control[3] * actuator_limit, actuator_limit), -actuator_limit);

								actuators.timestamp = hrt_absolute_time();
								orb_publish(ORB_ID(actuator_armed), armed_pub, &armed);
								orb_publish(ORB_ID(actuator_controls_0), actuator_pub_fd, &actuators);
							}



    				    	/* Plot to terminal */
    				    	if (hrt_absolute_time() - stime > 500000){
    				    		/* Plot pwm values  */
    				    		for (unsigned k = 0; k < 4; k++) {
    				    			servo_position_t spos;
    				    			ret = ioctl(fd, PWM_SERVO_GET(k), (unsigned long)&spos);
    				    			if (ret == OK) {
    				    				printf("pwm channel %u: %u us\n", k+1, spos);
    				    			}
    				    		}

    				    	/* Print sensor & EKF values */

    				    	printf("Snrs:\n Pres:\t%8.4f\n",
    				    			(double)raw.baro_pres_mbar);

    				    	printf("Temp:\t%8.4f\n",
    				    			(double)raw.baro_temp_celcius);

    				    	printf("Ang:\t%8.4f\t%8.4f\t%8.4f\n",
    				    			(double)_v_att.roll,
    				    			(double)_v_att.pitch,
    				    			(double)_v_att.yaw);
    				    	printf("Vel:\t%8.4f\t%8.4f\t%8.4f\n",
    				    			(double)_v_att.rollspeed,
    				    			(double)_v_att.pitchspeed,
    				    			(double)_v_att.yawspeed);
    				    	printf("Acc:\t%8.4f\t%8.4f\t%8.4f\n \n",
    				    			(double)_v_att.rollacc,
    				    			(double)_v_att.pitchacc,
    				    			(double)_v_att.yawacc);

    				    	printf("ADC 10:\t%8.4f\n",
   				    			(double)raw.adc_voltage_v[6]);
	    				    printf("BAT:\t%8.4f\t AMP:\t%8.4f\n \n",
	    				    		(double)_bat_stat.voltage_filtered_v,
	    				    		(double)_bat_stat.current_a);

	    				    printf("Autopilot:\t%d\t OldAutopilot:\t%d\t AutoDepth:\t%d\t Manual:\t%d\n \n",
	    				    	autopilot,oldautopilot,autodepth,manual);

    				    	printf("Control:\t%8.4f\t%8.4f\t%8.4f\t%8.4f\n",
    				    			(double)actuators.control[0],
    				    			(double)actuators.control[1],
    				    			(double)actuators.control[2],
    				    			(double)actuators.control[3]);

    	    				stime = hrt_absolute_time();
    				    	}
    				    	fflush( stdout );
    					usleep(10000);
    				}
    			}
    		}
    }
    return OK;
}
