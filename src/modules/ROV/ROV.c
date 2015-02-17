/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
 *   Author: Eugen Solowjow <eugen.solowjow@gmail.com>
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
			float k_ax = 0.1f; // gain for uprighting moment (pitch)
			float k_ay = 0.2f; // gain for roll compensator
			float c_ax = 0.5f; // gain for inverse influence on thrust and yaw when vehicle is not aligned in plane (x-axis)
			float c_ay = c_ax; // gain for inverse influence on thrust and yaw when vehicle is not aligned in plane (y-axis)
			float k_omz = 1.0f; // yaw rate gain
			float omz_set = 0.0f; // setpoint yaw rate
			float k_thrust = 1.0f; // forward thrust gain
			float thrust_set = 0.0f; // thrust setpoint

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

			    	for (unsigned k = 0; k < 4; k++) {
			    		actuators.control[k] = 0.0f;
			    	}

			    	usleep(20000);

    				ret = poll(&fds, 1, 0);
    				fflush(stdin);
    				    	if (ret > 0) {		// If key pressed event occured
    				    		read(0,&c_key,1);
    				    		fflush(stdin);

    				    		switch (c_key){
									//User abort
									case 0x63:         //c
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
									// Emergency stop
									case 0x20:         // space
										actuators.control[0] = 0.0f;
										actuators.control[1] = 0.0f;
										actuators.control[2] = 0.0f;
										actuators.control[3] = 0.0f;
									break;

				    		    	// roll
    				    			case 0x71:		// q
    				    				actuators.control[0] = -1.0f;
    				    		    break;
    				    			case 0x65:		// e
    				    				actuators.control[0] = +1.0f;
    				    		    break;
    								// pitch
    								case 0x77:		// w
    									actuators.control[1] = -1.0f;
    								break;
    								case 0x73:		// s
    									actuators.control[1] = +1.0f;
    								break;
									// yaw
    								case 0x61:		// a
    									actuators.control[2] = -1.0f;
    								break;
    								case 0x64:		// d
    									actuators.control[2] = +1.0f;
    								break;
									// Acc
    								case 0x72:         // r
    									actuators.control[3] = +1.0f;
									break;
    								// Rev. Acc
    							    case 0x66:         // f
    									actuators.control[3] = -1.0f;
    								break;

									// gyro & acc stabilized mode
    								//  keyboard control centered around j - neutral h,g left, k,l right, one/two row up forward, one row down backward
    							    default:
    							    	switch (c_key){
										// neutral position stabilizing
										case 0x6A:		// j
											thrust_set = 0.0f;
											omz_set =  0.0f;
										break;
										// slow left
										case 0x68:		// h
											thrust_set = 0.0f;
											omz_set = -0.4f;
										break;
										// hard left
										case 0x67:		// g
											thrust_set = 0.0f;
											omz_set = -1.0f;
										break;
										// slow right
										case 0x6B:		// k
											thrust_set = 0.0f;
											omz_set = 0.4f;
										break;
										// hard right
										case 0x6C:		// l
											thrust_set = 0.0f;
											omz_set = 1.0f;
										break;
										// forward
										case 0x75:		// u
											thrust_set = 0.3f;
											omz_set = 0.0f;
										break;
										// forward slow left
										case 0x7A:		// z
											thrust_set = 0.3f;
											omz_set = -0.4f;
										break;
										// forward hard left
										case 0x74:		// t
											thrust_set = 0.3f;
											omz_set = -1.0f;
										break;
										// forward slow right
										case 0x69:		// i
											thrust_set = 0.3f;
											omz_set = 0.4f;
										break;
										// forward hard right
										case 0x6F:		// o
											thrust_set = 0.3f;
											omz_set = 1.0f;
										break;
										// backward
										case 0x6D:		// m
											thrust_set = -0.3f;
											omz_set = 0.0f;
										break;
										// backward slow left
										case 0x6E:		// n
											thrust_set = -0.3f;
											omz_set = -0.4f;
										break;
										// backward hard left
										case 0x62:		// b
											thrust_set = -0.3f;
											omz_set = -1.0f;
										break;
										// backward slow right
										case 0x2C:		// ,
											thrust_set = -0.3f;
											omz_set = 0.4f;
										break;
										// backward hard right
										case 0x2E:		// .
											thrust_set = -0.3f;
											omz_set = 1.0f;
										break;
										case 0x55:     // U thrust gain +.1
											k_thrust = k_thrust + 0.1f;
											printf("thrust gain +.1 - k_thrust = %8.4f",(double)k_thrust);
										break;
										case 0x4D:     // M thrust gain -.1
											k_thrust = k_thrust - 0.1f;
											printf("thrust gain -.1 - k_thrust = %8.4f",(double)k_thrust);
										break;
										case 0x4B:     // K yaw rate gain +.1
											k_omz = k_omz + 0.1f;
											printf("yaw rate gain +.1 - k_omz = %8.4f",(double)k_omz);
										break;
										case 0x48:     // H yaw rate gain -.1
											k_omz = k_omz - 0.1f;
											printf("yaw rate gain -.1 - k_omz = %8.4f",(double)k_omz);
										break;
										case 0x5A:     // Z pitch stabilizer gain +.1
											k_ax = k_ax + 0.1f;
											printf("pitch stabilizer gain +.1 - k_ax = %8.4f",(double)k_ax);
										break;
										case 0x4E:     // N pitch stabilizer gain -.1
											k_ax = k_ax - 0.1f;
											printf("pitch stabilizer gain -.1 - k_ax = %8.4f",(double)k_ax);
										break;
										case 0x54:     // T roll stabilizer gain +.1
											k_ay = k_ay + 0.1f;
											printf("roll stabilizer gain +.1 - k_ay = %8.4f",(double)k_ay);
										break;
										case 0x42:     // B roll stabilizer gain -.1
											k_ay = k_ay - 0.1f;
											printf("roll stabilizer gain -.1 - k_ay = %8.4f",(double)k_ay);
										break;
										case 0x49:     // I pitch stabilizer dominance +.025
											c_ax = c_ax + 0.025f;
											printf("pitch stabilizer dominance +.025 - c_ax = %8.4f",(double)c_ax);
										break;
										case 0x3B:     // ; pitch stabilizer dominance -.025
											c_ax = c_ax - 0.025f;
											printf("pitch stabilizer dominance -.025 - c_ax = %8.4f",(double)c_ax);
										break;
										case 0x4F:     // O roll stabilizer dominance +.1
											c_ay = c_ay + 0.1f;
											printf("roll stabilizer dominance +.1 - c_ay = %8.4f",(double)c_ay);
										break;
										case 0x3A:     // : roll stabilizer dominance -.1
											c_ay = c_ay - 0.1f;
											printf("roll stabilizer dominance -.1 - c_ay = %8.4f",(double)c_ay);
										break;
										default:
											printf("Unidentified key pressed: %c",c_key);
											thrust_set = 0.0f;
											omz_set =  0.0f;
											actuators.control[0] = 0.0f;
											actuators.control[1] = 0.0f;
											actuators.control[2] = 0.0f;
											actuators.control[3] = 0.0f;
										break;
    							    }
									// gyro controlled
									// copy sensors raw data into local buffer
									orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
									// roll moment when y-axis is not horizontal
									actuators.control[0] = k_ay * raw.accelerometer_m_s2[1];
									// pitch moment when x-axis is not horizontal
									actuators.control[1] = k_ax * raw.accelerometer_m_s2[0];
									// yaw moment when omz not omz_set and y and x axes are in horizontal plane
									actuators.control[2] = k_omz * (omz_set - raw.gyro1_rad_s[2])
											/(1+abs(c_ax*raw.accelerometer_m_s2[0])+abs(c_ay*raw.accelerometer_m_s2[1]));
									// forward thrust when nose is directed horizontally
									actuators.control[3] = k_thrust * thrust_set
											/(1+abs(c_ax*raw.accelerometer_m_s2[0])+abs(c_ay*raw.accelerometer_m_s2[1]));
	    				    		break; // default


    				    		/*
									// Emergency stop
    								case 0x70:         // p
    									actuators.control[0] = 0.0f;
    									actuators.control[1] = 0.0f;
    									actuators.control[2] = 0.0f;
    									actuators.control[3] = 0.0f;
    								break;*/

								}
							}
							/* sanity check and publish actuator outputs */
							if (isfinite(actuators.control[0]) &&
								isfinite(actuators.control[1]) &&
								isfinite(actuators.control[2]) &&
								isfinite(actuators.control[3])) {

								actuators.timestamp = hrt_absolute_time();
								orb_publish(ORB_ID(actuator_armed), armed_pub, &armed);
								orb_publish(ORB_ID(actuator_controls_0), actuator_pub_fd, &actuators);
							}

							/* copy attitude topic which is produced by attitude estimator */
							orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
        				    /* copy sensors raw data into local buffer */
        				    orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
        				    /* copy battery status into local buffer */
        				    orb_copy(ORB_ID(battery_status), _bat_stat_sub, &_bat_stat);


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

//    				    	printf("Snrs:\n Pres:\t%8.4f\n",
//    				    			(double)raw.baro_pres_mbar);
//
//    				    	printf("Temp:\t%8.4f\n",
//    				    			(double)raw.baro_temp_celcius);
//
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
	    				    printf("BAT:\t%8.4f\n \n",
	    				    	(double)_bat_stat.voltage_filtered_v);

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
