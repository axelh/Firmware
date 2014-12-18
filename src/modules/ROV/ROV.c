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
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <sys/ioctl.h>

#include <nuttx/fs/nxffs.h>
#include <nuttx/fs/ioctl.h>
#include <arch/board/board.h>
#include "drivers/drv_pwm_output.h"
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

    /* subscribe to sensor_combined topic */
    int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
    orb_set_interval(sensor_sub_fd, 1000);

    /* Initialize aux variables and abs time variable */
    int i ;
    int ret;
    char c_key;
    float rcvalue = 0.0f;
    hrt_abstime stime;
	/* Initialize sensor struct */
	struct sensor_combined_s raw;

    /* Key pressed event */
    struct pollfd fds;
    		fds.fd = 0; /* stdin */
    		fds.events = POLLIN;


			// controller tuning parameters
			float k_ax = 0.05f;
			float k_ay = k_ax;
			float c_ax = 0.5f; //
			float c_ay = c_ax;
			float k_omz = 1.0f; // yaw rate gain
			float omz_set = 0.0f; // approx 30° per second setpoint
			float k_thrust = 0.0f;

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

    		if (c_key == 0x63) { // If "c" key pressed go into control gain setup
			    printf("Press key\n");
			    printf("1 for k_ax = %8.4f\n",(double)k_ax);
			    printf("2 for k_ay = %8.4f\n",(double)k_ay);
			    printf("3 for c_ax = %8.4f\n",(double)c_ax);
			    printf("4 for c_ay = %8.4f\n",(double)c_ay);
			    printf("5 for k_omz = %8.4f\n",(double)k_omz);
	    		read(0, &c_key, 1);
	    		switch (c_key){
	    			case 0x31: //1
	    				printf("function not implemented yet");
					break;
	    			default:
	    				printf("function not implemented yet");
					break;
	    		}
    		}

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
    				    	if (ret > 0) {
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
											k_thrust = 0.0f;
											omz_set =  0.0f;
										break;
										case 0x68:		// h
											k_thrust = 0.0f;
											omz_set = -0.4f;
										break;
										case 0x67:		// g
											k_thrust = 0.0f;
											omz_set = -1.0f;
										break;
										case 0x6B:		// k
											k_thrust = 0.0f;
											omz_set = 0.4f;
										break;
										case 0x6C:		// l
											k_thrust = 0.0f;
											omz_set = 1.0f;
										break;
										case 0x75:		// u
											k_thrust = 0.3f;
											omz_set = 0.0f;
										break;
										case 0x7A:		// z
											k_thrust = 0.3f;
											omz_set = -0.4f;
										break;
										case 0x74:		// t
											k_thrust = 0.3f;
											omz_set = -1.0f;
										break;
										case 0x69:		// i
											k_thrust = 0.3f;
											omz_set = 0.4f;
										break;
										case 0x6F:		// o
											k_thrust = 0.3f;
											omz_set = 1.0f;
										break;
										case 0x6D:		// m
											k_thrust = -0.3f;
											omz_set = 0.0f;
										break;
										case 0x6E:		// n
											k_thrust = -0.3f;
											omz_set = -0.4f;
										break;
										case 0x62:		// b
											k_thrust = -0.3f;
											omz_set = -1.0f;
										break;
										case 0x2C:		// ,
											k_thrust = -0.3f;
											omz_set = 0.4f;
										break;
										case 0x2E:		// .
											k_thrust = -0.3f;
											omz_set = 1.0f;
										break;
										default:
											k_thrust = 0.0f;
											omz_set = 0.0f;
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
									actuators.control[0] = - k_ay * raw.accelerometer_m_s2[1];
									// pitch moment when x-axis is not horizontal
									actuators.control[1] = k_ax * raw.accelerometer_m_s2[0];
									// yaw moment when omz not omz_set and y and x axes are in horizontal plane
									actuators.control[2] = k_omz * (omz_set - raw.gyro1_rad_s[2])
											/(1+abs(c_ax*raw.accelerometer_m_s2[0])+abs(c_ay*raw.accelerometer_m_s2[1]));
									// forward thrust when nose is directed horizontally
									actuators.control[3] = k_thrust
											/(1+abs(c_ax*raw.accelerometer_m_s2[0])+abs(c_ay*raw.accelerometer_m_s2[1]));
	    				    		break; // default


    				    		/*
									// gyro controlled
    								case 0x78:         // x
    		        				    // copy sensors raw data into local buffer
    		        				    orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
    		        				    // roll moment when y-axis is not horizontal
    		        				    actuators.control[0] = - k_ay * raw.accelerometer_m_s2[1];
    		        				    // pitch moment when x-axis is not horizontal
    		        				    actuators.control[1] = k_ax * raw.accelerometer_m_s2[0];
    		        				    // yaw moment when omz not omz_set and y and x axes are in horizontal plane
    		        				    actuators.control[2] = k_omz * (omz_set - raw.gyro1_rad_s[2])
    		        				    		/(1+abs(k_ax*raw.accelerometer_m_s2[0]))
    		        				    		/(1+abs(k_ay*raw.accelerometer_m_s2[1]));
    		        				    // forward thrust when nose is directed horizontally
    		        				    actuators.control[3] = k_thrust
    		        				    		/(1+abs(k_ax*raw.accelerometer_m_s2[0]))
    		        				    		/(1+abs(k_ay*raw.accelerometer_m_s2[1]));
    							    break;
									// Emergency stop
    								case 0x70:         // p
    									actuators.control[0] = 0.0f;
    									actuators.control[1] = 0.0f;
    									actuators.control[2] = 0.0f;
    									actuators.control[3] = 0.0f;
    								break;*/

    								}
        				    	/* sanity check and publish actuator outputs */
        				    	if (isfinite(actuators.control[0]) &&
        				    		isfinite(actuators.control[1]) &&
        				    		isfinite(actuators.control[2]) &&
        				    		isfinite(actuators.control[3])) {
        				    	}
    				    	}

			    			actuators.timestamp = hrt_absolute_time();
			    			orb_publish(ORB_ID(actuator_armed), armed_pub, &armed);
			    			orb_publish(ORB_ID(actuator_controls_0), actuator_pub_fd, &actuators);

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

        				    /* copy sensors raw data into local buffer */
        				    orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);

    				    	/* Print sensor values */

    				    	printf("Snrs:\n Pres:\t%8.4f\n",
    				    			(double)raw.baro_pres_mbar);

    				    	printf("Temp:\t%8.4f\n",
    				    			(double)raw.baro_temp_celcius);

        				    printf("Mag:\t%8.4f\t%8.4f\t%8.4f\n",
        				    		(double)raw.magnetometer_ga[0],
        				    		(double)raw.magnetometer_ga[1],
        				    		(double)raw.magnetometer_ga[2]);

    				    	printf("Gyro:\t%8.4f\t%8.4f\t%8.4f\n",
    				    			(double)raw.gyro1_rad_s[0],
    				    			(double)raw.gyro1_rad_s[1],
    				    			(double)raw.gyro1_rad_s[2]);

    				    	printf("Acc:\t%8.4f\t%8.4f\t%8.4f\n \n",
    				    			(double)raw.accelerometer_m_s2[0],
    				    			(double)raw.accelerometer_m_s2[1],
    				    			(double)raw.accelerometer_m_s2[2]);

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
