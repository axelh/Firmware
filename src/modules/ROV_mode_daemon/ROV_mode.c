/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
 *   Author: Eugen Solowjow <eugen.solowjow@tuhh.de>
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
 * Minimal application example for PX4 ROV_mode
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
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

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


__EXPORT int ROV_mode_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int ROV_mode_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

/* Variables */
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
//static struct params p;
//static struct param_handles ph;


int ROV_mode_thread_main(int argc, char *argv[])
{
/*
	 read arguments
	bool verbose = false;

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			verbose = true;
		}
	}
*/

	/* welcome user (warnx prints a line, including an appended\n, with variable arguments */
	warnx("[ROV_mode] started");

	/* initialize parameters, first the handles, then the values */
/*	parameters_init(&ph);
	parameters_update(&ph, &p);*/

	/*
	 * Declare and safely initialize all structs to zero.
	 *
	 */
	thread_running = true;

	struct vehicle_status_s vstatus;
	memset(&vstatus, 0, sizeof(vstatus));


	// char c;

	/* output structs - this is what is sent to the mixer */
	struct actuator_controls_s actuators;
	memset(&actuators, 0, sizeof(actuators));


	/* publish actuator controls with zero values */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++) {
		actuators.control[i] = 0.0f;
	}


/*	 * Advertise that this controller will publish actuator*/

	orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);

	/* subscribe to topics. */
/*	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	int global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	int manual_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
	int param_sub = orb_subscribe(ORB_ID(parameter_update));*/

	/* Setup of loop */
/*    int ret;
	struct pollfd fds;
	fds.fd = 0;  stdin;
	fds.events = POLLIN;*/

/*	 open for ioctl only
	const char *dev = PWM_OUTPUT_DEVICE_PATH;
	int fd = open(dev, 0);
	if (fd < 0)
		err(1, "can't open %s", dev);

	 get the number of servo channels
		unsigned servo_count;
		ret = ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);
		if (ret != OK)
			err(1, "PWM_SERVO_GET_COUNT");

	 set servos to neutral position
	for (unsigned i = 0; i < servo_count; i++) {
			ret = ioctl(fd, PWM_SERVO_SET(i), 1500);
			if (ret != OK)
				err(1, "PWM_SERVO_SET(%d)", i);
	}*/


    while(!thread_should_exit){

    	actuators.control[0] = 0.0f;
    	printf("on\n");

//    	ret = poll(&fds, 1, 0);

/*    	if (ret > 0) {
		read(0,&c,1);
		switch (c){
			case 0x77:
				printf("pitch+\n");
				actuators.control[1] = +1;
				break;
			case 0x61:
				printf("yaw-\n");
				actuators.control[2] = -1;
				break;
			case 0x64:
				printf("yaw+\n");
				actuators.control[2] = +1;
				break;
			case 0x73:
				printf("pitch-\n");
				actuators.control[1] = -1;
				break;
			case 0x6c:         // l
				printf("Acc\n");
				actuators.control[3] = +1;
				break;
			case 0x70:         // p
				printf("Stop\n");
				actuators.control[0] = 0;
				actuators.control[1] = 0;
				actuators.control[2] = 0;
				actuators.control[3] = 0;
				break;
			case 0x63:         //c
				warnx("User abort\n");
				thread_should_exit = true;
				break;
				}
    	}*/
    	/* sanity check and publish actuator outputs */
    					if (isfinite(actuators.control[0]) &&
    					    isfinite(actuators.control[1]) &&
    					    isfinite(actuators.control[2]) &&
    					    isfinite(actuators.control[3])) {
    						orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
    					}
    }

	printf("[ROV_mode] exiting, stopping all motors.\n");
	thread_running = false;

	fflush(stdout);

	/* set servos to neutral position */
/*	for (unsigned i = 0; i < servo_count; i++) {
			ret = ioctl(fd, PWM_SERVO_SET(i), 1500);
			if (ret != OK)
				err(1, "PWM_SERVO_SET(%d)", i);
	}*/

	return 0;
}

/* Startup Functions */

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	fprintf(stderr, "usage: ROV_mode {start|stop|status}\n\n");
	exit(1);
}

/**
 *
 * The actual stack size should be set in the call
 * to task_spawn_cmd().
 */
int ROV_mode_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("ROV_mode\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn_cmd("ROV_mode",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX,
					 2048,
					 ROV_mode_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tex_fixedwing_control is running\n");

		} else {
			printf("\tex_fixedwing_control not started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}
