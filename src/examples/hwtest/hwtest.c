/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author:    Lorenz Meier <lm@inf.ethz.ch>
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
 * @file hwtest.c
 *
 * Simple functional hardware test.
 *
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
#include <uORB/topics/actuator_armed.h>
#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <sys/ioctl.h>

#include <nuttx/fs/nxffs.h>
#include <nuttx/fs/ioctl.h>
#include <arch/board/board.h>
#include "drivers/drv_pwm_output.h"


__EXPORT int ex_hwtest_main(int argc, char *argv[]);

int ex_hwtest_main(int argc, char *argv[])
{
    struct actuator_controls_s actuators;
    memset(&actuators, 0, sizeof(actuators));
    orb_advert_t actuator_pub_fd = orb_advertise(ORB_ID(actuator_controls_0), &actuators);

    struct actuator_armed_s armed;
    armed.armed = true;
    /* lock down actuators if required, only in HIL */
    armed.lockdown = false;
    orb_advert_t armed_pub = orb_advertise(ORB_ID(actuator_armed), &armed);

    int i;
    int ret;

    struct pollfd fds;
    		fds.fd = 0; /* stdin */
    		fds.events = POLLIN;
    		int fd = open(&PWM_OUTPUT_DEVICE_PATH, 0);
    char c;
    printf("hi\n");
    float rcvalue = 1.0f;
    hrt_abstime stime;

    while (true) {
        stime = hrt_absolute_time();
        while (hrt_absolute_time() - stime < 1000000) {
            for (i=0; i<8; i++)
                actuators.control[i] = rcvalue;
            actuators.timestamp = hrt_absolute_time(); 
            orb_publish(ORB_ID(actuator_armed), armed_pub, &armed);
            orb_publish(ORB_ID(actuator_controls_0), actuator_pub_fd, &actuators);
            usleep(10000);
        }
        rcvalue -= 0.05f;
/* Plot pwm values to terminal */
		for (unsigned k = 0; k < 4; k++) {
			servo_position_t spos;
			ret = ioctl(fd, PWM_SERVO_GET(k), (unsigned long)&spos);
			if (ret == OK) {
				printf("channel %u: %u us\n", k+1, spos);
			}
		}

    	ret = poll(&fds, 1, 0);
    	if (ret > 0) {

    	read(0, &c, 1);
    		if (c == 0x03 || c == 0x63 || c == 'q') {
    			warnx("User abort\n");
    			exit(0);
    		}
    	}
    	usleep(10000);

    }

    return OK;
}
