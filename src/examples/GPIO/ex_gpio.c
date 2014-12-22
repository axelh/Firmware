/****************************************************************************
 *
<<<<<<< HEAD:src/examples/GPIO/ex_gpio.c
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Example User <mail@example.com>
=======
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
>>>>>>> d54b46355ce0f8c128a5e7fce94564c7cb338987:src/examples/hwtest/hwtest.c
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
<<<<<<< HEAD:src/examples/GPIO/ex_gpio.c
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
=======
 * @file hwtest.c
 *
 * Simple output test.
 * @ref Documentation https://pixhawk.org/dev/examples/write_output
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
>>>>>>> d54b46355ce0f8c128a5e7fce94564c7cb338987:src/examples/hwtest/hwtest.c
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <drivers/drv_gpio.h>




__EXPORT int ex_gpio_main(int argc, char *argv[]);

int ex_gpio_main(int argc, char *argv[])
{
	printf("Hello!\n");


	/* configure the GPIO */
	int fd = open(PX4FMU_DEVICE_PATH, 0);
	ioctl(fd, GPIO_SET_INPUT, GPIO_EXT_1);

	/* check the GPIO */
	uint32_t gpio_values;
	for (int i = 0; i < 500; i++) {
	ioctl(fd, GPIO_GET, &gpio_values);
	if (gpio_values & GPIO_EXT_1) {
		printf("0\n");
	      }else {
	printf("1\n");
	      }
	usleep(1000000);
	}


	return 0;

}
