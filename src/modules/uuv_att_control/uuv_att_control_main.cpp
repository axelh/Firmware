/****************************************************************************
 *
 *   Copyright (c) 2013, 2014, 2015 PX4 Development Team. All rights reserved.
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
 * @file uuv_att_control_main.cpp
 * UUV attitude controller.
 *
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Axel Hackbarth <axel.hackbarth@tuhh.de>
 * @author Eugen Solowjow <eugen.solowjow@tuhh.de>
 *
 * UUV ATTITUDE CONTROLER DESCRIPTION COMES HERE.
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/battery_status.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>

/**
 * Unmanned Underwater Vehicle attitude control app start / stop handling function
 *
 *
 */
extern "C" __EXPORT int uuv_att_control_main(int argc, char *argv[]);

class UuvAttitudeControl
{
public:

	UuvAttitudeControl();
	~UuvAttitudeControl();

	int		start();   // Start the UUV attitude control task and return OK on success

private:
	//******************flags & handlers******************************************************
	bool	_task_should_exit;		/**< if true, task_main() should exit */
	int		_control_task;			/**< task handle */

	/* handlers for subscriptions */
	int		_v_att_sub;				/**< vehicle attitude subscription */
	int     _bat_stat_sub;			   /**< battery status subscription */

	//handlers for publishers

	//*******************data containers***********************************************************
	struct vehicle_attitude_s			_v_att;				/**< vehicle attitude */
	struct battery_status_s			    _bat_stat;			/**< battery status */


	//*****************Member functions***********************************************************************

	static void		task_main_trampoline(int argc, char *argv[]); 	//Shim for calling task_main from task_create
	void			task_main(); 									//Main attitude control task.
};

// Define a pointer to class UuvAttitudeControl with namespace uuv_att_control
namespace uuv_att_control
{
UuvAttitudeControl	*g_control;
}

UuvAttitudeControl::UuvAttitudeControl() :

	_task_should_exit(false),
	_control_task(-1),
	/* subscriptions */
		_v_att_sub(-1)
	{
	memset(&_v_att, 0, sizeof(_v_att));
	}

UuvAttitudeControl::~UuvAttitudeControl()
	{
		if (_control_task != -1) {
			/* task wakes up every 100ms or so at the longest */
			_task_should_exit = true;

			/* wait for a second for the task to quit at our request */
			unsigned i = 0;

			do {
				/* wait 20ms */
				usleep(20000);

				/* if we have given up, kill it */
				if (++i > 50) {
					task_delete(_control_task);
					break;
				}
			} while (_control_task != -1);
		}

		uuv_att_control::g_control = nullptr;
	}

void
UuvAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	uuv_att_control::g_control->task_main();
}

void
UuvAttitudeControl::task_main()
{
	/*
 	 * do subscriptions
	 */
	_v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_bat_stat_sub = orb_subscribe(ORB_ID(battery_status));

	/* initialize parameters cache */

	/* wakeup source: vehicle attitude */
		struct pollfd fds[1];

		fds[0].fd = _v_att_sub;
		fds[0].events = POLLIN;

	while (!_task_should_exit) {

		/* wait for up to 100ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0)
			continue;

		/* Poll error */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}


		/* copy attitude topic which is produced by attitude estimator */
		orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
		orb_copy(ORB_ID(battery_status), _bat_stat_sub, &_bat_stat);

		/* Print ekf values */
		hrt_abstime stime;
		if (hrt_absolute_time() - stime > 500000){
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
		    				    	printf("BAT:\t%8.4f\n \n",
		    				    			(double)_bat_stat.voltage_filtered_v);

		    				    	stime = hrt_absolute_time();
		}

		usleep(20000);

	}

	warnx("exit");

	_control_task = -1;
	_exit(0);
}

int
UuvAttitudeControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = task_spawn_cmd("uuv_att_control",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       2000,
				       (main_t)&UuvAttitudeControl::task_main_trampoline,
				       nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int uuv_att_control_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: uuv_att_control {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (uuv_att_control::g_control != nullptr)
			errx(1, "already running");

		uuv_att_control::g_control = new UuvAttitudeControl;  //allocate dynamic memory to object UuvAttitudeControl

		if (uuv_att_control::g_control == nullptr)
			errx(1, "alloc failed");

		if (OK != uuv_att_control::g_control->start()) {
			delete uuv_att_control::g_control;
			uuv_att_control::g_control = nullptr;
			err(1, "start failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (uuv_att_control::g_control == nullptr)
			errx(1, "not running");

		delete uuv_att_control::g_control;
		uuv_att_control::g_control = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (uuv_att_control::g_control) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
