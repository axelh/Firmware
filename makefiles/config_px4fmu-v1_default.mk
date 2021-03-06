#
# Makefile for the px4fmu_default configuration
#

#
# Use the configuration's ROMFS.
#
ROMFS_ROOT	 = $(PX4_BASE)/ROMFS/px4fmu_common
ROMFS_OPTIONAL_FILES = $(PX4_BASE)/Images/px4io-v1_default.bin

#
# Board support modules
#
MODULES		+= drivers/device
MODULES		+= drivers/stm32
MODULES		+= drivers/stm32/adc
MODULES		+= drivers/stm32/tone_alarm
MODULES		+= drivers/led
MODULES		+= drivers/px4io
MODULES		+= drivers/px4fmu
MODULES		+= drivers/boards/px4fmu-v1
MODULES		+= drivers/ardrone_interface
MODULES		+= drivers/l3gd20
MODULES		+= drivers/mpu6000
MODULES		+= drivers/hmc5883
MODULES		+= drivers/ms5611
#MODULES		+= drivers/ll40ls
MODULES		+= drivers/trone
#MODULES		+= drivers/mb12xx
MODULES		+= drivers/gps
MODULES		+= drivers/hil
#MODULES		+= drivers/blinkm
MODULES		+= drivers/rgbled
MODULES		+= drivers/mkblctrl
MODULES		+= drivers/airspeed
#MODULES		+= drivers/ets_airspeed
MODULES		+= drivers/meas_airspeed
#MODULES		+= drivers/frsky_telemetry
MODULES		+= modules/sensors

#
# System commands
#
MODULES		+= systemcmds/mtd
MODULES		+= systemcmds/mixer
MODULES		+= systemcmds/param
MODULES		+= systemcmds/perf
MODULES		+= systemcmds/preflight_check
MODULES		+= systemcmds/pwm
MODULES		+= systemcmds/esc_calib
MODULES		+= systemcmds/reboot
MODULES		+= systemcmds/top
MODULES		+= systemcmds/config
MODULES		+= systemcmds/nshterm
MODULES		+= systemcmds/dumpfile
MODULES		+= systemcmds/ver

#
# General system control
#
#MODULES		+= modules/commander
#MODULES		+= modules/navigator
#MODULES		+= modules/mavlink
#MODULES		+= modules/gpio_led

#
# Estimation modules (EKF / other filters)
#
#MODULES		+= modules/attitude_estimator_ekf
#MODULES		+= modules/ekf_att_pos_estimator
#MODULES		+= modules/position_estimator_inav

#
# Vehicle Control
#
#MODULES		+= modules/fw_pos_control_l1
#MODULES		+= modules/fw_att_control
#MODULES		+= modules/mc_att_control
#MODULES		+= modules/mc_pos_control

#
# Logging
#
MODULES		+= modules/sdlog2

#
# Unit tests
#
#MODULES 	+= modules/unit_test
#MODULES 	+= modules/commander/commander_tests

#
# Library modules
#
MODULES		+= modules/systemlib
MODULES		+= modules/systemlib/mixer
MODULES		+= modules/controllib
MODULES		+= modules/uORB
MODULES		+= modules/dataman

#
# Libraries
#
LIBRARIES	+= lib/mathlib/CMSIS
MODULES		+= lib/mathlib
MODULES		+= lib/mathlib/math/filter
MODULES		+= lib/ecl
MODULES		+= lib/external_lgpl
MODULES		+= lib/geo
MODULES		+= lib/geo_lookup
MODULES		+= lib/conversion
MODULES		+= lib/launchdetection

#
# Demo apps
#
#MODULES		+= examples/math_demo
MODULES		+= examples/px4_simple_app
#MODULES		+= examples/px4_daemon_app
#MODULES		+= examples/px4_mavlink_debug
#MODULES	    += examples/fixedwing_control
#MODULES	    += examples/ex_gpio

# Hardware test
MODULES			+= examples/hwtest

# ROV
MODULES		+= modules/ROV

# Generate parameter XML file
GEN_PARAM_XML = 1

# Generate parameter XML file
GEN_PARAM_XML = 1

#
# Transitional support - add commands from the NuttX export archive.
#
# In general, these should move to modules over time.
#
# Each entry here is <command>.<priority>.<stacksize>.<entrypoint> but we use a helper macro
# to make the table a bit more readable.
#
define _B
	$(strip $1).$(or $(strip $2),SCHED_PRIORITY_DEFAULT).$(or $(strip $3),CONFIG_PTHREAD_STACK_DEFAULT).$(strip $4)
endef

#                  command                 priority                   stack  entrypoint
BUILTIN_COMMANDS := \
	$(call _B, sercon,                 ,                          2048,  sercon_main                ) \
	$(call _B, serdis,                 ,                          2048,  serdis_main                )
