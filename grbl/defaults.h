/*
  defaults.h - defaults settings configuration file
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* The defaults.h file serves as a central default settings selector for different machine
   types, from DIY CNC mills to CNC conversions of off-the-shelf machines. The settings
   files listed here are supplied by users, so your results may vary. However, this should
   give you a good starting point as you get to know your machine and tweak the settings for
   your nefarious needs.
   NOTE: Ensure one and only one of these DEFAULTS_XXX values is defined in config.h */

#ifndef defaults_h

#ifdef DEFAULTS_GRBL32

#define USE_DOUBLE_SERIAL 0

#define MODE_OFFSET 8
//机型选择
#define OLM            (1+MODE_OFFSET)
#define OLM_1          (2+MODE_OFFSET)
#define OLM_2          (3+MODE_OFFSET)
#define C40            (4+MODE_OFFSET)
#define OLM_MODEL_180  (5+MODE_OFFSET)
#define OLM_MODEL_210  (6+MODE_OFFSET)
#define OLM_MODEL_400  (7+MODE_OFFSET)
#define OLM_2_PRO	   (8+MODE_OFFSET)

//#define ORTUR_MODEL OLM_MODEL_400

#if ORTUR_MODEL==OLM
	#define ORTUR_MODEL_NAME "Ortur Laser Master"
	#define DEFAULT_DIRECTION_INVERT_MASK 1 // INVERT X
	#define DEFAULT_X_MAX_TRAVEL 160.0f // mm NOTE: Must be a positive value.
	#define DEFAULT_Y_MAX_TRAVEL 150.0f // mm NOTE: Must be a positive value.
	#define DEFAULT_Z_MAX_TRAVEL 1.0f // mm NOTE: Must be a positive value.	$132

    #define DEFAULT_X_MAX_RATE           (50*60.0f)  // $110  X Max rate [mm/min]
	#define DEFAULT_Y_MAX_RATE           (50*60.0f)  // $111  Y Max rate [mm/min]
	#define DEFAULT_Z_MAX_RATE           (20*60.0f)  // $112  Z Max rate [mm/min]

	#define DEFAULT_X_ACCELERATION   (1200.0f*60*60) // $120  X Acceleration [mm/sec^2]
    #define DEFAULT_Y_ACCELERATION   (1200.0f*60*60) // $121  Y Acceleration [mm/sec^2]
    #define DEFAULT_Z_ACCELERATION   (1200.0f*60*60) // $122  Z Acceleration [mm/sec^2]

#elif ORTUR_MODEL==OLM_MODEL_180
	#define ORTUR_MODEL_NAME "Ortur Laser Master Model 180"
	#define DEFAULT_DIRECTION_INVERT_MASK 1 // INVERT X
	#define DEFAULT_X_MAX_TRAVEL 180.0f // mm NOTE: Must be a positive value.
	#define DEFAULT_Y_MAX_TRAVEL 180.0f // mm NOTE: Must be a positive value.
	#define DEFAULT_Z_MAX_TRAVEL 1.0f // mm NOTE: Must be a positive value.	$132

	#define DEFAULT_X_MAX_RATE           (50*60.0f)  // $110  X Max rate [mm/min]
	#define DEFAULT_Y_MAX_RATE           (50*60.0f)  // $111  Y Max rate [mm/min]
	#define DEFAULT_Z_MAX_RATE           (20*60.0f)  // $112  Z Max rate [mm/min]

	#define DEFAULT_X_ACCELERATION   (1200.0f*60*60) // $120  X Acceleration [mm/sec^2]
    #define DEFAULT_Y_ACCELERATION   (1200.0f*60*60) // $121  Y Acceleration [mm/sec^2]
    #define DEFAULT_Z_ACCELERATION   (1200.0f*60*60) // $122  Z Acceleration [mm/sec^2]
#elif ORTUR_MODEL==OLM_1
	#define ORTUR_MODEL_NAME "Ortur Laser Master 1"
	#define DEFAULT_DIRECTION_INVERT_MASK 1 // INVERT X
	#define DEFAULT_X_MAX_TRAVEL 210.0f // mm NOTE: Must be a positive value.
	#define DEFAULT_Y_MAX_TRAVEL 210.0f // mm NOTE: Must be a positive value.
	#define DEFAULT_Z_MAX_TRAVEL 1.0f // mm NOTE: Must be a positive value.	$132

    #define DEFAULT_X_MAX_RATE           (3500.0f)  // $110  X Max rate [mm/min]
	#define DEFAULT_Y_MAX_RATE           (3500.0f)  // $111  Y Max rate [mm/min]
	#define DEFAULT_Z_MAX_RATE           (20*60.0f)  // $112  Z Max rate [mm/min]

	#define DEFAULT_X_ACCELERATION   (1500.0f*60*60) // $120  X Acceleration [mm/sec^2]
    #define DEFAULT_Y_ACCELERATION   (1200.0f*60*60) // $121  Y Acceleration [mm/sec^2]
    #define DEFAULT_Z_ACCELERATION   (1200.0f*60*60) // $122  Z Acceleration [mm/sec^2]
#elif ORTUR_MODEL==OLM_MODEL_210
	#define ORTUR_MODEL_NAME "Ortur Laser Master Model 210"
	#define DEFAULT_DIRECTION_INVERT_MASK 1 // INVERT X
	#define DEFAULT_X_MAX_TRAVEL 210.0f // mm NOTE: Must be a positive value.
	#define DEFAULT_Y_MAX_TRAVEL 210.0f // mm NOTE: Must be a positive value.
	#define DEFAULT_Z_MAX_TRAVEL 1.0f // mm NOTE: Must be a positive value.	$132

    #define DEFAULT_X_MAX_RATE           (3500.0f)  // $110  X Max rate [mm/min]
	#define DEFAULT_Y_MAX_RATE           (3500.0f)  // $111  Y Max rate [mm/min]
	#define DEFAULT_Z_MAX_RATE           (20*60.0f)  // $112  Z Max rate [mm/min]

	#define DEFAULT_X_ACCELERATION   (1500.0f*60*60) // $120  X Acceleration [mm/sec^2]
    #define DEFAULT_Y_ACCELERATION   (1200.0f*60*60) // $121  Y Acceleration [mm/sec^2]
    #define DEFAULT_Z_ACCELERATION   (1200.0f*60*60) // $122  Z Acceleration [mm/sec^2]
#elif ORTUR_MODEL==C40
	#define ORTUR_MODEL_NAME "C40"
  	#define DEFAULT_HOMING_ENABLE                 0 // $22   Homing cycle [boolean]
	#define DEFAULT_DIRECTION_INVERT_MASK 0
	#define DEFAULT_X_MAX_TRAVEL 400.0f // mm NOTE: Must be a positive value.
	#define DEFAULT_Y_MAX_TRAVEL 300.0f // mm NOTE: Must be a positive value.
	#define DEFAULT_Z_MAX_TRAVEL 1.0f // mm NOTE: Must be a positive value.	$132

	#define DEFAULT_X_MAX_RATE           (75*60.0f)  // $110  X Max rate [mm/min]
	#define DEFAULT_Y_MAX_RATE           (75*60.0f)  // $111  Y Max rate [mm/min]
	#define DEFAULT_Z_MAX_RATE           (20*60.0f)  // $112  Z Max rate [mm/min]

	#define DEFAULT_X_ACCELERATION   (2000.0f*60*60) // $120  X Acceleration [mm/sec^2]
    #define DEFAULT_Y_ACCELERATION   (1800.0f*60*60) // $121  Y Acceleration [mm/sec^2]
    #define DEFAULT_Z_ACCELERATION   (1200.0f*60*60) // $122  Z Acceleration [mm/sec^2]
#elif ORTUR_MODEL==OLM_2
	#define ORTUR_MODEL_NAME "Ortur Laser Master 2"
	#define DEFAULT_DIRECTION_INVERT_MASK 0
	#define DEFAULT_X_MAX_TRAVEL 400.0f // mm NOTE: Must be a positive value.
	#define DEFAULT_Y_MAX_TRAVEL 430.0f // mm NOTE: Must be a positive value.
	#define DEFAULT_Z_MAX_TRAVEL 1.0f // mm NOTE: Must be a positive value.	$132

    #define DEFAULT_X_MAX_RATE           (75*60.0f)  // $110  X Max rate [mm/min]
	#define DEFAULT_Y_MAX_RATE           (75*60.0f)  // $111  Y Max rate [mm/min]
	#define DEFAULT_Z_MAX_RATE           (20*60.0f)  // $112  Z Max rate [mm/min]

	#define DEFAULT_X_ACCELERATION   (2000.0f*60*60) // $120  X Acceleration [mm/sec^2]
    #define DEFAULT_Y_ACCELERATION   (1800.0f*60*60) // $121  Y Acceleration [mm/sec^2]
    #define DEFAULT_Z_ACCELERATION   (1200.0f*60*60) // $122  Z Acceleration [mm/sec^2]
#elif ORTUR_MODEL==OLM_2_PRO
	#define ORTUR_MODEL_NAME "Ortur Laser Master 2 Pro"
	#define DEFAULT_DIRECTION_INVERT_MASK 1
	#define DEFAULT_X_MAX_TRAVEL 400.0f // mm NOTE: Must be a positive value.
	#define DEFAULT_Y_MAX_TRAVEL 400.0f // mm NOTE: Must be a positive value.
	#define DEFAULT_Z_MAX_TRAVEL 1.0f // mm NOTE: Must be a positive value.	$132

    #define DEFAULT_X_MAX_RATE           (75*60.0f)  // $110  X Max rate [mm/min]
	#define DEFAULT_Y_MAX_RATE           (75*60.0f)  // $111  Y Max rate [mm/min]
	#define DEFAULT_Z_MAX_RATE           (20*60.0f)  // $112  Z Max rate [mm/min]

	#define DEFAULT_X_ACCELERATION   (2000.0f*60*60) // $120  X Acceleration [mm/sec^2]
    #define DEFAULT_Y_ACCELERATION   (1800.0f*60*60) // $121  Y Acceleration [mm/sec^2]
    #define DEFAULT_Z_ACCELERATION   (1200.0f*60*60) // $122  Z Acceleration [mm/sec^2]
#elif ORTUR_MODEL==OLM_MODEL_400
	#define ORTUR_MODEL_NAME "Ortur Laser Master Model 400"
	#define DEFAULT_HOMING_DIR_MASK       0b11 // $23   Homing dir invert [mask]
	#define DEFAULT_DIRECTION_INVERT_MASK 0
	#define DEFAULT_X_MAX_TRAVEL 400.0f // mm NOTE: Must be a positive value.
	#define DEFAULT_Y_MAX_TRAVEL 400.0f // mm NOTE: Must be a positive value.
	#define DEFAULT_Z_MAX_TRAVEL 100.0f // mm NOTE: Must be a positive value.	$132
 	#define DEFAULT_Z_STEPS_PER_MM 400.0f // $102

    #define DEFAULT_X_MAX_RATE           (5000.0f) // $110  X Max rate [mm/min]
	#define DEFAULT_Y_MAX_RATE           (50*60.0f)  // $111  Y Max rate [mm/min]
	#define DEFAULT_Z_MAX_RATE           (20*60.0f)  // $112  Z Max rate [mm/min]

	#define DEFAULT_X_ACCELERATION   (2000.0f*60*60) // $120  X Acceleration [mm/sec^2]
    #define DEFAULT_Y_ACCELERATION   (1200.0f*60*60) // $121  Y Acceleration [mm/sec^2]
    #define DEFAULT_Z_ACCELERATION   (1500.0f*60*60) // $122  Z Acceleration [mm/sec^2]
#else
	#error "Please select a machine model !"
#endif


  #define DEFAULT_STEP_PULSE_MICROSECONDS 	     2 // $0    Step pulse [microseconds]
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME        25 // $1    Step idle delay [milliseconds] (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STEPPING_INVERT_MASK 		   0b0 // $2    Step port invert [mask]
#ifndef DEFAULT_DIRECTION_INVERT_MASK
  #define DEFAULT_DIRECTION_INVERT_MASK        0b0 // $3    Direction port invert [mask]
#endif
  #define DEFAULT_INVERT_ST_ENABLE               0 // $4    Step enable invert [boolean]
  #define DEFAULT_INVERT_LIMIT_PINS              0 // $5    Limit pins invert [boolean]
  #define DEFAULT_INVERT_PROBE_PIN               0 // $6    Probe pin invert [boolean]

  #define DEFAULT_STATUS_REPORT_MASK             3 // $10   Status report [mask]  MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION         0.05f // $11   Junction deviation [mm]
  #define DEFAULT_ARC_TOLERANCE             0.002f // $12   Arc tolerance [mm]
  #define DEFAULT_REPORT_INCHES                  0 // $13   Report inches [boolean]

  #define DEFAULT_SOFT_LIMIT_ENABLE              1 // $20   Soft limits [boolean]
  #define DEFAULT_HARD_LIMIT_ENABLE              1 // $21   Hard limits [boolean]
#ifndef DEFAULT_HOMING_ENABLE
  #define DEFAULT_HOMING_ENABLE                  1 // $22   Homing cycle [boolean]
#endif
#ifndef  DEFAULT_HOMING_DIR_MASK
  #define DEFAULT_HOMING_DIR_MASK            0b111 // $23   Homing dir invert [mask]
#endif
  #define DEFAULT_HOMING_FEED_RATE      (10.0f*60) // $24   Homing feed [mm/min]
  #define DEFAULT_HOMING_SEEK_RATE      (50.0f*60) // $25   Homing seek [mm/min]
  #define DEFAULT_HOMING_DEBOUNCE_DELAY        100 // $26   Homing debounce [milliseconds] (0-65k)
  #define DEFAULT_HOMING_PULLOFF              3.0f // $27   Homing pull-off [mm]

  #define DEFAULT_SPINDLE_RPM_MAX  		   1000.0f // $30   Max spindle speed [RPM]
  #define DEFAULT_SPINDLE_RPM_MIN  			  0.0f // $31   Min spindle speed [RPM]
  #define DEFAULT_LASER_MODE                     1 // $32   Laser mode [boolean]

  #define DEFAULT_ACCEL_SENSITIVITY            250 // $33   Acceleration sensitivity [bit]

  #define DEFAULT_ANALOG_MAX              10000.0f // $40   analog value
  #define DEFAULT_VARIABLE_SPINDLE_ENABLE_PIN    0 // $50


  #define DEFAULT_X_STEPS_PER_MM 		     80.0f // $100		steps/mm
  #define DEFAULT_Y_STEPS_PER_MM 		     80.0f // $101
#ifndef DEFAULT_Z_STEPS_PER_MM
  #define DEFAULT_Z_STEPS_PER_MM 		     80.0f // $102
#endif
  #define DEFAULT_A_STEPS_PER_MM 		   3200.0f // $103
  #define DEFAULT_B_STEPS_PER_MM 		   3200.0f // $104
  #define DEFAULT_C_STEPS_PER_MM 		   3200.0f // $105


  #define DEFAULT_A_MAX_RATE           (150*60.0f) // $113  A Max rate [mm/min]
  #define DEFAULT_B_MAX_RATE           (150*60.0f) // $114  B Max rate [mm/min]
  #define DEFAULT_C_MAX_RATE           (150*60.0f) // $115  C Max rate [mm/min]

  #define DEFAULT_A_ACCELERATION   (1500.0f*60*60) // $123  A Acceleration [mm/sec^2]
  #define DEFAULT_B_ACCELERATION   (1500.0f*60*60) // $124  B Acceleration [mm/sec^2]
  #define DEFAULT_C_ACCELERATION   (1500.0f*60*60) // $125  C Acceleration [mm/sec^2]

  #define DEFAULT_A_MAX_TRAVEL              150.0f // $133  A Max travel [mm] NOTE: Must be a positive value.
  #define DEFAULT_B_MAX_TRAVEL              150.0f // $134  B Max travel [mm] NOTE: Must be a positive value.
  #define DEFAULT_C_MAX_TRAVEL              150.0f // $135  C Max travel [mm] NOTE: Must be a positive value.

#endif

#ifdef DEFAULTS_GENERIC
  // Grbl generic default settings. Should work across different machines.
  #define DEFAULT_X_STEPS_PER_MM 250.0
  #define DEFAULT_Y_STEPS_PER_MM 250.0
  #define DEFAULT_Z_STEPS_PER_MM 250.0
  #define DEFAULT_X_MAX_RATE 500.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 500.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 500.0 // mm/min
  #define DEFAULT_X_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 200.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Y_MAX_TRAVEL 200.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Z_MAX_TRAVEL 200.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_SPINDLE_RPM_MAX 1000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK 0
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.01 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 500.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
#endif

#ifdef DEFAULTS_SHERLINE_5400
  // Description: Sherline 5400 mill with three NEMA 23 Keling  KL23H256-21-8B 185 oz-in stepper motors,
  // driven by three Pololu A4988 stepper drivers with a 30V, 6A power supply at 1.5A per winding.
  #define MICROSTEPS 2
  #define STEPS_PER_REV 200.0
  #define MM_PER_REV (0.050*MM_PER_INCH) // 0.050 inch/rev leadscrew
  #define DEFAULT_X_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/MM_PER_REV)
  #define DEFAULT_Y_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/MM_PER_REV)
  #define DEFAULT_Z_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/MM_PER_REV)
  #define DEFAULT_X_MAX_RATE 635.0 // mm/min (25 ipm)
  #define DEFAULT_Y_MAX_RATE 635.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 635.0 // mm/min
  #define DEFAULT_X_ACCELERATION (50.0*60*60) // 50*60*60 mm/min^2 = 50 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (50.0*60*60) // 50*60*60 mm/min^2 = 50 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (50.0*60*60) // 50*60*60 mm/min^2 = 50 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 225.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Y_MAX_TRAVEL 125.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Z_MAX_TRAVEL 170.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_SPINDLE_RPM_MAX 2800.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((1<<Y_AXIS)|(1<<Z_AXIS))
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.01 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // true
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 50.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 635.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
#endif

#ifdef DEFAULTS_POCKETNC_FR4
  // Description: Pocket NC FR4 CNC mill.
  #define DEFAULT_X_STEPS_PER_MM 800.0
  #define DEFAULT_Y_STEPS_PER_MM 800.0
  #define DEFAULT_Z_STEPS_PER_MM 800.0
  #define DEFAULT_X_MAX_RATE 300.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 300.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 300.0 // mm/min
  #define DEFAULT_X_ACCELERATION (30.0*60*60) // 15*60*60 mm/min^2 = 15 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (30.0*60*60) // 15*60*60 mm/min^2 = 15 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (30.0*60*60) // 15*60*60 mm/min^2 = 15 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 225.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Y_MAX_TRAVEL 125.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Z_MAX_TRAVEL 170.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_SPINDLE_RPM_MAX 7000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((1<<Y_AXIS)|(1<<Z_AXIS))
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 250 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 3 // WPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.01 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 1 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 1  // false
  #define DEFAULT_HOMING_DIR_MASK 1 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 100.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 300.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 3.0 // mm
#endif

#ifdef DEFAULTS_SHAPEOKO
  // Description: Shapeoko CNC mill with three NEMA 17 stepper motors, driven by Synthetos
  // grblShield with a 24V, 4.2A power supply.
  #define MICROSTEPS_XY 8
  #define STEP_REVS_XY 400
  #define MM_PER_REV_XY (0.08*18*MM_PER_INCH) // 0.08 in belt pitch, 18 pulley teeth
  #define MICROSTEPS_Z 2
  #define STEP_REVS_Z 400
  #define MM_PER_REV_Z 1.250 // 1.25 mm/rev leadscrew
  #define DEFAULT_X_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Y_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Z_STEPS_PER_MM (MICROSTEPS_Z*STEP_REVS_Z/MM_PER_REV_Z)
  #define DEFAULT_X_MAX_RATE 1000.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 1000.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 1000.0 // mm/min
  #define DEFAULT_X_ACCELERATION (15.0*60*60) // 15*60*60 mm/min^2 = 15 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (15.0*60*60) // 15*60*60 mm/min^2 = 15 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (15.0*60*60) // 15*60*60 mm/min^2 = 15 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 200.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Y_MAX_TRAVEL 200.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Z_MAX_TRAVEL 200.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_SPINDLE_RPM_MAX 10000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((1<<Y_AXIS)|(1<<Z_AXIS))
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 255 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 250.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
#endif

#ifdef DEFAULTS_SHAPEOKO_2
  // Description: Shapeoko CNC mill with three NEMA 17 stepper motors, driven by Synthetos
  // grblShield at 28V.
  #define MICROSTEPS_XY 8
  #define STEP_REVS_XY 200
  #define MM_PER_REV_XY (2.0*20) // 2mm belt pitch, 20 pulley teeth
  #define MICROSTEPS_Z 2
  #define STEP_REVS_Z 200
  #define MM_PER_REV_Z 1.250 // 1.25 mm/rev leadscrew
  #define DEFAULT_X_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Y_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Z_STEPS_PER_MM (MICROSTEPS_Z*STEP_REVS_Z/MM_PER_REV_Z)
  #define DEFAULT_X_MAX_RATE 5000.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 5000.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 500.0 // mm/min
  #define DEFAULT_X_ACCELERATION (250.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (250.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (50.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 290.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Y_MAX_TRAVEL 290.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Z_MAX_TRAVEL 100.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_SPINDLE_RPM_MAX 10000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((1<<X_AXIS)|(1<<Z_AXIS))
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 255 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 250.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
#endif

#ifdef DEFAULTS_SHAPEOKO_3
  // Description: Shapeoko CNC mill with three NEMA 23 stepper motors, driven by CarbideMotion
  #define MICROSTEPS_XY 8
  #define STEP_REVS_XY 200
  #define MM_PER_REV_XY (2.0*20) // 2mm belt pitch, 20 pulley teeth
  #define MICROSTEPS_Z 8
  #define STEP_REVS_Z 200
  #define MM_PER_REV_Z (2.0*20) // 2mm belt pitch, 20 pulley teeth
  #define DEFAULT_X_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Y_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Z_STEPS_PER_MM (MICROSTEPS_Z*STEP_REVS_Z/MM_PER_REV_Z)
  #define DEFAULT_X_MAX_RATE 5000.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 5000.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 5000.0 // mm/min
  #define DEFAULT_X_ACCELERATION (400.0*60*60) // 400*60*60 mm/min^2 = 400 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (400.0*60*60) // 400*60*60 mm/min^2 = 400 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (400.0*60*60) // 400*60*60 mm/min^2 = 400 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 425.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Y_MAX_TRAVEL 465.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Z_MAX_TRAVEL 80.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_SPINDLE_RPM_MAX 10000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((1<<X_AXIS)|(1<<Z_AXIS))
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 255 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.01 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 100.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 1000.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 25 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 5.0 // mm
#endif

#ifdef DEFAULTS_X_CARVE_500MM
  // Description: X-Carve 3D Carver CNC mill with three 200 step/rev motors driven by Synthetos
  // grblShield at 24V.
  #define MICROSTEPS_XY 8
  #define STEP_REVS_XY 200
  #define MM_PER_REV_XY (2.0*20) // 2mm belt pitch, 20 pulley teeth
  #define MICROSTEPS_Z 2
  #define STEP_REVS_Z 200
  #define MM_PER_REV_Z 2.117 // ACME 3/8-12 Leadscrew
  #define DEFAULT_X_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Y_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Z_STEPS_PER_MM (MICROSTEPS_Z*STEP_REVS_Z/MM_PER_REV_Z)
  #define DEFAULT_X_MAX_RATE 8000.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 8000.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 500.0 // mm/min
  #define DEFAULT_X_ACCELERATION (500.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (500.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (50.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 290.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Y_MAX_TRAVEL 290.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Z_MAX_TRAVEL 100.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_SPINDLE_RPM_MAX 10000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((1<<X_AXIS)|(1<<Y_AXIS))
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 255 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 3 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 750.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
#endif

#ifdef DEFAULTS_X_CARVE_1000MM
  // Description: X-Carve 3D Carver CNC mill with three 200 step/rev motors driven by Synthetos
  // grblShield at 24V.
  #define MICROSTEPS_XY 8
  #define STEP_REVS_XY 200
  #define MM_PER_REV_XY (2.0*20) // 2mm belt pitch, 20 pulley teeth
  #define MICROSTEPS_Z 2
  #define STEP_REVS_Z 200
  #define MM_PER_REV_Z 2.117 // ACME 3/8-12 Leadscrew
  #define DEFAULT_X_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Y_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Z_STEPS_PER_MM (MICROSTEPS_Z*STEP_REVS_Z/MM_PER_REV_Z)
  #define DEFAULT_X_MAX_RATE 8000.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 8000.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 500.0 // mm/min
  #define DEFAULT_X_ACCELERATION (500.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (500.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (50.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 740.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Y_MAX_TRAVEL 790.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Z_MAX_TRAVEL 100.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_SPINDLE_RPM_MAX 10000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((1<<X_AXIS)|(1<<Y_AXIS))
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 255 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 3 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 750.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
#endif

#ifdef DEFAULTS_BOBSCNC_E3
  // Grbl settings for Bob's CNC E3 Machine
  // https://www.bobscnc.com/products/e3-cnc-engraving-kit
  #define DEFAULT_X_STEPS_PER_MM 80.0
  #define DEFAULT_Y_STEPS_PER_MM 80.0
  #define DEFAULT_Z_STEPS_PER_MM 2267.717
  #define DEFAULT_X_MAX_RATE 10000.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 10000.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 500.0 // mm/min
  #define DEFAULT_X_ACCELERATION (500.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (500.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (300.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 450.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Y_MAX_TRAVEL 390.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Z_MAX_TRAVEL 85.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_SPINDLE_RPM_MAX 1000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 5
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK 0
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.01 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 1 // true
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 1 // true
  #define DEFAULT_SOFT_LIMIT_ENABLE 1 // true
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 1  // true
  #define DEFAULT_HOMING_DIR_MASK 3 // move xy -dir, z dir
  #define DEFAULT_HOMING_FEED_RATE 500.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 4000.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 5.0 // mm
#endif

#ifdef DEFAULTS_BOBSCNC_E4
  // Grbl settings for Bob's CNC E4 Machine
  // https://www.bobscnc.com/products/e4-cnc-router
  #define DEFAULT_X_STEPS_PER_MM 80.0
  #define DEFAULT_Y_STEPS_PER_MM 80.0
  #define DEFAULT_Z_STEPS_PER_MM 2267.717
  #define DEFAULT_X_MAX_RATE 10000.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 10000.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 500.0 // mm/min
  #define DEFAULT_X_ACCELERATION (500.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (500.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (300.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 610.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Y_MAX_TRAVEL 610.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Z_MAX_TRAVEL 85.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_SPINDLE_RPM_MAX 1000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 5
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK 0
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.01 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 1 // true
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 1 // true
  #define DEFAULT_SOFT_LIMIT_ENABLE 1 // true
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 1  // true
  #define DEFAULT_HOMING_DIR_MASK 3 // move xy -dir, z dir
  #define DEFAULT_HOMING_FEED_RATE 500.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 4000.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 5.0 // mm
#endif

#ifdef DEFAULTS_ZEN_TOOLWORKS_7x7
  // Description: Zen Toolworks 7x7 mill with three Shinano SST43D2121 65oz-in NEMA 17 stepper motors.
  // Leadscrew is different from some ZTW kits, where most are 1.25mm/rev rather than 8.0mm/rev here.
  // Driven by 30V, 6A power supply and TI DRV8811 stepper motor drivers.
  #define MICROSTEPS 8
  #define STEPS_PER_REV 200.0
  #define MM_PER_REV 8.0 // 8 mm/rev leadscrew
  #define DEFAULT_X_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/MM_PER_REV)
  #define DEFAULT_Y_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/MM_PER_REV)
  #define DEFAULT_Z_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/MM_PER_REV)
  #define DEFAULT_X_MAX_RATE 6000.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 6000.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 6000.0 // mm/min
  #define DEFAULT_X_ACCELERATION (600.0*60*60) // 600*60*60 mm/min^2 = 600 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (600.0*60*60) // 600*60*60 mm/min^2 = 600 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (600.0*60*60) // 600*60*60 mm/min^2 = 600 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 190.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Y_MAX_TRAVEL 180.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Z_MAX_TRAVEL 150.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_SPINDLE_RPM_MAX 10000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((1<<Y_AXIS))
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 250.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
#endif

#ifdef DEFAULTS_OXCNC
  // Grbl settings for OpenBuilds OX CNC Machine
  // http://www.openbuilds.com/builds/openbuilds-ox-cnc-machine.341/
  #define DEFAULT_X_STEPS_PER_MM 26.670
  #define DEFAULT_Y_STEPS_PER_MM 26.670
  #define DEFAULT_Z_STEPS_PER_MM 50
  #define DEFAULT_X_MAX_RATE 500.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 500.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 500.0 // mm/min
  #define DEFAULT_X_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 500.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Y_MAX_TRAVEL 750.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Z_MAX_TRAVEL 80.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_SPINDLE_RPM_MAX 1000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK 0
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 500.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
#endif

#ifdef DEFAULTS_SIMULATOR
  // Settings only for Grbl Simulator (www.github.com/grbl/grbl-sim)
  // Grbl generic default settings. Should work across different machines.
  #define DEFAULT_X_STEPS_PER_MM 1000.0
  #define DEFAULT_Y_STEPS_PER_MM 1000.0
  #define DEFAULT_Z_STEPS_PER_MM 1000.0
  #define DEFAULT_X_MAX_RATE 1000.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 1000.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 1000.0 // mm/min
  #define DEFAULT_X_ACCELERATION (100.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (100.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (100.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 1000.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Y_MAX_TRAVEL 1000.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Z_MAX_TRAVEL 1000.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_SPINDLE_RPM_MAX 1000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK 0
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.01 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 500.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
#endif

#endif
