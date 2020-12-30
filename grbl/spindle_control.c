/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2012-2017 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#include "grbl.h"

#include "spindle_control.h"

#ifdef VARIABLE_SPINDLE
  static float pwm_gradient; // Precalulated value to speed up rpm to PWM conversions.
  //spindle speed value, pwm value
  SPINDLE_PWM_TYPE spindle_speed = 0;
#endif

#ifdef DELAY_OFF_SPINDLE

#ifndef _MAX
  #define _MAX(a,b) ((a)>(b)?(a):(b))
#endif

#ifndef _MIN
  #define _MIN(a,b) ((a)<(b)?(a):(b))
#endif

  /* 主轴/激光 是否关闭的标识 */
  uint8_t spindle_disable_by_grbl = 0;
  /* 主轴/激光 被关闭的时间 */
  uint32_t spindle_disabled_time = 0;
  /* 主轴/激光 累积热量*/
  uint32_t spindle_cumulative_heat = 0;
  /* 主轴/激光 是否挂起的标识 */
  uint8_t spindle_suspend_flag = 0;
  /* 主轴/激光 风扇延时时间*/
  uint32_t spindle_fan_delay_time = 0;

#endif

void spindle_init()
{
#ifdef STM32
	spindle_speed=0;
    pwm_gradient = SPINDLE_PWM_RANGE / (settings.rpm_max - settings.rpm_min);
	Spindle_Timer_Init();

#elif ATMEGA328P
  #ifdef VARIABLE_SPINDLE
    // Configure variable spindle PWM and enable pin, if requried. On the Uno, PWM and enable are
    // combined unless configured otherwise.
    SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT); // Configure as PWM output pin.
    SPINDLE_TCCRA_REGISTER = SPINDLE_TCCRA_INIT_MASK; // Configure PWM output compare timer
    SPINDLE_TCCRB_REGISTER = SPINDLE_TCCRB_INIT_MASK;
    #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
      SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
    #else
      #ifndef ENABLE_DUAL_AXIS
        SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); // Configure as output pin.
      #endif
    #endif
    pwm_gradient = SPINDLE_PWM_RANGE/(settings.rpm_max-settings.rpm_min);
  #else
    SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
    #ifndef ENABLE_DUAL_AXIS
      SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); // Configure as output pin.
    #endif
  #endif
#endif

  spindle_stop();
}


uint8_t spindle_get_state()
{
#ifdef STM32
  uint8_t pin = 0;
  #ifdef VARIABLE_SPINDLE
    #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
      pin = GPIO_ReadInputData(SPINDLE_ENABLE_PORT);
      // No spindle direction output pin.
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        if (bit_isfalse(pin,(1<<SPINDLE_ENABLE_BIT))) { return(SPINDLE_STATE_CW); }
      #else
        if (bit_istrue(pin,(1<<SPINDLE_ENABLE_BIT))) { return(SPINDLE_STATE_CW); }
      #endif //INVERT_SPINDLE_ENABLE_PIN
    #else
      pin = GPIO_ReadInputData(SPIN_DIR_GPIO_Port);
      if (pin & SPIN_DIR_Pin) { return(SPINDLE_STATE_CCW); }
        else { return(SPINDLE_STATE_CW); }
    #endif //USE_SPINDLE_DIR_AS_ENABLE_PIN
  #else
  pin = GPIO_ReadInputData(SPINDLE_ENABLE_PORT);
    #ifdef INVERT_SPINDLE_ENABLE_PIN
      if (bit_isfalse(pin,(1<<SPINDLE_ENABLE_BIT))) {
    #else
      if (bit_istrue(pin,(1<<SPINDLE_ENABLE_BIT))) {
    #endif
      if (pin & (1 << SPINDLE_DIRECTION_BIT)) { return(SPINDLE_STATE_CCW); }
      else { return(SPINDLE_STATE_CW); }
    }
  #endif //VARIABLE_SPINDLE


#elif ATMEGA328P
	#ifdef VARIABLE_SPINDLE
    #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
		  // No spindle direction output pin. 
			#ifdef INVERT_SPINDLE_ENABLE_PIN
			  if (bit_isfalse(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT))) { return(SPINDLE_STATE_CW); }
	    #else
	 			if (bit_istrue(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT))) { return(SPINDLE_STATE_CW); }
	    #endif
    #else
      if (SPINDLE_TCCRA_REGISTER & (1<<SPINDLE_COMB_BIT)) { // Check if PWM is enabled.
        if (SPINDLE_DIRECTION_PORT & (1<<SPINDLE_DIRECTION_BIT)) { return(SPINDLE_STATE_CCW); }
        else { return(SPINDLE_STATE_CW); }
      }
    #endif
	#else
		#ifdef INVERT_SPINDLE_ENABLE_PIN
		  if (bit_isfalse(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT))) {
		#else
		  if (bit_istrue(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT))) {
		#endif
      if (SPINDLE_DIRECTION_PORT & (1<<SPINDLE_DIRECTION_BIT)) { return(SPINDLE_STATE_CCW); }
      else { return(SPINDLE_STATE_CW); }
    }
	#endif
#endif

	return(SPINDLE_STATE_DISABLE);
}


// Disables the spindle and sets PWM output to zero when PWM variable spindle speed is enabled.
// Called by various main program and ISR routines. Keep routine small, fast, and efficient.
// Called by spindle_init(), spindle_set_speed(), spindle_set_state(), and mc_reset().
void spindle_stop()
{
#ifdef STM32
  #ifdef VARIABLE_SPINDLE
	 spindle_speed = 0;
     Spindle_Disable();

      #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
        #ifdef INVERT_SPINDLE_ENABLE_PIN
          SetSpindleEnablebit();
        #else
          ResetSpindleEnablebit();
        #endif
      #endif
  #else
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        SetSpindleEnablebit();
      #else
        ResetSpindleEnablebit();
      #endif
  #endif

#elif ATMEGA328P
  #ifdef VARIABLE_SPINDLE
    SPINDLE_TCCRA_REGISTER &= ~(1<<SPINDLE_COMB_BIT); // Disable PWM. Output voltage is zero.
    #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);  // Set pin to high
      #else
        SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low
      #endif
    #endif
  #else
    #ifdef INVERT_SPINDLE_ENABLE_PIN
      SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);  // Set pin to high
    #else
      SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low
    #endif
  #endif
#endif
}

uint8_t is_spindle_Open()
{
  return readSpindleEnable() && (spindle_get_speed());
}



#ifdef VARIABLE_SPINDLE

  // Sets spindle speed PWM output and enable pin, if configured. Called by spindle_set_state()
  // and stepper ISR. Keep routine small and efficient.
  void spindle_set_speed(SPINDLE_PWM_TYPE pwm_value)
  {
  #ifdef STM32
	//get speed value
    spindle_speed = pwm_value;

#ifdef DELAY_OFF_SPINDLE
  	spindle_disabled_time = HAL_GetTick();
#endif

    #ifdef SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED
      if (pwm_value == SPINDLE_PWM_OFF_VALUE)
      {
        spindle_stop();
      }
      else
      {
    	  Spindle_Enable();
      }
    #endif
      if(is_spindle_suspend_flag_set()&&(pwm_value>0))
      {
    	  spindle_suspend_flag_set(0);
    	  Spindle_Enable();
      }

  #elif ATMEGA328P

    SPINDLE_OCR_REGISTER = pwm_value; // Set PWM output level.
    #ifdef SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED
      if (pwm_value == SPINDLE_PWM_OFF_VALUE) {
        spindle_stop();
      } else {
        SPINDLE_TCCRA_REGISTER |= (1<<SPINDLE_COMB_BIT); // Ensure PWM output is enabled.
        #ifdef INVERT_SPINDLE_ENABLE_PIN
          SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
        #else
          SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
        #endif
      }
    #else
      if (pwm_value == SPINDLE_PWM_OFF_VALUE) {
        SPINDLE_TCCRA_REGISTER &= ~(1<<SPINDLE_COMB_BIT); // Disable PWM. Output voltage is zero.
      } else {
        SPINDLE_TCCRA_REGISTER |= (1<<SPINDLE_COMB_BIT); // Ensure PWM output is enabled.
      }
    #endif
  #endif
  }

  #ifdef ENABLE_PIECEWISE_LINEAR_SPINDLE
  
    // Called by spindle_set_state() and step segment generator. Keep routine small and efficient.
  SPINDLE_PWM_TYPE spindle_compute_pwm_value(float rpm) // 328p PWM register is 8-bit.
    {
  	SPINDLE_PWM_TYPE pwm_value;
      rpm *= (0.010*sys.spindle_speed_ovr); // Scale by spindle speed override value.
      // Calculate PWM register value based on rpm max/min settings and programmed rpm.
      if ((settings.rpm_min >= settings.rpm_max) || (rpm >= RPM_MAX)) {
        rpm = RPM_MAX;
        pwm_value = SPINDLE_PWM_MAX_VALUE;
      } else if (rpm <= RPM_MIN) {
        if (rpm == 0.0) { // S0 disables spindle
          pwm_value = SPINDLE_PWM_OFF_VALUE;
        } else {
          rpm = RPM_MIN;
          pwm_value = SPINDLE_PWM_MIN_VALUE;
        }
      } else {
        // Compute intermediate PWM value with linear spindle speed model via piecewise linear fit model.
        #if (N_PIECES > 3)
          if (rpm > RPM_POINT34) {
            pwm_value = floor(RPM_LINE_A4*rpm - RPM_LINE_B4);
          } else 
        #endif
        #if (N_PIECES > 2)
          if (rpm > RPM_POINT23) {
            pwm_value = floor(RPM_LINE_A3*rpm - RPM_LINE_B3);
          } else 
        #endif
        #if (N_PIECES > 1)
          if (rpm > RPM_POINT12) {
            pwm_value = floor(RPM_LINE_A2*rpm - RPM_LINE_B2);
          } else 
        #endif
        {
          pwm_value = floor(RPM_LINE_A1*rpm - RPM_LINE_B1);
        }
      }
      sys.spindle_speed = rpm;
      return(pwm_value);
    }
    
  #else 
  
    // Called by spindle_set_state() and step segment generator. Keep routine small and efficient.
    SPINDLE_PWM_TYPE spindle_compute_pwm_value(float rpm)
    {
    	SPINDLE_PWM_TYPE pwm_value;
      rpm *= (0.010*sys.spindle_speed_ovr); // Scale by spindle speed override value.
      // Calculate PWM register value based on rpm max/min settings and programmed rpm.
      if ((settings.rpm_min >= settings.rpm_max) || (rpm >= settings.rpm_max)) {
        // No PWM range possible. Set simple on/off spindle control pin state.
        sys.spindle_speed = settings.rpm_max;
        pwm_value = SPINDLE_PWM_MAX_VALUE;
      } else if (rpm <= settings.rpm_min) {
        if (rpm == 0.0) { // S0 disables spindle
          sys.spindle_speed = 0.0;
          pwm_value = SPINDLE_PWM_OFF_VALUE;
        } else { // Set minimum PWM output
          sys.spindle_speed = settings.rpm_min;
          pwm_value = SPINDLE_PWM_MIN_VALUE;
        }
      } else { 
        // Compute intermediate PWM value with linear spindle speed model.
        // NOTE: A nonlinear model could be installed here, if required, but keep it VERY light-weight.
        sys.spindle_speed = rpm;
        pwm_value = (SPINDLE_PWM_TYPE)floor((rpm-settings.rpm_min)*pwm_gradient) + SPINDLE_PWM_MIN_VALUE;
      }
      return(pwm_value);
    }
    
  #endif
#endif


// Immediately sets spindle running state with direction and spindle rpm via PWM, if enabled.
// Called by g-code parser spindle_sync(), parking retract and restore, g-code program end,
// sleep, and spindle stop override.
#ifdef VARIABLE_SPINDLE
  void spindle_set_state(uint8_t state, float rpm)
#else
  void _spindle_set_state(uint8_t state)
#endif
{
  if (sys.abort) { return; } // Block during abort.
  if (state == SPINDLE_DISABLE) { // Halt or set spindle direction and rpm.

    #ifdef VARIABLE_SPINDLE
      sys.spindle_speed = 0.0;
    #endif
    spindle_stop();
  
  } else {
    
    #if !defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) && !defined(ENABLE_DUAL_AXIS)

	  if (state == SPINDLE_ENABLE_CW) {
        ResetSpindleDirectionBit();
      }
      else {
        SetSpindleDirectionBit();
      }
    #endif
  
    #ifdef VARIABLE_SPINDLE
      // NOTE: Assumes all calls to this function is when Grbl is not moving or must remain off.
      if (settings.flags & BITFLAG_LASER_MODE) { 
        if (state == SPINDLE_ENABLE_CCW) { rpm = 0.0; } // TODO: May need to be rpm_min*(100/MAX_SPINDLE_SPEED_OVERRIDE);
      }
      spindle_set_speed(spindle_compute_pwm_value(rpm));
      Spindle_Enable();
    #endif
    #if (defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) && \
        !defined(SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED)) || !defined(VARIABLE_SPINDLE)
      // NOTE: Without variable spindle, the enable bit should just turn on or off, regardless
      // if the spindle speed value is zero, as its ignored anyhow.
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        ResetSpindleEnablebit();
      #else
        SetSpindleEnablebit();
      #endif    
    #endif
  
  }
  
  sys.report_ovr_counter = 0; // Set to report change immediately
}


// G-code parser entry-point for setting spindle state. Forces a planner buffer sync and bails 
// if an abort or check-mode is active.
#ifdef VARIABLE_SPINDLE
  void spindle_sync(uint8_t state, float rpm)
  {
    if (sys.state == STATE_CHECK_MODE) { return; }
    protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
    spindle_set_state(state,rpm);
  }
#else
  void _spindle_sync(uint8_t state)
  {
    if (sys.state == STATE_CHECK_MODE) { return; }
    protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
    _spindle_set_state(state);
  }

#endif


#ifdef DELAY_OFF_SPINDLE

  uint8_t spindle_delay_stop(void)
  {
  	if(spindle_disable_by_grbl)
  	{
  		if(( HAL_GetTick() - spindle_disabled_time ) > spindle_fan_delay_time)
  		{
  			spindle_disable_by_grbl = 0;
  			spindle_cumulative_heat = 0;
  			#ifdef VARIABLE_SPINDLE_ENABLE_PIN
  			  if (settings.spindle_enable_pin_mode == 1)
  				ResetSpindleEnablebit();
  			  else
  				SetSpindleEnablebit();
  			#endif
  		}
  		else
  			return 0;
  	}

  	return 1;
  }

  void spindle_disable_by_grbl_set(uint8_t status)
  {
	spindle_disable_by_grbl = status;
	spindle_disabled_time = HAL_GetTick();
	if(status)
	{
		spindle_fan_delay_time = spindle_cumulative_heat / FAN_HEAT_DISSIPATION_PER_SECOND;
		spindle_fan_delay_time = _MAX(spindle_fan_delay_time,MIN_SPINDLE_FAN_TIME);
		spindle_fan_delay_time = _MIN(MAX_SPINDLE_FAN_TIME,spindle_fan_delay_time);
	}
  }

  void spindle_calculate_heat()
  {
	  static uint32_t last_time = 0;
	  if(HAL_GetTick() - last_time >= 1000)
	  {
		  //电源开启
		  if(readSpindleEnable())
		  {
			  //计算每秒产生的热量和丧失的热量
			  if(spindle_cumulative_heat < MAX_SPINDLE_HEAT)
				  spindle_cumulative_heat += spindle_speed;
			  if(spindle_cumulative_heat >= FAN_HEAT_DISSIPATION_PER_SECOND)
				  spindle_cumulative_heat -= FAN_HEAT_DISSIPATION_PER_SECOND ;
		  }
		  else
		  {
			  if(spindle_cumulative_heat >= AIR_HEAT_DISSIPATION_PER_SECOND)
				  spindle_cumulative_heat -= AIR_HEAT_DISSIPATION_PER_SECOND ;
		  }
		  last_time = HAL_GetTick();
	  }
  }

#endif
