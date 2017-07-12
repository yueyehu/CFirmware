/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file mc_pos_control_params.c
 * Multicopter position controller parameters.
 *
 * @author Anton Babushkin <anton@px4.io>
 */

/**
 * Minimum thrust in auto thrust control
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.05
 * @max 1.0
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_MIN_S, 0.12f);

/**
 * Hover thrust
 *
 * Vertical thrust required to hover.
 * This value is mapped to center stick for manual throttle control.
 * With this value set to the thrust required to hover, transition
 * from manual to ALTCTL mode while hovering will occur with the
 * throttle stick near center, which is then interpreted as (near)
 * zero demand for vertical speed.
 *
 * @min 0.2
 * @max 0.8
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_HOVER_S, 0.5f);

/**
 * ALTCTL throttle curve breakpoint
 *
 * Halfwidth of deadband or reduced sensitivity center portion of curve.
 * This is the halfwidth of the center region of the ALTCTL throttle
 * curve. It extends from center-dz to center+dz.
 *
 * @min 0.0
 * @max 0.2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_ALTCTL_DZ_S, 0.1f);

/**
 * Maximum thrust in auto thrust control
 *
 * Limit max allowed thrust. Setting a value of one can put
 * the system into actuator saturation as no spread between
 * the motors is possible any more. A value of 0.8 - 0.9
 * is recommended.
 *
 * @min 0.0
 * @max 0.95
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_MAX_S, 0.9f);

/**
 * Minimum manual thrust
 *
 * Minimum vertical thrust. It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MANTHR_MIN_S, 0.08f);

/**
 * Maximum manual thrust
 *
 * Limit max allowed thrust. Setting a value of one can put
 * the system into actuator saturation as no spread between
 * the motors is possible any more. A value of 0.8 - 0.9
 * is recommended.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MANTHR_MAX_S, 0.9f);

/**
 * Proportional gain for vertical position error
 *
 * @min 0.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_P_S, 1.0f);

/**
 * Proportional gain for vertical velocity error
 *
 * @min 0.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_P_S, 0.2f);

/**
 * Integral gain for vertical velocity error
 *
 * Non zero value allows hovering thrust estimation on stabilized or autonomous takeoff.
 *
 * @min 0.0
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_I_S, 0.02f);

/**
 * Differential gain for vertical velocity error
 *
 * @min 0.0
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_D_S, 0.0f);

/**
 * Maximum vertical velocity
 *
 * Maximum vertical velocity in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL).
 *
 * @unit m/s
 * @min 0.0
 * @max 8.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_MAX_S, 3.0f);

/**
 * Vertical velocity feed forward
 *
 * Feed forward weight for altitude control in stabilized modes (ALTCTRL, POSCTRL). 0 will give slow responce and no overshot, 1 - fast responce and big overshot.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_FF_S, 0.5f);

/**
 * Proportional gain for horizontal position error
 *
 * @min 0.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_P_S, 1.0f);

/**
 * Proportional gain for horizontal velocity error
 *
 * @min 0.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_P_S, 0.1f);

/**
 * Integral gain for horizontal velocity error
 *
 * Non-zero value allows to resist wind.
 *
 * @min 0.0
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_I_S, 0.02f);

/**
 * Differential gain for horizontal velocity error. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_D_S, 0.01f);

/**
 * Maximum horizontal velocity
 *
 * Maximum horizontal velocity in AUTO mode and endpoint for position stabilized mode (POSCTRL).
 *
 * @unit m/s
 * @min 0.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_MAX_S, 5.0f);

/**
 * Horizontal velocity feed forward
 *
 * Feed forward weight for position control in position control mode (POSCTRL). 0 will give slow responce and no overshot, 1 - fast responce and big overshot.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_FF_S, 0.5f);

/**
 * Maximum tilt angle in air
 *
 * Limits maximum tilt in AUTO and POSCTRL modes during flight.
 *
 * @unit degree
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TILTMAX_AIR_S, 45.0f);

/**
 * Maximum tilt during landing
 *
 * Limits maximum tilt angle on landing.
 *
 * @unit degree
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TILTMAX_LND_S, 12.0f);

/**
 * Landing descend rate
 *
 * @unit m/s
 * @min 0.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_LAND_SPEED_S, 0.5f);

/**
 * Takeoff climb rate
 *
 * @unit m/s
 * @min 0.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TKO_SPEED_S, 1.5f);

/**
 * Max manual roll
 *
 * @unit degree
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MAN_R_MAX_S, 35.0f);

/**
 * Max manual pitch
 *
 * @unit degree
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MAN_P_MAX_S, 35.0f);

/**
 * Max manual yaw rate
 *
 * @unit degree / s
 * @min 0.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MAN_Y_MAX_S, 120.0f);

/**
 * Deadzone of X,Y sticks where position hold is enabled
 *
 * @unit %
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_HOLD_XY_DZ_S, 0.1f);

/**
 * Maximum horizontal velocity for which position hold is enabled (use 0 to disable check)
 *
 * @unit m/s
 * @min 0.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_HOLD_MAX_XY_S, 0.5f);

/**
 * Maximum vertical velocity for which position hold is enabled (use 0 to disable check)
 *
 * @unit m/s
 * @min 0.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_HOLD_MAX_Z_S, 0.5f);

/**
 * Low pass filter cut freq. for numerical velocity derivative
 *
 * @unit Hz
 * @min 0.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VELD_LP_S, 5.0f);

/**
 * Maximum horizonal acceleration in velocity controlled modes
 *
 * @unit m/s/s
 * @min 2.0
 * @max 10.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_ACC_HOR_MAX_S, 4.0f);

/**
 * Maximum height in climb rate control
 *
 * @unit m/s/s
 * @min 2.0
 * @max 10.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_POS_HEIGHT_MAX_S, 2.0f);