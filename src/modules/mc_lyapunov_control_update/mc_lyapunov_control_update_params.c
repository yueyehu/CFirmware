/**
 * @file MC_LYAPUNOV_UP_control_params.c
 * Multicopter lyapunov controller parameters.
 *
 * @author hukaijian<jerry_git@163.com>
 */

/**
 * memotum
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 4
 * @group Multicopter Lyapunov Control
 */
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_UP_IX, 0.0095f);
/**
 * memotum
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 4
 * @group Multicopter Lyapunov Control
 */
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_UP_IY, 0.0095f);
/**
 * memotum
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 4
 * @group Multicopter Lyapunov Control
 */
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_UP_IZ, 0.0186f);
/**
 * mass
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 4
 * @group Multicopter Lyapunov Control
 */
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_UP_M, 1.0230f);
/**
 * K
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group Multicopter Lyapunov Control
 */
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_UP_K1, 15f);
/**
 * K
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group Multicopter Lyapunov Control
 */
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_UP_K2, 8f);
/**
 * K
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group Multicopter Lyapunov Control
 */
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_UP_K3, 15f);
/**
 * K
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group Multicopter Lyapunov Control
 */
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_UP_K4, 8f);
/**
 * K
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group Multicopter Lyapunov Control
 */
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_UP_K5, 2.5f);
/**
 * K
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group Multicopter Lyapunov Control
 */
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_UP_K6, 2.5f);
/**
 * K
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group Multicopter Lyapunov Control
 */
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_UP_K7, 2.0f);
/**
 * K
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group Multicopter Lyapunov Control
 */
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_UP_K8, 2.0f);
/**
 * K
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group Multicopter Lyapunov Control
 */
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_UP_K9, 2.0f);
/**
 * K
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group Multicopter Lyapunov Control
 */
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_UP_K10, 2.0f);
/**
 * K
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group Multicopter Lyapunov Control
 */
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_UP_K11, 2.0f);
/**
 * K
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group Multicopter Lyapunov Control
 */
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_UP_K12, 2.0f);
/**
 * U_MAX
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.0
 * @max 40.0
 * @decimal 2
 * @group Multicopter Lyapunov Control
 */
PARAM_DEFINE_FLOAT(MC_LYA_U1_UP_MAX, 30.0f);
/**
 * U_MAX
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 4
 * @group Multicopter Lyapunov Control
 */
PARAM_DEFINE_FLOAT(MC_LYA_U2_UP_MAX, 5.00f);
/**
 * U_MAX
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group Multicopter Lyapunov Control
 */
PARAM_DEFINE_FLOAT(MC_LYA_U3_UP_MAX, 5.00f);
/**
 * U_MAX
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group Multicopter Lyapunov Control
 */
PARAM_DEFINE_FLOAT(MC_LYA_U4_UP_MAX, 5.00f);
