/**
 * @file mc_lyapunov_control_params.c
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
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_IX, 0.0095f);
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
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_IY, 0.0095f);
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
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_IZ, 0.0186f);
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
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_M, 1.0230f);
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
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_K1, 17.4f);
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
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_K2, 8.7f);
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
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_K3, 17.4f);
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
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_K4, 8.7f);
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
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_K5, 3.5f);
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
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_K6, 3.6f);
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
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_K7, 2.0f);
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
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_K8, 2.0f);
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
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_K9, 2.0f);
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
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_K10, 2.0f);
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
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_K11, 1.5f);
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
PARAM_DEFINE_FLOAT(MC_LYAPUNOV_K12, 1.5f);
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
PARAM_DEFINE_FLOAT(MC_LYA_U1_MAX, 30.0f);
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
PARAM_DEFINE_FLOAT(MC_LYA_U2_MAX, 5.00f);
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
PARAM_DEFINE_FLOAT(MC_LYA_U3_MAX, 5.00f);
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
PARAM_DEFINE_FLOAT(MC_LYA_U4_MAX, 5.00f);
