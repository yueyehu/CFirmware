
/**
 * @file mc_lyapunov_control_main.cpp
 * Multicopter position controller.
 *
 * @author hukaijian<jerry_git@163.com>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <functional>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/mc_virtual_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_attitude.h>

#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <mavlink/mavlink_log.h>
#include <platforms/px4_defines.h>

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#define ONE_G	9.8066f

/**
 * Multicopter position control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_lyapunov_control_main(int argc, char *argv[]);

class MulticopterLyapunovControl : public control::SuperBlock
{
public:
    /**
     * Constructor
     */
    MulticopterLyapunovControl();

    /**
     * Destructor, also kills task.
     */
    ~MulticopterLyapunovControl();

    /**
     * Start task.
     *
     * @return		OK on success.
     */
    int		start();

private:
    bool	_task_should_exit;		/**< if true, task should exit */
    int		_control_task;			/**< task handle for task */
    int		_mavlink_fd;			/**< mavlink fd */

    int		_vehicle_status_sub;		/**< vehicle status subscription */
    int		_ctrl_state_sub;		/**< control state subscription */
    int		_att_sp_sub;			/**< vehicle attitude setpoint */
    int		_control_mode_sub;		/**< vehicle control mode subscription */
    int		_params_sub;			/**< notification of parameter updates */
    int		_manual_sub;			/**< notification of manual control updates */
    int		_arming_sub;			/**< arming status of outputs */
    int		_local_pos_sub;			/**< vehicle local position */
    int		_pos_sp_triplet_sub;		/**< position setpoint triplet */
    int		_local_pos_sp_sub;		/**< offboard local position setpoint */
    int		_global_vel_sp_sub;		/**< offboard global velocity setpoint */
    int     _vehicle_att_sub;

    orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
    orb_advert_t	_local_pos_sp_pub;		/**< vehicle local position setpoint publication */
    orb_advert_t	_global_vel_sp_pub;		/**< vehicle global velocity setpoint publication */
    orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */

    orb_id_t _attitude_setpoint_id;

    struct vehicle_status_s 			_vehicle_status; 	/**< vehicle status */
    struct control_state_s				_ctrl_state;			/**< vehicle attitude */
    struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
    struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
    struct vehicle_control_mode_s			_control_mode;		/**< vehicle control mode */
    struct actuator_armed_s				_arming;		/**< actuator arming status */
    struct vehicle_local_position_s			_local_pos;		/**< vehicle local position */
    struct position_setpoint_triplet_s		_pos_sp_triplet;	/**< vehicle global position setpoint triplet */
    struct vehicle_local_position_setpoint_s	_local_pos_sp;		/**< vehicle local position setpoint */
    struct vehicle_global_velocity_setpoint_s	_global_vel_sp;		/**< vehicle global velocity setpoint */
    struct actuator_controls_s			_actuators;			/**< actuator controls */
    struct {
        param_t Ix;
        param_t Iy;
        param_t Iz;
        param_t m;
        param_t k1;
        param_t k2;
        param_t k3;
        param_t k4;
        param_t k5;
        param_t k6;
        param_t k7;
        param_t k8;
        param_t k9;
        param_t k10;
        param_t k11;
        param_t k12;
        param_t u1_max;
        param_t u2_max;
        param_t u3_max;
        param_t u4_max;
    }_params_handles;		/**< handles for interesting parameters */

    struct {
        float Ix;
        float Iy;
        float Iz;
        float m;
        float k1;
        float k2;
        float k3;
        float k4;
        float k5;
        float k6;
        float k7;
        float k8;
        float k9;
        float k10;
        float k11;
        float k12;
        float u1_max;
        float u2_max;
        float u3_max;
        float u4_max;
    }_params;

    struct map_projection_reference_s _ref_pos;
    float _ref_alt;
    hrt_abstime _ref_timestamp;

    math::Vector<3> _pos;
    math::Vector<3> _pos_sp;
    math::Vector<3> _vel;
    math::Vector<3> _thrust_sp_prev;

    bool _in_landing;	/**< the vehicle is in the landing descent */

    /**
     * Update our local parameter cache.
     */
    int	parameters_update(bool force);

    /**
     * Check for changes in subscribed topics.
     */
    void poll_subscriptions();

    void update_ref();
    /**
     * Set position setpoint for AUTO
     */
    void control_auto(float dt);

    /**
     * Shim for calling task_main from task_create.
     */
    static void	task_main_trampoline(int argc, char *argv[]);

    /**
     * Main sensor collection task.
     */
    void task_main();
    float limitMinMax(float input,float min,float max);
};

namespace lyapunov_control
{

MulticopterLyapunovControl	*g_control;
}

MulticopterLyapunovControl::MulticopterLyapunovControl() :
    SuperBlock(NULL, "MPC"),
    _task_should_exit(false),
    _control_task(-1),
    _mavlink_fd(-1),

    /* subscriptions */
    _ctrl_state_sub(-1),
    _att_sp_sub(-1),
    _control_mode_sub(-1),
    _params_sub(-1),
    _manual_sub(-1),
    _arming_sub(-1),
    _local_pos_sub(-1),
    _pos_sp_triplet_sub(-1),
    _global_vel_sp_sub(-1),
    _vehicle_att_sub(-1),

    /* publications */
    _att_sp_pub(nullptr),
    _local_pos_sp_pub(nullptr),
    _global_vel_sp_pub(nullptr),
    _actuators_0_pub(nullptr),
    _attitude_setpoint_id(0),
    _ref_alt(0.0f),
    _ref_timestamp(0),
    _in_landing(false)
{
    memset(&_vehicle_status, 0, sizeof(_vehicle_status));
    memset(&_ctrl_state, 0, sizeof(_ctrl_state));
    _ctrl_state.q[0] = 1.0f;
    memset(&_att_sp, 0, sizeof(_att_sp));
    memset(&_manual, 0, sizeof(_manual));
    memset(&_control_mode, 0, sizeof(_control_mode));
    memset(&_arming, 0, sizeof(_arming));
    memset(&_local_pos, 0, sizeof(_local_pos));
    memset(&_pos_sp_triplet, 0, sizeof(_pos_sp_triplet));
    memset(&_local_pos_sp, 0, sizeof(_local_pos_sp));
    memset(&_global_vel_sp, 0, sizeof(_global_vel_sp));
    memset(&_actuators, 0, sizeof(_actuators));

    memset(&_ref_pos, 0, sizeof(_ref_pos));

    _pos.zero();
    _pos_sp.zero();
    _vel.zero();

    _params_handles.Ix		= param_find("MC_LYAPUNOV_IX");
    _params_handles.Iy		= param_find("MC_LYAPUNOV_IY");
    _params_handles.Iz		= param_find("MC_LYAPUNOV_IZ");
    _params_handles.m		= param_find("MC_LYAPUNOV_M");
    _params_handles.k1		= param_find("MC_LYAPUNOV_K1");
    _params_handles.k2		= param_find("MC_LYAPUNOV_K2");
    _params_handles.k3		= param_find("MC_LYAPUNOV_K3");
    _params_handles.k4		= param_find("MC_LYAPUNOV_K4");
    _params_handles.k5		= param_find("MC_LYAPUNOV_K5");
    _params_handles.k6		= param_find("MC_LYAPUNOV_K6");
    _params_handles.k7		= param_find("MC_LYAPUNOV_K7");
    _params_handles.k8		= param_find("MC_LYAPUNOV_K8");
    _params_handles.k9		= param_find("MC_LYAPUNOV_K9");
    _params_handles.k10		= param_find("MC_LYAPUNOV_K10");
    _params_handles.k11		= param_find("MC_LYAPUNOV_K11");
    _params_handles.k12		= param_find("MC_LYAPUNOV_K12");
    _params_handles.u1_max  = param_find("MC_LYA_U1_MAX");
    _params_handles.u2_max  = param_find("MC_LYA_U2_MAX");
    _params_handles.u3_max  = param_find("MC_LYA_U3_MAX");
    _params_handles.u4_max  = param_find("MC_LYA_U4_MAX");
    
    /* fetch initial parameter values */
    parameters_update(true);
}

MulticopterLyapunovControl::~MulticopterLyapunovControl()
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
                px4_task_delete(_control_task);
                break;
            }
        } while (_control_task != -1);
    }

    lyapunov_control::g_control = nullptr;
}

int
MulticopterLyapunovControl::parameters_update(bool force)
{
    bool updated;
    struct parameter_update_s param_upd;

    orb_check(_params_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(parameter_update), _params_sub, &param_upd);
    }

    if (updated || force) {
        /* update C++ param system */
        updateParams();

        /* update legacy C interface params */
        param_get(_params_handles.Ix,&_params.Ix);
        param_get(_params_handles.Iy,&_params.Iy);
        param_get(_params_handles.Iz,&_params.Iz);
        param_get(_params_handles.m,&_params.m);
        param_get(_params_handles.k1,&_params.k1);
        param_get(_params_handles.k2,&_params.k2);
        param_get(_params_handles.k3,&_params.k3);
        param_get(_params_handles.k4,&_params.k4);
        param_get(_params_handles.k5,&_params.k5);
        param_get(_params_handles.k6,&_params.k6);
        param_get(_params_handles.k7,&_params.k7);
        param_get(_params_handles.k8,&_params.k8);
        param_get(_params_handles.k9,&_params.k9);
        param_get(_params_handles.k10,&_params.k10);
        param_get(_params_handles.k11,&_params.k11);
        param_get(_params_handles.k12,&_params.k12);
        param_get(_params_handles.u1_max,&_params.u1_max);
        param_get(_params_handles.u2_max,&_params.u2_max);
        param_get(_params_handles.u3_max,&_params.u3_max);
        param_get(_params_handles.u4_max,&_params.u4_max);
    }

    return OK;
}

void
MulticopterLyapunovControl::poll_subscriptions()
{
    bool updated;

    orb_check(_vehicle_status_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);      
    }

    orb_check(_ctrl_state_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);
    }

    orb_check(_att_sp_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
    }

    orb_check(_control_mode_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
    }

    orb_check(_manual_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
    }

    orb_check(_arming_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(actuator_armed), _arming_sub, &_arming);
    }

    orb_check(_local_pos_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
    }
}

void
MulticopterLyapunovControl::task_main_trampoline(int argc, char *argv[])
{
    lyapunov_control::g_control->task_main();
}

void
MulticopterLyapunovControl::update_ref()
{
    if (_local_pos.ref_timestamp != _ref_timestamp) {
        double lat_sp, lon_sp;
        float alt_sp = 0.0f;

        if (_ref_timestamp != 0) {
            /* calculate current position setpoint in global frame */
            map_projection_reproject(&_ref_pos, _pos_sp(0), _pos_sp(1), &lat_sp, &lon_sp);
            alt_sp = _ref_alt - _pos_sp(2);
        }

        /* update local projection reference */
        map_projection_init(&_ref_pos, _local_pos.ref_lat, _local_pos.ref_lon);
        _ref_alt = _local_pos.ref_alt;

        if (_ref_timestamp != 0) {
            /* reproject position setpoint to new reference */
            map_projection_project(&_ref_pos, lat_sp, lon_sp, &_pos_sp.data[0], &_pos_sp.data[1]);
            _pos_sp(2) = -(alt_sp - _ref_alt);
        }

        _ref_timestamp = _local_pos.ref_timestamp;
    }
}

float
MulticopterLyapunovControl::limitMinMax(float input, float _min, float _max){
    if(input < _min){
        return _min;
    }
    else if(input > _max){
        return _max;
    }
    else{
        return input;
    }
}

void
MulticopterLyapunovControl::control_auto(float dt)
{

    //Poll position setpoint
    bool updated;
    orb_check(_pos_sp_triplet_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);

        //Make sure that the position setpoint is valid
        if (!PX4_ISFINITE(_pos_sp_triplet.current.lat) ||
                !PX4_ISFINITE(_pos_sp_triplet.current.lon) ||
                !PX4_ISFINITE(_pos_sp_triplet.current.alt)) {
            _pos_sp_triplet.current.valid = false;
        }
    }

    bool current_setpoint_valid = false;

    math::Vector<3> curr_sp;
    curr_sp.zero();

    if (_pos_sp_triplet.current.valid) {

        /* project setpoint to local frame */
        map_projection_project(&_ref_pos,
                       _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon,
                       &curr_sp.data[0], &curr_sp.data[1]);
        curr_sp(2) = -(_pos_sp_triplet.current.alt - _ref_alt);

        if (PX4_ISFINITE(curr_sp(0)) &&
                PX4_ISFINITE(curr_sp(1)) &&
                PX4_ISFINITE(curr_sp(2))) {
            current_setpoint_valid = true;
        }
        if(current_setpoint_valid){
            _pos_sp(0)=curr_sp(0);
            _pos_sp(1)=curr_sp(1);
            _pos_sp(2)=curr_sp(2);
        }
    } else {
        /* no waypoint, do nothing, setpoint was already reset */
    }
}

void
MulticopterLyapunovControl::task_main()
{

    _mavlink_fd = px4_open(MAVLINK_LOG_DEVICE, 0);

    /*
     * do subscriptions
     */
    _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
    _ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
    _att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
    _control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
    _params_sub = orb_subscribe(ORB_ID(parameter_update));
    _manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    _arming_sub = orb_subscribe(ORB_ID(actuator_armed));
    _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    _pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
    _local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
    _global_vel_sp_sub = orb_subscribe(ORB_ID(vehicle_global_velocity_setpoint));

    parameters_update(true);
    //mavlink_log_info(_mavlink_fd, "[mc_lyapunov]param:%8.4f,%8.4f,%8.4f",(double)_params.Ix,(double)_params.Iy,(double)_params.Iz);
    /* initialize values of critical structs until first regular update */
    _arming.armed = false;

    /* get an initial update for all sensor and status data */
    poll_subscriptions();

    hrt_abstime t_prev = 0;

    float phi,theta,psi;
    float dphi,dtheta,dpsi;
    phi=theta=psi=0;
    dphi=dtheta=dpsi=0;

    float m_x,m_y,m_z;
    float m_phi,m_theta,m_psi;
    m_x=m_y=m_z=0;
    m_phi=m_theta=m_psi=0;

    math::Vector<13> e;
    e.zero();

    math::Vector<3> alpha;
    alpha.zero();
    math::Vector<3> beta;
    beta.zero();

    float phi_cmd;
    float the_cmd;
    float psi_cmd;

    math::Vector<4> U;
    U.zero();

    float f1,f2;
    f1=f2=0;
    float g1,g2,g3;
    g1=g2=g3=0;

    /* wakeup source */
    px4_pollfd_struct_t fds[1];

    fds[0].fd = _local_pos_sub;
    fds[0].events = POLLIN;

    while (!_task_should_exit) {
        /* wait for up to 500ms for data */
        int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

        /* timed out - periodic check for _task_should_exit */
        if (pret == 0) {
            continue;
        }

        /* this is undesirable but not much we can do */
        if (pret < 0) {
            warn("poll error %d, %d", pret, errno);
            continue;
        }

        poll_subscriptions();

        parameters_update(false);

        hrt_abstime t = hrt_absolute_time();
        float dt = t_prev != 0 ? (t - t_prev) * 0.000001f : 0.0f;
        t_prev = t;

        // set dt for control blocks
        setDt(dt);

        update_ref();

        /* Update velocity derivative,
         * independent of the current flight mode
         */
        if (_local_pos.timestamp > 0) {

            if (PX4_ISFINITE(_local_pos.x) &&
                    PX4_ISFINITE(_local_pos.y) &&
                    PX4_ISFINITE(_local_pos.z)) {
                _pos(0) = _local_pos.x;
                _pos(1) = _local_pos.y;
                _pos(2) = _local_pos.z;
            }

            if (PX4_ISFINITE(_local_pos.vx) &&
                    PX4_ISFINITE(_local_pos.vy) &&
                    PX4_ISFINITE(_local_pos.vz)) {
                _vel(0) = _local_pos.vx;
                _vel(1) = _local_pos.vy;
                _vel(2) = _local_pos.vz;
                m_x = _params.m*_vel(0);
                m_y = _params.m*_vel(1);
                m_z = _params.m*_vel(2);
            }
        }
        if (_ctrl_state.timestamp > 0){
            math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
            math::Vector<3> euler= q_att.to_euler();
            phi = euler(0);
            theta = euler(1);
            psi = euler(2);
            dphi = _ctrl_state.roll_rate;
            dtheta = _ctrl_state.pitch_rate;
            dpsi = _ctrl_state.yaw_rate;
            mavlink_log_info(_mavlink_fd, "[mc_lyapunov]degree:%8.4f,%8.4f",(double)psi,(double)dpsi);
            m_phi = _params.Ix*dphi-_params.Ix*sinf(theta)*dpsi;
            m_theta = (_params.Iy*cosf(phi)*cosf(phi)+_params.Iz*(sinf(phi)*sinf(phi)))*dtheta+(_params.Iy-_params.Iz)*cosf(phi)*sinf(phi)*cosf(theta)*dpsi;
            m_psi = -_params.Ix*sinf(theta)*dphi+(_params.Iy-_params.Iz)*cosf(phi)*sinf(phi)*cosf(theta)*dtheta+\
                    (_params.Ix*sinf(theta)*sinf(theta)+_params.Iy*sinf(phi)*sinf(phi)*cosf(theta)*cosf(theta)+_params.Iz*cosf(phi)*cosf(phi)*cosf(theta)*cosf(theta))*dpsi;
        }
        else{
            mavlink_log_info(_mavlink_fd, "[mc_lyapunov]log:attitude error");
            continue;
        }
        g1 = (_params.Iy*_params.Iz+(_params.Ix-_params.Iz)*_params.Iy*sinf(theta)*sinf(theta)-(_params.Iy-_params.Iz)*_params.Ix*(float)(sinf(phi)*sinf(phi)*sinf(theta)*sinf(theta)))/\
                (_params.Ix*_params.Iy*_params.Iy*cosf(theta)*cosf(theta));
        g2 = (_params.Iy*sinf(phi)*sinf(phi)+_params.Iz*cosf(phi)*cosf(phi))/(_params.Iy*_params.Iz);
        g3 = (_params.Iy*cosf(phi)*cosf(phi)+_params.Iz*sinf(phi)*sinf(phi))/(_params.Iy*_params.Iz*cosf(theta)*cosf(theta));

        f1 = -(_params.Iy-_params.Iz)*(dtheta*dtheta*sinf(phi)*cosf(phi)-dtheta*dpsi*cosf(2*phi)*cosf(theta)-dpsi*dpsi*sinf(phi)*cosf(phi)*cosf(theta)*cosf(theta));
        f2 = (_params.Ix-_params.Iy*sinf(phi)*sinf(phi)-_params.Iz*cosf(phi)*cosf(phi))*dpsi*dpsi*sinf(theta)*cosf(theta)\
                -(_params.Iy-_params.Iz)*dtheta*dpsi*sinf(phi)*cosf(phi)*sinf(theta)-dphi*dpsi*_params.Ix*cosf(theta);

        //mavlink_log_info(_mavlink_fd, "[mc_lyapunov]log:%8.4f,%8.4f,%8.4f",(double)g1,(double)g2,(double)g3);

        if (_control_mode.flag_control_auto_enabled) {
            PX4_WARN("[mc_lyapunov]log:auto mode is set");
            control_auto(dt);

            if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid
                    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
                /* idle state, don't run controller and set zero thrust */
                for(int i=0;i<4;i++){
                    _actuators.control[i] = 0;
                }
                _actuators.timestamp = hrt_absolute_time();
                _actuators.timestamp_sample = _ctrl_state.timestamp;
                if (_actuators_0_pub != nullptr) {
                    orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);

                } else {
                    _actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
                }
            }else {
                e(7) = _pos(0)-_pos_sp(0);
                e(7) = limitMinMax(e(7),-1,1);
                e(8) = m_x/_params.m + _params.k7*e(7);
                e(9) = _pos(1)-_pos_sp(1);
                e(9) = limitMinMax(e(9),-1,1);
                e(10) = m_y/_params.m + _params.k9*e(9);
                e(11) = _pos(2)-_pos_sp(2);
                e(11) = limitMinMax(e(11),-1,1);
                e(12) = m_z/_params.m + _params.k11*e(11);

                alpha(0) = _params.m*((_params.k7*_params.k7-1)*e(7)-(_params.k7+_params.k8)*e(8));
                alpha(1) = _params.m*((_params.k9*_params.k9-1)*e(9)-(_params.k9+_params.k10)*e(10));
                alpha(2) = -_params.m*ONE_G+_params.m*((_params.k11*_params.k11-1)*e(11)-(_params.k11+_params.k12)*e(12));

                beta(0) = alpha(0)*cosf(psi)+alpha(1)*sinf(psi);
                beta(1) = -alpha(0)*sinf(psi)+alpha(1)*cosf(psi);
                beta(2) = alpha(2);

                the_cmd = atanf(beta(0)/beta(2));
                phi_cmd = atanf(-beta(1)*cosf(the_cmd)/beta(2));
                U(0) = beta(2)/(cosf(phi_cmd)*cosf(the_cmd));

                the_cmd = limitMinMax(the_cmd,-0.3,0.3);
                phi_cmd = limitMinMax(phi_cmd,-0.3,0.3);
                psi_cmd = 0;

                e(1) = phi - phi_cmd;
                e(2) = m_phi/_params.Ix + _params.k1*e(1);
                e(3) = theta - the_cmd;
                e(4) = m_theta/_params.Iy + _params.k3*e(3);
                e(5) = psi - psi_cmd;
                e(6) = m_psi/_params.Iz + _params.k5*e(5);

                U(1) = -f1+(-(_params.k1*g1*_params.Ix+_params.k2)*e(2)+(g1*_params.Ix*_params.k1*_params.k1-g1*_params.Ix)*e(1))*_params.Ix;
                U(2) = -f2+(-(_params.k3*g2*_params.Iy+_params.k4)*e(4)+(g2*_params.Iy*_params.k3*_params.k3-g2*_params.Iy)*e(3))*_params.Iy;
                U(3) = (-(_params.k5*g3*_params.Iz+_params.k6)*e(6)+(g3*_params.Iz*_params.k5*_params.k5-g3*_params.Iz)*e(5))*_params.Iz;

                mavlink_log_info(_mavlink_fd, "control data:%8.4f,%8.4f,%8.4f,%8.4f",(double)U(0),(double)U(1),(double)U(2),(double)U(3));
                if (U(0)>0){
                    U(0) = 0.0;
                }
                else if(U(0)<-_params.u1_max){
                    U(0) = 1.0;
                }
                else{
                    U(0) = fabsf(U(0))/_params.u1_max;
                }
                if (fabsf(U(1))>_params.u2_max){
                    U(1) = fabsf(U(1))/U(1);
                }
                else{
                    U(1) = U(1)/_params.u2_max;
                }
                if (fabsf(U(2))>_params.u3_max){
                    U(2) = fabsf(U(2))/U(2);
                }
                else{
                    U(2) = U(2)/_params.u3_max;
                }
                if (fabsf(U(3))>_params.u4_max){
                    U(3) = fabsf(U(3))/U(3);
                }
                else{
                    U(3) = U(3)/_params.u4_max;
                }
                U(1)=limitMinMax(U(1),-0.3,0.3);
                U(2)=limitMinMax(U(2),-0.3,0.3);
                U(3)=limitMinMax(U(3),-0.3,0.3);
                //mavlink_log_info(_mavlink_fd, "control data:%8.4f,%8.4f,%8.4f,%8.4f",(double)U(0),(double)U(1),(double)U(2),(double)U(3));
                _actuators.control[0] = (PX4_ISFINITE(U(1))) ? U(1) : 0.0f;
                _actuators.control[1] = (PX4_ISFINITE(U(2))) ? U(2) : 0.0f;
                _actuators.control[2] = (PX4_ISFINITE(U(3))) ? U(3) : 0.0f;
                _actuators.control[3] = (PX4_ISFINITE(U(0))) ? U(0) : 0.0f;

                _actuators.timestamp = hrt_absolute_time();
                _actuators.timestamp_sample = _ctrl_state.timestamp;
                if (_actuators_0_pub != nullptr) {
                    orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);
                } else {
                    _actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
                }
            }
        }
    }

    mavlink_log_info(_mavlink_fd, "[mpc] stopped");

    _control_task = -1;
}

int
MulticopterLyapunovControl::start()
{
    ASSERT(_control_task == -1);

    /* start the task */
    _control_task = px4_task_spawn_cmd("mc_lyapunov_control",
                       SCHED_DEFAULT,
                       SCHED_PRIORITY_MAX - 5,
                       1900,
                       (px4_main_t)&MulticopterLyapunovControl::task_main_trampoline,
                       nullptr);

    if (_control_task < 0) {
        warn("task start failed");
        return -errno;
    }

    return OK;
}

int
mc_lyapunov_control_main(int argc, char *argv[])
{
    if (argc < 2) {
        warnx("usage: mc_pos_control {start|stop|status}");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {

        if (lyapunov_control::g_control != nullptr) {
            warnx("already running");
            return 1;
        }

        lyapunov_control::g_control = new MulticopterLyapunovControl;

        if (lyapunov_control::g_control == nullptr) {
            warnx("alloc failed");
            return 1;
        }

        if (OK != lyapunov_control::g_control->start()) {
            delete lyapunov_control::g_control;
            lyapunov_control::g_control = nullptr;
            warnx("start failed");
            return 1;
        }

        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        if (lyapunov_control::g_control == nullptr) {
            warnx("not running");
            return 1;
        }

        delete lyapunov_control::g_control;
        lyapunov_control::g_control = nullptr;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (lyapunov_control::g_control) {
            warnx("running");
            return 0;

        } else {
            warnx("not running");
            return 1;
        }
    }

    warnx("unrecognized command");
    return 1;
}
