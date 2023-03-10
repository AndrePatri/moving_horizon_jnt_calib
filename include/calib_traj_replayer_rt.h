#ifndef CALIB_TRAJ_REPLAYER_RT
#define CALIB_TRAJ_REPLAYER_RT

#include <matlogger2/matlogger2.h>

#include <xbot2/xbot2.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>

#include <thread>

#include <xbot2/ros/ros_support.h>

#include <std_msgs/Bool.h>

#include <concert_jnt_calib/CalibTrajStatus.h>
#include <concert_jnt_calib/PerformCalibTraj.h>

#include <concert_jnt_calib/JntCalibStatus.h>
#include <concert_jnt_calib/JntCalibRt.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <awesome_utils/awesome_utils/traj_utils.hpp>
#include <awesome_utils/xbot2_utils/xbot2_utils.hpp>
#include <awesome_utils/awesome_utils/sign_proc_utils.hpp>
#include <awesome_utils/awesome_utils/calib_utils.hpp>

using namespace XBot;
using namespace XBot::Cartesian;
using namespace TrajUtils;
using namespace CalibUtils;
using namespace SignProcUtils;
using namespace Xbot2Utils;

typedef Eigen::Array<bool, Eigen::Dynamic, 1> VectorXb;

/**
 * @brief
 */
class CalibTrajReplayerRt : public ControlPlugin
{

public:

    using ControlPlugin::ControlPlugin;

    // initialization method; the plugin won't be run
    // if this returns 'false'
    bool on_initialize() override;

    // callback for the 'Starting' state
    // start_completed() must be called to switch
    // to 'Run' state
    void starting() override;

    // callback for 'Run' state
    void run() override;

    // callback for 'On Stop' state
    void stopping() override;

    // callback for 'On Stop' state
    void on_stop() override;

    // callback for 'On Abort' state
    void on_close() override;

    // callback for 'On Abort' state
    void on_abort() override;


private:

    bool _is_first_run = true,
        _send_pos_ref = true, _send_vel_ref = true,  _send_eff_ref = false,
        _is_sim = true, _is_dummy = false,
        _reduce_dumped_sol_size = false,
        _verbose = false,
        _go2calib_traj = false, _approach_traj_started = false, _approach_traj_finished = false,
        _perform_traj = false, _traj_started = false, _traj_finished = false,
        _idle = true;

    int _n_jnts_robot,
        _performed_traj_n = 0;

    std::string _dump_mat_suffix = "traj_replay",
                _hw_type,
                _dump_path = "/tmp/CalibTrajReplayerRt";

    double _plugin_dt,
        _loop_time = 0.0, _loop_timer_reset_time = 3600.0,
        _traj_time = 0.0, _traj_execution_time = 10.0,
        _approach_traj_time = 0.0, _approach_traj_exec_time = 3.0, _approach_traj_phase = 0.0,
        _matlogger_buffer_size = 1e5,
        _omega0_s = 0, _omegaf_s = 0, _t_exec_omega_s = 10.0, _q_ub_s = 0.0, _q_lb_s = 0.0,
        _q_temp, _q_dot_temp, _q_ddot_temp,
        _sweep_min_t_exec = 0.5;


    std::vector<std::string> _jnt_list;
    std::vector<int> _jnt_indxs;
    std::vector<double> _omega0;
    std::vector<double> _omegaf;
    std::vector<double> _t_exec_omega;
    std::vector<double> _q_ub;
    std::vector<double> _q_lb;
    std::vector<SweepCos> _sweep_trajs;

    std::vector<std::string> _enbld_jnt_names;

    std::vector<double> _phase_omega;
    std::vector<double> _ramp_up;
    std::vector<double> _omega_k;
    std::vector<double> _time_ref;

    std::vector<bool> _cal_mask;

    Eigen::VectorXd _q_p_meas,
                    _q_p_dot_meas, _q_p_dot_meas_filt,
                    _q_p_ddot_meas, _q_p_ddot_meas_filt,
                    _tau_meas, _tau_meas_filt,
                    _iq_meas, _iq_meas_filt,
                    _jnt_cal_sol_millis, _alpha_f0, _alpha_f1,
                    _K_d0, _K_d1, _rot_MoI, _K_t, _red_ratio,
                    _K_d0_ig, _K_d1_ig, _rot_MoI_ig, _K_t_ig,
                    _K_d0_nom, _K_d1_nom, _rot_MoI_nom, _K_t_nom,
                    _q_p_cmd, _q_p_dot_cmd, _q_p_ddot_cmd,
                    _q_p_safe_cmd,
                    _q_min, _q_max, _q_dot_lim,
                    _q_p_init_appr_traj, _q_p_trgt_appr_traj;

    std::vector<double> _q_p_cmd_vect, _q_p_dot_cmd_vect, _q_p_ddot_cmd_vect;

    std::vector<double> _q_p_ddot_est_vect,
                        _q_p_dot_meas_vect,
                        _tau_meas_vect,
                        _K_t_vect,
                        _K_d0_vect,
                        _K_d1_vect,
                        _rot_MoI_vect,
                        _red_ratio_vect,
                        _alpha_f0_vect,
                        _alpha_f1_vect,
                        _iq_meas_vect,
                        _jnt_cal_sol_millis_vect;

    PeisekahTrans _peisekah_utils;

    MatLogger2::Ptr _dump_logger;

    RosSupport::UniquePtr _ros;

    CallbackQueue _queue;

    ServiceServerPtr<concert_jnt_calib::PerformCalibTrajRequest,
                     concert_jnt_calib::PerformCalibTrajResponse> _perform_traj_srvr;

    ServiceServerPtr<concert_jnt_calib::JntCalibRtRequest,
                     concert_jnt_calib::JntCalibRtResponse> _set_cal_srvr;

    PublisherPtr<concert_jnt_calib::CalibTrajStatus> _traj_status_pub;
    PublisherPtr<concert_jnt_calib::JntCalibStatus> _jnt_calib_pub;

    SubscriberPtr<xbot_msgs::CustomState> _aux_signals_sub;

    IqOutRosGetter _iq_getter;

    MovAvrgFilt _mov_avrg_filter_tau;
    MovAvrgFilt _mov_avrg_filter_q_dot;
    MovAvrgFilt _mov_avrg_filter_q_ddot;
    int _mov_avrg_window_size = 10;
    double _mov_avrg_cutoff_freq= 15.0;

    NumDiff _num_diff;

    void get_params_from_config();

    void init_vars();
    void init_clocks();
    void init_ros_bridge();
    void init_dump_logger();

    void reset_clocks();

    void saturate_cmds();
    
    void update_state();

    void update_clocks();

    void set_approach_trajectory();
    void set_calib_trajectory();

    void set_cmds();
    void send_cmds();

    void add_data2dump_logger();
    void add_data2bedumped();

    void spawn_nrt_thread();

    void is_sim(std::string sim_string);
    void is_dummy(std::string dummy_string);

    void pub_replay_status();
    void pub_calib_status();

    void run_jnt_calib();

    bool on_perform_traj_received(const concert_jnt_calib::PerformCalibTrajRequest& req,
                                  concert_jnt_calib::PerformCalibTrajResponse& res);

    bool on_jnt_cal_received(const concert_jnt_calib::JntCalibRtRequest& req,
                             concert_jnt_calib::JntCalibRtResponse& res);
};

#endif // CALIB_TRAJ_REPLAYER_RT
