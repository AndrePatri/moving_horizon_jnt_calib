// Copyright (C) 2023  Andrea Patrizi (AndrePatri, andreapatrizi1b6e6@gmail.com)
// 
// This file is part of moving_horizon_jnt_calib and distributed under the General Public License version 2 license.
// 
// moving_horizon_jnt_calib is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 2 of the License, or
// (at your option) any later version.
// 
// moving_horizon_jnt_calib is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with moving_horizon_jnt_calib.  If not, see <http://www.gnu.org/licenses/>.
// 
#ifndef CALIB_TRAJ_REPLAYER_RT
#define CALIB_TRAJ_REPLAYER_RT

#include <yaml-cpp/yaml.h>

#include <matlogger2/matlogger2.h>

#include <xbot2/xbot2.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>

#include <thread>

#include <xbot2/ros/ros_support.h>

#include <std_msgs/Bool.h>

#include <moving_horizon_jnt_calib/CalibTrajStatus.h>
#include <moving_horizon_jnt_calib/PerformCalibTraj.h>
#include <moving_horizon_jnt_calib/SetCalibParams.h>

#include <moving_horizon_jnt_calib/JntCalibStatus.h>
#include <moving_horizon_jnt_calib/StartCalib.h>

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
        _q_dot_temp, _q_ddot_temp,
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

    YAML::Node _jnt_cal_yaml;

    Eigen::VectorXd _q_min, _q_max, _q_dot_lim,
                    _q_p_init_appr_traj, _q_p_trgt_appr_traj,
                    _q_p_cmd, _q_p_dot_cmd, _q_p_ddot_cmd,
                    _q_p_safe_cmd, _q_p_meas;

    std::vector<double> _q_p_cmd_vect, _q_p_dot_cmd_vect, _q_p_ddot_cmd_vect;

    PeisekahTrans _peisekah_utils;

    MatLogger2::Ptr _dump_logger;

    RosSupport::UniquePtr _ros;

    CallbackQueue _queue;

    ServiceServerPtr<moving_horizon_jnt_calib::PerformCalibTrajRequest,
                     moving_horizon_jnt_calib::PerformCalibTrajResponse> _perform_traj_srvr;

    PublisherPtr<moving_horizon_jnt_calib::CalibTrajStatus> _traj_status_pub;

    void get_params_from_config();
    void param_dims_ok_or_throw();

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

    bool on_perform_traj_received(const moving_horizon_jnt_calib::PerformCalibTrajRequest& req,
                                  moving_horizon_jnt_calib::PerformCalibTrajResponse& res);

};

#endif // CALIB_TRAJ_REPLAYER_RT
