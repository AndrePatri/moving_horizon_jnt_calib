#include "calib_traj_replayer_rt.h"

#include <math.h> 

void CalibTrajReplayerRt::init_clocks()
{
    _loop_time = 0.0; // reset loop time clock

    _traj_time = 0.0; // reset traj time clock

    _approach_traj_time = 0.0; // reset time for approach trajectory

}

void CalibTrajReplayerRt::reset_clocks()
{

    _traj_time = 0.0;
    _approach_traj_time = 0.0;

}

void CalibTrajReplayerRt::update_clocks()
{
    // Update timer(s)
    _loop_time += _plugin_dt;


    if(_perform_traj)
    {
        _traj_time += _plugin_dt;
    }

    if(_traj_finished)
    {
        _traj_time = _traj_execution_time;
    }

    if(_go2calib_traj)
    {
        _approach_traj_time += _plugin_dt;
    }

    if(_approach_traj_finished)
    {
        _approach_traj_time = _approach_traj_exec_time;
    }
}

void CalibTrajReplayerRt::init_vars()
{
    _q_p_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_dot_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_ddot_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_safe_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);

    _q_p_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_dot_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_ddot_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _tau_meas = Eigen::VectorXd::Zero(_n_jnts_robot);

    _iq_meas = Eigen::VectorXd::Zero(_jnt_list.size());
    _iq_meas_filt = Eigen::VectorXd::Zero(_jnt_list.size());
    _q_p_dot_meas_red = Eigen::VectorXd::Zero(_jnt_list.size());
    _q_p_dot_meas_red_filt = Eigen::VectorXd::Zero(_jnt_list.size());
    _q_p_ddot_meas_red = Eigen::VectorXd::Zero(_jnt_list.size());
    _q_p_ddot_meas_red_filt = Eigen::VectorXd::Zero(_jnt_list.size());
    _tau_meas_red  = Eigen::VectorXd::Zero(_jnt_list.size());
    _tau_meas_red_filt = Eigen::VectorXd::Zero(_jnt_list.size());

    _jnt_cal_sol_millis = Eigen::VectorXd::Zero(_jnt_list.size());
    _alpha_f0 = Eigen::VectorXd::Zero(_jnt_list.size());
    _alpha_f1 = Eigen::VectorXd::Zero(_jnt_list.size());
    _tau_friction = Eigen::VectorXd::Zero(_jnt_list.size());
    _tau_mot = Eigen::VectorXd::Zero(_jnt_list.size());
    _tau_inertial = Eigen::VectorXd::Zero(_jnt_list.size());

    _K_d0 = Eigen::VectorXd::Zero(_jnt_list.size());
    _K_d1 = Eigen::VectorXd::Zero(_jnt_list.size());
    _rot_MoI = Eigen::VectorXd::Zero(_jnt_list.size());
    _K_t = Eigen::VectorXd::Zero(_jnt_list.size());
    _red_ratio = Eigen::VectorXd::Zero(_jnt_list.size());
    _K_d0_ig = Eigen::VectorXd::Zero(_jnt_list.size());
    _K_d1_ig = Eigen::VectorXd::Zero(_jnt_list.size());
    _rot_MoI_ig = Eigen::VectorXd::Zero(_jnt_list.size());
    _K_t_ig = Eigen::VectorXd::Zero(_jnt_list.size());

    _q_p_cmd_vect = std::vector<double>(_n_jnts_robot);
    _q_p_dot_cmd_vect = std::vector<double>(_n_jnts_robot);
    _q_p_ddot_cmd_vect = std::vector<double>(_n_jnts_robot);

    _q_min =  Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_max = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_dot_lim = Eigen::VectorXd::Zero(_n_jnts_robot);

    _q_p_init_appr_traj = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_trgt_appr_traj = Eigen::VectorXd::Zero(_n_jnts_robot);

    _jnt_indxs = std::vector<int>(_jnt_list.size());
    std::fill(_jnt_indxs.begin(), _jnt_indxs.end(), -1);

    _omega0 = std::vector<double>(_jnt_list.size());
    _omegaf = std::vector<double>(_jnt_list.size());
    _t_exec_omega = std::vector<double>(_jnt_list.size());

    _phase_omega = std::vector<double>(_jnt_list.size());
    _ramp_up = std::vector<double>(_jnt_list.size());
    _omega_k = std::vector<double>(_jnt_list.size());
    _time_ref = std::vector<double>(_jnt_list.size());

    std::fill(_omega0.begin(), _omega0.end(), _omega0_s);
    std::fill(_omegaf.begin(), _omegaf.end(), _omegaf_s);
    std::fill(_t_exec_omega.begin(), _t_exec_omega.end(), _t_exec_omega_s);

    _sweep_trajs = std::vector<SweepCos>(_jnt_list.size());
    for (int i = 0; i < _jnt_list.size(); i++)
    {
        _sweep_trajs[i] = SweepCos(_omega0[i], _omegaf[i], _t_exec_omega[i],
                                   _q_lb[i], _q_ub[i],
                                   _plugin_dt);
    }

    _q_p_ddot_est_vect = std::vector<double>(_jnt_list.size());
    _q_p_dot_meas_vect = std::vector<double>(_jnt_list.size());
    _tau_meas_vect = std::vector<double>(_jnt_list.size());
    _K_t_vect = std::vector<double>(_jnt_list.size());
    _K_d0_vect = std::vector<double>(_jnt_list.size());
    _K_d1_vect = std::vector<double>(_jnt_list.size());
    _rot_MoI_vect = std::vector<double>(_jnt_list.size());
    _red_ratio_vect = std::vector<double>(_jnt_list.size());
    _alpha_f0_vect = std::vector<double>(_jnt_list.size());
    _alpha_f1_vect = std::vector<double>(_jnt_list.size());
    _iq_meas_vect = std::vector<double>(_jnt_list.size());
    _jnt_cal_sol_millis_vect = std::vector<double>(_jnt_list.size());


}

void CalibTrajReplayerRt::get_params_from_config()
{
    // Reading some parameters from XBot2 config. YAML file

    _verbose = getParamOrThrow<bool>("~verbose");

    _traj_execution_time = getParamOrThrow<double>("~traj_execution_time");

    _approach_traj_exec_time = getParamOrThrow<double>("~approach_traj_exec_time");

    _jnt_list = getParamOrThrow<std::vector<std::string>>("~jnt_list");

    _omega0_s = 2 * M_PI * getParamOrThrow<double>("~f0");
    _omegaf_s = 2 * M_PI * getParamOrThrow<double>("~ff");

    if(getParamOrThrow<double>("~f0") >= 1/_sweep_min_t_exec )
    {
        _omega0_s  = 2 * M_PI * 1/_sweep_min_t_exec;
    }
    if(getParamOrThrow<double>("~ff") >= 1/_sweep_min_t_exec )
    {
        _omegaf_s = 2 * M_PI * 1/_sweep_min_t_exec;
    }

    _t_exec_omega_s = getParamOrThrow<double>("~t_exec_f");
    _q_lb = getParamOrThrow<std::vector<double>>("~q_lb");
    _q_ub = getParamOrThrow<std::vector<double>>("~q_ub");

    _red_ratio = getParamOrThrow<Eigen::VectorXd>("~red_ratio");

    _K_t = getParamOrThrow<Eigen::VectorXd>("~K_t");
    _rot_MoI = getParamOrThrow<Eigen::VectorXd>("~rotor_MoI");
    _K_d0 = getParamOrThrow<Eigen::VectorXd>("~K_d0");
    _K_d1 = getParamOrThrow<Eigen::VectorXd>("~K_d1");

    _K_t_ig = getParamOrThrow<Eigen::VectorXd>("~K_t_ig");
    _rot_MoI_ig = getParamOrThrow<Eigen::VectorXd>("~rot_MoI_ig");
    _K_d0_ig = getParamOrThrow<Eigen::VectorXd>("~K_d0_ig");
    _K_d1_ig = getParamOrThrow<Eigen::VectorXd>("~K_d1_ig");

    _K_t_nom = getParamOrThrow<Eigen::VectorXd>("~K_t_nom");
    _rot_MoI_nom = getParamOrThrow<Eigen::VectorXd>("~rot_MoI_nom");
    _K_d0_nom = getParamOrThrow<Eigen::VectorXd>("~K_d0_nom");
    _K_d1_nom = getParamOrThrow<Eigen::VectorXd>("~K_d1_nom");

    _red_ratio = getParamOrThrow<Eigen::VectorXd>("~red_ratio");

    _cal_mask = getParamOrThrow<std::vector<bool>>("~cal_mask");
    _rot_calib_window_size = getParamOrThrow<int>("~rot_calib_window_size");
    _lambda = getParamOrThrow<Eigen::VectorXd>("~lambda");
    _alpha = getParamOrThrow<int>("~alpha");
    _q_dot_3sigma = getParamOrThrow<double>("~q_dot_3sigma");
}

void CalibTrajReplayerRt::is_sim(std::string sim_string = "sim")
{
    XBot::Context ctx;
    auto& pm = ctx.paramManager();
    _hw_type = pm.getParamOrThrow<std::string>("/xbot_internal/hal/hw_type");

    size_t sim_found = _hw_type.find(sim_string);

    if (sim_found != std::string::npos) { // we are running the plugin in simulation

        _is_sim = true;
    }
    else // we are running on the real robot
    {
        _is_sim = false;
    }

}

void CalibTrajReplayerRt::is_dummy(std::string dummy_string = "dummy")
{
    XBot::Context ctx;
    auto& pm = ctx.paramManager();
    _hw_type = pm.getParamOrThrow<std::string>("/xbot_internal/hal/hw_type");

    size_t dummy_found = _hw_type.find(dummy_string);


    if (dummy_found != std::string::npos) { // we are running the plugin in dummy mode

        _is_dummy = true;
    }
    else // we are running on the real robot
    {
        _is_dummy = false;
    }

}

void CalibTrajReplayerRt::update_state()
{    
    // "sensing" the robot
    _robot->sense();

    // Getting robot state
    _robot->getJointPosition(_q_p_meas);
    _robot->getMotorVelocity(_q_p_dot_meas);  
    _robot->getJointEffort(_tau_meas);

    // extracting only relevant info
    for (int i = 0; i < _jnt_list.size(); i++)
    {
        _q_p_dot_meas_red(i) = _q_p_dot_meas(_jnt_indxs[i]);

        _tau_meas_red(i) = _tau_meas(_jnt_indxs[i]);

    }

    // estimating joint acceleration
    _num_diff.add_sample(_q_p_dot_meas_red);
    _num_diff.dot(_q_p_ddot_meas_red);

    if(_is_sim || _is_dummy)
    {
        update_iq_estimation();

    }
    else
    { // we're on the real robot

        _iq_getter.get_last_iq_out(_iq_meas); // raw

    }

    // filtering data (all with same frequency)
    _mov_avrg_filter_q_dot.add_sample(_q_p_dot_meas_red);
    _mov_avrg_filter_q_ddot.add_sample(_q_p_ddot_meas_red);
    _mov_avrg_filter_tau.add_sample(_tau_meas_red);

    _mov_avrg_filter_q_dot.get(_q_p_dot_meas_red_filt);
    _mov_avrg_filter_q_ddot.get(_q_p_ddot_meas_red_filt);
    _mov_avrg_filter_tau.get(_tau_meas_red_filt);

    _iq_getter.get_last_iq_out_filt(_iq_meas_filt); // filtered

}

void CalibTrajReplayerRt::update_iq_estimation()
{

    _iq_estimator.set_current_state(_q_p_dot_meas_red_filt, _q_p_ddot_meas_red_filt, _tau_meas_red_filt);

    _iq_estimator.get_iq_estimate(_iq_meas);

}

void CalibTrajReplayerRt::send_cmds()
{

        if(_send_pos_ref)
        {  

            _robot->setPositionReference(_q_p_cmd);
        }

        if(_send_vel_ref)
        {  

            _robot->setVelocityReference(_q_p_dot_cmd);
        }
    
    _robot->move(); // send commands to the robot
}

void CalibTrajReplayerRt::set_approach_trajectory()
{
    _approach_traj_phase = _approach_traj_time / _approach_traj_exec_time; // phase ([0, 1] inside the approach traj

    _peisekah_utils.compute_peisekah_vect_val(_approach_traj_phase, _q_p_init_appr_traj, _q_p_trgt_appr_traj, _q_p_cmd);

    jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
           std::string("\n _approach_traj_time {} \n"), _approach_traj_time);

    jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
           std::string("\n _approach_traj_exec_time {} \n"), _approach_traj_exec_time);

}

void CalibTrajReplayerRt::set_calib_trajectory()
{

    // we only override the desired joints
    for (int i = 0; i < _jnt_list.size(); i++)
    {
        _sweep_trajs[i].eval_at(_traj_time, _q_p_cmd[_jnt_indxs[i]], _q_p_dot_cmd[_jnt_indxs[i]], _q_p_ddot_cmd[_jnt_indxs[i]]);

        _sweep_trajs[i].get_stuff(_phase_omega[i], _ramp_up[i], _omega_k[i], _time_ref[i]);


    }

}

void CalibTrajReplayerRt::set_cmds()
{ // always called in each plugin loop
  // is made of a number of phases, each signaled by suitable flags
  // remember to increase the sample index at the end of each phase,
  // if necessary

    if (_is_first_run)
    { // set cmds to safe values

        if (_verbose)
        {
            jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                       "\n (first run) \n");
        }

        _q_p_cmd = _q_p_meas;

        _q_p_dot_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);

    }

    if(_go2calib_traj)
    { // we go towards the first sample of the trajectory

        _idle = false;

        if (_approach_traj_time > _approach_traj_exec_time - 0.00001)
        {
            _go2calib_traj = false; // finished approach traj
            _approach_traj_finished = true;
            _approach_traj_started = false;

            reset_clocks();

            jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                   std::string("\n Approach trajectory finished... ready to perform calibration trajectory \n"));

        }
        else
        {// set approach traj cmds

            if (_verbose)
            {
                jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "\n (setting approach trajectory...) \n");
            }

            set_approach_trajectory();
        }
    }

    if(_perform_traj)
    { // we go towards the first sample of the trajectory

        _idle = false;

        if (_traj_time > _traj_execution_time - 0.00001)
        {
            _perform_traj = false; // finished approach traj
            _traj_finished = true;
            _traj_started = false;

            reset_clocks();

            jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                   std::string("\n Calibration trajectory finished\n"));

        }
        else
        {// set approach traj cmds

            if (_verbose)
            {
                jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "\n (setting calibration trajectory...) \n");
            }

            set_calib_trajectory();
        }
    }

}

void CalibTrajReplayerRt::init_dump_logger()
{

    // // Initializing logger for debugging
    MatLogger2::Options opt;
    opt.default_buffer_size = _matlogger_buffer_size; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression

    _dump_logger = MatLogger2::MakeLogger(_dump_path, opt); // date-time automatically appended

    _dump_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    _dump_logger->add("plugin_dt", _plugin_dt);
    _dump_logger->add("is_sim", int(_is_sim));
    
    _dump_logger->add("send_pos_ref", int(_send_pos_ref));
    _dump_logger->add("send_vel_ref", int(_send_vel_ref));
    _dump_logger->add("send_eff_ref", int(_send_eff_ref));

    _dump_logger->add("traj_execution_time", _traj_execution_time);

    _dump_logger->create("loop_time", 1);
    _dump_logger->create("traj_time", 1, 1, _matlogger_buffer_size);

    _dump_logger->create("q_p_meas", _n_jnts_robot), 1, _matlogger_buffer_size;
    _dump_logger->create("q_p_dot_meas", _n_jnts_robot, 1, _matlogger_buffer_size);

    _dump_logger->create("q_p_cmd", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("q_p_dot_cmd", _n_jnts_robot, 1, _matlogger_buffer_size);

}

void CalibTrajReplayerRt::add_data2dump_logger()
{

    _dump_logger->add("loop_time", _loop_time);
    _dump_logger->add("traj_time", _traj_time);

    _dump_logger->add("q_p_meas", _q_p_meas);
    _dump_logger->add("q_p_dot_meas", _q_p_dot_meas);

    _dump_logger->add("q_p_cmd", _q_p_cmd);
    _dump_logger->add("q_p_dot_cmd", _q_p_dot_cmd);

    _dump_logger->add("plugin_time", _loop_time);




}

void CalibTrajReplayerRt::add_data2bedumped()
{
    
    add_data2dump_logger();
    
}

void CalibTrajReplayerRt::init_ros_bridge()
{    
    ros::NodeHandle nh(getName());

    _ros = std::make_unique<RosSupport>(nh);

    /* Service servers */
    _perform_traj_srvr = _ros->advertiseService(
        "perform_calib_traj",
        &CalibTrajReplayerRt::on_perform_traj_received,
        this,
        &_queue);

    _set_cal_srvr = _ros->advertiseService("set_jnt_cal",
                                           &CalibTrajReplayerRt::on_jnt_cal_received,
                                           this,
                                           &_queue);
    /* Publishers */
    moving_horizon_jnt_calib::CalibTrajStatus replay_st_prealloc;
    _traj_status_pub = _ros->advertise<moving_horizon_jnt_calib::CalibTrajStatus>(
        "calib_traj_status", 1, replay_st_prealloc);

    moving_horizon_jnt_calib::JntCalibStatus jnt_cal_st_prealloc;
    _jnt_calib_pub = _ros->advertise<moving_horizon_jnt_calib::JntCalibStatus>(
        "jnt_calib_status", 1, jnt_cal_st_prealloc);

    /* Subscribers */
    _aux_signals_sub = _ros->subscribe("/xbotcore/aux",
                                &Xbot2Utils::IqOutRosGetter::on_aux_signal_received,
                                &_iq_getter,
                                1,  // queue size
                                &_queue);

}

void CalibTrajReplayerRt::saturate_cmds()
{

    _robot->enforceJointLimits(_q_p_cmd);
    _robot->enforceVelocityLimit(_q_p_dot_cmd);

}

void CalibTrajReplayerRt::pub_replay_status()
{
    auto status_msg = _traj_status_pub->loanMessage();

    status_msg->msg().approach_traj_started = _approach_traj_started;
    status_msg->msg().traj_started = _traj_started;

    status_msg->msg().approach_traj_finished = _approach_traj_finished;
    status_msg->msg().traj_finished = _traj_finished;

    status_msg->msg().send_pos = _send_pos_ref;
    status_msg->msg().send_vel = _send_vel_ref;

    status_msg->msg().performed_traj_n = _performed_traj_n;

    status_msg->msg().phase_omega = _phase_omega;
    status_msg->msg().ramp_up = _ramp_up;
    status_msg->msg().omega_k = _omega_k;
    status_msg->msg().time_ref = _time_ref;

    for(int i = 0; i < _n_jnts_robot; i++)
    {
        _q_p_cmd_vect[i] = _q_p_cmd(i);
        _q_p_dot_cmd_vect[i] = _q_p_dot_cmd(i);
        _q_p_ddot_cmd_vect[i] = _q_p_ddot_cmd(i);

    }

    status_msg->msg().pos_ref = _q_p_cmd_vect;
    status_msg->msg().vel_ref = _q_p_dot_cmd_vect;
    status_msg->msg().acc_ref = _q_p_ddot_cmd_vect;

    _traj_status_pub->publishLoaned(std::move(status_msg));
}

void CalibTrajReplayerRt::pub_calib_status()
{
    auto status_msg = _jnt_calib_pub->loanMessage();

    status_msg->msg().jnt_names = _enbld_jnt_names;

    for(int i = 0; i < _jnt_list.size(); i++)
    {
        _iq_meas_vect[i] = _iq_meas(i);
        _q_p_dot_meas_vect[i] = _q_p_dot_meas_red_filt(i);
        _q_p_ddot_est_vect[i] = _q_p_ddot_meas_red_filt(i);
        _tau_meas_vect[i] = _tau_meas_red_filt(i);
        _alpha_f0_vect[i] = _alpha_f0[i];
        _alpha_f1_vect[i] = _alpha_f1[i];
        _K_d0_vect[i] = _K_d0[i];
        _K_d1_vect[i] = _K_d1[i];
        _K_t_vect[i] = _K_t[i];
        _rot_MoI_vect[i] = _rot_MoI[i];
        _jnt_cal_sol_millis_vect[i] = _jnt_cal_sol_millis(i);

    }

    status_msg->msg().iq = _iq_meas_vect;
    status_msg->msg().q_dot = _q_p_dot_meas_vect;
    status_msg->msg().q_ddot = _q_p_ddot_est_vect;
    status_msg->msg().alpha_f0 = _alpha_f0_vect;
    status_msg->msg().alpha_f1 = _alpha_f1_vect;
    status_msg->msg().K_d0_cal = _K_d0_vect;
    status_msg->msg().K_d1_cal = _K_d1_vect;
    status_msg->msg().K_t_cal = _K_t_vect;
    status_msg->msg().rotor_MoI_cal = _rot_MoI_vect;
    status_msg->msg().red_ratio = _red_ratio_vect;

    status_msg->msg().sol_millis = _jnt_cal_sol_millis_vect;


    _jnt_calib_pub->publishLoaned(std::move(status_msg));
}

bool CalibTrajReplayerRt::on_perform_traj_received(const moving_horizon_jnt_calib::PerformCalibTrajRequest& req,
                              moving_horizon_jnt_calib::PerformCalibTrajResponse& res)
{

    bool approach_state_changed = _go2calib_traj != req.go2calib_traj;
    bool cal_traj_state_changed = _perform_traj != req.perform_calib_traj;

    bool success = false;

    if((approach_state_changed && !cal_traj_state_changed) || (!approach_state_changed && cal_traj_state_changed))
    { // we can either ramp towards the initial state of the calibration trajectory or perform the calib. trajectory
      // (not both at the same time)

        if(approach_state_changed)
        {
            _go2calib_traj = req.go2calib_traj;
            _perform_traj = false;

        }

        if(cal_traj_state_changed)
        {
            _go2calib_traj = false;
            _perform_traj = req.perform_calib_traj;

        }

        success = true;
    }
    else
    {

        success = false;
    }

    if(_verbose)
    {
        jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                      "CalibTrajReplayerRt: received PerformCalibTrajRequest: -->\n");
        jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                      "perform_calib_traj: {}\n", _go2calib_traj);
        jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                      "go2calib_traj: {}\n", _perform_traj);
    }

    if(success && _go2calib_traj)
    {
        reset_clocks();

        _q_p_init_appr_traj = _q_p_cmd;
        _q_p_trgt_appr_traj = _q_p_cmd;

        // we only override the desired joints
        for (int i = 0; i < _jnt_list.size(); i++)
        {
            _sweep_trajs[i].eval_at(_approach_traj_time, _q_p_trgt_appr_traj[_jnt_indxs[i]], _q_dot_temp, _q_ddot_temp);

        }

        _q_p_dot_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);


        _approach_traj_started = true;

        _approach_traj_finished = false;
    }

    if(success && _perform_traj)
    { // only if the approach traj. was already executed
        reset_clocks();

        _traj_started = true;

        _traj_finished = false;
    }

    res.success = success;

    return success;

}

bool CalibTrajReplayerRt::on_jnt_cal_received(const moving_horizon_jnt_calib::JntCalibRtRequest& req,
                              moving_horizon_jnt_calib::JntCalibRtResponse& res)
{

    res.success = true;

    return res.success;

}


void CalibTrajReplayerRt::run_jnt_calib()
{

}

void CalibTrajReplayerRt::apply_calib_mask()
{
    _rot_dyn_calib.set_solution_mask(_cal_mask);

    if(_cal_mask[0] = false)
    { // Kt
        _rot_dyn_calib.set_ig_Kt(_K_t_nom);
    }
    else
    {
        _rot_dyn_calib.set_ig_Kt(_K_t_ig);
    }

    if(_cal_mask[1] = false)
    { // rot_MoI
        _rot_dyn_calib.set_ig_MoI(_rot_MoI_nom);
    }
    else
    {
        _rot_dyn_calib.set_ig_MoI(_rot_MoI_ig);
    }
    if(_cal_mask[2] = false)
    { // Kd0
        _rot_dyn_calib.set_ig_Kd0(_K_d0_nom);
    }
    else
    {
        _rot_dyn_calib.set_ig_Kd0(_K_d0_ig);
    }

    if(_cal_mask[3] = false)
    { // Kd1
        _rot_dyn_calib.set_ig_Kd1(_K_d1_nom);
    }
    else
    {
        _rot_dyn_calib.set_ig_Kd1(_K_d1_ig);

    }

}

bool CalibTrajReplayerRt::on_initialize()
{ 
    std::string sim_flagname = "sim";
    is_sim(sim_flagname); // see if we are running a simulation

    std::string dummy_flagname = "dummy";
    is_dummy(dummy_flagname); // see if we are running in dummy mode

    get_params_from_config(); // load params from yaml file

    _plugin_dt = getPeriodSec();

    _n_jnts_robot = _robot->getJointNum();
    _robot->getVelocityLimits(_q_dot_lim);
    _robot->getJointLimits(_q_min, _q_max);
    _enbld_jnt_names = _robot->getEnabledJointNames();

    init_vars();

    init_ros_bridge();

    jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                          "\n CalibTrajReplayerRt::on_initialize(): enabled joints list --> \n");

    for(int i = 0; i < _enbld_jnt_names.size(); i ++)
    {
        jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                              "{}\n", _enbld_jnt_names[i]);
    }


    for(int i = 0; i < _jnt_list.size(); i ++)
    {
        for(int j = 0; j < _enbld_jnt_names.size(); j++)
        {
            if(_jnt_list[i] == _enbld_jnt_names[j])
            {// joint found

                _jnt_indxs[i] = j;

                 break; // we have the mapping for this joint --> let's exit this loop
            }

        }


    } // non-existent joints will have an index of -1 --> we throw an error in case we provided a non-valid
      // joint name

    jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                          "\n CalibTrajReplayerRt::on_initialize(): joint which will be controlled --> \n");

    for(int i = 0; i < _jnt_list.size(); i ++)
    {
        jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                              "{}, ID: {}\n", _jnt_list[i], _jnt_indxs[i]);
    }

    bool all_jnts_valid = !(std::count(_jnt_indxs.begin(), _jnt_indxs.end(), -1));

    if(!all_jnts_valid)
    {
        std::string exception = std::string("Not all joint names in jnt_list are valid. Check them!!!\n");

        throw std::invalid_argument(exception);
    }

    if(_q_lb.size() != _q_ub.size())
    {
        std::string exception = std::string("Mismatching dimensions in _q_lb({}) and _q_ub({})!!!\n", _q_lb.size(), _q_ub.size());

        throw std::invalid_argument(exception);
    }
    if(_q_lb.size() != _jnt_list.size())
    {
        std::string exception = std::string("Mismatching dimensions of _q_lb and _q_ub ({}) with _jnt_list!!!\n", _q_lb.size(), _jnt_list.size());

        throw std::invalid_argument(exception);
    }

    // initializing calibration-related stuff

    _iq_getter = IqOutRosGetter(_jnt_list, _plugin_dt, _mov_avrg_cutoff_freq); // getter for quadrature current measurements from ros topic

    //filter for tau_meas
    _mov_avrg_filter_tau = MovAvrgFilt(_jnt_list.size(), _plugin_dt, _mov_avrg_cutoff_freq);
    _mov_avrg_filter_tau.get_window_size(_mov_avrg_window_size); // get computed window size

    //filter for q_dot
    _mov_avrg_filter_q_dot = MovAvrgFilt(_jnt_list.size(), _plugin_dt, _mov_avrg_cutoff_freq);
    _mov_avrg_filter_tau.get_window_size(_mov_avrg_window_size); // get computed window size

    //filter for q_ddot
    _mov_avrg_filter_q_ddot = MovAvrgFilt(_jnt_list.size(), _plugin_dt, _mov_avrg_cutoff_freq);
    _mov_avrg_filter_q_ddot.get_window_size(_mov_avrg_window_size); // get computed window size

    // numerical differentiation
    _num_diff = NumDiff(_jnt_list.size(), _plugin_dt, 1);

    // actuator dynamics calibration

    _rot_dyn_calib = RotDynCal(_rot_calib_window_size,
                                _red_ratio,
                                _K_t_ig,
                                _rot_MoI_ig,
                                _K_d0_ig,
                                _K_d1_ig,
                                _lambda,
                                _alpha,
                                _q_dot_3sigma,
                                _verbose);

    apply_calib_mask();
    _rot_dyn_calib.set_lambda(_lambda);
    _rot_dyn_calib.add_sample(_q_p_dot_meas_red_filt,
                              _q_p_ddot_meas_red_filt,
                              _iq_meas_filt,
                              _tau_meas_red_filt);

    _rot_dyn_calib.solve();

    _rot_dyn_calib.get_sol_millis(_jnt_cal_sol_millis);
    _rot_dyn_calib.get_opt_Kd0(_K_d0);
    _rot_dyn_calib.get_opt_Kd1(_K_d1);
    _rot_dyn_calib.get_opt_Kt(_K_t);
    _rot_dyn_calib.get_opt_rot_MoI(_rot_MoI);

    _rot_dyn_calib.get_tau_friction(_tau_friction);
    _rot_dyn_calib.get_alpha(_alpha_f0, _alpha_f1);
    _rot_dyn_calib.get_tau_motor(_tau_mot);
    _rot_dyn_calib.get_tau_inertial(_tau_inertial);

    // iq estimator, just to check calibration in simulation

    _iq_estimator = IqEstimator(_K_t_nom,
                                _K_d0_nom, _K_d1_nom,
                                _rot_MoI_nom,
                                _red_ratio,
                                _alpha,
                                _q_dot_3sigma); // object to compute iq estimate
    return true;
    
}

void CalibTrajReplayerRt::starting()
{

    init_dump_logger();

    init_clocks(); // initialize clocks timers

    // we let all types of commands pass
    _robot->setControlMode(ControlMode::Position() + ControlMode::Velocity() + ControlMode::Effort() + ControlMode::Stiffness() +
            ControlMode::Damping());

    // Move on to run()
    start_completed();
    
}

void CalibTrajReplayerRt::run()
{  

    _idle = true;

    update_state(); // update all necessary states

    _queue.run(); // process server callbacks and update triggers' states, if necessary

    set_cmds(); // compute and set command references

    if(!_idle)
    {
        saturate_cmds();

        send_cmds(); // send commands to the robot
    }


    pub_replay_status(); // publishes info from the plugin to ros and internal xbot2 topics

    pub_calib_status(); // publishes calibration status

    add_data2dump_logger(); // add data for debugging purposes

    update_clocks(); // last, update the clocks (loop + any additional one). Each clock is incremented by a plugin dt

    if (_is_first_run)
    { // next control loops are aware that it is not the first control loop
        _is_first_run = !_is_first_run;
    }

}

void CalibTrajReplayerRt::on_stop()
{

    _is_first_run = true;

    init_clocks();

    _dump_logger.reset();
}

void CalibTrajReplayerRt::stopping()
{

    stop_completed();

}

void CalibTrajReplayerRt::on_abort()
{
    // Destroy logger and dump .mat file
    _dump_logger.reset();
}

void CalibTrajReplayerRt::on_close()
{
    jinfo("Closing CalibTrajReplayerRt");
}

XBOT2_REGISTER_PLUGIN(CalibTrajReplayerRt, calib_traj_replayer_rt)

