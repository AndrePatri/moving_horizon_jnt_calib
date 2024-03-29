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

}

void CalibTrajReplayerRt::get_params_from_config()
{
    // Reading some parameters from XBot2 config. YAML file

    _verbose = getParamOrThrow<bool>("~verbose");

    _traj_execution_time = getParamOrThrow<double>("~traj_execution_time");

    _approach_traj_exec_time = getParamOrThrow<double>("~approach_traj_exec_time");

    _jnt_cal_yaml = getParamOrThrow<YAML::Node>("~jnt_cal_yaml/content");

    _jnt_list = _jnt_cal_yaml["jnt_list"].as<std::vector<std::string>>();

    _omega0_s = 2 * M_PI * _jnt_cal_yaml["f0"].as<double>();
    _omegaf_s = 2 * M_PI * _jnt_cal_yaml["ff"].as<double>();

    if(_jnt_cal_yaml["f0"].as<double>() >= 1/_sweep_min_t_exec )
    {
        _omega0_s  = 2 * M_PI * 1/_sweep_min_t_exec;
    }
    if(_jnt_cal_yaml["ff"].as<double>() >= 1/_sweep_min_t_exec )
    {
        _omegaf_s = 2 * M_PI * 1/_sweep_min_t_exec;
    }

    _t_exec_omega_s = _jnt_cal_yaml["t_exec_f"].as<double>();
    _q_lb = _jnt_cal_yaml["q_lb"].as<std::vector<double>>();
    _q_ub = _jnt_cal_yaml["q_ub"].as<std::vector<double>>();

    param_dims_ok_or_throw();
}

void CalibTrajReplayerRt::param_dims_ok_or_throw()
{

    if((_q_ub.size() != _q_lb.size()) ||
       (_q_lb.size() != _jnt_list.size()))
    {
        std::string exception = std::string("\nCalibTrajReplayerRt::param_dims_ok_or_throw(): dimension mismatch in YAML parameters. Check the XBot2 config file for errors! "
                                            "All dimensions should be coherent with the provided calibration jnt_list (dim: ") +
                                std::to_string(_jnt_list.size()) + std::string(").\n");

        throw std::invalid_argument(exception);
    }
}

void CalibTrajReplayerRt::is_sim(std::string sim_string = "sim")
{
    XBot::Context ctx;
    auto& pm = ctx.paramManager();
    _hw_type = pm.getParamOrThrow<std::string>("/xbot/hal/hw_type");

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
    _hw_type = pm.getParamOrThrow<std::string>("/xbot/hal/hw_type");

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
    _robot->getPositionReference(_q_p_meas);

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

    if(_verbose)
    {
        jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
               std::string("\n _approach_traj_time {} \n"), _approach_traj_time);

        jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
               std::string("\n _approach_traj_exec_time {} \n"), _approach_traj_exec_time);
    }

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

    _dump_logger->create("loop_time", 1, 1, _matlogger_buffer_size);
    _dump_logger->create("traj_time", 1, 1, _matlogger_buffer_size);


}

void CalibTrajReplayerRt::add_data2dump_logger()
{


    _dump_logger->add("loop_time", _loop_time);
    _dump_logger->add("traj_time", _traj_time);

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


    /* Publishers */
    moving_horizon_jnt_calib::CalibTrajStatus replay_st_prealloc;
    _traj_status_pub = _ros->advertise<moving_horizon_jnt_calib::CalibTrajStatus>(
        "calib_traj_status", 1, replay_st_prealloc);


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

        if(_verbose)
        {
            jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                          "CalibTrajReplayerRt: starting approach trajectory\n");
        }

    }

    if(success && _perform_traj)
    { // only if the approach traj. was already executed
        reset_clocks();

        _traj_started = true;

        _traj_finished = false;

        for (int i = 0; i < _jnt_list.size(); i++)
        {
            _sweep_trajs[i].reset();
        }
    }

    if(success && !_perform_traj)
    { // when stopping the traj, we reset the flags
        reset_clocks();

        _traj_started = false;

        _traj_finished = false;

        _approach_traj_started = false;

        _approach_traj_finished = false;
    }

    res.success = success;

    return success;

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
                          "\n CalibTrajReplayerRt::on_initialize(): joint to be controlled --> \n");

    for(int i = 0; i < _jnt_list.size(); i ++)
    {
        jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                              "{}, ID: {}\n", _jnt_list[i], _jnt_indxs[i]);
    }

    bool all_jnts_valid = !(std::count(_jnt_indxs.begin(), _jnt_indxs.end(), -1));

    if(!all_jnts_valid)
    {
        std::string exception = std::string("CalibTrajReplayerRt::on_initialize(): Not all joint names in jnt_list are valid. Check them!!!\n");

        throw std::invalid_argument(exception);
    }

    if(_q_lb.size() != _q_ub.size())
    {
        std::string exception = std::string("CalibTrajReplayerRt::on_initialize(): Mismatching dimensions in _q_lb({}) and _q_ub({})!!!\n", _q_lb.size(), _q_ub.size());

        throw std::invalid_argument(exception);
    }
    if(_q_lb.size() != _jnt_list.size())
    {
        std::string exception = std::string("CalibTrajReplayerRt::on_initialize(): Mismatching dimensions of _q_lb and _q_ub ({}) with _jnt_list!!!\n", _q_lb.size(), _jnt_list.size());

        throw std::invalid_argument(exception);
    }

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

