#include "calib_traj_replayer_rt.h"

#include <math.h> 

void CalibTrajReplayerRt::init_clocks()
{
    _loop_time = 0.0; // reset loop time clock

    _traj_time = 0.0;

}

void CalibTrajReplayerRt::reset_clocks()
{

    _traj_time = 0.0;

}

void CalibTrajReplayerRt::update_clocks()
{
    // Update timer(s)
    _loop_time += _plugin_dt;


    if(!_perform_traj)
    {
        _traj_time += _plugin_dt;
    }

    if(_traj_finished)
    {
        _traj_time = _traj_execution_time;
    }
}

void CalibTrajReplayerRt::init_vars()
{
    _q_p_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_dot_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);

    _q_p_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_dot_meas = Eigen::VectorXd::Zero(_n_jnts_robot);

    _q_p_safe_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);

    _tau_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);

    _q_p_cmd_vect = std::vector<double>(_n_jnts_robot);
    _q_p_dot_cmd_vect = std::vector<double>(_n_jnts_robot);
    _tau_cmd_vect = std::vector<double>(_n_jnts_robot);

}

void CalibTrajReplayerRt::get_params_from_config()
{
    // Reading some parameters from XBot2 config. YAML file

    _verbose = getParamOrThrow<bool>("~verbose");

    _traj_execution_time = getParamOrThrow<double>("~traj_execution_time");

    _cal_jnt_names = getParamOrThrow<std::vector<std::string>>("~jnt_list");

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

        if (_send_eff_ref)
        {
            _robot->setEffortReference(_tau_cmd);
        }

    }
    
    _robot->move(); // send commands to the robot
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
    _dump_logger->create("tau_cmd", _n_jnts_robot, 1, _matlogger_buffer_size);

}

void CalibTrajReplayerRt::add_data2dump_logger()
{


    _dump_logger->add("q_p_cmd", _q_p_cmd);
    _dump_logger->add("q_p_dot_cmd", _q_p_dot_cmd);
    _dump_logger->add("tau_cmd", _tau_cmd);

    _dump_logger->add("plugin_time", _loop_time);

    _dump_logger->add("q_p_meas", _q_p_meas);
    _dump_logger->add("q_p_dot_meas", _q_p_dot_meas);

    _dump_logger->add("loop_time", _loop_time);

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
    _go2takeoff_config_srvr = _ros->advertiseService(
        "perform_calib_traj",
        &CalibTrajReplayerRt::on_perform_traj_received,
        this,
        &_queue);

    /* Publishers */
    _traj_status_pub = concert_jnt_calib::JntCalibStatus calib_status_;
    _replay_status_pub = _ros->advertise<concert_jnt_calib::CalibTrajStatus>(
        "calib_traj_status", 1, replay_st_prealloc);


}
void CalibTrajReplayerRt::saturate_cmds()
{

    _robot->enforceJointLimits(_q_p_cmd);
    _robot->enforceVelocityLimit(_q_p_dot_cmd);
    _robot->enforceEffortLimit(_tau_cmd);

}

void CalibTrajReplayerRt::set_cmds()
{ // always called in each plugin loop
  // is made of a number of phases, each signaled by suitable flags
  // remember to increase the sample index at the end of each phase, 
  // if necessary

    if (_is_first_run)
    { // set impedance vals and pos ref to safe values at first plugin loop

        if (_verbose)
        {
            jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                       "\n (first run) \n");
        }

        _q_p_cmd = _q_p_meas;

    }

}

void CalibTrajReplayerRt::pub_replay_status()
{

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
    _robot->getVelocityLimits(_jnt_vel_limits);

    init_vars();

    init_ros_bridge();

    load_opt_data(); // load trajectory from file (to be placed here in starting because otherwise
    // a seg fault will arise)

    _peisekah_utils = PeisekahTrans();

    _q_p_trgt_appr_traj = _q_p_ref.block(1, 0, _n_jnts_robot, 1); // target pos. for the approach traj

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

    // Destroy logger and dump .mat file (will be recreated upon plugin restart)
    _dump_logger->add("performed_jumps", _performed_jumps);
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

