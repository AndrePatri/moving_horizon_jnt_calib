#include "jnt_cal_rt.h"

#include <math.h> 

void JntCalRt::init_clocks()
{
    _loop_time = 0.0; // reset loop time clock
}

void JntCalRt::init_vars()
{

    _q_p_meas = Eigen::VectorXd::Zero(_n_jnts_robot);

    _q_p_dot_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_dot_meas_filt = Eigen::VectorXd::Zero(_n_jnts_robot);

    _tau_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _tau_meas_filt = Eigen::VectorXd::Zero(_n_jnts_robot);

    _iq_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _iq_meas_filt = Eigen::VectorXd::Zero(_n_jnts_robot);

    _q_p_ddot_est = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_ddot_est_filt = Eigen::VectorXd::Zero(_n_jnts_robot);

    _iq_est = Eigen::VectorXd::Zero(_n_jnts_robot);
    _iq_jnt_names = std::vector<std::string>(_n_jnts_robot);

    _iq_friction_torque = Eigen::VectorXd::Zero(_n_jnts_robot);

    _iq_friction_torque_cal = Eigen::VectorXd::Zero(_n_jnts_robot);
    _tau_rot_est = Eigen::VectorXd::Zero(_n_jnts_robot);

    _alpha_f0 = Eigen::VectorXd::Zero(_n_jnts_robot);
    _alpha_f1 = Eigen::VectorXd::Zero(_n_jnts_robot);
    _K_d0_cal = Eigen::VectorXd::Zero(_n_jnts_robot);
    _K_d1_cal = Eigen::VectorXd::Zero(_n_jnts_robot);
    _K_d0 = Eigen::VectorXd::Zero(_n_jnts_robot);
    _K_d1 = Eigen::VectorXd::Zero(_n_jnts_robot);

    _iq_cal_sol_millis = Eigen::VectorXd::Zero(_n_jnts_robot);

    // used to convert to ros messages-compatible types
    _iq_est_vect = std::vector<double>(_n_jnts_robot);
    _q_p_ddot_est_vect = std::vector<double>(_n_jnts_robot);
    _q_p_ddot_est_filt_vect = std::vector<double>(_n_jnts_robot);
    _q_p_dot_meas_vect = std::vector<double>(_n_jnts_robot);
    _q_p_dot_meas_filt_vect = std::vector<double>(_n_jnts_robot);
    _tau_meas_vect = std::vector<double>(_n_jnts_robot);
    _K_t_vect = std::vector<double>(_n_jnts_robot);
    _K_d0_vect = std::vector<double>(_n_jnts_robot);
    _K_d1_vect = std::vector<double>(_n_jnts_robot);
    _rot_MoI_vect = std::vector<double>(_n_jnts_robot);
    _red_ratio_vect = std::vector<double>(_n_jnts_robot);
    _iq_friction_torque_vect = std::vector<double>(_n_jnts_robot);
    _tau_meas_filt_vect = std::vector<double>(_n_jnts_robot);
    _alpha_f0_vect = std::vector<double>(_n_jnts_robot);
    _alpha_f1_vect = std::vector<double>(_n_jnts_robot);
    _K_d0_cal_vect = std::vector<double>(_n_jnts_robot);
    _K_d1_cal_vect = std::vector<double>(_n_jnts_robot);
    _iq_meas_vect = std::vector<double>(_n_jnts_robot);
    _iq_meas_filt_vect = std::vector<double>(_n_jnts_robot);

    _iq_friction_torque_cal_vect = std::vector<double>(_n_jnts_robot);
    _tau_rot_est_vect = std::vector<double>(_n_jnts_robot);

    _iq_cal_sol_millis_vect = std::vector<double>(_n_jnts_robot);
}

void JntCalRt::reset_flags()
{

    _sample_index = 0.0; // resetting samples index, in case the plugin stopped and started again


}

void JntCalRt::update_clocks()
{
    // Update timer(s)
    _loop_time += _plugin_dt;

    // Reset timers, if necessary
    if (_loop_time >= _loop_timer_reset_time)
    {
        _loop_time = _loop_time - _loop_timer_reset_time;
    }
    
}

void JntCalRt::get_params_from_config()
{
    // Reading some parameters from XBot2 config. YAML file

    _urdf_path = getParamOrThrow<std::string>("~urdf_path"); 

    _mat_path = getParamOrThrow<std::string>("~mat_path"); 
    _dump_mat_suffix = getParamOrThrow<std::string>("~dump_mat_suffix"); 
    _matlogger_buffer_size = getParamOrThrow<double>("~matlogger_buffer_size");

    _verbose = getParamOrThrow<bool>("~verbose");

    _red_ratio = getParamOrThrow<Eigen::VectorXd>("~red_ratio");
    _K_t = getParamOrThrow<Eigen::VectorXd>("~K_t");
    _K_d0_ig = getParamOrThrow<Eigen::VectorXd>("~K_d0_ig");
    _K_d1_ig = getParamOrThrow<Eigen::VectorXd>("~K_d1_ig");
    _rot_MoI = getParamOrThrow<Eigen::VectorXd>("~rotor_axial_MoI");

    _der_est_order = getParamOrThrow<int>("~der_est_order");
    _mov_avrg_cutoff_freq_iq = getParamOrThrow<double>("~mov_avrg_cutoff_freq_iq");
    _mov_avrg_cutoff_freq_tau = getParamOrThrow<double>("~mov_avrg_cutoff_freq_tau");
    _mov_avrg_cutoff_freq_iq_meas = getParamOrThrow<double>("~mov_avrg_cutoff_freq_iq_meas");
    _mov_avrg_cutoff_freq_q_dot = getParamOrThrow<double>("~mov_avrg_cutoff_freq_q_dot");

    _iq_calib_window_size = getParamOrThrow<int>("~iq_calib_window_size");

    _lambda_qp_reg = getParamOrThrow<double>("~lambda_qp_reg");
    _alpha = getParamOrThrow<int>("~alpha");

    _q_dot_3sigma = getParamOrThrow<double>("~q_dot_3sigma");

}

void JntCalRt::is_sim(std::string sim_string = "sim")
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

void JntCalRt::is_dummy(std::string dummy_string = "dummy")
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

void JntCalRt::update_state()
{    
    // "sensing" the robot
    _robot->sense();

    // Getting robot state
    _robot->getJointPosition(_q_p_meas);
    _robot->getMotorVelocity(_q_p_dot_meas);  
    _robot->getJointEffort(_tau_meas);

    // Getting q_p_ddot via numerical differentiation
    _num_diff.add_sample(_q_p_dot_meas);
    _num_diff.dot(_q_p_ddot_est);

    // Removing noise from q_p_dot (mostly used for introducing a uniform lag
    // w.r.t. other data)
    _mov_avrg_filter_q_dot.add_sample(_q_p_dot_meas);
    _mov_avrg_filter_q_dot.get(_q_p_dot_meas_filt);

    // Removing noise from q_p_ddot
    _mov_avrg_filter_iq.add_sample(_q_p_ddot_est);
    _mov_avrg_filter_iq.get(_q_p_ddot_est_filt);

    // Removing noise from tau_meas
    _mov_avrg_filter_tau.add_sample(_tau_meas);
    _mov_avrg_filter_tau.get(_tau_meas_filt);

    //getting iq measure
    _iq_getter.get_last_iq_out(_iq_meas);

    // Removing noise from iq_meas
    _mov_avrg_filter_iq_meas.add_sample(_iq_meas);
    _mov_avrg_filter_iq_meas.get(_iq_meas_filt);

}

void JntCalRt::init_dump_logger()
{

    // // Initializing logger for debugging
    MatLogger2::Options opt;
    opt.default_buffer_size = _matlogger_buffer_size; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    if (_dump_mat_suffix == std::string(""))
    { // if empty, then dump to tmp with plugin name
        _dump_logger = MatLogger2::MakeLogger("/tmp/JntCalRt", opt); // date-time automatically appended
    }
    else
    {
        if (!_is_sim)
        {
            _dump_logger = MatLogger2::MakeLogger(_mat_path + std::string("test_") + _dump_mat_suffix, opt); // date-time automatically appended
        }
        else
        {
            _dump_logger = MatLogger2::MakeLogger(_mat_path + std::string("sim_") + _dump_mat_suffix, opt); // date-time automatically appended
        }
        
    }

    _dump_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    _dump_logger->add("plugin_dt", _plugin_dt);
    _dump_logger->add("is_sim", int(_is_sim));

    _dump_logger->create("sol_milliseconds", _n_jnts_robot, 1, _matlogger_buffer_size);

    _dump_logger->create("q_p_meas", _n_jnts_robot, 1, _matlogger_buffer_size);

    _dump_logger->create("q_p_dot_meas", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("q_p_dot_meas_filt", _n_jnts_robot, 1, _matlogger_buffer_size);

    _dump_logger->create("q_p_ddot_est", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->add("der_est_order", _der_est_order);

    _dump_logger->create("q_p_ddot_est_filt", _n_jnts_robot, 1, _matlogger_buffer_size);

    _dump_logger->create("tau_meas", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("tau_meas_filt", _n_jnts_robot, 1, _matlogger_buffer_size);

    _dump_logger->create("iq_meas", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("iq_meas_filt", _n_jnts_robot, 1, _matlogger_buffer_size);

    _dump_logger->create("iq_est", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("iq_friction_torque", _n_jnts_robot, 1, _matlogger_buffer_size);

    _dump_logger->create("K_t", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("K_d0", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("K_d1", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("rot_MoI", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("red_ratio", _n_jnts_robot, 1, _matlogger_buffer_size);

    _dump_logger->create("tau_total_rot", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("iq_friction_torque_cal", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("tau_rot_est", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("alpha_f0", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("alpha_f1", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("K_d0_cal", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("K_d1_cal", _n_jnts_robot, 1, _matlogger_buffer_size);

    _dump_logger->add("mov_avrg_window_size_iq", _mov_avrg_window_size_iq);
    _dump_logger->add("mov_avrg_cutoff_freq_iq", _mov_avrg_cutoff_freq_iq);

    _dump_logger->add("mov_avrg_window_size_tau", _mov_avrg_window_size_iq);
    _dump_logger->add("mov_avrg_cutoff_freq_tau", _mov_avrg_cutoff_freq_iq);

}

void JntCalRt::add_data2dump_logger()
{

    _dump_logger->add("sol_milliseconds", _iq_cal_sol_millis);

    _dump_logger->add("q_p_meas", _q_p_meas);

    _dump_logger->add("q_p_dot_meas", _q_p_dot_meas);
    _dump_logger->add("q_p_dot_meas_filt", _q_p_dot_meas_filt);

    _dump_logger->add("tau_meas", _tau_meas);
    _dump_logger->add("tau_meas_filt", _tau_meas_filt);

    _dump_logger->add("q_p_ddot_est", _q_p_ddot_est);
    _dump_logger->add("q_p_ddot_est_filt", _q_p_ddot_est_filt);

    _dump_logger->add("iq_meas", _iq_meas);
    _dump_logger->add("iq_meas_filt", _iq_meas_filt);

    _dump_logger->add("iq_est", _iq_est);
    _dump_logger->add("iq_friction_torque", _iq_friction_torque);

    _dump_logger->add("K_t", _K_t);
    _dump_logger->add("K_d0", _K_d0);
    _dump_logger->add("K_d1", _K_d1);
    _dump_logger->add("rot_MoI", _rot_MoI);
    _dump_logger->add("red_ratio", _red_ratio);

    _dump_logger->add("iq_friction_torque_cal", _iq_friction_torque_cal);
    _dump_logger->add("tau_rot_est", _tau_rot_est);
    _dump_logger->add("alpha_f0", _alpha_f0);
    _dump_logger->add("alpha_f1", _alpha_f1);
    _dump_logger->add("K_d0_cal", _K_d0_cal);
    _dump_logger->add("K_d1_cal", _K_d1_cal);

}

void JntCalRt::init_nrt_ros_bridge()
{
    ros::NodeHandle nh(getName());

    _ros = std::make_unique<RosSupport>(nh);

    /* Subscribers */
    _aux_signals_sub = _ros->subscribe("/xbotcore/aux",
                                &Xbot2Utils::IqRosGetter::on_aux_signal_received,
                                &_iq_getter,
                                1,  // queue size
                                &_queue);

    _js_signals_sub = _ros->subscribe("/xbotcore/joint_states",
                                &Xbot2Utils::IqRosGetter::on_js_signal_received,
                                &_iq_getter,
                                1,  // queue size
                                &_queue);

    /* Publishers */

    std::vector<std::string> iq_jnt_names_prealloc(_n_jnts_robot);

    // iq model status publisher
    awesome_leg::IqEstStatus iq_status_prealloc;

    std::vector<double> iq_est_prealloc(_n_jnts_robot);
    std::vector<double> q_p_ddot_est_prealloc(_n_jnts_robot);
    std::vector<double> q_p_ddot_est_filt_prealloc(_n_jnts_robot);
    std::vector<double> q_p_dot_meas_prealloc(_n_jnts_robot);
    std::vector<double> tau_meas_prealloc(_n_jnts_robot);
    std::vector<double> Kt_prealloc(_n_jnts_robot);
    std::vector<double> Kd0_prealloc(_n_jnts_robot);
    std::vector<double> Kd1_prealloc(_n_jnts_robot);
    std::vector<double> rot_MoI_prealloc(_n_jnts_robot);
    std::vector<double> red_ratio_prealloc(_n_jnts_robot);
    std::vector<double> iq_friction_torque_prealloc(_n_jnts_robot);

    iq_status_prealloc.iq_est = iq_est_prealloc;
    iq_status_prealloc.q_p_ddot_est = q_p_ddot_est_prealloc;
    iq_status_prealloc.q_p_ddot_est_filt = q_p_ddot_est_prealloc;
    iq_status_prealloc.q_p_dot_meas = q_p_dot_meas_prealloc;
    iq_status_prealloc.tau_meas = tau_meas_prealloc;
    iq_status_prealloc.K_t = Kt_prealloc;
    iq_status_prealloc.K_d0 = Kd0_prealloc;
    iq_status_prealloc.K_d1 = Kd1_prealloc;
    iq_status_prealloc.rot_MoI = rot_MoI_prealloc;
    iq_status_prealloc.red_ratio = red_ratio_prealloc;
    iq_status_prealloc.tau_friction = iq_friction_torque_prealloc;
    iq_status_prealloc.der_est_order = _der_est_order;

    iq_status_prealloc.iq_jnt_names = iq_jnt_names_prealloc;

    _iq_est_pub = _ros->advertise<awesome_leg::IqEstStatus>(
        "iq_est_node", 1, iq_status_prealloc);

    // iq model calibration status publisher
    awesome_leg::IqCalStatus iq_cal_prealloc;

    std::vector<double> tau_linkside_prealloc(_n_jnts_robot);
    std::vector<double> q_p_dot_filt_prealloc(_n_jnts_robot);
    std::vector<double> tau_total_prealloc(_n_jnts_robot);
    std::vector<double> tau_friction_meas(_n_jnts_robot);
    std::vector<double> alpha_f0_prealloc(_n_jnts_robot);
    std::vector<double> alpha_f1_prealloc(_n_jnts_robot);
    std::vector<double> K_d0_cal_prealloc(_n_jnts_robot);
    std::vector<double> K_d1_cal_prealloc(_n_jnts_robot);
    std::vector<double> iq_meas_prealloc(_n_jnts_robot);
    std::vector<double> iq_meas_filt_prealloc(_n_jnts_robot);
    std::vector<double> sol_millis_prealloc(_n_jnts_robot);

    iq_cal_prealloc.tau_linkside_filt = tau_linkside_prealloc;
    iq_cal_prealloc.q_dot_filt = q_p_dot_filt_prealloc;
    iq_cal_prealloc.tau_total = tau_total_prealloc;
    iq_cal_prealloc.tau_friction_meas = tau_friction_meas;
    iq_cal_prealloc.alpha_f0 = alpha_f0_prealloc;
    iq_cal_prealloc.alpha_f1 = alpha_f1_prealloc;
    iq_cal_prealloc.K_d0_cal = K_d0_cal_prealloc;
    iq_cal_prealloc.K_d1_cal = K_d1_cal_prealloc;
    iq_cal_prealloc.iq_meas = iq_meas_prealloc;
    iq_cal_prealloc.iq_meas_filt = K_d1_cal_prealloc;

    iq_cal_prealloc.iq_jnt_names = iq_jnt_names_prealloc;

    iq_cal_prealloc.sol_millis = sol_millis_prealloc;

    _iq_cal_pub = _ros->advertise<awesome_leg::IqCalStatus>(
        "iq_cal_node", 1, iq_cal_prealloc);

    /* Services */

    _stop_calibration_srv = _ros->advertiseService(
        "stop_calibration_srv",
        &JntCalRt::on_stop_calib_msg_rec,
        this,
        &_queue);
}

bool JntCalRt::on_stop_calib_msg_rec(const awesome_leg::JntCalRtRequest& req,
                                           awesome_leg::JntCalRtResponse& res)
{
    _stop_calibration = req.stop; // setting flag

    std::string message;

    res.message = (_stop_calibration == true) ? "Stopping calibration!! Gains will be fixed to the latest ones!" : "Resuming calibration!!";

    res.success = true;

    return res.success;
}

void JntCalRt::add_data2bedumped()
{

    add_data2dump_logger();
    
}

void JntCalRt::pub_iq_est()
{
    auto iq_est_msg = _iq_est_pub->loanMessage();

    // mapping EigenVectorXd data to std::vector, so that they can be published
    Eigen::Map<Eigen::VectorXd>(&_iq_est_vect[0], _iq_est.size(), 1) = _iq_est;
    Eigen::Map<Eigen::VectorXd>(&_q_p_ddot_est_vect[0], _q_p_ddot_est.size(), 1) = _q_p_ddot_est;
    Eigen::Map<Eigen::VectorXd>(&_q_p_ddot_est_filt_vect[0], _q_p_ddot_est_filt.size(), 1) = _q_p_ddot_est_filt;
    Eigen::Map<Eigen::VectorXd>(&_q_p_dot_meas_vect[0], _q_p_dot_meas.size(), 1) = _q_p_dot_meas;
    Eigen::Map<Eigen::VectorXd>(&_tau_meas_vect[0], _tau_meas.size(), 1) = _tau_meas;
    Eigen::Map<Eigen::VectorXd>(&_K_t_vect[0], _K_t.size(), 1) = _K_t;
    Eigen::Map<Eigen::VectorXd>(&_K_d0_vect[0], _K_d0.size(), 1) = _K_d0;
    Eigen::Map<Eigen::VectorXd>(&_K_d1_vect[0], _K_d1.size(), 1) = _K_d1;
    Eigen::Map<Eigen::VectorXd>(&_rot_MoI_vect[0], _rot_MoI.size(), 1) = _rot_MoI;
    Eigen::Map<Eigen::VectorXd>(&_red_ratio_vect[0], _red_ratio.size(), 1) = _red_ratio;
    Eigen::Map<Eigen::VectorXd>(&_iq_friction_torque_vect[0], _iq_friction_torque.size(), 1) = _iq_friction_torque;

    // filling message
    iq_est_msg->msg().iq_est = _iq_est_vect;
    iq_est_msg->msg().q_p_ddot_est = _q_p_ddot_est_vect;
    iq_est_msg->msg().q_p_ddot_est_filt = _q_p_ddot_est_filt_vect;
    iq_est_msg->msg().q_p_dot_meas = _q_p_dot_meas_vect;
    iq_est_msg->msg().tau_meas = _tau_meas_vect;
    iq_est_msg->msg().K_t = _K_t_vect;
    iq_est_msg->msg().K_d0 = _K_d0_vect;
    iq_est_msg->msg().K_d1 = _K_d1_vect;
    iq_est_msg->msg().rot_MoI = _rot_MoI_vect;
    iq_est_msg->msg().red_ratio = _red_ratio_vect;
    iq_est_msg->msg().tau_friction = _iq_friction_torque_vect;

    iq_est_msg->msg().der_est_order = _der_est_order;

    iq_est_msg->msg().iq_jnt_names = _iq_jnt_names;

    _iq_est_pub->publishLoaned(std::move(iq_est_msg));

}

void JntCalRt::pub_iq_cal()
{

    auto iq_cal_msg = _iq_cal_pub->loanMessage();

    // mapping EigenVectorXd data to std::vector, so that they can be published
    Eigen::Map<Eigen::VectorXd>(&_tau_meas_filt_vect[0], _tau_meas_filt.size(), 1) = _tau_meas_filt;
    Eigen::Map<Eigen::VectorXd>(&_q_p_dot_meas_filt_vect[0], _q_p_dot_meas_filt.size(), 1) = _q_p_dot_meas_filt;
    Eigen::Map<Eigen::VectorXd>(&_tau_rot_est_vect[0], _tau_rot_est.size(), 1) = _tau_rot_est;
    Eigen::Map<Eigen::VectorXd>(&_iq_friction_torque_cal_vect[0], _iq_friction_torque_cal.size(), 1) = _iq_friction_torque_cal;
    Eigen::Map<Eigen::VectorXd>(&_alpha_f0_vect[0], _alpha_f0.size(), 1) = _alpha_f0;
    Eigen::Map<Eigen::VectorXd>(&_alpha_f1_vect[0], _alpha_f1.size(), 1) = _alpha_f1;
    Eigen::Map<Eigen::VectorXd>(&_K_d0_cal_vect[0], _K_d0_cal.size(), 1) = _K_d0_cal;
    Eigen::Map<Eigen::VectorXd>(&_K_d1_cal_vect[0], _K_d1_cal.size(), 1) = _K_d1_cal;
    Eigen::Map<Eigen::VectorXd>(&_iq_meas_vect[0], _iq_meas.size(), 1) = _iq_meas;
    Eigen::Map<Eigen::VectorXd>(&_iq_meas_filt_vect[0], _iq_meas_filt.size(), 1) = _iq_meas_filt;
    Eigen::Map<Eigen::VectorXd>(&_iq_cal_sol_millis_vect[0], _iq_cal_sol_millis.size(), 1) = _iq_cal_sol_millis;

    // filling message
    iq_cal_msg->msg().tau_linkside_filt = _tau_meas_filt_vect;
    iq_cal_msg->msg().q_dot_filt = _q_p_dot_meas_filt_vect;
    iq_cal_msg->msg().tau_total = _tau_rot_est_vect;
    iq_cal_msg->msg().tau_friction_meas = _iq_friction_torque_cal_vect;
    iq_cal_msg->msg().alpha_f0 = _alpha_f0_vect;
    iq_cal_msg->msg().alpha_f1 = _alpha_f1_vect;
    iq_cal_msg->msg().K_d0_cal = _K_d0_cal_vect;
    iq_cal_msg->msg().K_d1_cal = _K_d1_cal_vect;
    iq_cal_msg->msg().iq_meas = _iq_meas_vect;
    iq_cal_msg->msg().iq_meas_filt = _iq_meas_filt_vect;

    iq_cal_msg->msg().iq_jnt_names = _iq_jnt_names;

    iq_cal_msg->msg().sol_millis = _iq_cal_sol_millis_vect;

    _iq_cal_pub->publishLoaned(std::move(iq_cal_msg));

}

void JntCalRt::run_iq_calib()
{

    // we update the calibration oject with fresh
    // states (we filter every input, so that we get the same delay).
    // Note that we don't really care about the
    // delay introduced here. This call updates all the necessary internal
    // data, but still doesn't update the calibration
    _iq_calib.add_sample(_q_p_dot_meas_filt,
                         _q_p_ddot_est_filt,
                         _iq_meas_filt,
                         _tau_meas_filt);

    _iq_calib.set_ig(_K_d0_ig, _K_d1_ig); // in case Kd0 and Kd1 i.g. are updated are run-time

    // we solve the calibration problem and get the current
    // optimal values
    if(!_stop_calibration)
    { // if we want to stop the calibration, we simply don't update anymore
      // the friction gains. The internal quantities are still computed, so
      // we can still monitor relevant quantities, but the optimal gains stay fix.

        _iq_calib.get_current_optimal_Kd(_K_d0_cal, _K_d1_cal);

    }

    // we set the ig to the new solution to
    // promote convergence (which is also controlled by
    // the chosen regularization gain)
    _K_d0_ig = _K_d0_cal;
    _K_d1_ig = _K_d1_cal;

    // we also set the gains for the iq estimaton to the just obtained
    // optimal values
    _K_d0 = _K_d0_cal;
    _K_d1 = _K_d1_cal;

    // getting useful additional data for debbugging
    _iq_calib.get_current_tau_total(_tau_rot_est);
    _iq_calib.get_current_tau_friction(_iq_friction_torque_cal);
    _iq_calib.get_current_alpha(_alpha_f0, _alpha_f1);
    _iq_calib.get_sol_millis(_iq_cal_sol_millis);

}

void JntCalRt::run_iq_estimation()
{

    // we update the state of the iq estimator with the most recent ones (filtered)
    _iq_estimator.set_current_state(_q_p_dot_meas_filt, _q_p_ddot_est_filt, _tau_meas_filt);

    // we compute and get the iq estimate using the (possibly) updated K_d0 and K_d1 gains
    _iq_estimator.get_iq_estimate(_iq_est, _K_d0, _K_d1);

    // we also get other useful quantities from the estimator
    _iq_estimator.get_tau_friction(_iq_friction_torque);

}

bool JntCalRt::on_initialize()
{ 
    std::string sim_flagname = "sim";
    is_sim(sim_flagname); // see if we are running a simulation

    std::string dummy_flagname = "dummy";
    is_dummy(dummy_flagname); // see if we are running in dummy mode

    get_params_from_config(); // load params from yaml file

    _plugin_dt = getPeriodSec();

    _n_jnts_robot = _robot->getJointNum();
    _jnt_names = _robot->getEnabledJointNames();;

    init_vars();

    init_nrt_ros_bridge();

    // using the order given by _jnt_names
    _iq_getter = IqRosGetter(_plugin_dt); // object to get the
    // quadrature current from XBot2 ROS topic (they need to be activated
    // manually)
    _iq_jnt_names = _jnt_names; // we will get (and estimate) the iq
    _iq_getter.set_jnt_names(_iq_jnt_names); // iq estimates will be ordered as _jnt_names

    // iq estimation model
    _iq_estimator = IqEstimator(_K_t,
                                _K_d0_ig, _K_d1_ig,
                                _rot_MoI,
                                _red_ratio,
                                _alpha,
                                _q_dot_3sigma); // object to compute iq estimate

    // numerical differentiation
    _num_diff = NumDiff(_n_jnts_robot, _plugin_dt, _der_est_order);

    // iq model calibration object
    _iq_calib = IqCalib(_iq_calib_window_size,
                        _K_t,
                        _rot_MoI,
                        _red_ratio,
                        _K_d0_ig,
                        _K_d1_ig,
                        _alpha,
                        _q_dot_3sigma,
                        _lambda_qp_reg);

    //* filtering --> we should filter everything, also the q_dot (which is not so noisy)
    // so that we get the same delay for each data necessary for the iq model calibration
    // (this means we should filter with the same cutoff frequency or window length)*//

    // filter for iq(estimate)
    _mov_avrg_filter_iq = MovAvrgFilt(_n_jnts_robot, _plugin_dt, _mov_avrg_cutoff_freq_iq);
    _mov_avrg_filter_iq.get_window_size(_mov_avrg_window_size_iq); // get computed window size

    // filter for iq(measurement)
    _mov_avrg_filter_iq_meas = MovAvrgFilt(_n_jnts_robot, _plugin_dt, _mov_avrg_cutoff_freq_iq_meas);
    _mov_avrg_filter_iq_meas.get_window_size(_mov_avrg_window_size_iq_meas); // get computed window size

    //filter for tau_meas
    _mov_avrg_filter_tau = MovAvrgFilt(_n_jnts_robot, _plugin_dt, _mov_avrg_cutoff_freq_iq);
    _mov_avrg_filter_tau.get_window_size(_mov_avrg_window_size_tau); // get computed window size

    //filter for q_dot
    _mov_avrg_filter_q_dot = MovAvrgFilt(_n_jnts_robot, _plugin_dt, _mov_avrg_cutoff_freq_iq);
    _mov_avrg_filter_tau.get_window_size(_mov_avrg_window_size_q_dot); // get computed window size

    return true;
    
}

void JntCalRt::starting()
{

    init_dump_logger(); // needs to be here

    reset_flags(); // reset flags, just in case

    init_clocks(); // initialize clocks timers

    update_state(); // read current jnt positions and velocities

    start_completed(); // Move on to run()

}

void JntCalRt::run()
{  

    update_state(); // update all necessary states

    _queue.run(); // processes callbacks

    // calibrate iq model
    run_iq_calib();

    // computing and updating iq estimates
    run_iq_estimation();

    // publish info
    pub_iq_est(); // publish estimates to topic

    pub_iq_cal(); // publish results of iq model calibration to topic

    add_data2dump_logger();

    update_clocks(); // last, update the clocks (loop + any additional one)

}

void JntCalRt::on_stop()
{
    // Destroy logger and dump .mat file (will be recreated upon plugin restart)
    _dump_logger.reset();
}

void JntCalRt::stopping()
{
    stop_completed();
}

void JntCalRt::on_abort()
{
    // Destroy logger and dump .mat file
    _dump_logger.reset();
}

void JntCalRt::on_close()
{
    jinfo("Closing JntCalRt");
}

XBOT2_REGISTER_PLUGIN(JntCalRt, jnt_calib_rt)

