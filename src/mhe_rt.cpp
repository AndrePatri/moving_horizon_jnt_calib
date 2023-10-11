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
#include "mhe_rt.h"

#include <math.h>

void MheRt::init_clocks()
{
    _loop_time = 0.0; // reset loop time clock

}

void MheRt::reset_clocks()
{
    _loop_time = 0.0;

}

void MheRt::update_clocks()
{
    // Update timer(s)
    _loop_time += _plugin_dt;

}

void MheRt::init_vars()
{

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
    _alpha_d0 = Eigen::VectorXd::Zero(_jnt_list.size());
    _alpha_d1 = Eigen::VectorXd::Zero(_jnt_list.size());
    _alpha_inertial = Eigen::VectorXd::Zero(_jnt_list.size());
    _alpha_kt = Eigen::VectorXd::Zero(_jnt_list.size());
    _tau_friction = Eigen::VectorXd::Zero(_jnt_list.size());
    _tau_mot = Eigen::VectorXd::Zero(_jnt_list.size());
    _tau_inertial = Eigen::VectorXd::Zero(_jnt_list.size());

    _K_d0 = Eigen::VectorXd::Zero(_jnt_list.size());
    _K_d1 = Eigen::VectorXd::Zero(_jnt_list.size());
    _rot_MoI = Eigen::VectorXd::Zero(_jnt_list.size());
    _K_t = Eigen::VectorXd::Zero(_jnt_list.size());

    _K_d0_ig_solv = Eigen::VectorXd::Zero(_jnt_list.size());
    _K_d1_ig_solv = Eigen::VectorXd::Zero(_jnt_list.size());
    _rot_MoI_ig_solv = Eigen::VectorXd::Zero(_jnt_list.size());
    _K_t_ig_solv = Eigen::VectorXd::Zero(_jnt_list.size());

    _jnt_indxs = std::vector<int>(_jnt_list.size());
    std::fill(_jnt_indxs.begin(), _jnt_indxs.end(), -1);

    _q_p_ddot_est_vect = std::vector<double>(_jnt_list.size());
    _q_p_dot_meas_vect = std::vector<double>(_jnt_list.size());
    _tau_meas_vect = std::vector<double>(_jnt_list.size());

    _red_ratio_vect = std::vector<double>(_jnt_list.size());
    _alpha_d0_vect = std::vector<double>(_jnt_list.size());
    _alpha_d1_vect = std::vector<double>(_jnt_list.size());
    _alpha_inertial_vect = std::vector<double>(_jnt_list.size());
    _alpha_kt_vect = std::vector<double>(_jnt_list.size());
    _iq_meas_vect = std::vector<double>(_jnt_list.size());
    _jnt_cal_sol_millis_vect = std::vector<double>(_jnt_list.size());

    _K_t_vect = std::vector<double>(_jnt_list.size());
    _K_d0_vect = std::vector<double>(_jnt_list.size());
    _K_d1_vect = std::vector<double>(_jnt_list.size());
    _rot_MoI_vect = std::vector<double>(_jnt_list.size());
    _K_t_ig_vect = std::vector<double>(_jnt_list.size());
    _K_d0_ig_vect = std::vector<double>(_jnt_list.size());
    _K_d1_ig_vect = std::vector<double>(_jnt_list.size());
    _rot_MoI_ig_vect = std::vector<double>(_jnt_list.size());
    _K_t_nom_vect = std::vector<double>(_jnt_list.size());
    _K_d0_nom_vect = std::vector<double>(_jnt_list.size());
    _K_d1_nom_vect = std::vector<double>(_jnt_list.size());
    _rot_MoI_nom_vect = std::vector<double>(_jnt_list.size());

    _cal_mask = std::vector<bool>(_lambda_des.size());
    _cal_mask_ros = std::vector<uint8_t>(_lambda_des.size());
    _cal_mask_des_ros = std::vector<uint8_t>(_lambda_des.size());

    _lambda = Eigen::VectorXd::Zero(_lambda_des.size());
    _lambda_high_solv = Eigen::VectorXd::Zero(_lambda_des.size());

    _lambda_vect = std::vector<double>(_lambda.size());
    _lambda_des_vect = std::vector<double>(_lambda_des.size());
    _lambda_high_vect = std::vector<double>(_lambda_des.size());

}

void MheRt::get_params_from_config()
{
    // Reading some parameters from XBot2 config. YAML file

    _verbose = getParamOrThrow<bool>("~verbose");

    _jnt_cal_yaml = getParamOrThrow<YAML::Node>("~jnt_cal_yaml/content");

    _force_iq_from_topic = getParamOrThrow<bool>("~force_iq_from_topic");

    _queue_size = getParamOrThrow<int>("~queue_size");

    _jnt_list = _jnt_cal_yaml["jnt_list"].as<std::vector<std::string>>();

    _cal_mask_des = _jnt_cal_yaml["cal_mask"].as<std::vector<bool>>();
    _rot_calib_window_size = _jnt_cal_yaml["rot_calib_window_size"].as<int>();

    _alpha = _jnt_cal_yaml["alpha"].as<int>();
    _q_dot_3sigma = _jnt_cal_yaml["q_dot_3sigma"].as<double>();
    _mov_avrg_cutoff_freq = _jnt_cal_yaml["mov_avrg_cutoff_freq"].as<double>();

    auto lambda_des = _jnt_cal_yaml["lambda"].as<std::vector<double>>();
    auto lambda_high = _jnt_cal_yaml["lambda_high"].as<std::vector<double>>();
    auto red_ratio = _jnt_cal_yaml["red_ratio"].as<std::vector<double>>();
    auto K_t_ig = _jnt_cal_yaml["K_t_ig"].as<std::vector<double>>();
    auto rot_MoI_ig = _jnt_cal_yaml["rot_MoI_ig"].as<std::vector<double>>();
    auto K_d0_ig = _jnt_cal_yaml["K_d0_ig"].as<std::vector<double>>();
    auto K_d1_ig = _jnt_cal_yaml["K_d1_ig"].as<std::vector<double>>();
    auto K_t_nom = _jnt_cal_yaml["K_t_nom"].as<std::vector<double>>();
    auto rot_MoI_nom = _jnt_cal_yaml["rot_MoI_nom"].as<std::vector<double>>();
    auto K_d0_nom = _jnt_cal_yaml["K_d0_nom"].as<std::vector<double>>();
    auto K_d1_nom = _jnt_cal_yaml["K_d1_nom"].as<std::vector<double>>();

    // mapping to VectorXd
    _lambda_des = Eigen::VectorXd::Map(lambda_des.data(),
                                 lambda_des.size());
    _lambda_high = Eigen::VectorXd::Map(lambda_high.data(),
                                 lambda_high.size());
    _red_ratio = Eigen::VectorXd::Map(red_ratio.data(),
                                 red_ratio.size());
    _red_ratio = _red_ratio.array().inverse(); // to comply with internal
    // conventions that 0 < red_ratio <= 1

    _K_t_ig = Eigen::VectorXd::Map(K_t_ig.data(),
                                 K_t_ig.size());
    _rot_MoI_ig = Eigen::VectorXd::Map(rot_MoI_ig.data(),
                                 rot_MoI_ig.size());
    _K_d0_ig = Eigen::VectorXd::Map(K_d0_ig.data(),
                                 K_d0_ig.size());
    _K_d1_ig = Eigen::VectorXd::Map(K_d1_ig.data(),
                                 K_d1_ig.size());
    _K_t_nom = Eigen::VectorXd::Map(K_t_nom.data(),
                                 K_t_nom.size());
    _rot_MoI_nom = Eigen::VectorXd::Map(rot_MoI_nom.data(),
                                 rot_MoI_nom.size());
    _K_d0_nom = Eigen::VectorXd::Map(K_d0_nom.data(),
                                 K_d0_nom.size());
    _K_d1_nom = Eigen::VectorXd::Map(K_d1_nom.data(),
                                 K_d1_nom.size());

    _set_ig_to_prev_sol = _jnt_cal_yaml["set_ig_to_prev_sol"].as<bool>();

    param_dims_ok_or_throw();
}

void MheRt::param_dims_ok_or_throw()
{

    if((_red_ratio.size() != _K_t_ig.size()) ||
       (_K_t_ig.size() != _rot_MoI_ig.size()) ||
       (_rot_MoI_ig.size() != _K_d0_ig.size()) ||
       (_K_d0_ig.size() != _K_d1_ig.size()) ||
       (_K_d1_ig.size() != _K_t_nom.size()) ||
       (_K_t_nom.size() != _rot_MoI_nom.size()) ||
       (_rot_MoI_nom.size() != _K_d0_nom.size()) ||
       (_K_d0_nom.size() != _K_d1_nom.size()) ||
       (_K_d1_nom.size() != _jnt_list.size()))
    {
        std::string exception = std::string("MheRt::param_dims_ok_or_throw(): Dimension mismatch in YAML parameters. Check the XBot2 config file for errors! "
                                            "All dimensions should be coherent with the provided calibration jnt_list (dim: ") +
                                std::to_string(_jnt_list.size()) + std::string(").\n");

        throw std::invalid_argument(exception);
    }
}

void MheRt::is_sim(std::string sim_string = "sim")
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

void MheRt::is_dummy(std::string dummy_string = "dummy")
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

void MheRt::update_state()
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

        _tau_meas_red(i) = _tau_meas(_jnt_indxs[i]);

        _q_p_dot_meas_red(i) = _q_p_dot_meas(_jnt_indxs[i]);


    }

    // estimating joint acceleration
    _num_diff.add_sample(_q_p_dot_meas_red);
    _num_diff.dot(_q_p_ddot_meas_red);

    if((_is_sim || _is_dummy) && !_force_iq_from_topic)
    {
        update_iq_estimation();

    }
    else
    { // we're on the real robot

        _iq_getter.get_last_iq_out(_iq_meas); // raw

        _iq_getter.get_last_iq_out_filt(_iq_meas_filt); // filtered

    }

    // filtering data (all with same frequency)
    _mov_avrg_filter_q_dot.add_sample(_q_p_dot_meas_red);
    _mov_avrg_filter_q_ddot.add_sample(_q_p_ddot_meas_red);
    _mov_avrg_filter_tau.add_sample(_tau_meas_red);

    _mov_avrg_filter_q_dot.get(_q_p_dot_meas_red_filt);
    _mov_avrg_filter_q_ddot.get(_q_p_ddot_meas_red_filt);
    _mov_avrg_filter_tau.get(_tau_meas_red_filt);

}

void MheRt::update_iq_estimation()
{

    _iq_estimator.set_current_state(_q_p_dot_meas_red_filt, _q_p_ddot_meas_red_filt, _tau_meas_red_filt);

    _iq_estimator.get_iq_estimate(_iq_meas_filt);

}

void MheRt::init_dump_logger()
{

    // // Initializing logger for debugging
    MatLogger2::Options opt;
    opt.default_buffer_size = _matlogger_buffer_size; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression

    _dump_logger = MatLogger2::MakeLogger(_dump_path, opt); // date-time automatically appended

    _dump_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    _dump_logger->add("plugin_dt", _plugin_dt);
    _dump_logger->add("is_sim", int(_is_sim));

    _dump_logger->create("loop_time", 1, 1, _matlogger_buffer_size);

    _dump_logger->create("jnt_cal_sol_millis", _jnt_list.size(), 1, _matlogger_buffer_size);
    _dump_logger->create("_alpha_d0", _jnt_list.size(), 1, _matlogger_buffer_size);
    _dump_logger->create("_alpha_d1", _jnt_list.size(), 1, _matlogger_buffer_size);
    _dump_logger->create("_alpha_inertial", _jnt_list.size(), 1, _matlogger_buffer_size);
    _dump_logger->create("_alpha_kt", _jnt_list.size(), 1, _matlogger_buffer_size);

    _dump_logger->create("lambda", _lambda_des.size(), 1, _matlogger_buffer_size);
    _dump_logger->create("lambda_des", _lambda_des.size(), 1, _matlogger_buffer_size);
    _dump_logger->create("lambda_high", _lambda_des.size(), 1, _matlogger_buffer_size);
    _dump_logger->create("cal_mask", _lambda_des.size(), 1, _matlogger_buffer_size);

    _dump_logger->create("q_p_dot_meas", _jnt_list.size(), 1, _matlogger_buffer_size);
    _dump_logger->create("q_p_ddot_meas", _jnt_list.size(), 1, _matlogger_buffer_size);
    _dump_logger->create("tau_meas", _jnt_list.size(), 1, _matlogger_buffer_size);
    _dump_logger->create("iq_meas", _jnt_list.size(), 1, _matlogger_buffer_size);

    _dump_logger->create("q_p_cmd", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("q_p_dot_cmd", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("q_p_ddot_cmd", _n_jnts_robot, 1, _matlogger_buffer_size);

    _dump_logger->create("K_d0", _jnt_list.size(), 1, _matlogger_buffer_size);
    _dump_logger->create("K_d1", _jnt_list.size(), 1, _matlogger_buffer_size);
    _dump_logger->create("rot_MoI", _jnt_list.size(), 1, _matlogger_buffer_size);
    _dump_logger->create("K_t", _jnt_list.size(), 1, _matlogger_buffer_size);
    _dump_logger->create("red_ratio", _jnt_list.size(), 1, _matlogger_buffer_size);

    _dump_logger->create("K_d0_ig", _jnt_list.size(), 1, _matlogger_buffer_size);
    _dump_logger->create("K_d1_ig", _jnt_list.size(), 1, _matlogger_buffer_size);
    _dump_logger->create("rot_MoI_ig", _jnt_list.size(), 1, _matlogger_buffer_size);
    _dump_logger->create("K_t_ig", _jnt_list.size(), 1, _matlogger_buffer_size);
    _dump_logger->create("red_ratio_ig", _jnt_list.size(), 1, _matlogger_buffer_size);

    _dump_logger->create("K_d0_nom", _jnt_list.size(), 1, _matlogger_buffer_size);
    _dump_logger->create("K_d1_nom", _jnt_list.size(), 1, _matlogger_buffer_size);
    _dump_logger->create("rot_MoI_nom", _jnt_list.size(), 1, _matlogger_buffer_size);
    _dump_logger->create("K_t_nom", _jnt_list.size(), 1, _matlogger_buffer_size);
    _dump_logger->create("red_ratio_nom", _jnt_list.size(), 1, _matlogger_buffer_size);



}

void MheRt::add_data2dump_logger()
{


    _dump_logger->add("loop_time", _loop_time);

    _dump_logger->add("jnt_cal_sol_millis", _jnt_cal_sol_millis);
    _dump_logger->add("_alpha_d0", _alpha_d0);
    _dump_logger->add("_alpha_d1", _alpha_d1);
    _dump_logger->add("_alpha_inertial", _alpha_inertial);
    _dump_logger->add("_alpha_kt", _alpha_kt);

    _dump_logger->add("lambda", _lambda);
    _dump_logger->add("lambda_des", _lambda_des);
    _dump_logger->add("lambda_high", _lambda_high);

    _dump_logger->add("q_p_dot_meas", _q_p_dot_meas_red_filt);
    _dump_logger->add("q_p_ddot_meas", _q_p_ddot_meas_red_filt);
    _dump_logger->add("tau_meas", _tau_meas_red_filt);
    _dump_logger->add("iq_meas", _iq_meas_filt);

    _dump_logger->add("K_d0", _K_d0);
    _dump_logger->add("K_d1", _K_d1);
    _dump_logger->add("rot_MoI", _rot_MoI);
    _dump_logger->add("K_t", _K_t);
    _dump_logger->add("red_ratio", _red_ratio);

    _dump_logger->add("K_d0_ig", _K_d0_ig_solv);
    _dump_logger->add("K_d1_ig", _K_d1_ig_solv);
    _dump_logger->add("rot_MoI_ig", _rot_MoI_ig_solv);
    _dump_logger->add("K_t_ig", _K_t_ig_solv);

    _dump_logger->add("K_d0_nom", _K_d0_nom);
    _dump_logger->add("K_d1_nom", _K_d1_nom);
    _dump_logger->add("rot_MoI_nom", _rot_MoI_nom);
    _dump_logger->add("K_t_nom", _K_t_nom);

}

void MheRt::add_data2bedumped()
{

    add_data2dump_logger();

}

void MheRt::init_ros_bridge()
{
    ros::NodeHandle nh(getName());

    _ros = std::make_unique<RosSupport>(nh);

    /* Service servers */
    _start_cal_srvr = _ros->advertiseService("start_jnt_cal",
                                           &MheRt::on_start_cal_received,
                                           this,
                                           &_queue);

    _set_cal_srvr = _ros->advertiseService("set_jnt_cal",
                                           &MheRt::on_set_cal_received,
                                           this,
                                           &_queue);

    /* Publishers */
    moving_horizon_jnt_calib::JntCalibStatus jnt_cal_st_prealloc;
    _jnt_calib_pub = _ros->advertise<moving_horizon_jnt_calib::JntCalibStatus>(
        "jnt_calib_status", 1, jnt_cal_st_prealloc);

    /* Subscribers */

    #if defined(EC_XBOT2_CLIENT_FOUND)

    // we use internal topics for mininum latency
    _aux_signals_sub = subscribe("/hal/joint_ec/aux",
                                &Xbot2Utils::IqOutRosGetter::on_aux_signal_received,
                                &_iq_getter,
                                100,  // queue size
                                &_queue);

    #else

    // we use ros topics(high latency)
    _aux_signals_sub = _ros->subscribe("/xbotcore/aux",
                                &Xbot2Utils::IqOutRosGetter::on_aux_signal_received_ros,
                                &_iq_getter,
                                100,  // queue size
                                &_queue);
   #endif

}


void MheRt::pub_calib_status()
{
    auto status_msg = _jnt_calib_pub->loanMessage();

    status_msg->msg().jnt_names = _jnt_list;

    status_msg->msg().is_window_full = _rot_dyn_calib.is_window_full();

    for(int i = 0; i < _jnt_list.size(); i++)
    {
        _iq_meas_vect[i] = _iq_meas_filt(i);
        _q_p_dot_meas_vect[i] = _q_p_dot_meas_red_filt(i);
        _q_p_ddot_est_vect[i] = _q_p_ddot_meas_red_filt(i);
        _tau_meas_vect[i] = _tau_meas_red_filt(i);
        _alpha_d0_vect[i] = _alpha_d0[i];
        _alpha_d1_vect[i] = _alpha_d1[i];
        _alpha_inertial_vect[i] = _alpha_inertial[i];
        _alpha_kt_vect[i] = _alpha_kt[i];

        _K_d0_vect[i] = _K_d0[i];
        _K_d1_vect[i] = _K_d1[i];
        _K_t_vect[i] = _K_t[i];
        _rot_MoI_vect[i] = _rot_MoI[i];

        _K_d0_ig_vect[i] = _K_d0_ig_solv[i];
        _K_d1_ig_vect[i] = _K_d1_ig_solv[i];
        _K_t_ig_vect[i] = _K_t_ig_solv[i];
        _rot_MoI_ig_vect[i] = _rot_MoI_ig_solv[i];

        _K_d0_nom_vect[i] = _K_d0_nom[i];
        _K_d1_nom_vect[i] = _K_d1_nom[i];
        _K_t_nom_vect[i] = _K_t_nom[i];
        _rot_MoI_nom_vect[i] = _rot_MoI_nom[i];

        _jnt_cal_sol_millis_vect[i] = _jnt_cal_sol_millis(i);

    }

    for (int i = 0; i < _lambda.size(); i++)
    {
        _lambda_vect[i] = _lambda(i);
        _lambda_des_vect[i] = _lambda_des(i);
        _lambda_high_vect[i] = _lambda_high_solv(i);
    }

    status_msg->msg().iq = _iq_meas_vect;
    status_msg->msg().q_dot = _q_p_dot_meas_vect;
    status_msg->msg().q_ddot = _q_p_ddot_est_vect;
    status_msg->msg().tau = _tau_meas_vect;

    status_msg->msg().alpha_d0 = _alpha_d0_vect;
    status_msg->msg().alpha_d1 = _alpha_d1_vect;
    status_msg->msg().alpha_inertial = _alpha_inertial_vect;
    status_msg->msg().alpha_kt = _alpha_kt_vect;

    status_msg->msg().red_ratio = _red_ratio_vect;

    status_msg->msg().K_d0_cal = _K_d0_vect;
    status_msg->msg().K_d1_cal = _K_d1_vect;
    status_msg->msg().K_t_cal = _K_t_vect;
    status_msg->msg().rotor_MoI_cal = _rot_MoI_vect;

    status_msg->msg().K_d0_ig = _K_d0_ig_vect;
    status_msg->msg().K_d1_ig = _K_d1_ig_vect;
    status_msg->msg().K_t_ig = _K_t_ig_vect;
    status_msg->msg().rotor_MoI_ig = _rot_MoI_ig_vect;

    status_msg->msg().K_d0_nom = _K_d0_nom_vect;
    status_msg->msg().K_d1_nom = _K_d1_nom_vect;
    status_msg->msg().K_t_nom = _K_t_nom_vect;
    status_msg->msg().rotor_MoI_nom = _rot_MoI_nom_vect;

    status_msg->msg().sol_millis = _jnt_cal_sol_millis_vect;

    for(int i = 0; i < 4; i++)
    {
        _cal_mask_ros[i] = _cal_mask[i];
        _cal_mask_des_ros[i] = _cal_mask_des[i];

    }

    status_msg->msg().cal_mask = _cal_mask_ros;
    status_msg->msg().cal_mask_des = _cal_mask_des_ros;

    status_msg->msg().lambda_des = _lambda_des_vect;
    status_msg->msg().lambda = _lambda_vect;
    status_msg->msg().lambda_high = _lambda_high_vect;

    _jnt_calib_pub->publishLoaned(std::move(status_msg));
}

bool MheRt::on_start_cal_received(const moving_horizon_jnt_calib::StartCalibRequest& req,
                              moving_horizon_jnt_calib::StartCalibResponse& res)
{


    _calibrate = req.calibrate;

    res.success = true;

    return res.success;

}

bool MheRt::on_set_cal_received(const moving_horizon_jnt_calib::SetCalibParamsRequest& req,
                              moving_horizon_jnt_calib::SetCalibParamsResponse& res)
{


    int err = 0;

    if(req.cal_mask.size() == _lambda_des.size())
    { // we can set the calibration mask
        for (int i = 0; i < _lambda_des.size(); i++)
        {
            _cal_mask_des[i] = req.cal_mask[i];
        }

    }
    else
    {
        err++;
    }

    if(req.lambda_.size() == _lambda_des.size())
    {

        for (int i = 0; i < _lambda_des.size(); i++)
        {
            _lambda_des(i) = req.lambda_[i];
        }

    }
    else
    {
        err++;
    }

    if(req.lambda_high.size() == _lambda_des.size())
    {

        for (int i = 0; i < _lambda_des.size(); i++)
        {
            _lambda_high(i) = req.lambda_high[i];
        }

    }
    else
    {
        err++;
    }
    if(req.Kd0_ig.size() == _jnt_list.size())
    {

        for (int i = 0; i < _jnt_list.size(); i++)
        {
            _K_d0_ig(i) = req.Kd0_ig[i];
        }

    }
    else
    {
        err++;
    }
    if(req.Kd0_nom.size() == _jnt_list.size())
    {

        for (int i = 0; i < _jnt_list.size(); i++)
        {
            _K_d0_nom(i) = req.Kd0_nom[i];
        }

    }
    else
    {
        err++;
    }
    if(req.Kd1_ig.size() == _jnt_list.size())
    {

        for (int i = 0; i < _jnt_list.size(); i++)
        {
            _K_d1_ig(i) = req.Kd1_ig[i];
        }

    }
    else
    {
        err++;
    }
    if(req.Kd1_nom.size() == _jnt_list.size())
    {

        for (int i = 0; i < _jnt_list.size(); i++)
        {
            _K_d1_nom(i) = req.Kd1_nom[i];
        }

    }
    else
    {
        err++;
    }
    if(req.rot_MoI_ig.size() == _jnt_list.size())
    {

        for (int i = 0; i < _jnt_list.size(); i++)
        {
            _rot_MoI_ig(i) = req.rot_MoI_ig[i];
        }

    }
    else
    {
        err++;
    }
    if(req.rot_MoI_nom.size() == _jnt_list.size())
    {

        for (int i = 0; i < _jnt_list.size(); i++)
        {
            _rot_MoI_nom(i) = req.rot_MoI_nom[i];
        }

    }
    else
    {
        err++;
    }
    if(req.K_t_ig.size() == _jnt_list.size())
    {

        for (int i = 0; i < _jnt_list.size(); i++)
        {
            _K_t_ig(i) = req.K_t_ig[i];
        }

    }
    else
    {
        err++;
    }
    if(req.K_t_nom.size() == _jnt_list.size())
    {

        for (int i = 0; i < _jnt_list.size(); i++)
        {
            _K_t_nom(i) = req.K_t_nom[i];
        }

    }
    else
    {
        err++;
    }

    _set_ig_to_prev_sol = req.set_ig_to_prev_sol;

    res.success = true;

    return res.success;

}

void MheRt::run_jnt_calib()
{

    _rot_dyn_calib.set_lambda(_lambda_des); // setting latest available regularization
    _rot_dyn_calib.set_lambda_high(_lambda_high); // setting latest available regularization

    apply_calib_mask(); // we first apply the calibration mask, just in case the mask
    // was changed externally

    _rot_dyn_calib.add_sample(_q_p_dot_meas_red_filt,
                              _q_p_ddot_meas_red_filt,
                              _iq_meas_filt,
                              _tau_meas_red_filt); // adding latest state for calibration


    if(_calibrate && _rot_dyn_calib.is_window_full())
    {// we start solving only if the window was completely filled with data
        _rot_dyn_calib.solve();
    }

}

void MheRt::get_calib_data()
{
    _rot_dyn_calib.get_sol_millis(_jnt_cal_sol_millis);
    _rot_dyn_calib.get_opt_Kd0(_K_d0);
    _rot_dyn_calib.get_opt_Kd1(_K_d1);
    _rot_dyn_calib.get_opt_Kt(_K_t);
    _rot_dyn_calib.get_opt_rot_MoI(_rot_MoI);

    _rot_dyn_calib.get_tau_friction(_tau_friction);
    _rot_dyn_calib.get_alpha_d(_alpha_d0, _alpha_d1);
    _rot_dyn_calib.get_alpha_inertial(_alpha_inertial);
    _rot_dyn_calib.get_alpha_kt(_alpha_kt);
    _rot_dyn_calib.get_tau_motor(_tau_mot);
    _rot_dyn_calib.get_tau_inertial(_tau_inertial);

    _rot_dyn_calib.get_cal_mask(_cal_mask);

    _rot_dyn_calib.get_lambda_des(_lambda_des);
    _rot_dyn_calib.get_lambda(_lambda);
    _rot_dyn_calib.get_lambda_high(_lambda_high_solv);

    _rot_dyn_calib.get_ig_Kd0(_K_d0_ig_solv);
    _rot_dyn_calib.get_ig_Kd1(_K_d1_ig_solv);
    _rot_dyn_calib.get_ig_Kt(_K_t_ig_solv);
    _rot_dyn_calib.get_ig_MoI(_rot_MoI_ig_solv);

}

void MheRt::apply_calib_mask()
{
    _rot_dyn_calib.set_solution_mask(_cal_mask_des);

    if(!_cal_mask_des[0])
    { // Kt
        _rot_dyn_calib.set_ig_Kt(_K_t_nom);
    }
    else
    {
        _rot_dyn_calib.set_ig_Kt(_K_t_ig);
    }
    if(!_cal_mask_des[1])
    { // Kd0
        _rot_dyn_calib.set_ig_Kd0(_K_d0_nom);
    }
    else
    {
        _rot_dyn_calib.set_ig_Kd0(_K_d0_ig);
    }

    if(!_cal_mask_des[2])
    { // Kd1
        _rot_dyn_calib.set_ig_Kd1(_K_d1_nom);
    }
    else
    {
        _rot_dyn_calib.set_ig_Kd1(_K_d1_ig);

    }
    if(!_cal_mask_des[3])
    { // rot_MoI
        _rot_dyn_calib.set_ig_MoI(_rot_MoI_nom);
    }
    else
    {
        _rot_dyn_calib.set_ig_MoI(_rot_MoI_ig);
    }

}

bool MheRt::on_initialize()
{
    std::string sim_flagname = "sim";
    is_sim(sim_flagname); // see if we are running a simulation

    std::string dummy_flagname = "dummy";
    is_dummy(dummy_flagname); // see if we are running in dummy mode

    get_params_from_config(); // load params from yaml file

    _plugin_dt = getPeriodSec();

    _n_jnts_robot = _robot->getJointNum();

    _enbld_jnt_names = _robot->getEnabledJointNames();

    init_vars();

    init_ros_bridge();

    jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                          "\nMheRt::on_initialize(): enabled joints list --> \n");

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
                          "\nMheRt::on_initialize(): joint to be calibrated --> \n");

    for(int i = 0; i < _jnt_list.size(); i ++)
    {
        jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                              "{}, ID: {}\n", _jnt_list[i], _jnt_indxs[i]);
    }

    bool all_jnts_valid = !(std::count(_jnt_indxs.begin(), _jnt_indxs.end(), -1));

    if(!all_jnts_valid)
    {
        std::string exception = std::string("MheRt::on_initialize(): not all joint names in jnt_list are valid. Check them!!!\n");

        throw std::invalid_argument(exception);
    }

    // initializing calibration-related stuff

    _iq_getter = IqOutRosGetter(_jnt_list,
                                _plugin_dt,
                                _mov_avrg_cutoff_freq,
                                _verbose); // getter for quadrature current measurements from ros topic

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
                                _lambda_des,
                                _alpha,
                                _q_dot_3sigma,
                                _verbose);

    // iq estimator, just to check calibration in simulation

    _iq_estimator = IqEstimator(_K_t_nom,
                                _K_d0_nom, _K_d1_nom,
                                _rot_MoI_nom,
                                _red_ratio,
                                _alpha,
                                _q_dot_3sigma,
                                _verbose); // object to compute iq estimate
    return true;

}

void MheRt::starting()
{

    init_dump_logger();

    init_clocks(); // initialize clocks timers

    // Move on to run()
    start_completed();

}

void MheRt::run()
{

    update_state(); // update all necessary states

    _queue.run(); // process server callbacks and update triggers' states, if necessary

    run_jnt_calib(); // run moving horizon joint estimation
    get_calib_data(); // getting calibration data

    pub_calib_status(); // publishes calibration status

    add_data2dump_logger(); // add data for debugging purposes

    update_clocks(); // last, update the clocks (loop + any additional one). Each clock is incremented by a plugin dt

    if(_set_ig_to_prev_sol && _calibrate && _rot_dyn_calib.is_window_full())
    { // we do this only is calibrating and if the window is full
      // we check is_window_full() to ensure we only solve when the window is full
        _K_t_ig = _K_t;
        _K_d0_ig = _K_d0;
        _K_d1_ig = _K_d1;
        _rot_MoI_ig = _rot_MoI;
    }

}

void MheRt::on_stop()
{

    init_clocks();

    _dump_logger.reset();

}

void MheRt::stopping()
{

    stop_completed();

}

void MheRt::on_abort()
{
    // Destroy logger and dump .mat file
    _dump_logger.reset();
}

void MheRt::on_close()
{
    jinfo("Closing MheRt");
}

XBOT2_REGISTER_PLUGIN(MheRt, mhe_rt)

