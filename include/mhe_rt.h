#ifndef MHE_RT_H
#define MHE_RT_H

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

#if defined(EC_XBOT2_CLIENT_FOUND)
#include <ec_xbot2/joint_ec.h>
#endif

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <awesome_utils/xbot2_utils/xbot2_utils.hpp>
#include <awesome_utils/awesome_utils/sign_proc_utils.hpp>
#include <awesome_utils/awesome_utils/calib_utils.hpp>

using namespace XBot;
using namespace XBot::Cartesian;
using namespace CalibUtils;
using namespace SignProcUtils;
using namespace Xbot2Utils;

/**
 * @brief
 */
class MheRt : public ControlPlugin
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

    bool _is_sim = true, _is_dummy = false,
        _verbose = false,
        _set_ig_to_prev_sol = true,
        _calibrate = false,
        _force_iq_from_topic = false;

    int _n_jnts_robot,
        _rot_calib_window_size = 10000,
        _alpha = 5;

    std::string _dump_mat_suffix = "mhr_rt",
                _hw_type,
                _dump_path = "/tmp/MheRt";

    double _plugin_dt,
        _matlogger_buffer_size = 1e5,
        _loop_time = 0.0,
        _q_dot_3sigma = 0.01;

    std::vector<std::string> _jnt_list;
    std::vector<int> _jnt_indxs;

    std::vector<std::string> _enbld_jnt_names;

    std::vector<double> _time_ref;

    std::vector<bool> _cal_mask, _cal_mask_des;
    std::vector<uint8_t> _cal_mask_ros, _cal_mask_des_ros;

    YAML::Node _jnt_cal_yaml;

    Eigen::VectorXd _q_p_meas,
                    _q_p_dot_meas,
                    _q_p_ddot_meas,
                    _tau_meas,
                    _K_d0, _K_d1, _rot_MoI, _K_t, _red_ratio,
                    _K_d0_ig, _K_d1_ig, _rot_MoI_ig, _K_t_ig,
                    _K_d0_ig_solv, _K_d1_ig_solv, _rot_MoI_ig_solv, _K_t_ig_solv, // inernal (solver) value of ig
                    _K_d0_nom, _K_d1_nom, _rot_MoI_nom, _K_t_nom;

    Eigen::VectorXd _iq_meas, _iq_meas_filt,
                    _q_p_dot_meas_red, _q_p_dot_meas_red_filt,
                    _q_p_ddot_meas_red, _q_p_ddot_meas_red_filt,
                    _tau_meas_red, _tau_meas_red_filt;

    Eigen::VectorXd _jnt_cal_sol_millis,
                    _alpha_d0, _alpha_d1, _alpha_inertial, _alpha_kt,
                    _tau_friction, _tau_mot, _tau_inertial;

    Eigen::VectorXd _lambda, _lambda_des,
                    _lambda_high, _lambda_high_solv;

    std::vector<double> _q_p_ddot_est_vect,
                        _q_p_dot_meas_vect,
                        _tau_meas_vect,
                        _K_t_vect,
                        _K_d0_vect,
                        _K_d1_vect,
                        _rot_MoI_vect,
                        _red_ratio_vect,
                        _alpha_d0_vect,
                        _alpha_d1_vect,
                        _alpha_inertial_vect,
                        _alpha_kt_vect,
                        _iq_meas_vect,
                        _jnt_cal_sol_millis_vect,
                        _lambda_vect, _lambda_des_vect, _lambda_high_vect;

    std::vector<double> _K_t_ig_vect,
                        _K_d0_ig_vect,
                        _K_d1_ig_vect,
                        _rot_MoI_ig_vect,
                        _K_t_nom_vect,
                        _K_d0_nom_vect,
                        _K_d1_nom_vect,
                        _rot_MoI_nom_vect;

    MatLogger2::Ptr _dump_logger;

    RosSupport::UniquePtr _ros;

    CallbackQueue _queue;

    ServiceServerPtr<moving_horizon_jnt_calib::StartCalibRequest,
                     moving_horizon_jnt_calib::StartCalibResponse> _start_cal_srvr;

    ServiceServerPtr<moving_horizon_jnt_calib::SetCalibParamsRequest,
                     moving_horizon_jnt_calib::SetCalibParamsResponse> _set_cal_srvr;

    PublisherPtr<moving_horizon_jnt_calib::JntCalibStatus> _jnt_calib_pub;


    #if defined(EC_XBOT2_CLIENT_FOUND)
    SubscriberPtr<XBot::Hal::JointEcAux> _aux_signals_sub;
    #else
    SubscriberPtr<xbot_msgs::CustomState> _aux_signals_sub;
    #endif

    IqOutRosGetter _iq_getter;

    IqEstimator _iq_estimator;

    RotDynCal _rot_dyn_calib;

    MovAvrgFilt _mov_avrg_filter_tau;
    MovAvrgFilt _mov_avrg_filter_q_dot;
    MovAvrgFilt _mov_avrg_filter_q_ddot;
    int _mov_avrg_window_size = 10;
    double _mov_avrg_cutoff_freq= 15.0;

    NumDiff _num_diff;

    void get_params_from_config();
    void param_dims_ok_or_throw();

    void init_vars();
    void init_clocks();
    void init_ros_bridge();
    void init_dump_logger();

    void reset_clocks();

    void update_state();
    void update_iq_estimation();

    void update_clocks();

    void add_data2dump_logger();
    void add_data2bedumped();

    void spawn_nrt_thread();

    void is_sim(std::string sim_string);
    void is_dummy(std::string dummy_string);

    void pub_calib_status();

    void run_jnt_calib();
    void get_calib_data();

    void apply_calib_mask();

    bool on_start_cal_received(const moving_horizon_jnt_calib::StartCalibRequest& req,
                               moving_horizon_jnt_calib::StartCalibResponse& res);

    bool on_set_cal_received(const moving_horizon_jnt_calib::SetCalibParamsRequest& req,
                             moving_horizon_jnt_calib::SetCalibParamsResponse& res);
};

#endif // MHE_RT_H
