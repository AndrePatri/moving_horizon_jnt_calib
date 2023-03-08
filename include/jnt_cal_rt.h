#ifndef JNT_CALIB_RT
#define JNT_CALIB_RT

#include <matlogger2/matlogger2.h>

#include <xbot2/xbot2.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>

#include <thread>

#include <awesome_utils/awesome_utils/calib_utils.hpp>
#include <awesome_utils/awesome_utils/sign_proc_utils.hpp>
#include <awesome_utils/xbot2_utils/xbot2_utils.hpp>

#include <std_msgs/Bool.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <xbot2/hal/dev_ft.h>

#include <xbot2/ros/ros_support.h>
#include <xbot_msgs/CustomState.h>
#include <xbot_msgs/JointState.h>

#include <cartesian_interface/ros/RosServerClass.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>

#include <awesome_leg/IqEstStatus.h>
#include <awesome_leg/IqCalStatus.h>
#include <awesome_leg/IqModelCalibRt.h>
#include <awesome_leg/IqModelCalibRtRequest.h>
#include <awesome_leg/IqModelCalibRtResponse.h>

using namespace XBot;
using namespace CalibUtils;
using namespace SignProcUtils;
using namespace Xbot2Utils;

//typedef Eigen::Array<bool, Eigen::Dynamic, 1> VectorXb;

/**
 * @brief The IqModelCalibRt class is a ControlPlugin
 * implementing ......
**/

class JntCalRt : public ControlPlugin
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

    bool _pause_started = false, _pause_finished = false,
        _rt_active, _nrt_exit,
        _is_sim = true, _is_dummy = false,
        _reduce_dumped_sol_size = false,
        _verbose = false,
        _jnt_names_were_set = false,
        _stop_calibration = false;

    int _n_jnts_model,
        _n_jnts_model_ft_est,
        _n_jnts_robot,
        _sample_index = 0,
        _der_est_order = 1,
        _iq_calib_window_size = 1000,
        _alpha = 10;

    std::string _mat_path, _dump_mat_suffix,
                _urdf_path,
                _hw_type;

    double _plugin_dt,
        _loop_time = 0.0, _loop_timer_reset_time = 3600.0,
        _matlogger_buffer_size = 1e4,
        _lambda_qp_reg = 1.0,
        _q_dot_3sigma = 0.001;

    Eigen::VectorXd _q_p_meas,
                    _q_p_dot_meas, _q_p_dot_meas_filt, _q_p_ddot_est, _q_p_ddot_est_filt,
                    _tau_meas, _tau_meas_filt,
                    _iq_meas, _iq_meas_filt,
                    _K_t, _K_d0_ig, _K_d1_ig, _K_d0, _K_d1, _rot_MoI, _red_ratio,
                    _tau_,
                    _iq_est, _iq_friction_torque,
                    _iq_friction_torque_cal, _tau_rot_est,
                    _alpha_f0, _alpha_f1,
                    _K_d0_cal, _K_d1_cal,
                    _iq_cal_sol_millis;

    std::vector<double> _iq_est_vect, _q_p_ddot_est_vect, _q_p_ddot_est_filt_vect,
                        _q_p_dot_meas_vect, _q_p_dot_meas_filt_vect,
                        _tau_meas_vect, _tau_meas_filt_vect,
                        _K_t_vect,
                        _K_d0_vect, _K_d1_vect,
                        _rot_MoI_vect, _red_ratio_vect, _iq_friction_torque_vect,
                        _iq_friction_torque_cal_vect,
                        _tau_rot_est_vect,
                        _alpha_f0_vect, _alpha_f1_vect,
                        _K_d0_cal_vect, _K_d1_cal_vect,
                        _iq_meas_vect,
                        _iq_meas_filt_vect,
                        _iq_cal_sol_millis_vect;

    std::vector<std::string> _jnt_names, _iq_jnt_names;

    MatLogger2::Ptr _dump_logger;

    // handle adapting ROS primitives for RT support
    RosSupport::UniquePtr _ros;

    // queue object to handle multiple subscribers/servers at once
    CallbackQueue _queue;

    SubscriberPtr<xbot_msgs::CustomState> _aux_signals_sub;
    SubscriberPtr<xbot_msgs::JointState> _js_signals_sub;

    PublisherPtr<awesome_leg::IqEstStatus> _iq_est_pub;
    PublisherPtr<awesome_leg::IqCalStatus> _iq_cal_pub;

    ServiceServerPtr<awesome_leg::IqModelCalibRtRequest,
                     awesome_leg::IqModelCalibRtResponse> _stop_calibration_srv;

    IqRosGetter _iq_getter;
    IqEstimator _iq_estimator;
    IqCalib _iq_calib;

    NumDiff _num_diff;

    MovAvrgFilt _mov_avrg_filter_iq;
    MovAvrgFilt _mov_avrg_filter_tau;
    MovAvrgFilt _mov_avrg_filter_iq_meas;
    MovAvrgFilt _mov_avrg_filter_q_dot;
    int _mov_avrg_window_size_iq = 10;
    double _mov_avrg_cutoff_freq_iq = 15.0;
    int _mov_avrg_window_size_tau = 10;
    double _mov_avrg_cutoff_freq_tau = 15.0;
    int _mov_avrg_window_size_iq_meas = 10;
    double _mov_avrg_cutoff_freq_iq_meas = 15.0;
    int _mov_avrg_window_size_q_dot= 10;
    double _mov_avrg_cutoff_freq_q_dot = 15.0;

    void get_params_from_config();

    void init_vars();
    void init_clocks();
    void init_dump_logger();

    void reset_flags();
    
    void update_state();

    void update_clocks();

    void add_data2dump_logger();
    void add_data2bedumped();

    void spawn_nrt_thread();

    void init_nrt_ros_bridge();

    void is_sim(std::string sim_string);
    void is_dummy(std::string dummy_string);

    void pub_iq_est();
    void pub_iq_cal();

    void run_iq_calib();
    void run_iq_estimation();

    bool on_stop_calib_msg_rec(const awesome_leg::IqModelCalibRtRequest& req,
                               awesome_leg::IqModelCalibRtResponse& res);
                 
};

#endif // JNT_CALIB_RT
