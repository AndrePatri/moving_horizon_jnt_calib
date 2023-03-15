#ifndef MHE_GUI_HPP
#define MHE_GUI_HPP

#include <ros/ros.h>
#include <ros/service_client.h>
#include <ros/subscriber.h>
#include <QWidget>
#include <QSlider>
#include <QVBoxLayout>
#include <QApplication>

#include <moving_horizon_jnt_calib/SetCalibParams.h>
#include <moving_horizon_jnt_calib/JntCalibStatus.h>

class MheGui : public QWidget {

public:
    MheGui(ros::NodeHandle& nh);

public:
    void sendMheState(std::vector<double> value);

    void mheStateCallback(const moving_horizon_jnt_calib::JntCalibStatus::ConstPtr& cal_status);

private:
    ros::NodeHandle nh_;
    ros::ServiceClient set_mhe_params_client;
    ros::Subscriber mhe_status_sub_;
    moving_horizon_jnt_calib::SetCalibParams mhe_param_req;
    QSlider* slider_;
};

#endif // MHE_GUI_HPP
