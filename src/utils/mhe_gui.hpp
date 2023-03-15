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

//class MheGui : public QWidget {
//    Q_OBJECT
//public:
//    MheGui(QWidget *parent = nullptr);

//    virtual ~MyWidget();

//public slots:
//    void onSliderMoved(int value);

//    void callback(const std_msgs::Int32::ConstPtr &msg);

//private:
//    ros::NodeHandle nh_;
//    ros::Subscriber sub_;
//    ros::ServiceClient client_;

//    QSlider *slider_;
//};

#endif // MHE_GUI_HPP
