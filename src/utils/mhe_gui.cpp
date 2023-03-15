#include <mhe_gui.hpp>


MheGui::MheGui(ros::NodeHandle& nh)
{

  // Create the ROS service client
  set_mhe_params_client = nh_.serviceClient<moving_horizon_jnt_calib::SetCalibParams>("set_mhe_cal_params");

  // Create the ROS subscriber
  mhe_status_sub_ = nh_.subscribe("/mhe/jnt_calib_status", 1, &MheGui::mheStateCallback, this);

  // Create the GUI elements
  slider_lambda_ = new QSlider(Qt::Horizontal);
  slider_lambda_->setRange(0, 100);
  slider_lambda_->setValue(50);
  slider_lambda_->setSingleStep(1);

  QLabel *lb = new QLabel("0", this);
  QLabel *ub = new QLabel("100", this);

  slider_lambda_high_ = new QSlider(Qt::Horizontal);
  slider_lambda_high_->setRange(0, 100);
  slider_lambda_high_->setValue(50);

//  QVBoxLayout* layout = new QVBoxLayout();
  QGridLayout* layout = new QGridLayout;

  layout->addWidget(slider_lambda_, 0, 0, 1, 4);
  layout->addWidget(lb, 1, 0, 1, 1);
  layout->addWidget(ub, 1, 4, 1, 1);
  layout->addWidget(slider_lambda_high_, 2, 0, 1, 4);

  setLayout(layout);

  connect(slider_lambda_, SIGNAL(valueChanged(int)), this, SLOT(sendMheState(std::vector<double> value)));
  connect(slider_lambda_high_, SIGNAL(valueChanged(int)), this, SLOT(sendMheState(std::vector<double> value)));

}

void MheGui::sendMheState(std::vector<double> value)
{
    // Update the command data
    mhe_param_req.request.lambda_ = value;

    // Call the ROS service with the updated command
    set_mhe_params_client.call(mhe_param_req);
}

void MheGui::mheStateCallback(const moving_horizon_jnt_calib::JntCalibStatus::ConstPtr& cal_status)
{
    // Update the GUI with the new state information
    // ...
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "mhe_gui_node");
    ros::NodeHandle nh;

    QApplication app(argc, argv);

    // Create the GUI widget
    MheGui widget(nh);
    widget.show();

    // Run the Qt event loop
    return app.exec();
}
