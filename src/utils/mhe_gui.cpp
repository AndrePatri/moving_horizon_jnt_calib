#include <mhe_gui.hpp>

class MheGui : public QWidget
{
    Q_OBJECT

public:
    MheGui(ros::NodeHandle& nh) : nh_(nh)
    {
        // Create the ROS service client
        set_mhe_params_client = nh_.serviceClient<moving_horizon_jnt_calib::SetCalibParams>("set_mhe_cal_params");

        // Create the ROS subscriber
        mhe_status_sub_ = nh_.subscribe("/mhe/jnt_calib_status", 1, &MheGui::mheStateCallback, this);

        // Create the GUI elements
        slider_ = new QSlider(Qt::Horizontal);
        slider_->setRange(0, 100);
        slider_->setValue(50);

        QVBoxLayout* layout = new QVBoxLayout();
        layout->addWidget(slider_);

        setLayout(layout);

        // Connect the slider valueChanged signal to the command update function
        connect(slider_, SIGNAL(valueChanged(int)), this, SLOT(sendMheState(int)));
    }

    ~MheGui() {}

private:
    void sendMheState(std::vector<double> value)
    {
        // Update the command data
        mhe_param_req.request.lambda_ = value;

        // Call the ROS service with the updated command
        set_mhe_params_client.call(mhe_param_req);
    }

    // Callback function called when a new state message is received
    void mheStateCallback(const moving_horizon_jnt_calib::JntCalibStatus::ConstPtr& cal_status)
    {
        // Update the GUI with the new state information
        // ...
    }

    ros::NodeHandle nh_;
    ros::ServiceClient set_mhe_params_client;
    ros::Subscriber mhe_status_sub_;
    moving_horizon_jnt_calib::SetCalibParams mhe_param_req;
    QSlider* slider_;
};

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "mhe_gui_node");
    ros::NodeHandle nh;

    // Create the GUI widget
    MheGui widget(nh);
    widget.show();

    // Run the Qt event loop
    return QApplication::exec();
}
