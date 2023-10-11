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
#include <std_msgs/Bool.h>

#include <moving_horizon_jnt_calib/CalibTrajStatus.h>
#include <moving_horizon_jnt_calib/PerformCalibTraj.h>

#include "ros/ros.h"
#include <ros/callback_queue.h>

#include <time.h>

#include <chrono>
#include <thread>

class StartCalibTraj
{
    public:

        StartCalibTraj(ros::NodeHandle *nh, double pause_time = 0.0)
        :_nh{nh}, _pause_time{pause_time}
        {
            _approach_traj_finished = false;
            _traj_finished = false;

            _cal_traj_srvr_topicname = "";

        }

        void init_client(std::string full_topicname = "/traj_replayer/perform_calib_traj")
        {
            _cal_traj_srvr_topicname = full_topicname;

            _client = _nh->serviceClient<moving_horizon_jnt_calib::PerformCalibTraj>(_cal_traj_srvr_topicname);

            _go2_init_conf.request.perform_calib_traj = false;
            _go2_init_conf.request.go2calib_traj = true;

            _start_cal_traj.request.perform_calib_traj = true;
            _start_cal_traj.request.go2calib_traj = false;

        }

        void start_cal_traj_subscriber()
        {
            _sub = _nh->subscribe("/traj_replayer/calib_traj_status", 1000, &StartCalibTraj::calib_traj_status_callback, this);
        }

        void spin_node()
        {

            ros::spin(); // all callbacks will be called
            //within the spin method, which is blocking

        }

    private:

        bool _approach_traj_finished  = false;
        bool _traj_finished = false;
        bool _traj_started = false;
        bool _traj_sign_already_sent = false;
        bool _approach_sign_already_sent = false;

        bool _pause_started = false;

        double _replay_pause_time = 0.0;
        double _pause_timer = 0.0;

        struct timer
        {
            typedef std::chrono::steady_clock clock ;
            typedef std::chrono::seconds seconds ;

            void reset() { start = clock::now() ; }

            unsigned long long seconds_elapsed() const
            { return std::chrono::duration_cast<seconds>( clock::now() - start ).count() ; }

            private: clock::time_point start = clock::now() ;
        };

        timer _timer;
        double _pause_time = 0.0;

        std::string _cal_traj_srvr_topicname;

        ros::NodeHandle *_nh;
        ros::ServiceClient _client;

        moving_horizon_jnt_calib::PerformCalibTraj _start_cal_traj, _go2_init_conf;

        ros::Subscriber _sub;

        void calib_traj_status_callback(const moving_horizon_jnt_calib::CalibTrajStatus& msg)
        {
            _approach_traj_finished = msg.approach_traj_finished;

            _traj_finished = msg.traj_finished;
            _traj_started = msg.traj_started;

//            print_traj_status();

            if(!_approach_traj_finished && !_traj_finished && !_approach_sign_already_sent)
            { // sending approach trajectory start signal
                if (_client.call(_go2_init_conf))
                {
                    ROS_INFO("\nSending signal for starting approach traj. ...\n");
                }
                else
                {
                    ROS_ERROR("Failed to call service for going to initial configuration.");
                }

                _pause_started = false;

                _approach_sign_already_sent = true;
            }

            if(_approach_traj_finished  && !_traj_finished  && !_traj_sign_already_sent)
            { // sending trajectory start signal
                if (_client.call(_start_cal_traj))
                {
                    ROS_INFO("\nSending signal for starting traj. ...\n");
                }
                else
                {
                    ROS_ERROR("Failed to call service for starting calibration trajectory.");
                }

                _pause_started = false;

                _traj_sign_already_sent = true;
            }

            if(_approach_traj_finished && _traj_started)
            {
                ros::shutdown();
            }

//            if(_approach_traj_finished  && _traj_started && !_pause_started)
//            {
//                _timer.reset() ;
//                _pause_started = true;
//            }

//            if(_pause_started)
//            {
//                double elapsed_seconds = _timer.seconds_elapsed();

//                if( elapsed_seconds > _replay_pause_time)
//                {
//                    _timer.reset() ;
//                    _pause_started = false; // trigger next jump sequence
//                    _approach_sign_already_sent = false;
//                    _traj_sign_already_sent = false;

//                    ROS_INFO("\n Pause ended\n");

//                }
//                else
//                {
//                    ROS_INFO("\n Pause timer: %f s\n", elapsed_seconds);
//                }
//            }

        }

        void print_traj_status()
        {
            std::cout << "" << std::endl;

            ROS_INFO("\n Approach_traj_finished: %i\n", (int)_approach_traj_finished);
            ROS_INFO("\n traj_finished: %i\n", (int)_traj_finished);

            std::cout << "" << std::endl;
        }

};


int main(int argc, char **argv)
{
    std::string node_name = "perform_calib_traj_client";

    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    double pause_time = (argc == 1) ? 3.0 : atof(argv[1]);

    StartCalibTraj start_cyclic_traj = StartCalibTraj(&nh, pause_time);

    start_cyclic_traj.init_client();

    start_cyclic_traj.start_cal_traj_subscriber();

    start_cyclic_traj.spin_node();

    return 0;
}
