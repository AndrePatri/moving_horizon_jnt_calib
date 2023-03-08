#include <vector>
#include <signal.h>
#include <future>
#include <chrono>
#include <functional>

#include <sys/wait.h>
#include <sys/select.h>
#include <sys/fcntl.h>

#include <ros/ros.h>
#include <xbot_msgs/JointState.h>
#include <xbot_msgs/CustomState.h>

#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <eigen_conversions/eigen_msg.h>

#include <XBotInterface/XBotInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <matlogger2/matlogger2.h>
#include <matlogger2/utils/mat_appender.h>
#include <matlogger2/mat_data.h>

#include <map>

using namespace std;

bool g_msg_received  = false;
bool g_needs_restart = false;

XBot::MatLogger2::Ptr g_logger;
XBot::MatAppender::Ptr g_appender;
XBot::XBotInterface * g_model;

volatile sig_atomic_t g_run = 1; // 0 only if SIGINT was received

volatile int aux_types_encode_number = 0; // global counter used to assign incremental values to aux signal types

volatile bool is_first_aux_msg= true; // auxiliary variable to know if an aux message was already received or not
vector<int> indices; // global variable for holding the mapping

// Empty map container for encoding (dynamically) the aux message types
std::map<string, int> aux_msg_type_map;
std::vector<string> js_names; // auxiliary vector where the joint names (as visible in the js message) are saved.

void sigint_handler(int)
{
    printf("caught signal SIGINT\n");
    g_run = 0;
}

void logger_sigint_handler(int)
{
    printf("logger: caught signal sigint\n");
    ros::shutdown();
}

bool check_host_reachable()
{
    std::string command = "ping -c 1 -W 1 " + ros::master::getHost() + " 1>/dev/null";
    return system(command.c_str()) == 0;
}

template <typename T, typename t_v >
int find_index(vector<T> input_v, t_v value)
{
    /**
    Finds the index of a value in an array.

    @param input_v Input vector.
    @param value Value to be searched within the input vector.

    @return  The index of the element (-1 if not present).
    */

    auto it = find(input_v.begin(), input_v.end(), value);
 
    // If element was found
    if (it != input_v.end())
    {
        // calculating the index
        int index = it - input_v.begin();
        return index;
    }
    else 
    {
        // The element is not present in the vector
        return -1;
    }
}

template <typename T>
vector<int> map_indices(vector<T> input_v1, vector<T> input_v2)
{
    /**
    Maps the elements of the first input array to the second.

    @param input_v1 First input vector.
    @param input_v2 Second input vector.

    @return  A vector of indices. indices_map[i] contains the index where the i-th element of input_v1 is present in input_v2.

    */
    
    int v1_size = input_v1.size(); //initialize output
    vector<int> indices_map(v1_size, -1);

    // Here insert a check (size of input_v1 must equal size of input_v2)
    for (int i = 0; i < input_v1.size(); i++)
    {
        indices_map[i] = find_index(input_v2, input_v1[i] );
    }
    return indices_map;
}

int aux_type_encoder(string msg_type)
{
    /**
    Computes the code ID associated with a given message type and saves this code to the .mat file, 
    so that other programs can interpret the signal-type information. To avoid the use of strings, a unique _aux_code suffix is employed.

    @param msg_type The message type name (std::string).
    @return The message code to be associated with the message type name
    */

    int msg_code;

    if (aux_msg_type_map.find(msg_type) == aux_msg_type_map.end()) // msg_type not present
    {   
        aux_types_encode_number++; // increment last signal code number by 1
        aux_msg_type_map.insert({msg_type, aux_types_encode_number}); // put the code in the dictionary
        g_logger -> add(msg_type + "_aux_code", aux_types_encode_number); // add the pair key-value associated with this signal to the .mat file for post-processing
    }

    msg_code = aux_msg_type_map.at(msg_type); // read the key associated with the msg_type from the aux code map

    return msg_code;
}

auto aux_mapper(xbot_msgs::CustomStateConstPtr msg)
{

    /**
    Loops through the chains and their joints and, based on the received message, assigns the IDs associated with each message type.

    @param msg Input message
    @return A vector of signal IDs with dimension (number of chains)*(number of joints)
    */

    int n_names = msg->name.size(); // number of joints

    std::vector<float> msg_value_remapped(n_names, -1); // output vector for msg values 
    std::vector<int> msg_type_remapped(n_names, -1); // output vector for msg types 

    if (is_first_aux_msg)
    {
        indices = map_indices(js_names, msg->name); // this runs only the first time an aux message is received (joint mapping is assumed to be constant throughout a session)
      
        is_first_aux_msg = false; // mapping not needed anymore 
    }

    for (int i = 0; i < n_names; i++) // mapping
    {
        int encoded_type = aux_type_encoder(msg->type[i]);
        msg_type_remapped[indices[i]] = encoded_type;
        msg_value_remapped[indices[i]] = msg->value[i];
    }

    return make_tuple(msg_type_remapped, msg_value_remapped);
}

void log_chain_ids(xbot_msgs::JointStateConstPtr msg)
{
    /**
    Log information of the chain IDs (the joint IDs associated with each kinematic chain are saved in a .mat file under the name of the chain itself).

    @param msg Pointer to the message.

    */
    ros::V_string ch_names;

    for(const auto& pair : g_model->getChainMap()) // slide through each model chain
    {
        vector<int> ch_idx;
        bool at_least1_jnt_found = false; // whether or not at least one joint from the msg was found in this chain

        for(const auto& jname : pair.second->getJointNames()) // slide through each joint name of the chain
        {
            auto it = std::find(msg->name.begin(), msg->name.end(), jname); 
            if(it != msg->name.end()) // found joint name of the robot in the received joint state message
            {
                ch_idx.push_back(it - msg->name.begin() + 1); // NOTE MATLAB 1-BASED INDEXING
                
                if (!at_least1_jnt_found){at_least1_jnt_found=true;}
                
            }
        }

        if (at_least1_jnt_found){ch_names.push_back(pair.first);} // only add chain to .mat if at least one joint is associated with it (normally this check wouldn't be necessary)

        g_logger->add(pair.first, ch_idx);
    }
    
    js_names = msg->name; // assigning the received joint names to an auxiliary global vector (joint names and their order do not change by hypothesis)

    auto jnames_cell = XBot::matlogger2::MatData::make_cell(js_names.size());
    auto chnames_cell = XBot::matlogger2::MatData::make_cell(ch_names.size());

    jnames_cell.asCell().assign(js_names.begin(), js_names.end());
    chnames_cell.asCell().assign(ch_names.begin(), ch_names.end());

    g_logger->save("joint_names", jnames_cell);
    g_logger->save("chain_names", chnames_cell);
}

void on_js_received(xbot_msgs::JointStateConstPtr msg)
{
    /**
    If a joint_states message is received, log the neccesary information to a .mat file.

    @param msg Input message
    */

    if(!g_logger)
    {
        XBot::MatLogger2::Options opt;
        opt.enable_compression = true;
        opt.default_buffer_size = 1000;

        std::string mat_dump_path;
        bool got_param = ros::param::get("/xbot_logger/mat_dump_path", mat_dump_path);
        if (!got_param)
        {   
            g_logger = XBot::MatLogger2::MakeLogger("/tmp/robot_state_log", opt);
        }
        else
        {
            g_logger = XBot::MatLogger2::MakeLogger(mat_dump_path + "/robot_state_log", opt);
        }

        g_appender->add_logger(g_logger);
        
        log_chain_ids(msg);
    }
    
    g_logger->add("link_position", msg->link_position);
    g_logger->add("motor_position", msg->motor_position);
    g_logger->add("link_velocity", msg->link_velocity);
    g_logger->add("motor_velocity", msg->motor_velocity);
    g_logger->add("stiffness", msg->stiffness);
    g_logger->add("damping", msg->damping);
    g_logger->add("effort", msg->effort);
    g_logger->add("position_reference", msg->position_reference);
    g_logger->add("velocity_reference", msg->velocity_reference);
    g_logger->add("effort_reference", msg->effort_reference);
    g_logger->add("fault", msg->fault);
    g_logger->add("temperature_driver", msg->temperature_driver);
    g_logger->add("temperature_motor", msg->temperature_motor);
    g_logger->add("time", msg->header.stamp.toSec());
    g_logger->add("seq", msg->header.seq);

    g_msg_received = true;
}

void on_aux_received(xbot_msgs::CustomStateConstPtr msg)
{
    /**
    If a aux message is received, log the neccesary information to a .mat file.

    @param msg Input message
    */
    
    if(!g_logger)
    {
        return;
    }
    
    auto remapped_tuple = aux_mapper(msg); // remapping aux types

    g_logger->add("aux_time", msg->header.stamp.toSec());
    g_logger->add("aux_seq", msg->header.seq);
    g_logger->add("aux_type", std::get<0>(remapped_tuple));
    g_logger->add("aux_value", std::get<1>(remapped_tuple));

}

void on_ft_received(geometry_msgs::WrenchStampedConstPtr msg, std::string name)
{
    if(!g_logger)
    {
        return;
    }

    Eigen::Vector6d w;
    tf::wrenchMsgToEigen(msg->wrench, w);

    g_logger->add(name + "_wrench", w);
    g_logger->add(name + "_ts", msg->header.stamp.toSec());
}

void on_imu_received(sensor_msgs::ImuConstPtr msg, std::string name)
{
    if(!g_logger)
    {
        return;
    }

    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(msg->orientation, q);

    Eigen::Vector3d omega;
    tf::vectorMsgToEigen(msg->angular_velocity, omega);

    Eigen::Vector3d acc;
    tf::vectorMsgToEigen(msg->linear_acceleration, acc);

    g_logger->add(name + "_rot", q.coeffs());
    g_logger->add(name + "_omega", omega);
    g_logger->add(name + "_lin_acc", acc);
    g_logger->add(name + "_ts", msg->header.stamp.toSec());

}

void on_timer_event(const ros::WallTimerEvent& event, 
                    std::function<void(void)> heartbeat)
{
    heartbeat();
    
    if(!g_logger && (!check_host_reachable() || !ros::master::check()))
    {
        g_logger.reset();
        heartbeat();
        std::cout << "Lost connection with master" << std::endl;
        g_needs_restart = true;
        ros::shutdown();
    }
    
    if(g_logger && !g_msg_received)
    {
        g_logger.reset();
        heartbeat();
        std::cout << "No message received from joint states" << std::endl;
        g_needs_restart = true;
        ros::shutdown();
    }
    
    g_msg_received = false;
    
}

int logger_main(int argc, char** argv, 
                std::function<void(void)> heartbeat)
{
    signal(SIGINT, logger_sigint_handler);
    
    ros::init(argc, argv,
              "robot_state_logger",
              ros::init_options::NoSigintHandler
              );
    
    std::cout << "Waiting for ROS master..." << std::endl;
    
    for(;;)
    {
        heartbeat();
        
        if(!ros::ok() || ros::isShuttingDown())
        {
            return EXIT_SUCCESS;
        }
        
        if(!check_host_reachable())
        {
            printf("Host '%s' is unreachable\n", ros::master::getHost().c_str());
            continue;
        }
        
        if(!ros::master::check())
        {
            printf("Ros master at '%s' is unreachable\n", ros::master::getURI().c_str());
            sleep(1);
            continue;
        }
        
        //         try
        //         {
        //             XBot::ConfigOptionsFromParamServer();
        //         }
        //         catch(std::exception& e)
        //         {
        //             printf("Unable to obtain model (%s)\n", e.what());
        //             sleep(1);
        //             continue;
        //         }
        
        break;
    }
    
    std::cout << "Connected to ROS master!" << std::endl;

    ros::NodeHandle nh("xbotcore");
    
    auto cfg = XBot::ConfigOptionsFromParamServer(nh);
    XBot::XBotInterface xbi;
    xbi.init(cfg);
    g_model = &xbi;
    

    std::vector<ros::Subscriber> subs;

    for(auto ft : xbi.getForceTorque())
    {
        auto ft_sub = nh.subscribe<geometry_msgs::WrenchStamped>(
                    "ft/" + ft.first, 15,
                    std::bind(on_ft_received, std::placeholders::_1, ft.first));
        subs.push_back(ft_sub);
    }

    for(auto imu : xbi.getImu())
    {
        auto imu_sub = nh.subscribe<sensor_msgs::Imu>(
                    "imu/" + imu.first, 15,
                    std::bind(on_imu_received, std::placeholders::_1, imu.first));
        subs.push_back(imu_sub);
    }
    
    ros::Subscriber js_sub =  nh.subscribe("joint_states",
                                           15,
                                           on_js_received);

    ros::Subscriber aux_sub =  nh.subscribe("aux",
                                           15,
                                           on_aux_received);                                       

    ros::WallTimer timer = nh.createWallTimer(ros::WallDuration(0.5),
                                              boost::bind(on_timer_event, _1, heartbeat));
    

    g_appender = XBot::MatAppender::MakeInstance();
    g_appender->start_flush_thread();
    timer.start();

    printf("Started spinning.. \n");
    
    ros::spin();
    
    g_appender.reset();
    g_logger.reset();
    
    printf("logger exiting with return code %d\n", g_needs_restart);

    return g_needs_restart;
    
}

int main(int argc, char **argv)
{
    signal(SIGINT, sigint_handler);
    
    bool restart = true;
    int logger_pid = -1;
    
    while(restart && g_run)
    {
        /* If logger pid is valid, kill it */
        if(logger_pid > 0 && waitpid(logger_pid, nullptr, WNOHANG) == 0)
        {
            printf("killing logger (pid %d)\n", logger_pid);
            kill(logger_pid, SIGKILL);
            printf("waiting logger exit..\n");
            waitpid(logger_pid, nullptr, 0);
        }
        
        /* Create pipe */
        int pipe_fd[2];
        if(pipe(pipe_fd) == -1)
        {
            perror("pipe");
        }
        auto heartbeat = [pipe_fd]()
        {
            int alive = 0;
            if(write(pipe_fd[1], &alive, sizeof(alive)) < 0)
            {
                perror("write");
            }
        };
        
        /* Fork the logger */
        logger_pid = fork();
        if(logger_pid == -1)
        {
            perror("fork");
        }
        
        if(logger_pid == 0)
        {
            close(pipe_fd[0]);
            return logger_main(argc, argv, heartbeat);
        }
        
        /* Parent process */
        close(pipe_fd[1]);
        fcntl(pipe_fd[0], F_SETFL, O_NONBLOCK);
        
        while(g_run)
        {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(pipe_fd[0], &fds);
            int max_fd = pipe_fd[0];
            
            timeval timeout;
            timeout.tv_sec = 2;
            timeout.tv_usec = 0;
            
            int ret = select(max_fd + 1, &fds, nullptr, nullptr, &timeout);
            
            if(ret < 0)
            {
                perror("select");
            }
            
            int status = -1;
            int pid_ret = waitpid(logger_pid, &status, WNOHANG);
            
            if(pid_ret == logger_pid && WIFEXITED(status))
            {
                int ret_code = WEXITSTATUS(status);
                printf("logger exited with return code %d\n", ret_code);
                logger_pid = -1;
                restart = ret != 0;
                break;
            }
            
            if(pid_ret == logger_pid && WIFSIGNALED(status))
            {
                int sig = WTERMSIG(status);
                printf("logger died from signal %d\n", sig);
                logger_pid = -1;
                restart = true;
                break;
            }
            
            if(ret > 0) // message received
            {
                int value = -1;
                if(read(pipe_fd[0], &value, sizeof(value)) < 0)
                {
                    perror("read");
                }
            }
            else // no message received
            {
                printf("logger is not alive\n");
                restart = true;
                break;
            }
        }
        
        close(pipe_fd[0]);
        
    }
    
    printf("waiting logger exit..\n");
    waitpid(logger_pid, nullptr, 0);
    printf("main exiting..\n");

    return 0;
    
    
}
