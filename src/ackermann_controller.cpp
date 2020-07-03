#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <string>
#include <sstream>
#include <tf/transform_listener.h>
#include <rm3_ackermann_controller/SetKvalues.h>
#include <rm3_ackermann_controller/ActivateController.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <std_srvs/Empty.h>

//filesystem
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <sys/stat.h>

using namespace std;

// Get roll, pitch and yaw
geometry_msgs::Point get_rpy(geometry_msgs::Quaternion quat)
{
    tf::Quaternion q(
        quat.x,
        quat.y,
        quat.z,
        quat.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    geometry_msgs::Point rpy;

    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;

    return rpy;
}

template <typename T>
int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}


class Ackermann_Controller
{

public:

    Ackermann_Controller(   const string &cmd_topic,
                            const string &joint_state_topic,
                            const string &odom_topic,
                            bool lyapunov_enable,
                            const string &lyapunov_topic,
                            const string &trajectory_pts_topic,
                            const string &waypoint_topic,
                            const string &yaw_topic,
                            double threshold_time,
                            double threshold_pose,
                            double threshold_trajectory,
                            double threshold_trajectory_near_waypoint,
                            bool dynamic_trajectory_enable,
                            const ros::NodeHandle &n)
        : m_sub_odom(),
          m_sub_trajectory(),
          m_sub_joint_state(),
          m_sub_waypoint(),
          m_sub_cmd(),
          m_sub_command_pose(),
          m_pub_cmd(),
          m_pub_yaw(),
          m_pub_lyapunov(),
          m_odom_msg(),
          m_joint_state_msg(),
          m_yaw_msg(),
          m_waypoint_msg(),
          m_cmd_msg(),
          m_position_msg(),
          m_serviceSet(),
          m_serviceActivate(),
          r_p(2),
          e_p(2),
          e_p_sat(2),
          r_p_des(2),
          e_p_des(2),
          r_th(0.0),
          e_th(0.0),
          r_th_des(0.0),
          e_th_des(0.0),
          rho(0.0),
          gamma(0.0),
          delta(0.0),
          bar_omega(0.0),
          v(0.0),
          phi(0.0),
          h(1.244),
          K1(1.f),
          K2(6.f),
          K3(3.f),
          isActive(false),
          trajectory_iter_(0),
          m_watchdog(ros::Time::now().toSec()),
          m_trajectory_recived(false),
          m_waypoint_recived(false),
          m_threshold_time(threshold_time),
          m_threshold_pose(threshold_pose),
          m_threshold_trajectory(threshold_trajectory),
          m_threshold_trajectory_near_waypoint(threshold_trajectory_near_waypoint),
          m_dynamic_trajectory_enable(dynamic_trajectory_enable),
          m_state(Idle),
          m_lyapunov_enable(lyapunov_enable),
          m_joint_state_topic(joint_state_topic),
          trajectoy_distances(30) //fixed
    {
        ros::NodeHandle nh;

        ROS_INFO("Ackermann Controller started");

        // odom subscriber
        m_sub_odom = nh.subscribe(odom_topic, 1, &Ackermann_Controller::odomChanged, this);
        m_sub_trajectory = nh.subscribe(trajectory_pts_topic, 1, &Ackermann_Controller::trajectoryChanged, this);

        // joint_state subscriber
        m_sub_joint_state = nh.subscribe(joint_state_topic, 1, &Ackermann_Controller::jointStateChanged, this);

        // waypoint subscriber
        m_sub_waypoint = nh.subscribe(waypoint_topic, 1, &Ackermann_Controller::waypointChanged, this);

        // cmd_vel subscriber
        m_sub_cmd = nh.subscribe(cmd_topic, 1, &Ackermann_Controller::cmdVelChanged, this);

        // command_pose subscriber
        m_sub_command_pose = nh.subscribe("/command/pose", 1, &Ackermann_Controller::commandPoseChanged, this);

        // cmd_vel publisher
        m_pub_cmd = nh.advertise<geometry_msgs::Twist>(cmd_topic, 1);

        // yaw_navigation publisher
        m_pub_yaw = nh.advertise<std_msgs::Float64>(yaw_topic, 1);

        // set K values service
        m_serviceSet = nh.advertiseService("set_k", &Ackermann_Controller::setKvalues, this);

        m_serviceActivate = nh.advertiseService("activate_controller", &Ackermann_Controller::activateController, this);

        if (m_lyapunov_enable)
        {
            // lyapunov publisher
            m_pub_lyapunov = nh.advertise<std_msgs::Float32>(lyapunov_topic, 1);
        }

    }

    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0 / frequency), &Ackermann_Controller::iteration, this);
        ros::spin();
    }

private:

    void iteration(const ros::TimerEvent &e)
    {
        // Disable old messages
        if (!(ros::Time::now().toSec() - m_watchdog < m_threshold_time)){
            m_trajectory_recived=false;
            m_waypoint_recived=false;
        }
        
        // Disable the publication of useless command velocity
        if ((m_trajectory_recived||m_waypoint_recived) && isActive)
        {
            update_values();

            //check if the robot is near desired pose (from waypoint or command_pose topic)
            bool check_threshold=true;
            if (m_trajectory_recived){
                check_threshold = trajectoy_distances(trajectory_iter_) < m_threshold_pose;
            }else if (m_waypoint_recived){
                check_threshold = (r_p - r_p_des).norm() < m_threshold_pose;
            }

            std::cout << "(r_p - r_p_des).norm(): " << (r_p - r_p_des).norm() << std::endl;


            if (check_threshold)
            //if ((r_p - r_p_des).norm() < m_threshold_pose)
            {
                v = 0;
                phi = 0;
            }
            else
            {
                compute_ackermann_to_pose();
            }

            m_cmd_msg.linear.x = fmax(-0.4, fmin(v, 0.4));
            m_cmd_msg.angular.z = fmax(-1.1, fmin(phi, 1.1));
            m_pub_cmd.publish(m_cmd_msg);

            if (m_lyapunov_enable)
            {
                calculate_lyapunov();
                m_pub_lyapunov.publish(m_lyapunov_msg);
            }
        }
        else if (isActive)
        {
            m_cmd_msg.linear.x = 0;
            m_cmd_msg.angular.z = 0;
            m_pub_cmd.publish(m_cmd_msg);
        }
        
    }

    void calculate_lyapunov()
    {

        V1 = pow(m_position_msg.x - m_waypoint_msg.x, 2) + pow(m_position_msg.y - m_waypoint_msg.y, 2);

        V2 = pow(atan2(-(m_position_msg.y - m_waypoint_msg.y), -(m_position_msg.x - m_waypoint_msg.x)) - m_position_msg.z, 2);

        V3 = K3 * pow((atan2(-(m_position_msg.y - m_waypoint_msg.y), -(m_position_msg.x - m_waypoint_msg.x)) - m_position_msg.z) + (m_position_msg.y - m_waypoint_msg.y), 2);

        m_lyapunov_msg.data = V1 + V2 + V3;
    }

    void compute_ackermann_to_pose()
    {

		//std::cout << "current position from odom_topic: " << r_p.transpose() << " " << r_th << std::endl;

        //traslation
        e_p = (r_p_des - r_p);

        //rotation
        e_th = r_th_des - r_th;
        e_th = remainder(e_th, 2.0 * M_PI);

        if (fabs(e_p(0)) < 1e-2)
        {
            e_p_sat(0) = 0;
            //ROS_INFO("e_p(0) SAT: %f",e_p(0));
        }
        else
        {
            e_p_sat(0) = e_p(0);
        }

        if (fabs(e_p(1)) < 1e-2)
        {
            e_p_sat(1) = 0;
            //ROS_INFO("e_p(1) SAT: %f",e_p(1));
        }
        else
        {
            e_p_sat(1) = e_p(1);
        }

        // motion direction
        m_d = sgn(cos(atan2(e_p_sat(1), e_p_sat(0)) - r_th));

        // Calculate rho
        rho = sqrt(pow(e_p(0), 2) + pow(e_p(1), 2));

        // Calculate gamma
        gamma = atan2(e_p_sat(1), e_p_sat(0)) - r_th - (m_d - 1) * M_PI / 2;

        // Calculate delta
        delta = gamma - e_th;

        gamma = remainder(gamma, 2.0 * M_PI); //between PI -PI
        delta = remainder(delta, 2.0 * M_PI); //between PI -PI

        //ROS_INFO("gamma: %f, delta: %f", gamma, delta);

        // Calculate v
        v = K1 * rho * m_d;

        //gamma
        if (gamma == 0.0)
        {
            bar_omega = K2 * gamma + K1 * (sgn(cos(gamma))) * (gamma + K3 * delta);
        }
        else
        {
            bar_omega = K2 * gamma + K1 * (sin(gamma) * (sgn(cos(gamma))) / gamma) * (gamma + K3 * delta);
        }
        bar_omega = bar_omega * h / v;

        // Calculate phi
        //phi = sgn(bar_omega)*acos(1/sqrt(pow(bar_omega,2)+1));
        phi = atan(bar_omega);

        //ROS_INFO("v: %f, phi: %f",v, phi);
    }

    void update_values()
    {

        //Update values
        r_p << m_position_msg.x,
            m_position_msg.y;
        r_th = m_position_msg.z;

        r_p_des << m_waypoint_msg.x,
            m_waypoint_msg.y;
        r_th_des = m_waypoint_msg.z;

    }

    double check_angle(double angle)
    {
        if (angle - 2 * M_PI > -M_PI)
            angle = check_angle(angle - 2 * M_PI);
        else if (angle + 2 * M_PI < M_PI)
            angle = check_angle(angle + 2 * M_PI);
        return angle;
    }

    double check_anglehalf(double angle)
    {
        if (angle - M_PI > -M_PI / 2)
            angle = check_anglehalf(angle - M_PI);
        else if (angle + M_PI < M_PI / 2)
            angle = check_anglehalf(angle + M_PI);

        return angle;
    }

    void updateTrajectoryDistances()
    {
        double trajectoy_distances_prev = 0;
        for(int i=0; i<trajectoy_distances.size(); i++){
            if(i==0){
                trajectoy_distances(i) = (
                                            Eigen::Vector2f(m_position_msg.x, m_position_msg.y) - 
                                            Eigen::Vector2f(trajectory.points[0].positions[i * 3], trajectory.points[0].positions[i * 3 + 1])
                                         ).norm();
            }else{
                trajectoy_distances(i) = trajectoy_distances(i-1) +
                                         (
                                           Eigen::Vector2f(trajectory.points[0].positions[(i-1) * 3], trajectory.points[0].positions[(i-1) * 3 + 1]) - 
                                           Eigen::Vector2f(trajectory.points[0].positions[  i   * 3], trajectory.points[0].positions[  i   * 3 + 1])
                                         ).norm();
            }            
        }
        std::cout << "trajectoy_distances:\n" << trajectoy_distances << std::endl;
    }   


    void trajectoryChanged(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
    {

        trajectory = *msg;
        trajectory_iter_ = 0;

        updateTrajectoryDistances();

        int iter = 0;
        m_waypoint_msg.x = trajectory.points[0].positions[iter * 3];
        m_waypoint_msg.y = trajectory.points[0].positions[iter * 3 + 1];
        m_waypoint_msg.z = trajectory.points[0].positions[iter * 3 + 2];
        trajectory_iter_= iter;

        std::cout << "Iter: " << iter << std::endl;


        bool waypoint_found=false;
        for (iter=1; iter<trajectoy_distances.size(); iter++){
            if (!waypoint_found && // check to set only the first waypoint
                trajectoy_distances(iter) > m_threshold_pose*(1+m_threshold_trajectory)){
                m_waypoint_msg.x = trajectory.points[0].positions[iter * 3];
                m_waypoint_msg.y = trajectory.points[0].positions[iter * 3 + 1];
                m_waypoint_msg.z = trajectory.points[0].positions[iter * 3 + 2];
                trajectory_iter_ = iter;

                std::cout << "Iter: " << iter << std::endl;
                waypoint_found=true;
            }
        }
        

//        do
//        {
//            m_waypoint_msg.x = trajectory.points[0].positions[iter * 3];
//            m_waypoint_msg.y = trajectory.points[0].positions[iter * 3 + 1];
//            m_waypoint_msg.z = trajectory.points[0].positions[iter * 3 + 2];

//            trajectory_iter_= iter;     

//            std::cout << "Iter: " << iter << std::endl;
//            iter++;
//        } while (iter < trajectoy_distances.size() && trajectoy_distances(iter) > m_threshold_pose && m_threshold_trajectory m_threshold_trajectory);

        std::cout << "near_check: " << (Eigen::Vector2f(m_command_pose_recived_msg.pose.pose.position.x, m_command_pose_recived_msg.pose.pose.position.y) - Eigen::Vector2f(m_position_msg.x, m_position_msg.y)).norm() << " " << m_threshold_trajectory_near_waypoint << std::endl;

        if((Eigen::Vector2f(m_command_pose_recived_msg.pose.pose.position.x, m_command_pose_recived_msg.pose.pose.position.y) - Eigen::Vector2f(m_position_msg.x, m_position_msg.y)).norm() < m_threshold_trajectory_near_waypoint){
            std::cout << "near" << std::endl;
            m_waypoint_msg.x = m_command_pose_recived_msg.pose.pose.position.x;
            m_waypoint_msg.y = m_command_pose_recived_msg.pose.pose.position.y;
            m_waypoint_msg.z = get_rpy(m_command_pose_recived_msg.pose.pose.orientation).z;
        }

        m_watchdog = ros::Time::now().toSec();
        m_trajectory_recived = true;

        std::cout << "Waypoint set to: " << " " << m_waypoint_msg.x << " " << m_waypoint_msg.y << " " << m_waypoint_msg.z << std::endl;

        std::cout << "Error(x,y,yaw): " << m_waypoint_msg.x-m_position_msg.x << " " << m_waypoint_msg.y-m_position_msg.y << " " << m_waypoint_msg.z-m_position_msg.z << std::endl;

        std::cout << "Error(Norm): " << (Eigen::Vector2f(m_waypoint_msg.x, m_waypoint_msg.y) - Eigen::Vector2f(m_position_msg.x, m_position_msg.y)).norm() << " " << m_threshold_trajectory << std::endl;        

    }

    void odomChanged(const nav_msgs::Odometry::ConstPtr &msg)
    {
        m_odom_msg = *msg;
        m_position_msg.x = m_odom_msg.pose.pose.position.x;
        m_position_msg.y = m_odom_msg.pose.pose.position.y;

        m_position_msg.z = get_rpy(m_odom_msg.pose.pose.orientation).z;
        
        std::cout << "Odometry set to: " << m_position_msg.x << " " << m_position_msg.y << " " << m_position_msg.z << std::endl;
        
    }

    void cmdVelChanged(const geometry_msgs::Twist::ConstPtr &msg)
    {
        m_cmd_recived_msg = *msg;
    }

    void commandPoseChanged(const nav_msgs::Odometry::ConstPtr &msg)
    {
        m_command_pose_recived_msg = *msg;
    }

    void jointStateChanged(const sensor_msgs::JointState::ConstPtr &msg)
    {
        m_joint_state_msg = *msg;
    }

    void waypointChanged(const geometry_msgs::Point::ConstPtr &msg)
    {
        m_waypoint_msg = *msg;
        m_watchdog = ros::Time::now().toSec();
        m_waypoint_recived = true;

        //std::cout << "Waypoint set to: " << m_waypoint_msg.x << " " << m_waypoint_msg.y << " " << m_waypoint_msg.z << std::endl;
    }

    bool setKvalues(rm3_ackermann_controller::SetKvalues::Request &req,
                    rm3_ackermann_controller::SetKvalues::Response &res)
    {
        K1 = req.k1;
        K2 = req.k2;
        K3 = req.k3;
        res.result = "Succesfully Called setKvalues service!";
        printf("\nAckermann_Controller: k1=%f,k2=%f,k3=%f\n", req.k1, req.k2, req.k3);

        return true;
    }

    bool activateController(rm3_ackermann_controller::ActivateController::Request &req,
                            rm3_ackermann_controller::ActivateController::Response &res)
    {
        isActive = req.is_active;
        res.result = "Succesfully Called ActivateController service!";

        if (isActive)
            printf("\nAckermann_Controller is ACTIVE\n");
        else
            printf("\nAckermann_Controller is NOT ACTIVE\n");

        return true;
    }

private:
    ros::Subscriber m_sub_odom;
    ros::Subscriber m_sub_trajectory;
    ros::Subscriber m_sub_joint_state;
    ros::Subscriber m_sub_waypoint;
    ros::Subscriber m_sub_jointstate;
    ros::Subscriber m_sub_cmd;
    ros::Subscriber m_sub_command_pose;

    ros::Publisher m_pub_cmd;
    ros::Publisher m_pub_yaw;
    ros::Publisher m_pub_lyapunov;

    nav_msgs::Odometry m_odom_msg;
    sensor_msgs::JointState m_joint_state_msg;
    geometry_msgs::Point m_waypoint_msg;
    trajectory_msgs::JointTrajectory trajectory;

    geometry_msgs::Twist m_cmd_recived_msg;
    geometry_msgs::Twist m_cmd_msg;
    nav_msgs::Odometry m_command_pose_recived_msg;
    std_msgs::Float64 m_yaw_msg;
    std_msgs::Float32 m_lyapunov_msg;

    geometry_msgs::Point m_position_msg;

    ros::ServiceServer m_serviceSet;
    ros::ServiceServer m_serviceActivate;

    Eigen::VectorXd r_p;     //x(2);
    Eigen::VectorXd r_p_des; //x_des(2);

    Eigen::VectorXd e_p;     //x(2);
    Eigen::VectorXd e_p_des; //x_des(2);

    Eigen::VectorXd e_p_sat;

    double r_th;
    double r_th_des;

    double e_th;
    double e_th_des;

    double m_d;

    double yaw_estimated;
    double yaw_offset;
    double yaw_imu;
    double yaw_bearing;
    double yaw_inc;

    double rho;
    double gamma;
    double delta;
    double bar_omega;
    double h;
    double v;
    double phi;
    double K1;
    double K2;
    double K3;
    double V1;
    double V2;
    double V3;
    bool isActive;
    int trajectory_iter_;

    double m_threshold_pose;
    double m_threshold_trajectory;

    double m_threshold_trajectory_near_waypoint;
    bool m_dynamic_trajectory_enable;

    Eigen::VectorXd trajectoy_distances;

    bool m_lyapunov_enable;

    double m_watchdog;
    double m_threshold_time;

    bool m_trajectory_recived;
    bool m_waypoint_recived;

    string m_joint_state_topic;

    enum State
    {
        Idle = 0,
        Moving_to_waypoint = 1
    };
    State m_state;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ackermann_controller_node");
    ros::NodeHandle nh("~");

    double frequency;
    nh.param("frequency", frequency, 20.0);

    string cmd_topic;
    nh.param<string>("cmd_topic", cmd_topic, "/base/base_pad/cmd_vel");

    string joint_state_topic;
    nh.param<string>("joint_state_topic", joint_state_topic, "/base/joint_states");

    string odom_topic;
    nh.param<string>("odom_topic", odom_topic, "/ekf_localization_slam_node/slam_odom_magnetic");

    bool lyapunov_enable;
    nh.param("lyapunov_enable", lyapunov_enable, false);

    string lyapunov_topic;
    nh.param<string>("lyapunov_topic", lyapunov_topic, "/lyapunov");

    string trajectory_pts_topic;
    nh.param<string>("trajectory_pts_topic", trajectory_pts_topic, "/sherpa/trajectory_pts");

    string waypoint_topic;
    nh.param<string>("waypoint_topic", waypoint_topic, "/waypoint");

    string yaw_topic;
    nh.param<string>("yaw_topic", yaw_topic, "/yaw_navigation");

    double threshold_time;
    nh.param("threshold_time", threshold_time, 2.0);

    double threshold_pose;
    nh.param("threshold_pose", threshold_pose, 0.20);

    double threshold_trajectory;
    nh.param("threshold_trajectory", threshold_trajectory, 0.30);

    double threshold_trajectory_near_waypoint;
    nh.param("threshold_trajectory_near_waypoint", threshold_trajectory_near_waypoint, 0.35);

    bool dynamic_trajectory_enable;
    nh.param("dynamic_trajectory_enable", dynamic_trajectory_enable, false);

    Ackermann_Controller ackermann_controller(  cmd_topic,
                                                joint_state_topic,
                                                odom_topic,
                                                lyapunov_enable,
                                                lyapunov_topic,
                                                trajectory_pts_topic,
                                                waypoint_topic,
                                                yaw_topic,
                                                threshold_time,
                                                threshold_pose,
                                                threshold_trajectory,
                                                threshold_trajectory_near_waypoint,
                                                dynamic_trajectory_enable,
                                                nh);
    ackermann_controller.run(frequency);

    return 0;
}
