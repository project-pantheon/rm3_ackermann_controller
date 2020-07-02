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
                            const ros::NodeHandle &n)
        : m_sub_odom(),
          m_sub_trajectory(),
          m_sub_joint_state(),
          m_sub_waypoint(),
          m_sub_cmd(),
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
          m_threshold_time(threshold_time),
          m_threshold_pose(threshold_pose),
          m_threshold_trajectory(threshold_trajectory),
          m_state(Idle),
          m_lyapunov_enable(lyapunov_enable),
          m_joint_state_topic(joint_state_topic)
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
//        ROS_INFO("isActive: %d",isActive);
//        ROS_INFO("ros::Time::now().toSec(): %f",ros::Time::now().toSec());
//        ROS_INFO("m_watchdog: %f",m_watchdog);
//        ROS_INFO("m_threshold_time: %f", m_threshold_time);
//        ROS_INFO("time-watch: %f", ros::Time::now().toSec() - m_watchdog);
        
        // Disable the publication of useless command velocity, and publish one time the final lyapunov
        if (ros::Time::now().toSec() - m_watchdog < m_threshold_time && isActive)
        {

            update_values();

            if ((r_p - r_p_des).norm() < m_threshold_pose)
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


    void trajectoryChanged(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
    {

        trajectory = *msg;
        trajectory_iter_ = 0;
        m_waypoint_msg.x = trajectory.points[0].positions[trajectory_iter_ * 3];
        m_waypoint_msg.y = trajectory.points[0].positions[trajectory_iter_ * 3 + 1];
        m_waypoint_msg.z = trajectory.points[0].positions[trajectory_iter_ * 3 + 2];

        int iter = 0;
        while ((Eigen::Vector2f(m_waypoint_msg.x, m_waypoint_msg.y) - Eigen::Vector2f(m_position_msg.x, m_position_msg.y)).norm() < m_threshold_trajectory)
        {
            ++iter;
            m_waypoint_msg.x = trajectory.points[0].positions[iter * 3];
            m_waypoint_msg.y = trajectory.points[0].positions[iter * 3 + 1];
            m_waypoint_msg.z = trajectory.points[0].positions[iter * 3 + 2];
        }

        m_watchdog = ros::Time::now().toSec();

        std::cout << "Waypoint set to: " << iter << " " << m_waypoint_msg.x << " " << m_waypoint_msg.y << " " << m_waypoint_msg.z << std::endl;
    }

    void odomChanged(const nav_msgs::Odometry::ConstPtr &msg)
    {
        m_odom_msg = *msg;
        m_position_msg.x = m_odom_msg.pose.pose.position.x;
        m_position_msg.y = m_odom_msg.pose.pose.position.y;

        m_position_msg.z = get_rpy(m_odom_msg.pose.pose.orientation).z;
        
        std::cout << "Odom set to: " << m_position_msg.x << " " << m_position_msg.y << " " << m_position_msg.z << std::endl;
        
    }

    void cmdVelChanged(const geometry_msgs::Twist::ConstPtr &msg)
    {
        m_cmd_recived_msg = *msg;
    }

    void jointStateChanged(const sensor_msgs::JointState::ConstPtr &msg)
    {
        m_joint_state_msg = *msg;
    }

    void waypointChanged(const geometry_msgs::Point::ConstPtr &msg)
    {
        m_waypoint_msg = *msg;
        m_watchdog = ros::Time::now().toSec();

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

    ros::Publisher m_pub_cmd;
    ros::Publisher m_pub_yaw;
    ros::Publisher m_pub_lyapunov;

    nav_msgs::Odometry m_odom_msg;
    sensor_msgs::JointState m_joint_state_msg;
    geometry_msgs::Point m_waypoint_msg;
    trajectory_msgs::JointTrajectory trajectory;

    geometry_msgs::Twist m_cmd_recived_msg;
    geometry_msgs::Twist m_cmd_msg;
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

    bool m_lyapunov_enable;

    double m_watchdog;
    double m_threshold_time;

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
                                                nh);
    ackermann_controller.run(frequency);

    return 0;
}
