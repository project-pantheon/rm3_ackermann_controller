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

#define BEARING_LINEAR 0.49
#define BEARING_ANGULAR 0.05

using namespace std;

enum Mode
{
    ENCODER = 1,
    GPS_BEARING = 2,
    GPS_IMU = 3,
    EKF = 4,
    GAZEBO = 5
};


// set default joint_state topic
string check_joint_state_topic(Mode mode, string joint_state_topic)
{
    if (strcmp(joint_state_topic.c_str(), "default") == 0)
    {
        switch (mode)
        {
        case ENCODER:
        {
            joint_state_topic = "/base/joint_states";
        }
        break;
        case GPS_BEARING:
        {
            joint_state_topic = "/base/joint_states";
        }
        break;
        case GPS_IMU:
        {
            joint_state_topic = "/base/joint_states";
        }
        break;
        case EKF:
        {
            joint_state_topic = "/base/joint_states";
        }
        break;
        case GAZEBO:
        {
            joint_state_topic = "/sherpa/joint_states";
        }
        break;
        }
    }
    return joint_state_topic;
}

// set default odom topic
string check_odom_topic(Mode mode, string odom_topic)
{
    if (strcmp(odom_topic.c_str(), "default") == 0)
    {
        switch (mode)
        {
        case ENCODER:
        {
            odom_topic = "/base/odom";
        }
        break;
        case GPS_BEARING:
        {
            odom_topic = "/gps";
        }
        break;
        case GPS_IMU:
        {
            odom_topic = "/odom_gps";
        }
        break;
        case EKF:
        {
            odom_topic = "/odom";
        }
        break;
        case GAZEBO:
        {
            odom_topic = "/odom";
        }
        break;
        }
    }
    return odom_topic;
}

// set default cmd topic
string check_cmd_topic(Mode mode, string cmd_topic)
{
    if (strcmp(cmd_topic.c_str(), "default") == 0)
    {
        switch (mode)
        {
        case ENCODER:
        {
            cmd_topic = "/base/base_pad/cmd_vel";
        }
        break;
        case GPS_BEARING:
        {
            cmd_topic = "/base/base_pad/cmd_vel";
        }
        break;
        case GPS_IMU:
        {
            cmd_topic = "/base/base_pad/cmd_vel";
        }
        break;
        case EKF:
        {
            cmd_topic = "/base/base_pad/cmd_vel";
        }
        break;
        case GAZEBO:
        {
            cmd_topic = "/sherpa/akrm_cmd";
        }
        break;
        }
    }
    return cmd_topic;
}

// Get time
string get_time()
{
    time_t t = time(nullptr);
    auto tm = *localtime(&t);

    ostringstream oss;
    oss << put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    auto str = oss.str();

    return str;
}

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

// Get home directory
std::string get_homeDirectory()
{
    struct passwd *pw = getpwuid(getuid());
    return pw->pw_dir;
}

class Ackermann_Controller
{

public:
    Ackermann_Controller(Mode mode,
                         const string &odom_topic,
                         const string &imu_topic,
                         const string &joint_state_topic,
                         const string &waypoint_topic,
                         const string &cmd_topic,
                         const string &yaw_topic,
                         double threshold,
                         double threshold_time,
                         bool lyapunov_enable,
                         double error,
                         int history_steps,
                         bool enable_yaw_estimation,
                         const ros::NodeHandle &n)
        : m_mode(mode),
          m_sub_odom(),
          m_sub_trajectory(),
        //   m_sub_imu(),
          m_sub_joint_state(),
          m_sub_waypoint(),
          m_sub_cmd(),
          m_pub_cmd(),
          m_pub_yaw(),
          m_pub_lyapunov(),
          m_pub_yaw_estimated(),
          m_pub_yaw_offset(),
          m_odom_msg(),
        //   m_imu_msg(),
          m_joint_state_msg(),
          m_yaw_msg(),
          m_waypoint_msg(),
          m_cmd_msg(),
          m_position_msg(),
          m_serviceSet(),
          m_serviceCalibration(),
          m_serviceFixOffset(),
          m_serviceActivate(),
          m_threshold(threshold),
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
          yaw_inc(0.01),
          yaw_offset(0.0),
          isActive(false),
          trajectory_pts_(false),
          trajectory_iter_(0),
          m_watchdog(ros::Time::now().toSec()),
          m_threshold_time(threshold_time),
          m_state(Idle),
          m_lyapunov_enable(lyapunov_enable),
          m_error(error),
          m_history_steps(history_steps),
          m_history_cnt(0),
          m_enable_yaw_estimation(enable_yaw_estimation),
          calibrationActive(false),
          offsetFixed(false),
          m_joint_state_topic(joint_state_topic)
    {
        ros::NodeHandle nh;

        ROS_INFO("Ackermann Controller started");

        // odom subscriber
        m_sub_odom = nh.subscribe(odom_topic, 1, &Ackermann_Controller::odomChanged, this);
        m_sub_trajectory = nh.subscribe("/sherpa/trajectory_pts", 1, &Ackermann_Controller::trajectoryChanged, this);

        // // orientation source
        // if (m_mode == GPS_IMU)
        // {
        //     // imu subscriber
        //     m_sub_imu = nh.subscribe(imu_topic, 1, &Ackermann_Controller::imuChanged, this);
        // }

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
        m_serviceCalibration = nh.advertiseService("calibration_controller", &Ackermann_Controller::calibration, this);
        m_serviceFixOffset = nh.advertiseService("fix_offset_controller", &Ackermann_Controller::fixOffset, this);

        m_serviceActivate = nh.advertiseService("activate_controller", &Ackermann_Controller::activateController, this);

        if (m_lyapunov_enable)
        {
            // lyapunov publisher
            m_pub_lyapunov = nh.advertise<std_msgs::Float32>("/lyapunov", 1);
        }

        if (m_enable_yaw_estimation)
        {
            m_pub_yaw_estimated = nh.advertise<std_msgs::Float64>("/yaw_estimated", 1);
            m_pub_yaw_offset = nh.advertise<std_msgs::Float64>("/yaw_offset", 1);
        }
    }

    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0 / frequency), &Ackermann_Controller::iteration, this);
        ros::spin();
    }

private:
    void updateTrajectoryWaypoint()
    {

        for (unsigned int iter = 0; iter < trajectory.points[0].positions.size() / 3 - 1; ++iter)
        {
            if ((Eigen::Vector2f(m_waypoint_msg.x, m_waypoint_msg.y) - Eigen::Vector2f(m_position_msg.x, m_position_msg.y)).norm() < 2e-1)
            {
                trajectory_iter_++;
                m_waypoint_msg.x = trajectory.points[0].positions[trajectory_iter_ * 3];
                m_waypoint_msg.y = trajectory.points[0].positions[trajectory_iter_ * 3 + 1];
                m_waypoint_msg.z = trajectory.points[0].positions[trajectory_iter_ * 3 + 2];
                std::cout << "Waypoint set to: " << m_waypoint_msg.x << " " << m_waypoint_msg.y << " " << m_waypoint_msg.z << "\n";
            }
        }
    }

    void iteration(const ros::TimerEvent &e)
    {
        // Disable the publication of useless command velocity, and publish one time the final lyapunov
        if (ros::Time::now().toSec() - m_watchdog < m_threshold_time && isActive)
        {

            //if(trajectory_pts_)
            //    updateTrajectoryWaypoint();

            // Update values
            update_values();
            //calculateKvalues();

            //add offset or generate it if calibration is enable
            calculate_yaw_estimated();

            //use yaw_imu + offset
            r_th = yaw_estimated;

            if ((r_p - r_p_des).norm() < 5e-2)
            {
                v = 0;
                phi = 0;
            }
            else
            {
                compute_ackermann_to_pose();
            }

            m_cmd_msg.linear.x = fmax(-0.5, fmin(v, 0.5));
            m_cmd_msg.angular.z = fmax(-1.1, fmin(phi, 1.1));
            m_pub_cmd.publish(m_cmd_msg);

            if (m_lyapunov_enable)
            {
                calculate_lyapunov();
                m_pub_lyapunov.publish(m_lyapunov_msg);
            }
        }
        else if (isActive && !calibrationActive)
        {
            m_cmd_msg.linear.x = 0;
            m_cmd_msg.angular.z = 0;
            m_pub_cmd.publish(m_cmd_msg);
        }
        else if (calibrationActive)
        {
            update_values();
            calculate_yaw_estimated();
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

        if (calibrationActive)
            update_history();
    }

    void update_history()
    {
        if (check_vel_for_bearing())
        {
            //check if the queue is full
            if (m_history_cnt < m_history_steps)
            {
                m_history_cnt++;
            }
            else
            {
                //remove the first position
                m_position_history.erase(m_position_history.begin(), m_position_history.begin() + 1);
            }
            //add last position
            m_position_history.push_back(m_position_msg);
        }
        else
        {
            m_position_history.clear();
            m_history_cnt = 0;
        }
    }

    bool check_vel_for_bearing()
    {
        //straight line
        return (abs(m_cmd_recived_msg.linear.x) > BEARING_LINEAR && abs(m_cmd_recived_msg.angular.z) < BEARING_ANGULAR);
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

    void calculate_yaw_estimated()
    {
        yaw_imu = m_position_msg.z;
        if (calibrationActive && !offsetFixed && m_history_cnt == m_history_steps)
        {
            calculate_yaw_bearing();
            yaw_offset = max(yaw_offset - yaw_inc, min(yaw_offset + yaw_inc, yaw_bearing - yaw_imu));
        }
        yaw_estimated = yaw_imu + yaw_offset;

        m_yaw_estimated_msg.data = yaw_estimated;
        m_yaw_offset_msg.data = yaw_offset;
        m_pub_yaw_estimated.publish(m_yaw_estimated_msg);
        m_pub_yaw_offset.publish(m_yaw_offset_msg);
    }

    void calculate_yaw_bearing()
    {
        double right, left, center, yaw_gps;
        geometry_msgs::Point position_prec;

        if (strcmp(m_joint_state_topic.c_str(), "/sherpa/joint_states") == 0)
        {
            left = m_joint_state_msg.position[2];
            right = m_joint_state_msg.position[5];
        }else{
            left = m_joint_state_msg.position[2];
            right = m_joint_state_msg.position[4];
        }

        position_prec = m_position_history[m_history_steps];
        center = (0.55 * right + 0.45 * left);
        yaw_gps = atan2((m_position_msg.y - position_prec.y), (m_position_msg.x - position_prec.x));
        yaw_estimated = max(yaw_estimated - yaw_inc, min(yaw_estimated + yaw_inc, yaw_gps - center));
    }

    // calibration on/off
    bool calibration(
        std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &res)
    {
        if (!offsetFixed)
        {
            calibrationActive = !calibrationActive;
            ROS_INFO("calibration %s", calibrationActive ? "active" : "not active");
        }
        else
        {
            ROS_INFO("This action required a not fixed offset");
        }
        return 1;
    }

    // fix_offset
    bool fixOffset(
        std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &res)
    {
        offsetFixed = !offsetFixed;
        if (offsetFixed)
        {
            calibrationActive = false;
            ROS_INFO("calibration not active");
        }
        ROS_INFO("offset %s", offsetFixed ? "fixed" : "not fixed");
        return 1;
    }

    void trajectoryChanged(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
    {

        trajectory = *msg;
        trajectory_iter_ = 0;
        m_waypoint_msg.x = trajectory.points[0].positions[trajectory_iter_ * 3];
        m_waypoint_msg.y = trajectory.points[0].positions[trajectory_iter_ * 3 + 1];
        m_waypoint_msg.z = trajectory.points[0].positions[trajectory_iter_ * 3 + 2];

        int iter = 0;
        while ((Eigen::Vector2f(m_waypoint_msg.x, m_waypoint_msg.y) - Eigen::Vector2f(m_position_msg.x, m_position_msg.y)).norm() < 1e-1)
        {
            ++iter;
            m_waypoint_msg.x = trajectory.points[0].positions[iter * 3];
            m_waypoint_msg.y = trajectory.points[0].positions[iter * 3 + 1];
            m_waypoint_msg.z = trajectory.points[0].positions[iter * 3 + 2];
        }

        m_watchdog = ros::Time::now().toSec();
        trajectory_pts_ = true;
        std::cout << "Waypoint set to: " << iter << " " << m_waypoint_msg.x << " " << m_waypoint_msg.y << " " << m_waypoint_msg.z << "\n";
    }

    void odomChanged(const nav_msgs::Odometry::ConstPtr &msg)
    {
        m_odom_msg = *msg;
        m_position_msg.x = m_odom_msg.pose.pose.position.x;
        m_position_msg.y = m_odom_msg.pose.pose.position.y;

        m_position_msg.z = get_rpy(m_odom_msg.pose.pose.orientation).z;
        m_position_msg.z = m_position_msg.z + m_error;

    }

    // void imuChanged(const sensor_msgs::Imu::ConstPtr &msg)
    // {
    //     m_imu_msg = *msg;
    //     switch (m_mode)
    //     {
    //     case GPS_IMU:
    //     {
    //         m_position_msg.z = get_rpy(m_imu_msg.orientation).z;
    //     }
    //     break;
    //     default:
    //     {
    //         //nothing
    //     }
    //     break;
    //     }
    // }

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
    }

    bool setKvalues(rm3_ackermann_controller::SetKvalues::Request &req,
                    rm3_ackermann_controller::SetKvalues::Response &res)
    {

        K1 = req.k1;
        K2 = req.k2;
        K3 = req.k3;
        res.result = "Succesfully Called setKvalues service!\n";
        printf("\nAckermann_Controller: k1=%f,k2=%f,k3=%f\n", req.k1, req.k2, req.k3);

        return true;
    }

    bool activateController(rm3_ackermann_controller::ActivateController::Request &req,
                            rm3_ackermann_controller::ActivateController::Response &res)
    {

        isActive = req.is_active;
        res.result = "Succesfully Called ActivateController service!\n";

        if (isActive)
            printf("\nAckermann_Controller is ACTIVE\n");
        else
            printf("\nAckermann_Controller is NOT ACTIVE\n");

        return true;
    }

    void calculateKvalues()
    {
        //        r_p <<     m_position_msg.x,
        //                   m_position_msg.y;
        //        r_th =     m_position_msg.z;

        //        r_p_des << m_waypoint_msg.x,
        //                   m_waypoint_msg.y;
        //        r_th_des = m_waypoint_msg.z;

        if (sqrt(pow(m_position_msg.x - m_waypoint_msg.x, 2) +
                 pow(m_position_msg.y - m_waypoint_msg.y, 2)) < 0.3)
        {
            K1 = 1.0;
            K2 = 6.0;
            K3 = 3.0;
        }
    }

private:
    ros::Subscriber m_sub_odom;
    ros::Subscriber m_sub_trajectory;
    // ros::Subscriber m_sub_imu;
    ros::Subscriber m_sub_joint_state;
    ros::Subscriber m_sub_waypoint;
    ros::Subscriber m_sub_jointstate;
    ros::Subscriber m_sub_cmd;

    ros::Publisher m_pub_cmd;
    ros::Publisher m_pub_yaw;
    ros::Publisher m_pub_lyapunov;
    ros::Publisher m_pub_yaw_estimated;
    ros::Publisher m_pub_yaw_offset;

    nav_msgs::Odometry m_odom_msg;
    // sensor_msgs::Imu m_imu_msg;
    sensor_msgs::JointState m_joint_state_msg;
    geometry_msgs::Point m_waypoint_msg;
    trajectory_msgs::JointTrajectory trajectory;

    geometry_msgs::Twist m_cmd_recived_msg;
    geometry_msgs::Twist m_cmd_msg;
    std_msgs::Float64 m_yaw_msg;
    std_msgs::Float32 m_lyapunov_msg;
    std_msgs::Float64 m_yaw_estimated_msg;
    std_msgs::Float64 m_yaw_offset_msg;

    geometry_msgs::Point m_position_msg;
    std::vector<geometry_msgs::Point> m_position_history;

    ros::ServiceServer m_serviceSet;
    ros::ServiceServer m_serviceActivate;
    ros::ServiceServer m_serviceCalibration;
    ros::ServiceServer m_serviceFixOffset;

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
    bool trajectory_pts_;
    int trajectory_iter_;

    double m_threshold;

    bool m_lyapunov_enable;

    double m_error;

    int m_history_cnt;
    int m_history_steps;
    bool m_enable_yaw_estimation;
    bool calibrationActive;
    bool offsetFixed;

    double m_watchdog;
    double m_threshold_time;

    string m_joint_state_topic;

    enum State
    {
        Idle = 0,
        Moving_to_waypoint = 1
    };
    State m_state;
    Mode m_mode;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ackermann_controller_node");
    ros::NodeHandle nh("~");

    double frequency;
    nh.param("frequency", frequency, 20.0);

    int mode;
    nh.param("mode", mode, 5);
    //REAL:{ENCODER, GPS_BEARING, GPS_IMU, EKF}
    //GAZEBO:{GAZEBO}
    //mode:   1 encoder
    //        2 gps_bearing
    //        3 gps_imu
    //        4 ekf
    //        5 gazebo

    string odom_topic;
    nh.param<string>("odom_topic", odom_topic, "default");
    odom_topic = check_odom_topic((Mode)mode, odom_topic);

    string imu_topic;
    nh.param<string>("imu_topic", imu_topic, "/imu");

    string joint_state_topic;
    nh.param<string>("joint_state_topic", joint_state_topic, "default");
    joint_state_topic = check_joint_state_topic((Mode)mode, joint_state_topic);

    string waypoint_topic;
    nh.param<string>("waypoint_topic", waypoint_topic, "/waypoint");

    string cmd_topic;
    nh.param<string>("cmd_topic", cmd_topic, "default");
    cmd_topic = check_cmd_topic((Mode)mode, cmd_topic);

    string yaw_topic;
    nh.param<string>("yaw_topic", yaw_topic, "/yaw_navigation"); // to evaluate check parameters

    double threshold;
    nh.param("threshold", threshold, 0.05);

    double threshold_time;
    nh.param("threshold_time", threshold_time, 2.0);

    bool lyapunov_enable;
    nh.param("lyapunov_enable", lyapunov_enable, false);

    double error;
    nh.param("error", error, 0.0);

    int history_steps;
    nh.param("history_steps", history_steps, 20);

    bool enable_yaw_estimation;
    nh.param("enable_yaw_estimation", enable_yaw_estimation, false);

    Ackermann_Controller ackermann_controller((Mode)mode,
                                              odom_topic,
                                              imu_topic,
                                              joint_state_topic,
                                              waypoint_topic,
                                              cmd_topic,
                                              yaw_topic,
                                              threshold,
                                              threshold_time,
                                              lyapunov_enable,
                                              error,
                                              history_steps,
                                              enable_yaw_estimation,
                                              nh);
    ackermann_controller.run(frequency);

    return 0;
}
