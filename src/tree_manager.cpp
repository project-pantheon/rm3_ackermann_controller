#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <rm3_ackermann_controller/GetTree.h>
#include <fstream>


using namespace std;


class Tree_Manager
{

public:

    Tree_Manager(const string& odom_topic, const string& trees_path,
	    ros::NodeHandle nh){

        ROS_INFO("Get Tree started");

        // odom subscriber
        m_sub_odom = nh.subscribe(odom_topic, 1, &Tree_Manager::odomChanged, this);

        // set K values service
        m_serviceGetTree = nh.advertiseService("get_tree", &Tree_Manager::getTree, this);
        

    }

    
private: 




    void odomChanged(const nav_msgs::Odometry::ConstPtr& msg)
    {
        m_odom_msg = *msg;
    }


    bool getTree(rm3_ackermann_controller::GetTree::Request &req,
        rm3_ackermann_controller::GetTree::Response &res) {

        m_odom_msg_fixed=m_odom_msg;
        
        m_trees.data.clear();
        
        //std::ifstream infile("/home/ciro/sherpa_pantheon_ws/src/rm3_ackermann_controller/src/trees.txt");
	std::ifstream infile("/home/renzo/lab_ws/src/rm3_ackermann_controller/src/trees.txt");
        
                    
        double x, y;

        ROS_INFO("test x:%f y:%f",x,y);
        while (infile >> x >> y)
        {
            ROS_INFO("test x:%f y:%f",x,y);
            ROS_INFO("ciao %f %f", m_odom_msg_fixed.pose.pose.position.x, m_odom_msg_fixed.pose.pose.position.y);             
            m_trees.data.push_back(x-m_odom_msg_fixed.pose.pose.position.x);
            m_trees.data.push_back(y-m_odom_msg_fixed.pose.pose.position.y);
            //m_trees.data.push_back(x);
            //m_trees.data.push_back(y);
        }

        res.trees=m_trees;
        std::cout << "get tree servce called \n";
        m_sub_odom.shutdown();

        return true;
    }
	
private:
    
    ros::Subscriber m_sub_odom;

    nav_msgs::Odometry m_odom_msg;
    nav_msgs::Odometry m_odom_msg_fixed;

    ros::ServiceServer m_serviceGetTree;

    std_msgs::Float32MultiArray m_trees;

    std::string m_trees_path;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tree_manager_node");
    ros::NodeHandle nh("~");


    string odom_topic;
    nh.param<string>("odom_topic", odom_topic, "/odom_gps_ekf");

    string trees_path;
    nh.param<string>("trees_path", trees_path, "/home/renzo/lab_ws/src/rm3_ackermann_controller/src/trees.txt");

    Tree_Manager tree_manager(odom_topic,trees_path, nh);

    ros::spin();

    return 0;
}
