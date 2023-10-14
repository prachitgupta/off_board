#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandTOL.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/takeoff");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.z = 1.0;
    pose.header.stamp = ros::Time::now();

    // send takeoff command
    mavros_msgs::CommandTOL takeoff_cmd;
    takeoff_cmd.request.altitude = 1.0;

    if (takeoff_client.call(takeoff_cmd) && takeoff_cmd.response.success) {
        ROS_INFO("Takeoff sent");
    } else {
        ROS_ERROR("Failed to send takeoff");
        return 1;
    }

    bool takeoff_sent = true;

    // wait for takeoff to complete
    while (ros::ok() && current_pose.pose.position.z < 0.95) {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Takeoff complete");

    // set waypoint 1 meter north of the takeoff position
    pose.pose.position.x = current_pose.pose.position.x + 1.0;
    pose.pose.position.y = current_pose.pose.position.y;
    pose.header.stamp = ros::Time::now();

    // navigate to waypoint
    while (ros::ok() && !waypoint_reached) {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();

        double distance_to_waypoint = sqrt(pow(current_pose.pose.position.x - pose.pose.position.x, 2) +
                                           pow(current_pose.pose.position))}
}