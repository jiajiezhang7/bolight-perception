#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <perception/detectStandsAction.h>  
#include <perception/detectBeamsAction.h> 


int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_stands_detect_client");


    actionlib::SimpleActionClient<perception::detectStandsAction> ac("lidar_stands_detect", true);

    ROS_INFO("Waiting for action server to start...");
    ac.waitForServer(); 

    ROS_INFO("Action server started, sending goal...");


    perception::detectStandsGoal goal;
    goal.ifDetect= true;


    ac.sendGoal(goal);

    // 等待 Action 完成，超时时间设置为 ros::Duration(30.0)
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    } else {
        ROS_WARN("Action did not finish before the timeout.");
    }
    // ros::init(argc, argv, "lidar_beams_detect_client");


    // actionlib::SimpleActionClient<perception::detectBeamsAction> ac("lidar_beams_detect", true);

    // ROS_INFO("Waiting for action server to start...");
    // ac.waitForServer(); 

    // ROS_INFO("Action server started, sending goal...");


    // perception::detectBeamsGoal goal;
    // goal.ifDetect= true;


    // ac.sendGoal(goal);

    // // 等待 Action 完成，超时时间设置为 ros::Duration(30.0)
    // bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    // if (finished_before_timeout) {
    //     actionlib::SimpleClientGoalState state = ac.getState();
    //     ROS_INFO("Action finished: %s", state.toString().c_str());
    // } else {
    //     ROS_WARN("Action did not finish before the timeout.");
    // }
    return 0;
}
