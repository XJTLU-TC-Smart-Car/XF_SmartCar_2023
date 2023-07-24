#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int16.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class UcarNav {
public:
    UcarNav() : ac("move_base", true) {
        ros::NodeHandle nh;

        flag = -1;
        arrive = 0;

        // Initialize the locations.
        locations = {
            {{4.991, -0.230, 0.99999077, 0.00429631}},
            {{2.033, -0.262, 1.000, -0.005}},
            {{2.344, 4.944, 0.704, 0.710}},
            {{3.944, 0.113, 0.708, 0.707}},
            {{3.943, 1.950, 0.711,0.703}},
            {{3.943, 1.950, -0.711, 0.703}},
            {{3.943, 0.113, 0.708, 0.707}}
        };

        qr_sub = nh.subscribe("/qr_res", 1, &UcarNav::qrCallback, this);

        while(!ac.waitForServer(ros::Duration(5.0))) {
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        // Loop over all the goals
        for (size_t i = 0; i < locations.size(); ++i) {
            moveToGoal(locations[i]);
            if (arrive == 0) { // If not arrive, break the loop
                break;
            }
        }
    }

private:
    MoveBaseClient ac;
    ros::Subscriber qr_sub;
    int flag;
    int arrive;
    std::vector<std::vector<double>> locations;

    void qrCallback(const std_msgs::Int16::ConstPtr& msg) {
        flag = msg->data;
        if (flag == 0) {
            // Play sound.
        } else if (flag == 1) {
            // Play another sound.
        } else if (flag == 2) {
            // Play a different sound.
        }
    }

    void moveToGoal(const std::vector<double>& goal) {
        move_base_msgs::MoveBaseGoal mbGoal;

        mbGoal.target_pose.header.frame_id = "map";
        mbGoal.target_pose.header.stamp = ros::Time::now();
        mbGoal.target_pose.pose.position.x = goal[0];
        mbGoal.target_pose.pose.position.y = goal[1];
        mbGoal.target_pose.pose.orientation.z = goal[2];
        mbGoal.target_pose.pose.orientation.w = goal[3];

        ROS_INFO("Sending goal");
        ac.sendGoal(mbGoal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Hooray, the base moved to the goal");
            arrive = 1;
        } else {
            ROS_INFO("The base failed to move for some reason");
            arrive = 0;
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ucar_nav_node");

    UcarNav ucarNav;

    ros::spin();

    return 0;
}
