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
                {{0.068, 4.075,  0.870,  0.493}},//B
                {{0.068, 4.075,  0.473,  0.881}},//B
                {{0.049, 0.855,  0.870,  -0.493}},//C
                {{0.049, 0.855,  -0.468, 0.884}},//C
                {{1.833, 4.260,  0.870,  0.493}},//D
                {{1.833, 4.260,  0.473,  0.881}},//D
                {{1.833, -0.171, 1.000,  0.000}},//E
                {{1.833, -0.171, 0.000,  1.000}},
                {{3.940, 0.113,  0.707,  0.707}},
                {{3.940, 2.127,  0.707,  0.707}},
                //{{3.920, 2.127,  -0.703, 0.711}},
                //{{3.920, 2.127,  0.703,  0.711}},
                {{3.912, 3.322,  1.000,  0.025}},//F1
                {{3.912, 3.322,  0.916,  0.402}},
                {{3.912, 3.322,  0.284,  0.959}},//F2
                {{3.232, 4.866, 0.802,  0.597}},
                {{4.802, 4.929,  0.473,  0.881}},
                {{3.498, 2.209,  -0.826, 0.563}},//F3
                //{{3.143, 2.121, 0.686, 0.728}},
                {{4.613, 1.945,  -0.587, 0.810}},//F4
                {{3.940, 2.127,  -0.707, 0.707}},
                {{3.940, 0.113,  -0.707, 0.707}},
                {{5.021, -0.378, -0.000, 1.000}}
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

