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
                //{{1.400, 0.388, -0.743, 0.669}},//B
                //{{1.861, -0.317, 1.000, -0.020}},//B
                //{{1.951, 5.191,0.018, 1.000}},//C
                //{{2.449, 4.901, 0.700, 0.714}},//C
               // {{1.951, 5.191,1.000, -0.004}},//D
                //{{1.449, 4.901, 0.700, 0.714}},//D
                //{{0.251,5.191,0.018,1.000}},//E
                //{{0.897, 4.451, 0.694, 0.720}},
                //{{0.251,5.191,1.000,-0.004}},
                //{{-0.603, 4.451, 0.694, 0.720}},
               // {{0.251,-0.309,0.017,1.000}},
                {{0.900, 0.388, -0.743, 0.669}},
                {{0.251,-0.309,1.000,-0.004}},//F1
                {{-0.600, 0.388, -0.743, 0.669}},
                {{3.071,5.191,1.000,-0.004}},//F2
                {{3.151, 4.720, 0.700, 0.714}},
                {{4.705, 4.725, 0.700, 0.714}},
                {{4.201,5.191,0.018,1.000}}//F3
                
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

