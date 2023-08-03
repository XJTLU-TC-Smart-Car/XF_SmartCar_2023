#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class UcarNav {
public:
    UcarNav() : ac("move_base", true) {
        ros::NodeHandle nh;

        flag = -1;
        arrive = 0;

        // Initialize the locations.
        locations = {


                //{{0.251,5.091,  0.700, 0.714},  false},//B  down植被固定点，第一次识别结束
                {{0.6625,4.6685,0.561, 0.828}},//B down植被固定点
                //{{0.678, 0.304,  -0.207, 0.978},  false},//C down植被固定点，第二次识别结束
                {{0.421, -0.015, -0.172, 0.985}},//C down植被固定点
                //{{1.972,  4.710,  0.333,  0.943},  false},//D 植被固定点，第三次识别结束
                {{2.037, 4.797, 0.560, 0.828}},//D 植被固定点
                
               
                {{1.748, 0.145, -0.812, 0.583}},//E 植被固定点


                {{3.930, 0.113,  0.707,  0.707}}, // 冲坡去水果区
                {{3.930, 2.127,  0.707,  0.707}}, // 过坡定位
                {{3.963, 3.537, 0.986, 0.167}},
                {{3.963, 3.537, 0.896, 0.443}},// F1 水果随机板，开始旋转
                {{3.963, 3.537, 0.478, 0.879}}, // F1 F2 中间点，切换视觉
                {{3.963, 3.537, 0.210, 0.978}},//F2 水果随机板， 结束旋转
                //{{3.151, 4.420,  0.912,  0.410},  false}, // F5 水果固定板，开始旋转
                {{3.505, 4.761, 0.970, 0.245}},  // F5 水果固定板，结束旋转
                //{{4.705, 4.725,  0.700,  0.714},  false}, // F6 水果固定板，结束旋转
                {{4.814, 4.681, 0.643, 0.766}},  // F6 水果固定板，结束旋转

                {{3.945, 2.127,  -0.707, 0.707}}, //回正点
                {{3.945, 2.127,  0.970,  -0.243}},  // F3 水果随机板，开始旋转
                {{3.945, 2.127,  -0.707, 0.707}}, // F3 F4 中间点，切换视觉
                {{3.945, 2.127,  -0.285, 0.958}},  // F4 水果随机板，开始旋转
                {{3.945, 2.127,  -0.707, 0.707}}, // 结束旋转，冲破
                {{3.945, 0.113,  -0.000, 1.000}}, // 过破
                {{4.900, -0.320, -0.000, 1.000}} // 终点
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

