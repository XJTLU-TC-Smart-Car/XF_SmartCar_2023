#include <ros/ros.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int16.h>
#include <xf_mic_asr_offline/Set_Awake_Word_srv.h>
#include <std_msgs/Int32.h>
#include <sound_play/sound_play.h>
#include <sound_play/SoundRequest.h>
#include <thread>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Trigger.h>
typedef actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> MoveBaseClient;

class UcarNav {
public:
    UcarNav() : ac(std::make_shared<MoveBaseClient>("move_base", true)), nh_("~") {
        ros::NodeHandle nh;

        sound_pub = nh.advertise<sound_play::SoundRequest>("/robotsound", 1);
        wake_up_words_ = "start";
        has_set_wake_word_ = false;
        has_wake_up_ = false;
        nh_.param("wake_up_words", wake_up_words_, std::string("开始导航"));
        flag = -1;
        arrive = 0;

        // Initialize the locations.
        locations = {
                {{4.991, -0.230, 0.99999077, 0.00429631}},
                {{2.033, -0.262, 1.000,      -0.005}},
                {{2.344, 4.944,  0.704,      0.710}}
        };

        qr_sub = nh.subscribe("/qr_res", 1, &UcarNav::qrCallback, this);


        set_wake_words_client_ = nh_.serviceClient<xf_mic_asr_offline::Set_Awake_Word_srv>(
                "/xf_asr_offline_node/set_awake_word_srv");

        // Subscribers and publishers
        wake_up_sub_ = nh_.subscribe("/mic/awake/angle", 10, &UcarNav::wakeUpCallback, this);

        // Set the wake up words
        setWakeWords();
    }


    void run() {
        ROS_INFO("wait for wake up.");
        ros::Rate loop_rate(10);
        while (ros::ok() && !has_wake_up_) {
            ros::spinOnce();
            loop_rate.sleep();
        }
        ROS_INFO("Start the game!");

        while (!ac->waitForServer(ros::Duration(5.0))) {
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        // Loop over all the goals
        for (size_t i = 0; i < locations.size(); ++i) {
            moveToGoal(locations[i]);
            if (arrive == 0) { // If not arrive, break the loop
                break;

            }

        }
        if (arrive == 1) {
            playSound("/home/ucar/ucar_ws/src/ucar_race/res/jia.wav");
        }
    }

private:
    std::shared_ptr <MoveBaseClient> ac;
    std::string wake_up_words_;
    bool has_set_wake_word_;
    bool has_wake_up_;
    ros::NodeHandle nh_;
    ros::ServiceClient set_wake_words_client_;
    ros::ServiceClient code_detect_client_;
    ros::ServiceClient park_detect_client_;
    ros::Subscriber wake_up_sub_;

    int flag;
    int arrive;
    std::vector <std::vector<double>> locations;
    ros::Subscriber qr_sub;

    ros::Publisher sound_pub;

    void setWakeWords() {
        set_wake_words_client_ = nh_.serviceClient<xf_mic_asr_offline::Set_Awake_Word_srv>(
                "/xf_asr_offline_node/set_awake_word_srv");
        xf_mic_asr_offline::Set_Awake_Word_srv set_wake_srv;
        set_wake_srv.request.awake_word = wake_up_words_;
        ros::Rate loop_rate(10);
        while (ros::ok() && !has_set_wake_word_) {
            try {
                set_wake_words_client_.call(set_wake_srv);
                if (set_wake_srv.response.result == std::string("ok")) {
                    has_set_wake_word_ = true;
                    break;
                } else if (set_wake_srv.response.result == std::string("false")) {
                    ROS_ERROR("set_wake_words faild. fail_reason: %s", set_wake_srv.response.fail_reason.c_str());
                } else {
                    ROS_ERROR("set_wake_words error. fail_reason: %s", set_wake_srv.response.fail_reason.c_str());
                }
            }
            catch (const std::exception &e) {
                std::cerr << "Call set_wake_word faild. Maybe service not ready." << e.what() << '\n';
            }
            loop_rate.sleep();
        }
        ROS_INFO("set wake up words as: %s", wake_up_words_.c_str());
    }

    void wakeUpCallback(const std_msgs::Int32::ConstPtr &msg) {
        if (has_set_wake_word_) {
            has_wake_up_ = true;
        }
    }

    void qrCallback(const std_msgs::Int16::ConstPtr &msg) {
        flag = msg->data;
        if (flag == 0) {
            // Play sound.
        } else if (flag == 1) {
            // Play another sound.
        } else if (flag == 2) {
            // Play a different sound.
        }
    }

    void moveToGoal(const std::vector<double> &goal) {
        move_base_msgs::MoveBaseGoal mbGoal;

        mbGoal.target_pose.header.frame_id = "map";
        mbGoal.target_pose.header.stamp = ros::Time::now();
        mbGoal.target_pose.pose.position.x = goal[0];
        mbGoal.target_pose.pose.position.y = goal[1];
        mbGoal.target_pose.pose.orientation.z = goal[2];
        mbGoal.target_pose.pose.orientation.w = goal[3];

        ROS_INFO("Sending goal");
        ac->sendGoal(mbGoal);

        ac->waitForResult();

        if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Hooray, the base moved to the goal");
            arrive = 1;
            ROS_INFO("arrive value is now: %d", arrive);
            std::thread detect_thread([&]() {
                code_detect_client_ = nh_.serviceClient<std_srvs::Trigger>("/detect_server");
                std_srvs::Trigger detect_srv;
                std::string cmd;
                while (ros::ok()) {
                    try {
                        ROS_INFO("Calling plant");
                        code_detect_client_.call(detect_srv);
                        if (detect_srv.response.success == true) {
                            ROS_INFO("Detection success. Results: %s", detect_srv.response.message);
                            break;
                        } else {
                            ROS_ERROR("Detection failed. Results: %s", detect_srv.response.message);
                        }
                    }
                    catch (const std::exception &e) {
                        std::cerr << "Call plant faild, error_info: " << e.what() << '\n';
                    }
                }
                system(cmd.c_str());
            });
            detect_thread.detach();
        } else {
            ROS_INFO("The base failed to move for some reason");
            arrive = 0;
        }
    }

    void playSound(const std::string &sound_file) {
        sound_play::SoundRequest sound;
        sound.sound = sound_play::SoundRequest::PLAY_FILE;
        sound.arg = sound_file;
        sound.arg2 = "";
        sound.command = sound_play::SoundRequest::PLAY_ONCE;
        sound.volume = 1.0;
        sound_pub.publish(sound);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ucar_nav_node");

    ros::NodeHandle n;
    ros::Publisher pub_wake_up = n.advertise<std_msgs::String>("/wake_up", 1000);
    UcarNav ucarNav;

    ucarNav.run();


    ros::spin();

    return 0;
}


