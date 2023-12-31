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
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

typedef actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> MoveBaseClient;

class UcarNav {
public:
    UcarNav() : ac(std::make_shared<MoveBaseClient>("move_base", true)), nh_("~"), stop_thread_flag(false) {
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
                {{0.6625, 4.6685, 0.577,  0.817},  true},//B down植被固定点
                {{0.6625, 4.6685, -0.797, 0.604},  false},
                {{-0.359, 2.494,  -0.697, 0.717},  false},
                {{0.634,  -0.020, -0.564, 0.826},  true},//C down植被固定点
                {{1.735,  4.731,  0.822,  0.569},  true},//D 植被固定点
                {{1.748,  0.145,  -0.812, 0.583},  true},//E 植被固定点
                {{3.930,  0.113,  0.707,  0.707},  false},//冲坡去水果区
                {{3.930,  2.127,  0.707,  0.707},  false},//过坡定位

                // 八卦站中间
                {{3.963,  3.537,  0.703,  0.700},  false},

                {{3.963,  3.537,  1.000,  0.000},  true},// F1 水果随机板，开始旋转
                {{3.963,  3.537,  -0.365, 0.931},  false}, // F1 F2 中间点，切换视觉
                {{3.963,  3.537,  0.006,  1.000},  true},//F2 水果随机板， 结束旋转
                //{{3.151, 4.420,  0.912,  0.410},  false}, // F5 水果固定板，开始旋转
                //{{3.455, 4.761, 0.970, 0.245},  true},  // F5 水果固定板，结束旋转
                //{{4.705, 4.725,  0.700,  0.714},  false}, // F6 水果固定板，结束旋转
                {{3.202,  4.540,  0.775,  0.632},  true},  // F5 水果固定板，结束旋转

                {{3.945,  2.127,  -0.707, 0.707},  false}, //回正点
                {{3.945,  2.127,  -0.449, 0.893},  true},  // F3 水果随机板，开始旋转
                {{3.945,  2.127,  -0.707, 0.707},  false}, // F3 F4 中间点，切换视觉
                {{3.945,  2.127,  0.893,  -0.449}, true},  // F4 水果随机板，开始旋转
                {{3.945,  2.127,  -0.707, 0.707},  false}, // 结束旋转，冲破
                {{3.945,  0.113,  -0.000, 1.000},  false}, // 过破
                {{5.000,  -0.370, -0.000, 1.000},  false} // 终点
        };
        qr_sub = nh.subscribe("/qr_res", 1, &UcarNav::qrCallback, this);
        set_wake_words_client_ = nh_.serviceClient<xf_mic_asr_offline::Set_Awake_Word_srv>(
                "/xf_asr_offline_node/set_awake_word_srv");
        wake_up_sub_ = nh_.subscribe("/mic/awake/angle", 10, &UcarNav::wakeUpCallback, this);
        setWakeWords();

    }


    void run() {
        ROS_WARN("wait for wake up.");
        ros::Rate loop_rate(10);

        while (ros::ok() && !has_wake_up_) {
            ros::spinOnce();
            loop_rate.sleep();
        }
        ROS_WARN("Start the game!");

        while (!ac->waitForServer(ros::Duration(5.0))) {
            ROS_WARN("Waiting for the move_base action server to come up");
        }
        // Loop over all the goals

        for (size_t i = 0; i < locations.size(); ++i) {
            moveToGoal(locations[i].first);
            if (i == 17) {
                stop_thread_flag = true;
//                processHighestConfidence(detection_results_tmp, detection_results_tmp_9)
            }
            if (locations[i].second) {
                startDetectThread(i);
            }
            if (arrive == 0)
                break;
            if (i == 5) {
                std::thread t([this, i]() {  // 注意这里，我们添加了this
                    while (i != 6) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    }
                    ros::Publisher vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
                    geometry_msgs::Twist vel_msg;
                    vel_msg.linear.x = 0.5;  // 设置前进速度为1.0 m/s
                    vel_msg.angular.z = 0;  // 设置转向速度为0，保证直线前进

                    ros::Rate rate(10);  // 设置发布频率为10Hz
                    for (int j = 0; j < 45; ++j) {
                        vel_pub.publish(vel_msg);
                        rate.sleep();
                    }
                    ROS_WARN("start move");
                    vel_msg.linear.x = 0;  // 停止前进
                    vel_pub.publish(vel_msg);  // 发布停止命令
                });
                t.detach();
            } else if (i == 18) {
                std::thread t([this, i]() {  // 注意这里，我们添加了this
                    while (i != 19) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    }
                    ros::Publisher vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
                    geometry_msgs::Twist vel_msg;
                    vel_msg.linear.x = 0.5;  // 设置前进速度为1.0 m/s
                    vel_msg.angular.z = 0;  // 设置转向速度为0，保证直线前进

                    ros::Rate rate(10);  // 设置发布频率为10Hz
                    for (int j = 0; j < 45; ++j) {
                        vel_pub.publish(vel_msg);
                        rate.sleep();
                    }
                    ROS_WARN("start move");
                    vel_msg.linear.x = 0;  // 停止前进
                    vel_pub.publish(vel_msg);  // 发布停止命令
                });
                t.detach();
            }

        }

        if (arrive == 1) {
            playSoundsBasedOnResults();
        }
    }

private:
    std::shared_ptr <MoveBaseClient> ac;
    std::string wake_up_words_;
    std::string cmd;
    std::shared_ptr <std::thread> detect_thread;
    struct DetectionResult {
        int classIndex;
        float average_confidence;
    };
    std::vector<int> detection_results_final_plant;
    std::vector<int> detection_results_final_fruit;
    std::vector <DetectionResult> detection_results_plant;
    std::vector <DetectionResult> detection_results_fruit;
    std::vector <DetectionResult> detection_results_tmp;
    std::vector <DetectionResult> detection_results_tmp_9;
    int fruit_num[4] = {0, 0, 0, 0};

    bool has_set_wake_word_;
    bool has_wake_up_;
    ros::NodeHandle nh_;
    ros::ServiceClient set_wake_words_client_;
    ros::ServiceClient code_detect_client_;
    ros::ServiceClient park_detect_client_;
    ros::Subscriber wake_up_sub_;

    int flag;
    int arrive;
    std::vector<std::pair<std::vector < double>, bool>>
    locations;
    ros::Subscriber qr_sub;

    ros::Publisher sound_pub;

    tf::TransformListener listener;
    std::atomic<bool> stop_thread_flag;

    std::map<int, std::string> result_to_sound_file_map = {
            {1, "/home/ucar/ucar_ws/src/ucar_race/res/yumi.wav"},
            {2, "/home/ucar/ucar_ws/src/ucar_race/res/huanggua.wav"},
            {3, "/home/ucar/ucar_ws/src/ucar_race/res/shuidao.wav"},
            {4, "/home/ucar/ucar_ws/src/ucar_race/res/xiaomai.wav"}
    };

    std::vector <std::string> location_sounds = {
            "/home/ucar/ucar_ws/src/ucar_race/res/B.wav",
            "/home/ucar/ucar_ws/src/ucar_race/res/C.wav",
            "/home/ucar/ucar_ws/src/ucar_race/res/D.wav",
            "/home/ucar/ucar_ws/src/ucar_race/res/E.wav"
    };
    int fruit_count[7] = {0, 1, 2, 1, 2, 3, 1};
    std::map<int, std::string> fruit_sound_files = {
            {1, "/home/ucar/ucar_ws/src/ucar_race/res/yumi.wav"},
            {2, "/home/ucar/ucar_ws/src/ucar_race/res/huanggua.wav"},
            {3, "/home/ucar/ucar_ws/src/ucar_race/res/xigua.wav"}
    };

    std::map<int, std::string> quantity_sound_files = {
            {1, "/home/ucar/ucar_ws/src/ucar_race/res/1ge.wav"},
            {2, "/home/ucar/ucar_ws/src/ucar_race/res/2ge.wav"},
            {3, "/home/ucar/ucar_ws/src/ucar_race/res/3ge.wav"},
            {4, "/home/ucar/ucar_ws/src/ucar_race/res/4ge.wav"},
            {5, "/home/ucar/ucar_ws/src/ucar_race/res/5ge.wav"},
            {6, "/home/ucar/ucar_ws/src/ucar_race/res/6ge.wav"},
            {7, "/home/ucar/ucar_ws/src/ucar_race/res/7ge.wav"},
            {8, "/home/ucar/ucar_ws/src/ucar_race/res/8ge.wav"},
            {9, "/home/ucar/ucar_ws/src/ucar_race/res/9ge.wav"}

    };

    std::string getSoundFileForResult(int result) {
        // 检查结果是否在 map 中
        ROS_INFO("result: %d", result);
        if (result_to_sound_file_map.find(result) != result_to_sound_file_map.end()) {
            ROS_INFO("in result: %d", result);
            // 如果在 map 中，返回对应的音频文件路径
            return result_to_sound_file_map[result];
        } else {
            // 如果不在 map 中，返回空字符串
            return "";
        }
    }


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
        ROS_WARN("set wake up words as: %s", wake_up_words_.c_str());
        sleep(3);
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

        ROS_WARN("Sending goal");
        ac->sendGoal(mbGoal);

        ac->waitForResult();

        if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_WARN("Hooray, the base moved to the goal");
            arrive = 1;
            ROS_WARN("arrive value is now: %d", arrive);

        } else {
            ROS_WARN("The base failed to move for some reason");
            arrive = 0;
        }
    }

    void rotateInPlace() {
        // Get current pose
        tf::StampedTransform transform;
        try {
            listener.lookupTransform("/map", "/base_link",
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        geometry_msgs::PoseStamped current_pose;
        current_pose.pose.position.x = transform.getOrigin().x();
        current_pose.pose.position.y = transform.getOrigin().y();
        current_pose.pose.position.z = transform.getOrigin().z();
        current_pose.pose.orientation.x = transform.getRotation().x();
        current_pose.pose.orientation.y = transform.getRotation().y();
        current_pose.pose.orientation.z = transform.getRotation().z();
        current_pose.pose.orientation.w = transform.getRotation().w();

        // Create goal
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        // Set goal position to current position
        goal.target_pose.pose.position = current_pose.pose.position;

        // Set goal orientation to current orientation plus 360 degrees rotation around Z axis
        tf::Quaternion q(current_pose.pose.orientation.x, current_pose.pose.orientation.y,
                         current_pose.pose.orientation.z, current_pose.pose.orientation.w);
        tf::Quaternion q_rot = tf::createQuaternionFromYaw(tf::getYaw(q) + 2.0 * M_PI);
        goal.target_pose.pose.orientation.x = q_rot.x();
        goal.target_pose.pose.orientation.y = q_rot.y();
        goal.target_pose.pose.orientation.z = q_rot.z();
        goal.target_pose.pose.orientation.w = q_rot.w();

        // Send goal
        ac->sendGoal(goal);
        ac->waitForResult();
    }

    void processHighestConfidence(std::vector<DetectionResult>& detection_results_tmp, std::vector<DetectionResult>& detection_results_tmp_9) {
        // 如果detection_results_tmp_9为空，直接返回
        if (detection_results_tmp_9.empty()) {
            ROS_WARN("detection_results_tmp_9 is empty.");
            return;
        }

        // 使用lambda函数对detection_results_tmp_9中的元素按average_confidence降序排序
        std::sort(detection_results_tmp_9.begin(), detection_results_tmp_9.end(), [](const DetectionResult& a, const DetectionResult& b) {
            return a.average_confidence > b.average_confidence;
        });

        // 打印detection_results_tmp_9中的所有元素
        for (const auto& result : detection_results_tmp_9) {
            ROS_WARN("Class Index: %d, Average Confidence: %.2f", result.classIndex, result.average_confidence);
        }

        // 将置信度最高的元素添加到detection_results_tmp
        detection_results_tmp.push_back(detection_results_tmp_9[0]);

        // 如果detection_results_tmp_9有多个元素，将置信度第二高的元素也添加到detection_results_tmp
        if (detection_results_tmp_9.size() > 1) {
            detection_results_tmp.push_back(detection_results_tmp_9[1]);
        }

        // 清空detection_results_tmp_9
        detection_results_tmp_9.clear();
    }


    // 定义结构体
    void startDetectThread(int location_i) {
//        stop_thread_flag = false; // 重置标志
        detect_thread = std::make_shared<std::thread>([&]() {
            auto start_time = std::chrono::system_clock::now();
            while (!stop_thread_flag) { // 在循环中检查标志
                // 设置 ROS 服务客户端
                code_detect_client_ = nh_.serviceClient<std_srvs::Trigger>("/detect_server");
                std_srvs::Trigger detect_srv;
                if (ros::ok()) {
                    try {
                        code_detect_client_.call(detect_srv);
                        if (detect_srv.response.success == true) {
                            ROS_WARN("Detection success. Results: %s", detect_srv.response.message.c_str());
                            std::stringstream ss(detect_srv.response.message);
                            std::string token;
                            std::getline(ss, token, ',');
                            int classIndex = std::stoi(token);
                            std::getline(ss, token, ',');
                            float average_confidence = std::stof(token);
                            // 创建一个DetectionResult对象并添加到detection_results中
                            if (location_i < 0) {
                                ROS_WARN("init cam!");
                                continue;
                            }
                            DetectionResult result;
                            result.classIndex = classIndex;
                            result.average_confidence = average_confidence;
                            detection_results_tmp.push_back(result);
                            return;
                            if (location_i != 9) {
                                detection_results_tmp.push_back(result);
                                return;
                            } else{
                                detection_results_tmp_9.push_back(result);
                            }


                        } else {
                            ROS_ERROR("Detection failed. Results: %s", detect_srv.response.message.c_str());
                        }
                    }
                    catch (const std::exception &e) {
                        std::cerr << "Call failed, error_info: " << e.what() << '\n';
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 让出一些CPU时间
                auto current_time = std::chrono::system_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
                    return;
            }
        });
        detect_thread->detach();
    }

    void playSound(const std::string &sound_file) {
        ROS_WARN("Playing sound: %s", sound_file.c_str());
        sound_play::SoundRequest sound;
        sound.sound = sound_play::SoundRequest::PLAY_FILE;
        sound.arg = sound_file;
        sound.arg2 = "";
        sound.command = sound_play::SoundRequest::PLAY_ONCE;
        sound.volume = 1.0;
        sound_pub.publish(sound);
    }

    bool isClassIndexPresent(int i, std::vector <DetectionResult> &detection_results) {
        auto it = std::find_if(detection_results.begin(), detection_results.end(),
                               [i](const DetectionResult &dr) { return dr.classIndex == i; });
        if (it == detection_results_plant.end())
            return false;
        return true;
    }

    void processResults_Fruit(std::vector <DetectionResult> &detection_results,
                              std::vector<int> &detection_results_final) {
        for (auto &result: detection_results)
            ROS_WARN("Fruit_classIndex: %d", result.classIndex);
        for (const auto &result: detection_results) {
            detection_results_final.push_back(result.classIndex);
        }
    }

    void processResults_Plant(std::vector <DetectionResult> &detection_results,
                              std::vector<int> &detection_results_final) {
        // 第一步
        bool allPresent = true;
        int classbegin = 1;
        int classend = 4;

        for (auto &result: detection_results)
            ROS_WARN("Plant_classIndex: %d", result.classIndex);
        for (int i = classbegin; i <= classend; i++) {
            if (!isClassIndexPresent(i, detection_results)) {
                allPresent = false;
                break;
            }
        }

        if (!allPresent) {
            // 第二步
            std::map<int, int> classIndexCounts;
            for (const auto &result: detection_results) {
                classIndexCounts[result.classIndex]++;
            }

            for (auto &pair: classIndexCounts) {

                if (pair.second == 2) {
                    // 找到重复的classIndex
                    int duplicateClassIndex = pair.first;
                    // 找到average_confidence较小的那个结果
                    DetectionResult *minConfidenceResult = nullptr;
                    for (auto &result: detection_results) {
                        if (result.classIndex == duplicateClassIndex) {
                            if (minConfidenceResult == nullptr ||
                                result.average_confidence < minConfidenceResult->average_confidence) {
                                minConfidenceResult = &result;
                            }
                        }
                    }
                    // 找到一个没有出现的classIndex
                    int newClassIndex = classbegin;
                    for (; newClassIndex <= classend; newClassIndex++) {
                        if (!isClassIndexPresent(newClassIndex, detection_results)) {
                            break;
                        }
                    }
                    // 修改average_confidence较小的那个结果的classIndex
                    minConfidenceResult->classIndex = newClassIndex;
                }
            }
        }
        // 第三步
        for (const auto &result: detection_results) {
            detection_results_final.push_back(result.classIndex);
        }
    }

    void dealVector() {
        // 检查tmp是否有足够的元素
        if (detection_results_tmp.size() < 4) {
            std::cout << "tmp vector does not have enough elements." << std::endl;
            return;
        }
        // 将tmp的前四个元素移动到plant
        detection_results_plant.insert(detection_results_plant.end(), detection_results_tmp.begin(),
                                       detection_results_tmp.begin() + 4);

        // 将剩下的元素移动到fruit
        if (detection_results_tmp.size() > 4) {
            detection_results_fruit.insert(detection_results_fruit.end(), detection_results_tmp.begin() + 4,
                                           detection_results_tmp.end());
        }

        // 清空tmp
        detection_results_tmp.clear();
        ROS_WARN("Plant size: %d, Fruit size: %d", detection_results_plant.size(), detection_results_fruit.size());
        processResults_Plant(detection_results_plant, detection_results_final_plant);
        processResults_Fruit(detection_results_fruit, detection_results_final_fruit);

    }

    void playFruitSound(int maxfruit_index, int maxfruit) {
        playSound("/home/ucar/ucar_ws/src/ucar_race/res/F.wav"); //单独播报F区
        ROS_WARN("ID: %d Num: %d", maxfruit_index, maxfruit);
        ros::Duration(3.0).sleep();
        // 播报最多的水果种类
        if (fruit_sound_files.find(maxfruit_index) != fruit_sound_files.end()) {
            std::string fruit_sound = fruit_sound_files[maxfruit_index];
            playSound(fruit_sound);
            ros::Duration(3.0).sleep();
        } else {
            ROS_WARN("No sound file for fruit index: %d", maxfruit_index);
        }

        // 播报数量
        if (quantity_sound_files.find(maxfruit) != quantity_sound_files.end()) {
            std::string quantity_sound = quantity_sound_files[maxfruit];
            playSound(quantity_sound);
            ros::Duration(3.0).sleep();
        } else {
            ROS_WARN("No sound file for quantity: %d", maxfruit);
        }
    }

    void playSoundsBasedOnResults() {
        // 检查是否有检测结果
        dealVector();
        if (detection_results_final_plant.empty()) {
            ROS_WARN("No detection results to play sounds for.");
            return;
        }
        std::vector <std::string> sound_files_to_play;
        // 对于每个检测结果，播放相应的音频
        for (size_t i = 0; i < detection_results_final_plant.size(); ++i) {
            // 找到与检测结果对应的音频文件

            std::string sound_file = getSoundFileForResult(detection_results_final_plant[i]);
            if (!sound_file.empty()) {
                // 播放位置的音频文件
                if (i < location_sounds.size()) {
                    sound_files_to_play.push_back(location_sounds[i]);
                }
                // 将该音频文件添加到播放列表中
                sound_files_to_play.push_back(sound_file);
            } else {
                // 如果没有找到音频文件，打印一条消息
                ROS_WARN("No sound file associated with result!");
            }

        }
        sleep(5);
        for (auto &file: sound_files_to_play) {
            playSound(file);
            ros::Duration(3.0).sleep(); // 延迟3秒，你可以根据你的音频文件的长度来调整这个值

        }

        ROS_WARN("Finish play Plant sound");
        int classid = 0;
        for (size_t i = 0; i < detection_results_final_fruit.size(); ++i) {
            if (detection_results_final_fruit[i] == 6)
                classid = 3;
            if (detection_results_final_fruit[i] == 1 || detection_results_final_fruit[i] == 2)
                classid = 1;
            if (detection_results_final_fruit[i] == 3 || detection_results_final_fruit[i] == 4 ||
                detection_results_final_fruit[i] == 5)
                classid = 2;
            fruit_num[classid] += fruit_count[detection_results_final_fruit[i]];
        }
        int maxfruit_index = 0, maxfruit = 0;
        for (int i = 1; i <= 3; i++) {
            if (fruit_num[i] > maxfruit) {
                maxfruit = fruit_num[i];
                maxfruit_index = i;
            }
        }
        ROS_WARN("maxfruit_index: %d,maxfruit_num: %d", maxfruit_index, maxfruit);
        playFruitSound(maxfruit_index, maxfruit);
        ROS_WARN("Finish play Fruit sound");
        ROS_WARN("Finish ALL");
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


