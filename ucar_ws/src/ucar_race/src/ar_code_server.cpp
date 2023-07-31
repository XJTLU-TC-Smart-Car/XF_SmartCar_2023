#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv2/aruco.hpp>
#include <stdio.h>
#include <iostream>
#include <string>
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <jetson-inference/imageNet.h>
#include <jetson-utils/loadImage.h>
#include <cstring>

using namespace cv;
using namespace std;

class ARCodeNode {
public:
    ARCodeNode();

    ~ARCodeNode() {}

    double fx = 421.168623;
    double cx = 316.360115;
    double fy = 420.844814;
    double cy = 242.163279;
    double k1 = -0.315867;
    double k2 = 0.094285;
    double p1 = -0.000475;
    double p2 = -0.000111;
    double k3 = 0;


    double marker_size = 100.0;//5.3; // 4.25; // cm
    int id_to_find = 0;
    double d_x_ = 0.40;// m
    double d_y_ = 0.0;// m
    double d_yaw = 0.0;// m
    double xy_goal_tolerance_ = 0.3;
    double yaw_goal_tolerance = 0.3;

    double min_vel_ = 0.03;
    double angle_min_vel_ = 0.15;
    double max_vel_ = 0.4;
    double accel_ = 0.2;
    double angle_max_vel_ = 1.5;
    double angle_accel_ = 1.5;
    double tri_s = 0.5 * max_vel_ * max_vel_ / accel_;
    nav_msgs::Odometry current_odom_;
    bool first_odom_arrived_ = false;
    ros::NodeHandle nh;
    bool if_show_debug_;
    std::string cam_dev_;
    cv::VideoCapture inputVideo;
    cv::Ptr <cv::aruco::Dictionary> dictionary;
    Mat cameraMatrix, distCoeffs;
    imageNet *net_1 = imageNet::Create(NULL, "/home/ucar/classfication_trt/plant.onnx", NULL,
                                       "/home/ucar/classfication_trt/label_plant.txt", "input", "output");

    imageNet *net_2 = imageNet::Create(NULL, "/home/ucar/classfication_trt/fruit.onnx", NULL,
                                       "/home/ucar/classfication_trt/label_fruit.txt", "input", "output");

    vector <string> class_names_1 = {"Background", "Corn_Plant", "Cucumber_Plant", "Rice_Plant", "Wheat_Plant"};
    vector <string> class_names_2 = {"Background", "Corn_1", "Corn_2", "Corn_3", "Corn_4",
                                     "Cucumber_1", "Cucumber_2", "Cucumber_3", "Cucumber_4",
                                     "Watermelon_1", "Watermelon_2", "Watermelon_3", "Watermelon_4"};

    uchar3 *imgBufferRGB = NULL;
    int detect_success = 0;
    bool detect_flag[17] = {0};

    struct DetectionResult {
        int classIndex;
        float average_confidence;
    };
    std::vector <DetectionResult> detection_results_tmp;

    int width = 640;
    int height = 480;
    int x_offset = width * 0.35;
    int y_offset_up = height * 0.4;
    int y_offset_down = height * 0.3;


    bool detectCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool parkingCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    void odomCallback(const nav_msgs::Odometry &msg);

    bool ez_cmd_move(Eigen::Vector3d position, Eigen::Vector3d euler_angle);

    bool getGoalPose(Eigen::Vector3d &T_goal_in_base, Eigen::Vector3d &Angle_goal_in_base);


};

ARCodeNode::ARCodeNode() {
    ros::NodeHandle pravite_nh("~");
    pravite_nh.param("offset_x", d_x_, 0.0);
    pravite_nh.param("offset_y", d_y_, 0.0);
    pravite_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.03);
    pravite_nh.param("if_show_debug", if_show_debug_, true);
    pravite_nh.param("cam_dev", cam_dev_, std::string("/dev/ucar_video"));
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cameraMatrix = (cv::Mat_<float>(3, 3) <<
                                          fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0);
    distCoeffs = (cv::Mat_<float>(5, 1) << k1, k2, p1, p2, k3);
    ros::ServiceServer detect_service = nh.advertiseService("detect_server", &ARCodeNode::detectCB, this);
    ros::ServiceServer parking_service = nh.advertiseService("parking_service", &ARCodeNode::parkingCB, this);
    ros::Subscriber sub = nh.subscribe("/odom", 1, &ARCodeNode::odomCallback, this);
    ROS_INFO("ar_code_server ready.");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
}

bool ARCodeNode::detectCB(std_srvs::Trigger::Request &req,
                          std_srvs::Trigger::Response &res) {
    cudaMalloc((void **) &imgBufferRGB, (width - 2 * x_offset) * sizeof(uchar3) * (height - y_offset_up - y_offset_down));
    ROS_INFO("detectCB: receive detect request.");
    int detect_timer = 0;
    int dettime_small = 0;
    int last_classIndex = -1;
    float confidence_sum = 0;
    if (!inputVideo.isOpened()) {
        inputVideo.open(cam_dev_);
        inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        inputVideo.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    }
    while (inputVideo.isOpened() && ros::ok()) {

        dettime_small++;
        cv::Mat frame, imageCopy, imageFlip;
        inputVideo >> frame;
        flip(frame, frame, 1);
        imageFlip.copyTo(imageCopy);
        if (detect_success >= 0) { //固定板参数,前两块板是大板
            x_offset = width *  0.3;
            y_offset_up = height * 0.3;
            y_offset_down = height * 0.3;
        }
//        if (detect_success >= 0) { //移动版参数，前两块板子是小板
//            x_offset = width * 0.35;
//            y_offset_up = height * 0.4;
//            y_offset_down = height * 0.3;
//        }
        if (detect_success >= 2) { //最后两个植被大识别板
            x_offset = width * 0.26;
            y_offset_up = height * 0.26;
            y_offset_down = height * 0.26;
        }
        if (detect_success >= 4) { //A点中心是特大板
            x_offset = width * 0.13;
            y_offset_up = height * 0.13;
            y_offset_down = height * 0.13;
        }
        if (detect_success >= 6) { //最后两个水果是小板
            x_offset = width * 0.31;
            y_offset_up = height * 0.35;
            y_offset_down = height * 0.32;
        }
        if (detect_success >= 8) { //最后两个水果是小板
            x_offset = width * 0.37;
            y_offset_up = height * 0.4;
            y_offset_down = height * 0.35;
        }
        Rect roi(x_offset, y_offset_up, frame.cols - 2 * x_offset, frame.rows - y_offset_up - y_offset_down);
        Mat cropped_frame = frame(roi);
        rectangle(frame, roi, Scalar(0, 255, 0), 2);
        float confidence = 0.0;
        std::string save_path = "/home/ucar/record_pic/";
//        cv::resize(cropped_frame, cropped_frame, cv::Size(224, 224));
        cvtColor(cropped_frame, cropped_frame, COLOR_BGR2RGB);

        cudaMemcpy2D((void *) imgBufferRGB, (width - 2 * x_offset) * sizeof(uchar3),
                     (void *) cropped_frame.data, cropped_frame.step,
                     (width - 2 * x_offset) * sizeof(uchar3),
                     (height - y_offset_up - y_offset_down), cudaMemcpyHostToDevice);
//        cudaMemcpy2D((void *) imgBufferRGB, (224) * sizeof(uchar3),
//                     (void *) cropped_frame.data, cropped_frame.step,
//                     (224) * sizeof(uchar3),
//                     (224), cudaMemcpyHostToDevice);

        int classIndex = 0;
        if (detect_success < 4)
            classIndex = net_1->Classify(imgBufferRGB, (width - 2 * x_offset), (height - y_offset_up - y_offset_down), &confidence);
        else
            classIndex = net_2->Classify(imgBufferRGB, (width - 2 * x_offset), (height - y_offset_up - y_offset_down), &confidence);


        float confthred = 0.6;
        if (detect_success >= 4)
            confthred = 0.5;
        if (confidence > confthred) {
            detect_timer++;
            if (classIndex != last_classIndex) {
                detect_timer = 0;
                last_classIndex = classIndex;
                confidence_sum = 0.0;
            } else {

                confidence_sum += confidence;
                std::string item_name = std::to_string(classIndex); //识别到的物品名称
                std::string count = std::to_string(detect_timer); //第几个
                std::string success_time = std::to_string(detect_success); //第几次识别
                std::string full_path_2 =
                        save_path + "success_" + success_time + "_detect_" + item_name + "_" + count + ".jpg";
                cvtColor(cropped_frame, cropped_frame, COLOR_RGB2BGR);

                cv::imwrite(full_path_2, frame);

                if (detect_timer >= 2) {
                    float average_confidence = confidence_sum / 2.0;
                    DetectionResult result;
                    result.classIndex = classIndex;
                    result.average_confidence = average_confidence;
                    detection_results_tmp.push_back(result);
                    detect_success++;
                    detect_flag[classIndex] = 1;
                    cout << "average_confidence: " << average_confidence << "detect_success:" << detect_success << endl;

                    ROS_INFO("detectCB: get id: %d", classIndex);
                    detect_timer = 0;
                    res.message = std::to_string(classIndex) + "," + std::to_string(average_confidence);
                    res.success = true;
                    inputVideo.release();
                    return true;
                }

            }
        } else {
            int classIndex_replace = 0, classbegin = 1, classend = 4;
            if (dettime_small >= 5) {
                if (detect_success < 4)
                    for (int i = classbegin; i <= classend; i++) {
                        auto it = std::find_if(detection_results_tmp.begin(), detection_results_tmp.end(),
                                               [i](const DetectionResult &dr) { return dr.classIndex == i; });
                        if (it == detection_results_tmp.end()) {
                            classIndex_replace = i;
                            break;
                        }
                    }
                detect_success++;
                res.success = true;
                ROS_INFO("detectCB: Can't detect Marker.Replace with %d", classIndex_replace);
                res.message = std::to_string(classIndex_replace) + "," + std::to_string(0.1);
                inputVideo.release();
                return true;
            }
        }
    }
    inputVideo.release();
    return true;
}


bool ARCodeNode::parkingCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    ROS_INFO("parkingCB: receive approach request.");
    Eigen::Vector3d T_goal_in_base;
    Eigen::Vector3d Angle_goal_in_base;
    bool pose_accept;
    int move_timer = 0;
    while (ros::ok() && !pose_accept) {
        move_timer++;
        bool result;
        result = getGoalPose(T_goal_in_base, Angle_goal_in_base);
        if (result == false) {//���ʧ��
            res.success = false;
            res.message = "failed.";
            return true;
        }

        cout << "T_goal_in_base" << T_goal_in_base.transpose() << endl;
        cout << "Angle_goal_in_base" << Angle_goal_in_base.transpose() << endl;
        //pose��ȷ������Χ�ڣ�
        if (T_goal_in_base[0] < xy_goal_tolerance_ &&
            T_goal_in_base[1] < xy_goal_tolerance_ &&
            Angle_goal_in_base[2] < yaw_goal_tolerance) {
            ROS_INFO("goal arrived.");
            res.success = true;
            res.message = "goal arrived.";
            return true;
        } else {
            // cmd_move(T_goal_in_base,Angle_goal_in_base);
            ez_cmd_move(T_goal_in_base, Angle_goal_in_base);
        }
        if (move_timer > 5) {
            res.success = false;
            res.message = "tried.";
            return true;
        }
    }
}

void ARCodeNode::odomCallback(const nav_msgs::Odometry &msg) {
    if (!first_odom_arrived_) {
        first_odom_arrived_ = true;
        ROS_INFO("odomCallback: receive odom_msg.");
    }
    current_odom_ = msg;
}

bool ARCodeNode::ez_cmd_move(Eigen::Vector3d position, Eigen::Vector3d euler_angle) {
    std::cout << "position: " << position << std::endl;
    std::cout << "euler_angle: " << euler_angle / 3.14159 * 180 << std::endl;
    // tf::TransformBroadcaster markers_broadcaster_;
    ros::Publisher vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Rate r(20);//20HZ
    bool move_finish = false;
    double delta_t = 0.05; // 50ms
    geometry_msgs::Twist vel_tmp;
    vel_tmp.linear.x = 0.0;
    vel_tmp.linear.y = 0.0;
    vel_tmp.angular.z = 0.0;
    double lx = abs(position[0]);
    double ly = abs(position[1]);
    double ld = std::max(sqrt(lx * lx + ly * ly) - 0.3, 0.0);
    double lyaw = abs(euler_angle[2]);
//    std::cout << "lx: " << lx << std::endl;
//    std::cout << "ly: " << ly << std::endl;
//    std::cout << "ld: " << ld << std::endl;
//    std::cout << "lyaw: " << lyaw << std::endl;


    position[0] > 0 ? vel_tmp.linear.x = lx / ld * 0.2 : vel_tmp.linear.x = lx / ld * -0.2;
    position[1] > 0 ? vel_tmp.linear.y = ly / ld * 0.2 : vel_tmp.linear.y = ly / ld * -0.2;
    while (ros::ok() && !move_finish) {
        ros::Duration tt;
        tt = ros::Time::now() - current_odom_.header.stamp;
        if (tt.toSec() > 0.05) {
            ROS_WARN("approach_server_zbar: odom msg is %f ago.", tt.toSec());
        }

        // ld -=  sqrt(vel_tmp.linear.x*vel_tmp.linear.x + vel_tmp.linear.y*vel_tmp.linear.y)*delta_t;
        ld -= sqrt(current_odom_.twist.twist.linear.x * current_odom_.twist.twist.linear.x +
                   current_odom_.twist.twist.linear.y * current_odom_.twist.twist.linear.y) * delta_t;
        // vel_tmp.linear.x = lx/ld * 0.3;
        // vel_tmp.linear.y = ly/ld * 0.3;
        vel_pub_.publish(vel_tmp);
        if (ld <= -0.01) {//&& lyaw<=0
            move_finish = true;
            geometry_msgs::Twist vel_stop;
            vel_stop.linear.x = 0.0;
            vel_stop.linear.y = 0.0;
            vel_stop.angular.z = 0.0;
            vel_pub_.publish(vel_stop);
            r.sleep();
            break;
        }
        r.sleep();
    }
    std::cout << "before rotate lyaw: " << lyaw << std::endl;
    bool rotate_finish = false;
    vel_tmp.linear.x = 0.0;
    vel_tmp.linear.y = 0.0;
    vel_tmp.angular.z = 0.0;
    r.sleep();
    while (ros::ok() && !rotate_finish) {
        // vel_yaw ����
        lyaw -= abs(current_odom_.twist.twist.angular.z) * delta_t;
        euler_angle[2] > 0 ? vel_tmp.angular.z = 0.5 : vel_tmp.angular.z = -0.5;
        // vel_tmp.angular.z =  0.5;
        vel_pub_.publish(vel_tmp);
        if (lyaw <= 0) {//&& lyaw<=0
            move_finish = true;
            geometry_msgs::Twist vel_stop;
            vel_stop.linear.x = 0.0;
            vel_stop.linear.y = 0.0;
            vel_stop.angular.z = 0.0;
            vel_pub_.publish(vel_stop);
            r.sleep();
            break;
        }
        r.sleep();
    }
    return true;
}

bool ARCodeNode::getGoalPose(Eigen::Vector3d &T_goal_in_base, Eigen::Vector3d &Angle_goal_in_base) {
    int detect_timer = 0;
    if (!inputVideo.isOpened()) {
        inputVideo.open(cam_dev_);
        inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        inputVideo.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    }
    while (inputVideo.grab() && ros::ok()) {
        cv::Mat image, imageCopy, imageFlip;
        inputVideo >> image;//ץȡ��Ƶ�е�һ����Ƭ
        if (image.empty())
            return false;
        flip(image, imageFlip, 1);//1����ˮƽ������ת180��
        imageFlip.copyTo(imageCopy);
        cvtColor(imageCopy, imageCopy, COLOR_BGR2GRAY);
        std::vector<int> ids;
        std::vector <std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(imageCopy, dictionary, corners, ids);//���б�
        if (ids.size() <= 0) {
            detect_timer++;
            if (detect_timer >= 10) {
                ROS_INFO("parkingCB: Can't detect Marker.");
                // res.success = false;
                return false;
            } else {
                continue;
            }
        }
        std::vector <cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, marker_size, cameraMatrix, distCoeffs, rvecs, tvecs);//���Pose
        int id_id = 0;
        cv::Mat R_mat;
        cv::Rodrigues(rvecs[id_id], R_mat);
        Eigen::Matrix3d matrix3;
        cv2eigen(R_mat, matrix3);
        //mark_pose_in_camera
        Eigen::Vector3d T_mark_in_cam = Eigen::Vector3d(tvecs[id_id][0] / 1000.0, tvecs[id_id][1] / 1000.0,
                                                        tvecs[id_id][2] / 1000.0); //camera frame �� marker�� ƽ��
        Eigen::Quaterniond Q_mark_in_cam(matrix3); //camera frame �� marker�� ��ת
        // tf::Quaternion quat(Q_mark_in_cam.x(),Q_mark_in_cam.y(),Q_mark_in_cam.z(),Q_mark_in_cam.w());

        //goal_pose_in_mark
        Eigen::Vector3d T_goal_in_mark = Eigen::Vector3d(0.0, d_y_, d_x_); //mark frame �� goal�� ƽ��
        Eigen::Quaterniond Q_goal_in_mark =                          //mark frame �� goal�� ��ת
                Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(3.14159 / 2, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(-3.14159 / 2, Eigen::Vector3d::UnitX());

        //camera_pose_in_base
        Eigen::Vector3d T_camera_in_base = Eigen::Vector3d(0.15, 0.0, 0.15); //mark frame �� goal�� ƽ��
        Eigen::Quaterniond Q_camera_in_base =                          //mark frame �� goal�� ��ת
                Eigen::AngleAxisd(-3.14159 * 90.0 / 180.0, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(-3.14159 * 110.0 / 180.0, Eigen::Vector3d::UnitX());

        //mark_pose_in_base
        Eigen::Vector3d T_goal_in_cam;
        T_goal_in_cam = Q_mark_in_cam * T_goal_in_mark + T_mark_in_cam;
        T_goal_in_base = Q_camera_in_base * T_goal_in_cam + T_camera_in_base;           // position
        Eigen::Quaterniond Q_goal_in_base = Q_camera_in_base * Q_mark_in_cam * Q_goal_in_mark;
        tf::Quaternion quat_goal_in_base(Q_goal_in_base.x(), Q_goal_in_base.y(), Q_goal_in_base.z(),
                                         Q_goal_in_base.w());
        double roll, pitch, yaw;//����洢r\p\y������
        tf::Matrix3x3(quat_goal_in_base).getRPY(roll, pitch, yaw);//����ת��
        std::cout << "Angle_goal_in_base(roll, pitch, yaw): (" << roll / 3.14159 * 180 << ", " << pitch / 3.14159 * 180
                  << ", " << yaw / 3.14159 * 180 << ") " << std::endl;
        Angle_goal_in_base << roll, pitch, yaw;
        break;
    }
    inputVideo.release();
    return true;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "ar_code_server");
    ARCodeNode AR;
    return 0;
}
