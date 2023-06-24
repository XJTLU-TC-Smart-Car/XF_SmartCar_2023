#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <xf_mic_asr_offline/Set_Awake_Word_srv.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Trigger.h>
#include <boost/thread.hpp>
#include <sstream>

class UCarRace
{
public:
  UCarRace();
  ~UCarRace(){}
  ros::NodeHandle nh_;
  bool if_show_debug_;
  std::string wake_up_words_;
  ros::ServiceClient set_wake_words_client_;
  bool has_set_wake_word_;
  ros::Subscriber wake_up_sub_;
  ros::Publisher init_pub_;
  bool has_wake_up_;
  
  std::string detect_voice_1_, detect_voice_2_, detect_voice_3_, finish_voice_1_, finish_voice_2_, finish_voice_3_;
  double code_point_6_x_,  code_point_6_y_,  code_point_6_yaw_, 
         code_point_1_x_,  code_point_1_y_,  code_point_1_yaw_, 
         code_point_2_x_,  code_point_2_y_,  code_point_2_yaw_, 
         code_point_3_x_,  code_point_3_y_,  code_point_3_yaw_, 
         code_point_4_x_,  code_point_4_y_,  code_point_4_yaw_,
         code_point_5_x_,  code_point_5_y_,  code_point_5_yaw_,
         end_point_1_x_, end_point_1_y_, end_point_1_yaw_, 
         start_point_y_, start_point_x_,start_point_yaw_;
  move_base_msgs::MoveBaseGoal  code_point, end_point;
  ros::ServiceClient code_detect_client_;
  std::string code_info_;
  int code_id_;
  // ros::ServiceClient approach_client_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *MBC_ptr_;
  void wakeUpCallBack(std_msgs::Int32 msg){
    if (has_set_wake_word_)
    {
      has_wake_up_ = true;
      ROS_INFO("wakeUpCallBack: Get wake angle msg: [%d]", msg.data);
      ROS_INFO("wakeUpCallBack: has_set_wake_words: [%d]", wake_up_words_);
    }
    ROS_INFO("wakeUpCallBack: Get wake angle msg: [%d]",msg.data);
  }
};

UCarRace::UCarRace(): if_show_debug_(true), has_set_wake_word_(false), has_wake_up_(false), code_id_(-99)
{
  ros::NodeHandle pravite_nh("~");
  pravite_nh.param("wake_up_words",   wake_up_words_,  std::string("开始导航"));
  pravite_nh.param("detect_voice_1",  detect_voice_1_, std::string(""));
  pravite_nh.param("detect_voice_2",  detect_voice_2_, std::string(""));
  pravite_nh.param("detect_voice_3",  detect_voice_3_, std::string(""));
  pravite_nh.param("finish_voice_1",  finish_voice_1_, std::string(""));
  pravite_nh.param("finish_voice_2",  finish_voice_2_, std::string(""));
  pravite_nh.param("finish_voice_3",  finish_voice_3_, std::string(""));
  pravite_nh.param("start_point_x_",   start_point_x_,   0.0);
  pravite_nh.param("start_point_y_",   start_point_y_,   0.0);
  pravite_nh.param("start_point_yaw_", start_point_yaw_, 0.0);
  pravite_nh.param("code_point_1_x_",    code_point_1_x_,    0.0);
  pravite_nh.param("code_point_1_y_",    code_point_1_y_,    0.0);
  pravite_nh.param("code_point_1_yaw_",  code_point_1_yaw_,  0.0); 
  pravite_nh.param("code_point_2_x_",    code_point_2_x_,    0.0);
  pravite_nh.param("code_point_2_y_",    code_point_2_y_,    0.0);
  pravite_nh.param("code_point_2_yaw_",  code_point_2_yaw_,  0.0);
  pravite_nh.param("code_point_3_x_",    code_point_3_x_,    0.0);
  pravite_nh.param("code_point_3_y_",    code_point_3_y_,    0.0);
  pravite_nh.param("code_point_3_yaw_",  code_point_3_yaw_,  0.0);
  pravite_nh.param("code_point_4_x_",    code_point_4_x_,    0.0);
  pravite_nh.param("code_point_4_y_",    code_point_4_y_,    0.0);
  pravite_nh.param("code_point_4_yaw_",  code_point_4_yaw_,  0.0);
  pravite_nh.param("code_point_5_x_",    code_point_5_x_,    0.0);
  pravite_nh.param("code_point_5_y_",    code_point_5_y_,    0.0);
  pravite_nh.param("code_point_5_yaw_",  code_point_5_yaw_,  0.0);  
  pravite_nh.param("code_point_6_x_",    code_point_6_x_,    0.0);
  pravite_nh.param("code_point_6_y_",    code_point_6_y_,    0.0);
  pravite_nh.param("code_point_6_yaw_",  code_point_6_yaw_,  0.0);
  pravite_nh.param("end_point_x_",   end_point_1_x_,   0.0);
  pravite_nh.param("end_point_y_",   end_point_1_y_,   0.0);
  pravite_nh.param("end_point_yaw_", end_point_1_yaw_, 0.0);
 
  pravite_nh.param("if_show_debug",   if_show_debug_,   true);
  wake_up_sub_ = nh_.subscribe("/mic/awake/angle", 10, &UCarRace::wakeUpCallBack, this);
  init_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1);
  MBC_ptr_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base",true);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::Duration(0.5).sleep();

  // step_0: 设置初始位置
  geometry_msgs::PoseWithCovarianceStamped init_pose;
  init_pose.header.stamp = ros::Time::now();
  init_pose.header.frame_id = std::string("map");
  init_pose.pose.pose.position.x = start_point_x_;
  init_pose.pose.pose.position.y = start_point_y_;
  init_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(start_point_yaw_);
  init_pose.pose.covariance[6 * 0 + 0] = 0.5 * 0.5;
  init_pose.pose.covariance[6 * 1 + 1] = 0.5 * 0.5;
  init_pose.pose.covariance[6 * 3 + 3] = 3.141593 / 12.0 * 3.141593 / 12.0;
  init_pub_.publish(init_pose);
  ROS_INFO("Set pose finish.");

  // step_1: 设置唤醒词
  set_wake_words_client_ = nh_.serviceClient<xf_mic_asr_offline::Set_Awake_Word_srv>("/xf_asr_offline_node/set_awake_word_srv");
  xf_mic_asr_offline::Set_Awake_Word_srv set_wake_srv;
  set_wake_srv.request.awake_word = wake_up_words_;
  ros::Rate loop_rate(10);
  while (ros::ok()&&!has_set_wake_word_)
  {
    try
    {
      set_wake_words_client_.call(set_wake_srv);
      if (set_wake_srv.response.result == std::string("ok"))
      {
        has_set_wake_word_ = true;
        break;
      }
      else if(set_wake_srv.response.result == std::string("false"))
      {
        ROS_ERROR("set_wake_words faild. fail_reason: %s",set_wake_srv.response.fail_reason);
      }
      else
      {
        ROS_ERROR("set_wake_words error. fail_reason: %s",set_wake_srv.response.fail_reason);
      }
    }
    catch(const std::exception& e)
    {
      std::cerr << "Call set_wake_word faild. Maybe service not ready." << e.what() << '\n';
    }
    loop_rate.sleep();
  }
  ROS_INFO("set wake up words as: %s", wake_up_words_);

  
  // step_2: 等待唤醒
  ROS_INFO("wait for wake up.");
  while (ros::ok()&&!has_wake_up_)
  {
    loop_rate.sleep();
  }
  ROS_INFO("Start the game!");

  // step_3: 导航至1点
  ROS_INFO("Go to first point.");
  code_point.target_pose.header.stamp = ros::Time::now();
  code_point.target_pose.header.frame_id = "map";
  code_point.target_pose.pose.position.x = code_point_1_x_;
  code_point.target_pose.pose.position.y = code_point_1_y_;
  code_point.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(code_point_1_yaw_);

  MBC_ptr_->sendGoal(code_point);
  
  while (ros::ok())
  {
    MBC_ptr_->waitForResult();
    if(MBC_ptr_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Arrived detect point. ");
      break;
    }
    else{
      ROS_INFO("Car failed to move to detect_point.");
      // todo 完成自己的异常处理逻辑
    }
  }
 // step_3: 导航至2点
  ROS_INFO("Go to second point.");
  code_point.target_pose.header.stamp = ros::Time::now();
  code_point.target_pose.header.frame_id = "map";
  code_point.target_pose.pose.position.x = code_point_1_x_;
  code_point.target_pose.pose.position.y = code_point_1_y_;
  code_point.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(code_point_1_yaw_);

  MBC_ptr_->sendGoal(code_point);
  
  while (ros::ok())
  {
    MBC_ptr_->waitForResult();
    if(MBC_ptr_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Arrived detect point. ");
      break;
    }
    else{
      ROS_INFO("Car failed to move to detect_point.");
      // todo 完成自己的异常处理逻辑
    }
  }

  
  


  // step_5: 导航至终点
    ROS_INFO("Go to end point.");
    end_point.target_pose.header.stamp = ros::Time::now();
    end_point.target_pose.header.frame_id = "map";
    end_point.target_pose.pose.position.x = end_point_1_x_;
    end_point.target_pose.pose.position.y = end_point_1_y_;
    end_point.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(end_point_1_yaw_);
  

  MBC_ptr_->sendGoal(end_point);
  while (ros::ok())
  {
    MBC_ptr_->waitForResult();
    if(MBC_ptr_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Arrived end point");
      break;
    }
    else{
      ROS_INFO("The car failed to move to end_point.");
      // todo 完成自己的异常处理逻辑
    }
  }

 
  
}


int main(int argc, char** argv){
  ros::init(argc, argv, "CarRace_node");
  UCarRace UR;
  return 0;
}