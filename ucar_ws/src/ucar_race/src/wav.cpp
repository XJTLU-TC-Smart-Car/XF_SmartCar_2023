#include <ros/ros.h>
#include <sound_play/sound_play.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "sound_play_test"); // 初始化ROS节点
    ros::NodeHandle nh; // 创建节点句柄

    // 创建SoundClient对象
    sound_play::SoundClient sc;

    // 允许初始化
    sleep(1);

    // 播放文件
    sc.startWaveFromPkg("ucar_race", "res/jia.wav");

    // 等待文件播放完成
    sleep(3);
    
    return 0;
}
