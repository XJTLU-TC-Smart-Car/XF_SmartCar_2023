#include <ros/ros.h>
#include <sound_play/SoundRequest.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sound_play_test_node");
    ros::NodeHandle nh;

    ros::Publisher sound_pub = nh.advertise<sound_play::SoundRequest>("robotsound", 1);

    sound_play::SoundRequest sound;
    sound.sound = sound_play::SoundRequest::PLAY_FILE;
    sound.command = sound_play::SoundRequest::PLAY_ONCE;
    sound.arg = "/home/ucar/ucar_ws/src/ucar_race/res/jia.wav";  // your audio file path
    sound.volume = 1.0;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        sound_pub.publish(sound);
        loop_rate.sleep();
    }

    return 0;
}

