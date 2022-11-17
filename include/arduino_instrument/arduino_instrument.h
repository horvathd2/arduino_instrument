#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Byte.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <unistd.h>

class ArduinoInstrument
{
    public:
        ArduinoInstrument(ros::NodeHandle node, float loopRate, std::string rosserialTopic, 
                          std::string interfaceCommandsTopic, std::string arduinoEncodersTopic,
                          std::string spacenavTopic);

        ~ArduinoInstrument();

        void publishInstrumentData();

    private:

        ros::NodeHandle node;
        ros::Rate loopRate;

        ros::Publisher rosserial_pub;
        ros::Subscriber interface_commands_sub;
        ros::Subscriber arduino_encoders_sub;
        ros::Subscriber spacenav_sub;

        std::string rosserialTopic;
        std::string interfaceCommandsTopic;
        std::string arduinoEncodersTopic;
        std::string spacenavTopic;

        std_msgs::Float64MultiArray interfaceCommands;
        std_msgs::Byte commandByte;
        geometry_msgs::Vector3 encoderMessage;

        bool switchState;
        bool manual;
        bool homing;
        bool ok;

        bool forward;
        bool backward;

        unsigned int fwdM1;
        unsigned int bwdM1;
        unsigned int fwdM2;
        unsigned int bwdM2;

        unsigned int microsecond;

        int encoderValues[2];

        double insertionSpeed;
        double insertionDepth;
        long inDepthRes;

        void InterfaceCommandsCallback(const std_msgs::Float64MultiArray::ConstPtr &data);

        void InstrumentEncodersCallback(const geometry_msgs::Vector3::ConstPtr &data);

        void SpacenavCallback(const geometry_msgs::Vector3::ConstPtr &data);
};