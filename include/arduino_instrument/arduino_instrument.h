#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Byte.h>
#include <geometry_msgs/Vector3.h>

class ArduinoInstrument
{
    public:
        ArduinoInstrument(ros::NodeHandle node, float loopRate, std::string rosserialTopic, 
                          std::string interfaceCommandsTopic, std::string arduinoEncodersTopic);

        ~ArduinoInstrument();

        void publishInstrumentData();

    private:

        ros::NodeHandle node;
        ros::Rate loopRate;

        ros::Publisher rosserial_pub;
        ros::Subscriber interface_commands_sub;
        ros::Subscriber arduino_encoders_sub;

        std::string rosserialTopic;
        std::string interfaceCommandsTopic;
        std::string arduinoEncodersTopic;

        std_msgs::Float64MultiArray interfaceCommands;
        std_msgs::Byte commandByte;
        geometry_msgs::Vector3 encoderMessage;

        bool switchState;
        bool manual;
        bool homing;
        int encoderValues[2];

        double insertionSpeed;
        double insertionDepth;

        void InterfaceCommandsCallback(const std_msgs::Float64MultiArray::ConstPtr &data);

        void InstrumentEncodersCallback(const geometry_msgs::Vector3::ConstPtr &data);
};