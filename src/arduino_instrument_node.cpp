#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <memory>
#include <arduino_instrument/arduino_instrument.h>

int main(int argc, char** argv){

    ros::init(argc,argv,"arduino_instrument_node");
    ros::NodeHandle node;

    ArduinoInstrument instrument(node, 600, "/arduino_instrument/commands", "/interface/commands",
                                            "/arduino_instrument/encoders", "/spacenav/joy");

    instrument.publishInstrumentData();

    ros::shutdown();

}