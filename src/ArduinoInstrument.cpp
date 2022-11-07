#include <arduino_instrument/arduino_instrument.h>

ArduinoInstrument::ArduinoInstrument(ros::NodeHandle node, float loopRate, std::string rosserialTopic, 
                                     std::string interfaceCommandsTopic, std::string arduinoEncodersTopic):
                         
                            node(node), loopRate(loopRate), interfaceCommandsTopic(interfaceCommandsTopic), 
                            rosserialTopic(rosserialTopic), arduinoEncodersTopic(arduinoEncodersTopic){

                            this->encoderValues[2] = {0}; 
                            this->switchState = false;
                            this->manual = true;
                            this->homing = false;

                            this->commandByte.data=0;

                            this->insertionSpeed = 0.0;
                            this->insertionDepth = 0.0;
                            this->inDepthRes = 0;

                            this->fwdM1 = 0;
                            this->fwdM2 = 0;
                            this->bwdM1 = 0;
                            this->bwdM2 = 0;

                            this->rosserial_pub = this->node.advertise<std_msgs::Byte>(this->rosserialTopic.c_str(),1);  
                            this->interface_commands_sub = this->node.subscribe<std_msgs::Float64MultiArray>(this->interfaceCommandsTopic.c_str(),1, &ArduinoInstrument::InterfaceCommandsCallback, this);
                            this->arduino_encoders_sub = this->node.subscribe<geometry_msgs::Vector3>(this->arduinoEncodersTopic.c_str(),1, &ArduinoInstrument::InstrumentEncodersCallback, this);

                            }

ArduinoInstrument::~ArduinoInstrument(){
    //Virtual Constructor
}

void ArduinoInstrument::publishInstrumentData(){
    ros::AsyncSpinner spinner(2);
    spinner.start();

    std::cout<<"[LOG] Starting node for communication between MATLAB interface and ROSSERIAL..."<<std::endl;

    if(ros::ok())
        std::cout<<"[LOG] Started!"<<std::endl;

    while(ros::ok()){
        this->rosserial_pub.publish(commandByte);
            
        this->loopRate.sleep();
    }

    spinner.stop();
    std::cout<<"[LOG] Shutting down..."<<std::endl;
}

/// @brief 
/// @param data 
void ArduinoInstrument::InterfaceCommandsCallback(const std_msgs::Float64MultiArray::ConstPtr &data){
    // MESSAGE STRUCTURE ACCORDING TO nonstd_msgs/Float69MultiArray

    //-----------------------------------------------------------------
    if(data->data[12]==1.0){                            // RFA START
        if(data->data[3]==1.0)
            this->manual = true;                        // MANUAL
        else
            this->manual = false;                       // AUTO

        if(data->data[9]==1.0)
            this->homing = true;                        // HOMING MOTORS
        else
            this->homing = false;                       // HOMING DISABLED

        if(data->data[10]==1.0)
            this->switchState = true;                   // HAPTIC PRESSED
        else{
            this->switchState = false;                  // HAPTIC RELEASED
            this->commandByte.data = 0;
        }

        if(homing){
            fwdM1=21;
            bwdM1=17;
            fwdM2=26;
            bwdM2=18;
        }else{                                          
            if(data->data[8]==1.0){                     // INSERTION SPEED FAST NOT HOMING
                fwdM1=37;
                bwdM1=33;
                fwdM2=42;
                bwdM2=34;
            }else{                                      // INSERTION SPEED SLOW NOT HOMING
                fwdM1=5;
                bwdM1=1;
                fwdM2=10;
                bwdM2=2;
            }
        }                       
            
        this->insertionDepth = data->data[7];           // INSERTION DEPTH
        this->inDepthRes = (insertionDepth*73000)/9.5;
        std::cout<<inDepthRes<<std::endl;

    //-----------------------------------------------------------------
        if(manual){
            if(data->data[4]==1.0){                                 // NEEDLE CONTROL

                if(data->data[5]==1.0 && data->data[10]==1.0){ 
                    this->commandByte.data = fwdM1;                 // >NEEDLE FORWARD<
                }else if(data->data[5]==0.0 && data->data[10]==1.0){ 
                    this->commandByte.data = bwdM1;                 // >NEEDLE BACKWARD<
                }

            }else if(data->data[4]==0.0){                           // ELECTRODE CONTROL

                if(data->data[6]==1.0 && data->data[10]==1.0){  
                    this->commandByte.data = fwdM2;                 // >ELECTRODE FORWARD<
                }else if(data->data[6]==0.0 && data->data[10]==1.0){
                    this->commandByte.data = bwdM2;                 // >ELECTRODE BACKWARD<
                }

            }
        }else{ 
            if(data->data[11]==1.0){ 
                if(encoderValues[1]>=inDepthRes)                    // >AUTO INSERT<               
                    this->commandByte.data = fwdM2;                 //AUTO INSERT ELECTRODE
                else
                    this->commandByte.data = fwdM1;                 //AUTO INSERT NEEDLE

            }else if(data->data[11]==0.0){                                              
                if(encoderValues[0]<=0)                             // >AUTO RETRACT<
                    this->commandByte.data = bwdM1;                 //AUTO RETRACT NEEDLE
                else
                    this->commandByte.data = bwdM2;                 //AUTO RETRACT ELECTRODE

            }else if(homing){
                if(encoderValues[0]<=0)                             // >HOME RETRACT<
                    this->commandByte.data = bwdM1;                 //RETRACT NEEDLE
                else
                    this->commandByte.data = bwdM2;                 //RETRACT ELECTRODE
            }
        }
        
    //-----------------------------------------------------------------
    }else{
        this->commandByte.data = 0;                                 // STOP ALL
    }
    //-----------------------------------------------------------------
}

void ArduinoInstrument::InstrumentEncodersCallback(const geometry_msgs::Vector3::ConstPtr &data){
    this->encoderValues[0] = data->x;
    this->encoderValues[1] = data->y;

    std::cout<<encoderValues[0]<<" "<<encoderValues[1]<<std::endl;
}