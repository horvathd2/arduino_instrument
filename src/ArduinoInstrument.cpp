#include <arduino_instrument/arduino_instrument.h>

ArduinoInstrument::ArduinoInstrument(ros::NodeHandle node, float loopRate, std::string rosserialTopic, 
                                     std::string interfaceCommandsTopic, std::string arduinoEncodersTopic):
                         
                            node(node), loopRate(loopRate), interfaceCommandsTopic(interfaceCommandsTopic), 
                            rosserialTopic(rosserialTopic), arduinoEncodersTopic(arduinoEncodersTopic){

                            this->encoderValues[2] = {0}; 
                            this->switchState = false;
                            this->manual = true;
                            this->homing = false;
                            this->ok = false;
                            this->commandByte.data=0;

                            this->insertionSpeed = 0.0;
                            this->insertionDepth = 0.0;

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
        if(manual){
            if(switchState){
                this->rosserial_pub.publish(commandByte);      // PUBLISH ONLY IF SWITCH PRESSED
                ok = true;
                std::cout<<"PRESSED"<<std::endl;
            }
                
            //std::cout<<"MANUAL"<<std::endl;
        }else{
            this->rosserial_pub.publish(commandByte);          // PUBLISH CONTINUOUSLY ON AUTO
            //std::cout<<"AUTO"<<std::endl;
        }

        if(homing){
            this->rosserial_pub.publish(commandByte);          // PUBLISH CONTINUOUSLY WHILE HOMING
            //std::cout<<"HOMING"<<std::endl;
        }
            
        this->loopRate.sleep();
    }

    spinner.stop();
    std::cout<<"[LOG] Shutting down..."<<std::endl;
}

void ArduinoInstrument::InterfaceCommandsCallback(const std_msgs::Float64MultiArray::ConstPtr &data){
    // MESSAGE STRUCTURE ACCORDING TO nonstd_msgs/Float69MultiArray

    //--------------------------------------------------
    if(data->data[12]==1.0){                            // RFA START
        if(data->data[3]==1.0)
            this->manual = true;                        // MANUAL
        else
            this->manual = false;                       // AUTO

        if(data->data[9]==1.0)
            this->homing = true;                        // HOMING MOTORS
        else{
            this->homing = false;                       // HOMING DISABLED
        }

        if(data->data[10]==1.0)
            this->switchState = true;                   // HAPTIC PRESSED
        else{
            this->switchState = false;                  // HAPTIC RELEASED
            ok=false;
        }

        this->insertionSpeed = data->data[8];           // INSERTION SPEED
        this->insertionDepth = data->data[7];           // INSERTION DEPTH

    //--------------------------------------------------
        if(manual){
            if(data->data[4]==0.0){                     // NEEDLE CONTROL

                if(data->data[5]==1.0){ 
                    this->commandByte.data=37;          // >NEEDLE FORWARD<
                }else if(data->data[5]==0.0){ 
                    this->commandByte.data=33;          // >NEEDLE BACKWARD<
                }

            }else{                                      // ELECTRODE CONTROL

                if(data->data[6]==1.0){  
                    this->commandByte.data=42;;         // >ELECTRODE FORWARD<
                }else if(data->data[6]==0.0){
                    this->commandByte.data=34;          // >ELECTRODE BACKWARD<
                }

            }
        }else{
            if(data->data[11]==1.0){                
                this->commandByte.data=0;               // >AUTO INSERT<
            }else if(data->data[11]==0.0){       
                this->commandByte.data=0;               // >AUTO RETRACT<
            }
        }

        if(homing){
            this->commandByte.data=16;                  // HOME MOTORS
        }
    //--------------------------------------------------
    }else{
        this->commandByte.data=0;                       // STOP ALL
        ok=false;
    }
}

void ArduinoInstrument::InstrumentEncodersCallback(const geometry_msgs::Vector3::ConstPtr &data){
    this->encoderValues[0] = data->x;
    this->encoderValues[1] = data->y;

    std::cout<<encoderValues[0]<<" "<<encoderValues[1]<<std::endl;
}