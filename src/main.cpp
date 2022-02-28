#include <NTURT_CAN_Parser.hpp>
#include <iostream>

// This program is to 
// - Validate the CAN Parser
// - Demostrate the usage
int main(){
    // For example, we want to recognize 
    // "front box frame 1" and collect data
    Parser myparser;
    myparser.assign_id(_CP_FB1, 0x040AD091);
    // Say, we are in some callbacks and got a
    // can message.  Then we store the can id 
    // and data into our variables.
    int id = 0x040AD091;
    int data[8];
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    // Then we can decode the CAN data and get, for
    // example, front wheel speed
    double fwsr, fwsl;
    int tmptype = myparser.get_type(id);
    if(tmptype == _CP_FB1){
        if(myparser.decode(_CP_FB1, data)==-1){
            myparser.get_FWS(fwsl, fwsr);
        }
    }
    std::cout<<"Front wheel speed left: "<<fwsl
            <<", right: "<<fwsr<<std::endl;
    // Another example is that we may want to encode torque data
    // which we are going to send it to Inverter.
    double tbe[1];
    double torque = 0;
    tbe[0] = torque;
    myparser.encode(_CP_INV, tbe, data);
    // Then we can send the "data" to CAN
    return 0;
}