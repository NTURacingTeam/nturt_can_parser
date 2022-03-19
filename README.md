# nturt_can_parser
A package that enable a program to parse and encode CAN message defined on NTURT race car.
- [HackMD Doc Here](https://hackmd.io/4qtuOHkrRgCJhWhzTcpmWA?view)
# CAN message parser
###### tags: `ROS` `NTURT`
## Infos
- [Github page](https://github.com/NTURacingTeam/nturt_can_parser)
- Maintainer: TieneSabor
    - [Email](tw88712@gmail.com)

## Motivation
- Every package can use the same CAN parser without re-writing one.
### Some alias
- We have some terms which represent roughly same idea as another term.
    - CAN message/CAN frame
    - CAN ID/CAN address
- Those 2 terms are different:
    - Frame Type: it seperates CAN frames with different CAN id.
    - Data Type: it seperates CAN data with different physical meaning.
### Concept
- We will need to **Initialize** the CAN parser before doing anything.
- When we get a CAN message, we want to **REVERT** the CAN data to its physical meaning by its address.  We do this by following steps:
    - Identify the frame type by its CAN id.
    - **DECODE** the CAN data according to its frame type.
    - Acquire latest data of certain data type.
- Before we send a CAN message, we want to **TRANSFER** the physical information to CAN data by the Frame type.  We do this by following steps:
    - We get the CAN id by the Frame type.
    - We **ENCODE** the physical information to CAN data structure according to the Frame type.

## pseudo-Code/API
### MACROs
#### Frame Type
- They are the type for CAN frame(message).  Basically, every frame contains its own CAN id(address) and data(8 byte).
```c++=
// define Frame type here
#define _CP_FB1 1  // front box
#define _CP_FB2 2  // front box
#define _CP_RBX 3  // rear box
#define _CP_INV 4  // inverter
#define _CP_BMS 5  // BMS
#define _CP_HIA 6  // accelerometer frame from Honeywell IMU
#define _CP_HIG 7  // gyroscope frame from Honeywell IMU
#define _CP_HIC 8  // pose frame from Honeywell IMU
#define _CP_OIA 9  // accelerometer frame from Open IMU
#define _CP_OIG 10 // gyroscope frame from Open IMU
#define _CP_OIC 11 // pose frame from Open IMU
```
#### Data Type
- A CAN frame may contains several data type.  For example, Front Box CAN message has steer angle, throttle, brake and wheel speeds.
- To acquire certain data after decoding, we use get_DATATYPE() function to get latest information.
    - For example: get_FWS() gives you the latest wheel speed.
- The MACRO itself should **NOT** appear in the code which includes Can Parser.
```c++
// define data type here
#define _CP_ACC 1  // accelerometer
#define _CP_GYR 2  // gyrscope
#define _CP_CMP 3  // compass
#define _CP_FWS 4  // front wheel speed
#define _CP_RWS 5  // rear wheel speed
#define _CP_THR 6  // throttle
#define _CP_BRK 7  // brake
#define _CP_STR 8  // steer
#define _CP_BTP 9  // battery temp
#define _CP_BVT 10 // battery voltag
```

#### Number of Type
- If we added more types for Frames or Data, we will need to change these constants.
```c++=
// define some parameters
#define Frame_NUM 11 // how many kinds of frame here?
#define DATA_NUM 10  // how many kinds of data here?
```

#### Function Result
- If the function execute successfully, it returns required value or OK. Otherwise, it returns ERR.
```c++=
// define result state here
#ifndef OK_ERR
#define OK_ERR

#define OK -1
#define ERR 0

#endif
```

#### APIs
- Please check out [HEADER](https://github.com/NTURacingTeam/nturt_can_parser/blob/main/inc/NTURT_CAN_Parser.hpp) file.

### Usage
#### receive data
- say, we want to collect accelerometer data in this node:
```c++=
NTURT_CAN_parser parser;
int id;
int data[8];
double ax,ay,az;
// Like, we will get a new CAN message everytime
// Assume we had stored lastest CAN id and data into 'id' and 'data'
int type = parser.get_type(id);
if(type == _CP_AIU){
    int res = parser.decode(type, data);
    if(res==OK){
        parser.get_ACC(ax,ay,az);
    }
}
```

#### send data
- say, we want to send torque data to inverter:
```c++=
NTURT_CAN_parser parser;
int id;
double tbe[1];
double torque;
int data[8];
// When we are ready to send a torque data
tbe[0] = torque;
id = parser.encode(_CP_INV, tbe, data);
// Then we can publish the data and id to ROS CAN driver!
```

### Integration with ROS
- Here are examples for using **nturt_can_parser** in other ROS package.
    - [CMakeLists.txt](https://github.com/NTURacingTeam/nturt_sideslip_estimator/blob/main/CMakeLists.txt)
    - [package.xml](https://github.com/NTURacingTeam/nturt_sideslip_estimator/blob/main/package.xml)

## Development
### TODOs
- Haven't added in the parser
    - Inverter
    - BMS
    - 2nd frame of the Front box
    - Dashboard
- A configurable text file for "ID<->DATA" pair input
    - yaml/xml/txt...
    - Not hurry
    - RUN TIME Configurable?
