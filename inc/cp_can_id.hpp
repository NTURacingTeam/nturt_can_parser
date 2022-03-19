#ifndef _CP_CAN_ID_HPP
#define _CP_CAN_ID_HPP

// define Frame ID here
#define _CAN_FB1 0x040AD091 // front box 1
#define _CAN_FB2 0x040AD092 // front box 2
#define _CAN_RB1 0x040AD093 // rear box 1
#define _CAN_RB2 0x040AD094 // rear box 2
#define _CAN_DB1 5          // Dash Board 1
#define _CAN_DB2 6          // Dash Board 2
#define _CAN_MCM 0x0C0      // inverter
#define _CAN_BMS 8          // BMS
#define _CAN_HIS 0x0CF029E2 // slope frame from Honeywell IMU
#define _CAN_HIA 0x08F02DE2 // accelerometer frame from Honeywell IMU
#define _CAN_HIG 0x0CF02AE2 // gyroscope frame from Honeywell IMU
#define _CAN_OIS 0x0CF029E2 // slope frame from OpenIMU
#define _CAN_OIA 0x08F02DE2 // accelerometer frame from OpenIMU
#define _CAN_OIG 0x0CF02AE2 // gyroscope frame from OpenIMU
#define _CAN_OIC 0x18FF6AE2 // pose frame from OpenIMU

#endif