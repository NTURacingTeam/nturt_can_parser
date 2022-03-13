#ifndef CAN_PARSER__H
#define CAN_PARSER__H

#include <iostream>
// define result state here
#ifndef OK_ERR
#define OK_ERR

#define OK -1
#define ERR 0

#endif
// define some parameters
#define Frame_NUM 15 // how many kinds of frame here?
#define DATA_NUM 15  // how many kinds of data here?
// define Frame type here
#define _CP_FB1 1  // front box 1
#define _CP_FB2 2  // front box 2
#define _CP_RB1 3  // rear box 1
#define _CP_RB2 4  // rear box 2
#define _CP_DB1 5  // Dash Board 1
#define _CP_DB2 6  // Dash Board 2
#define _CP_INV 7  // inverter
#define _CP_BMS 8  // BMS
#define _CP_HIA 9  // accelerometer frame from Honeywell IMU
#define _CP_HIG 10 // gyroscope frame from Honeywell IMU
#define _CP_HIC 11 // slope frame from Honeywell IMU
#define _CP_OIS 12 // slope frame from Open IMU
#define _CP_OIA 13 // accelerometer frame from Open IMU
#define _CP_OIG 14 // gyroscope frame from Open IMU
#define _CP_OIC 15 // pose frame from Open IMU
// and so on...
// define data type here
#define _CP_ACC 1   // accelerometer
#define _CP_GYR 2   // gyrscope
#define _CP_CMP 3   // compass
#define _CP_SLP 4   // slope
#define _CP_FWS 5   // front wheel speed
#define _CP_RWS 6   // rear wheel speed
#define _CP_FWT 7   // front wheel temp
#define _CP_RWT 8   // rear wheel temp
#define _CP_FSS 9   // front wheel suspension stroke
#define _CP_RSS 10  // rear wheel suspension stroke
#define _CP_THR 11  // throttle
#define _CP_BRK 12  // brake
#define _CP_STR 13  // steer
#define _CP_BTP 14  // battery temp
#define _CP_BVT 15  // battery voltage
// and so on...
// define Constants
#define TWOPOW08 256
#define TWOPOW26 67108864

class NTURT_CAN_parser {
public:
  /* parameter:
   *   Frame type
   *   CAN id (or PGN of the id)
   * return:
   *   return OK or ERR
   */
  int assign_id(int type, int id) {
    int idsize = sizeof(id_) / sizeof(int); // Should be Frame_NUM
    if (type < idsize) {
      id_[type] = id;
      return OK;
    } else {
      return ERR;
    }
  }

  /* parameter:
   * return:
   *   return OK or ERR
   */
  int print_id() {
    for (int i = 1; i < Frame_NUM; i++) {
      std::cout << "type: " << i << ", id: " << id_[i] << std::endl;
    }
    return OK;
  }

  /* parameter:
   * return:
   *   return OK or ERR
   * Get PGN for a CAN ID
   * Check here for PGN meanings:
   * https://www.kvaser.com/about-can/higher-layer-protocols/j1939-introduction/
   */
  int get_PGN(int id) {
    // we want the
    id /= TWOPOW08;
    id = id % TWOPOW26;
    return id;
  }

  /* parameter:
   *   none
   * return:
   *   OK or ERR
   * assign all ID
   */
  int init_parser();

  /* parameter:
   *   CAN id (or PGN of the id)
   * return:
   *   Frame type
   */
  int get_type(int id) {
    // Check with full ID
    for (int i = 1; i <= Frame_NUM; i++) {
      if (id_[i] == id) {
        return i;
      }
    }
    // check with PGN
    for (int i = 1; i <= Frame_NUM; i++) {
      if (id_[i] == get_PGN(id)) {
        return i;
      }
    }
    return ERR;
  }

  /* parameter:
   *   Frame type
   * return:
   *   CAN id (or PGN of the id)
   */
  int get_id(int type) { return id_[type]; }

  /* parameters:
   *   Frame type
   *   CAN data (uint[8])
   * return:
   *   OK or ERR
   * It decode data according to Frame type,
   * and it store the result to the class member variable.
   */
  int decode(int type, int *data);

  /* parameters:
   *   Frame type
   *   data to-be-encoded (double [NUM])
   *   CAN data (uint[8])
   * return:
   *   CAN id
   * It encode tbe (data to-be-encoded) according to Frame type,
   * and it store the result to data (int[8]).
   */
  int encode(int type, double *tbe, int *data);

  /* parameters:
   *   variables to-be-updated(i.e. sensor data)
   *   Here they are accelerometer data
   * return:
   *   OK or ERR
   */
  int get_ACC(double &ax, double &ay, double &az);

  /* parameters:
   *   variables to-be-updated(i.e. sensor data)
   *   Here they are gyroscope data
   * return:
   *   OK or ERR
   */
  int get_GYR(double &gx, double &gy, double &gz);

  /* parameters:
   *   variables to-be-updated(i.e. sensor data)
   *   Here they are pose (compass) data (roll pitch yaw)
   * return:
   *   OK or ERR
   */
  int get_CMP(double &cx, double &cy, double &cz);

  /* parameters:
   *   variables to-be-updated(i.e. sensor data)
   *   Here they are front wheel speed data (left/right)
   * return:
   *   OK or ERR
   */
  int get_FWS(double &fws_l, double &fws_r);

  /* parameters:
   *   variables to-be-updated(i.e. sensor data)
   *   Here they are rear wheel speed data (left/right)
   * return:
   *   OK or ERR
   */
  int get_RWS(double &rws_l, double &rws_r);

  /* parameters:
   *   variables to-be-updated(i.e. sensor data)
   *   Here they are throttle data
   * return:
   *   OK or ERR
   */
  int get_THR(double &th_1, double &th_2);

  /* parameters:
   *   variables to-be-updated(i.e. sensor data)
   *   Here they are brake data
   * return:
   *   OK or ERR
   */
  int get_BRK(double &brk);

  /* parameters:
   *   variables to-be-updated(i.e. sensor data)
   *   Here they are steer data
   * return:
   *   OK or ERR
   */
  int get_STR(double &str);
  // and BMS data...
private:
  /* The id of different CAN message frame.
   * ex: id[FBX] is the id of front box messages
   */
  int id_[Frame_NUM + 1];

  /* The flag of different sensor data, which indicates if there is new data.
   * ex: flag[ACC] means if we got new accelerometer data.
   * If we got message from a node,
   * then flag[data from that node] will be 1.
   * If we accessed this data via get_XXX(),
   * then flag[the data] will be 0
   */
  int flag_[DATA_NUM + 1];

  // declare every data as member function
  // IMU
  double accx_, accy_, accz_, gyrx_, gyry_, gyrz_, cmpx_, cmpy_, cmpz_;
  // Front box
  double brk_, thr1_, thr2_, str_;
  // Wheel Speed
  double fws_l_, fws_r_, rws_l_, rws_r_;
  // Suspension ride height
  double frh_l_, frh_r_, rrh_l_, rrh_r_;
  // Rear box
  // BMS
  // Inverter
  // Dashboard
  // and so on.......
};

typedef NTURT_CAN_parser Parser;

#endif
