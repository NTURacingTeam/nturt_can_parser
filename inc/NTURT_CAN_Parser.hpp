#ifndef CAN_PARSER__H
#define CAN_PARSER__H

#define BOOST_ARRAY

#include <math.h>
#include <stdio.h>

#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#ifdef BOOST_ARRAY
#include <boost/array.hpp>
#endif
// define result state here
#ifndef OK_ERR
#define OK_ERR

#define OK -1
#define ERR 0

#endif
// define can data properties
#define _CP_BIT 1
#define _CP_BYTE 2
#define _CP_BIG 3
#define _CP_LITTLE 4
// define Constants
#define TWOPOW08 256
#define TWOPOW26 67108864
// define mask
#define _CP_MASK_LAST_8_BIT 255 // b0000000011111111

// for each frame, thay have id and a list of data, we f()store the key and its
// component, so there are 2 string
typedef struct can_frame {
  std::vector<std::pair<std::string, std::string>> data_key;
} Frame;

// for each data type, there is certain rule to convert between can data and
// physical value
typedef struct can_data_rule {
  // can frame id and type
  int id;
  // store in a manner of bit or byte
  int bitbyte;
  // big/small endian
  int endian;
  // if start = 2 and stop = 4, we consider 2nd and 3rd bit/byte
  int startbyte, stopbyte;
  int startbit, stopbit;
  int ifsigned;
  // physical value = (can data * scale) + offset
  double scale, offset;
} Rule;

class NTURT_CAN_parser {
public:
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

  /*  */
  int check_key(int id, std::string key, std::string comp) {
    for (auto keys : frame_[id].data_key) {
      if (keys.first.compare(key) == 0) {
        if (keys.second.compare(comp) == 0) {
          return OK;
        }
      }
    }
    return ERR;
  }
  int check_key(int id, std::string key) {
    for (auto keys : frame_[id].data_key) {
      if (keys.first.compare(key) == 0) {
        return OK;
      }
    }
    return ERR;
  }

  /*
   */
  std::vector<std::pair<std::string, std::string>> get_key(int id) {
    if (frame_.find(id) == frame_.end()) {
      err_log(__func__, "No such frame id");
      return std::vector<std::pair<std::string, std::string>>();
    }
    return frame_[id].data_key;
  }

  /* parameter:
   *   none
   * return:
   *   OK or ERR
   * assign all ID
   */
  int init_parser();

  /**/
  int update_rule(std::string key, std::string comp, int &id, int &bitbyte,
                  int &endian, int &startbyte, int &stopbyte, int &startbit,
                  int &stopbit, int &ifsigned, double &scale, double &offset);

  /*  */
  int import_rule(std::string path);

  /*  */
  void err_log(const char *fun, std::string msg) {
    err += "[" + std::string(fun) + "] " + msg + "\r\n";
  }

  /*  */
  void get_err_log(std::string &_err) {
    _err = err;
    err.clear();
  }

  /*  */
  void print_err_log() {
    std::cout << err;
    err.clear();
  }

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
  int encode(int type, int *data);
#ifdef BOOST_ARRAY
  int decode(int id, const boost::array<unsigned char, 8> data);
  int encode(int id, boost::array<unsigned char, 8> &data);
#endif

  /**/
  int set_tbe(std::string key, std::map<std::string, double> &_tbe);

  /**/
  int set_tbe(std::string key, std::string comp, double _tbe);

  /**/
  int get_afd(std::string key, std::map<std::string, double> &_afd);

  /**/
  double get_afd(std::string key, std::string comp);

  /* parameters:
   *   variables to-be-updated(i.e. sensor data)
   *   Here they are accelerometer data
   * return:
   *   OK or ERR
   */
private:
  /* The id of different CAN message frame.
   * ex: id[FBX] is the id of front box messages
   */
  // int id_[Frame_NUM + 1];
  std::map<int, Frame> frame_;

  /* The flag of different sensor data, which indicates if there is new data.
   * ex: flag[ACC] means if we got new accelerometer data.
   * If we got message from a node,
   * then flag[data from that node] will be 1.
   * If we accessed this data via get_XXX(),
   * then flag[the data] will be 0
   */
  // int flag_[DATA_NUM + 1];
  std::map<std::string, std::map<std::string, bool>> flag_;

  /*  */
  std::map<std::string, std::map<std::string, Rule>> rule_;

  /*  */
  unsigned long pow256[8];
  unsigned long pow2[8];

  /*  */
  std::string err;
  // declare every data as member function
  // The physical value to be encoded
  std::map<std::string, std::map<std::string, double>> tbe;
  // The physical value after decoded
  std::map<std::string, std::map<std::string, double>> afd;
};

typedef NTURT_CAN_parser Parser;

#endif