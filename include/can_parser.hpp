#ifndef CAN_PARSER_HPP
#define CAN_PARSER_HPP

#include <string>
#include <cmath>
#include <boost/array.hpp>
#include <memory>
#include <bitset>
#include <iostream>
#include <string>
#include <map>
#include <vector>

#include "yaml_loader.hpp"

#define OK -1
#define ERR 0

#define _CP_BIT 0       // used to be 1
#define _CP_BYTE 1      // used to be 2
#define _CP_BIG 0       // used to be 3
#define _CP_LITTLE 1    // used to be 4 
// define Constants
#define TWOPOW08 256
#define TWOPOW26 67108864
// define mask
#define _CP_MASK_LAST_8_BIT 255 // b0000000011111111

using std::vector; using std::map; using std::string; using std::pair; using std::cout; 

class CanParser{
public:
    CanParser();

    void print_err_log();
    void map_print();
    int init_parser();
    int check_key(int id, string key, string comp);
    int check_key(int id, string key);
    int decode(int id, int *data);
    //int decode(int id, const boost::array<unsigned char, 8> data);    // not sure
    int encode(int id, int *data);
    int set_tbe(string frame_name, string comp_name, double val);
    double get_afd(string key, string comp);
    vector<pair<string, string>> get_key(int id);

    map<string, Frame> frameset_;
    map<string, map<string, bool>> flag_;
    map<string, map<string, double>> after_decode;
    map<string, map<string, double>> to_be_encode;
    //map<string, vector<bool>> flag_;
    map<int, vector<pair<string, string>>> find_id_to_get_two_name;
    map<int, string> find_id_to_get_frame;
    
    unsigned long pow256[8];
    unsigned long pow2[8];
};

#endif // CAN_PARSER_HPP
