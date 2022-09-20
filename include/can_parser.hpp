/**
 * @file can_parser.hpp
 * @author your name (you@domain.com)
 * @brief 
 */

#ifndef CAN_PARSER_HPP
#define CAN_PARSER_HPP

// std include
#include <bitset>
#include <cmath>
#include <iostream>
#include <string>
#include <map>
#include <memory>
#include <vector>

// boost include
#include <boost/array.hpp>

// nturt include
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

using std::vector; using std::map; using std::string; using std::pair;

/**
 * @author your name (you@domain.com)
 * @brief 
 */
class CanParser{
    public:
        CanParser();
        int init_parser();
        void print_err_log();
        void map_print();
        int check_key(int id, std::string key, std::string comp);
        int check_key(int id, std::string key);
        int decode(int id, int *data);
        //int decode(int id, const boost::array<unsigned char, 8> data);    // not sure
        int encode(int id, int *data);
        int set_tbe(std::string frame_name, std::string comp_name, double val);
        double get_afd(std::string key, std::string comp);
        std::vector<std::pair<std::string, std::string>> get_key(int id);

    private:
        /// @brief Map storing can frame.
        std::map<std::string, FramePtr> frameset_;
        std::map<std::string, std::map<std::string, bool>> flag_;
        std::map<std::string, std::map<std::string, double>> after_decode;
        std::map<std::string, std::map<std::string, double>> to_be_encode;
        //map<string, vector<bool>> flag_;
        std::map<int, std::vector<std::pair<std::string, std::string>>> find_id_to_get_two_name;
        std::map<int, std::string> find_id_to_get_frame;
        
        const unsigned long pow256[8] = {1, 256, 65536, 16777216, 4294967296, 1099511627776, 281474976710656, 72057594037927940};
        const unsigned long pow2[8] = {1, 2, 4, 8, 16, 32, 64, 128};
};

#endif // CAN_PARSER_HPP
