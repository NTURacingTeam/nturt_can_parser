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

        /**
         * @brief Function to publish the periodically published frame.
         * @param[in] _dt The time difference between tthis and last call of the function.
         * @param[in] publish_callback The callback function to publish the frame, whose arguments are id and data of the frame.
         */
        void publish(double _dt, void (*publish_callback)(int, const boost::array<u_int8_t, 8>&));

        /**
         * @brief Function to update the can data of a frame.
         * @param[in] _id The id of the frame.
         * @param[in] _data The data of the frame.
         * @return The pointer to the updated frame.
         */
        FramePtr update_frame(int _id, const boost::array<u_int8_t, 8> &_data);

        /**
         * @brief Get the can data using the data's name.
         * @param[in] _name The name of the can data.
         * @return The value of the data.
         */
        double get_data(std::string _name);
        
        /**
         * @brief Get the frameset stored in the can parser.
         * @return Frameset, a map storing pointer to can frame, with key being the id of the can frame.
         */
        Frameset get_frameset();

        int init_parser();
        void print_err_log();
        void map_print();
        int check_key(int id, std::string key, std::string comp);
        int check_key(int id, std::string key);
        int decode(int _id, int *_data);
        //int decode(int id, const boost::array<unsigned char, 8> data);    // not sure
        int encode(int _id, int *_data);
        int set_tbe(int _id, string _data_name, double _value);
        double get_afd(int _id, std::string _data_name);
        std::vector<std::pair<std::string, std::string>> get_key(int id);

    private:
        /// @brief Map storing can frame.
        Frameset frameset_;
        std::map<int, std::map<std::string, bool>> flag_;
        std::map<int, std::map<std::string, double>> after_decode;
        std::map<int, std::map<std::string, double>> to_be_encode;
        //map<string, vector<bool>> flag_;
        std::map<int, std::vector<std::pair<std::string, std::string>>> find_id_to_get_two_name;
        
        const unsigned long pow256[8] = {1, 256, 65536, 16777216, 4294967296, 1099511627776, 281474976710656, 72057594037927940};
        const unsigned long pow2[8] = {1, 2, 4, 8, 16, 32, 64, 128};
};

#endif // CAN_PARSER_HPP
