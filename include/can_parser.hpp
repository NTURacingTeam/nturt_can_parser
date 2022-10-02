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
#include <functional>
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

// type definition
typedef std::function<void(const FramePtr&, const boost::array<u_int8_t, 8>&)> PublishFun;

/**
 * @author your name (you@domain.com)
 * @brief 
 */
class CanParser{
    public:
        /**
         * @brief Initialize can parser and load yaml file.
         * @param _file Path to the can yaml file.
         */
        void init(std::string _file);

        /**
         * @brief Function to publish the periodically published frame.
         * @param[in] _name Name of the can frame to be published.
         * @param[in] publish_fun Function to publish the frame, whose arguments are frame pointer and data of the frame.
         * @return True if the frame is published successfully, or false if failed.
         */
        bool publish(const std::string &_name, const PublishFun publish_fun) const;

        /**
         * @brief Function to publish the periodically published frame.
         * @param[in] _dt The time difference between this and last call of this function.
         * @param[in] publish_fun Function to publish the frame, whose arguments are frame pointer and data of the frame.
         */
        void periodic_publish(const double &_dt, const PublishFun publish_fun) const;

        /**
         * @brief Function to update the can data.
         * @param[in] _name The name of the can data.
         * @param[in] _value The value of the can data.
         * @return Pointer to can data, nullptr if not found.
         */
        DataPtr update_data(const std::string &_name, const double &_value);

        /**
         * @brief Function to update the can data of a frame.
         * @param[in] _id The id of the frame.
         * @param[in] _data The data of the frame.
         * @return Pointer to the updated frame, nullptr if not found.
         */
        FramePtr update_frame(const int &_id, const boost::array<u_int8_t, 8> &_data);

        /**
         * @brief Get can data using the data's name.
         * @param[in] _name The name of the can data.
         * @return Pointer to the can data, nullptr if not found.
         */
        DataPtr get_data(const std::string &_name) const;

        /**
         * @brief Get can frame using the frame's id.
         * @param[in] _id Id of the can frame.
         * @return Pointer to the can frame, nullptr if not found.
         */
        FramePtr get_frame(const int &_id) const;

        /**
         * @brief Get frame using the frame's name.
         * @param[in] _name Name of the can frame.
         * @return Pointer to the can frame, nullptr if not found.
         */
        FramePtr get_frame(const std::string &_name) const;
        
        /**
         * @brief Get dataset stroed in the can parser.
         * @return Dataset, a map storing pointer to can data of all frames, with key being the name of the can data.
         */
        Dataset get_dataset() const;

        /**
         * @brief Get id frameset stored in the can parser.
         * @return Frameset, a map storing pointer to can frame, with key being the id of the can frame.
         */
        IdFrameset get_id_frameset() const;

        /**
         * @brief Get name frameset stored in the can parser.
         * @return Frameset, a map storing pointer to can frame, with key being the name of the can frame.
         */
        NameFrameset get_name_frameset() const;

        void print_err_log();
        void map_print();
        int check_key(int id, std::string key, std::string comp);
        int check_key(int id, std::string key);
        int decode(int _id, int *_data);
        int decode(int _id, const boost::array<unsigned char, 8> &_data);    // not sure
        int encode(int _id, int *_data);
        int set_tbe(int _id, std::string _data_name, double _value);
        double get_afd(int _id, std::string _data_name);
        std::vector<std::pair<std::string, std::string>> get_key(int id);

        /// @brief Map storing pointer to can frame, with key being the id of the can frame.
        IdFrameset id_frameset_;

        /// @brief Map storing pointer to can frame, with key being the name of the can frame.
        NameFrameset name_frameset_;

        /// @brief Map storing pointer to can data, with key being the name of the can data.
        Dataset dataset_;

        std::map<int, std::map<std::string, bool>> flag_;
        std::map<int, std::map<std::string, double>> after_decode; // move to last_data_ in Data, should be deleted
        std::map<int, std::map<std::string, double>> to_be_encode; // so does this
        //map<string, vector<bool>> flag_;
        std::map<int, std::vector<std::pair<std::string, std::string>>> find_id_to_get_two_name;
        
        /// @brief 2 to the power of N;
        const unsigned long pow2[8] = {1, 2, 4, 8, 16, 32, 64, 128};

        /// @brief 256 to the power of N;
        const unsigned long pow256[8] = {1, 256, 65536, 16777216, 4294967296, 1099511627776, 281474976710656, 72057594037927940};
};

#endif // CAN_PARSER_HPP
