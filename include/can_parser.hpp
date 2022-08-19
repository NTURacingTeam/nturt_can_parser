#ifndef CAN_PARSER_HPP
#define CAN_PARSER_HPP

// std include
#include <iostream>
#include <map>
#include <string>

// yaml include
#include "yaml-cpp/yaml.h"

// ros include
#include <ros/ros.h>

struct Data {
    int start_byte_;
    int stop_byte_;
    int start_bit_;
    int stop_bit_;
    double resolution_;
    double offset_;
    bool is_signed_;
    bool is_little_endian_;
    std::string get_string();
};

struct Frame {
    unsigned int id_;
    bool is_extended_id_;
    int dlc_;
    double period_;
    std::map<std::string, Data> dataset_;
    std::string get_string();
};

ostream &operator<<(ostream &_os, const Data &_data) {
    _os << _data.get_string();
    return _os;
}

ostream &operator<<(ostream &_os, const Frame &_frame) {
    _os << _frame.get_string();
    return _os;
}

namespace YAML {
template<>


} // namespace YAML

#endif // CAN_PARSER_HPP