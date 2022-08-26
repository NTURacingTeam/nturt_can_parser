#ifndef CAN_PARSER_HPP
#define CAN_PARSER_HPP

#include <string>
#include <boost/array.hpp>
#include <memory>
#include <bitset>
#include <iostream>
#include <string>
#include <map>
#include <vector>

#include "yaml_loader.hpp"

class CanParser{
    public:
        CanParser();
    private:
        std::map<std::string, Frame> frameset_;
};

CanParser::CanParser() {
    std::cout << "CanParser constructing\n";
    frameset_ = load_yaml("/home/docker/ws/src/nturt_can_parser/doc/can.yaml");
    for (auto it = frameset_.begin(); it != frameset_.end(); it++) {
        std::cout << it->second << "\n";
        for (auto it_ = it->second.datavector_.begin(); it_ != it->second.datavector_.end(); it_++) {
            std::cout << *it_ << "\n";
        }
    }
    std::cout << "CanParser constructed\n";
}
#endif // CAN_PARSER_HPP
