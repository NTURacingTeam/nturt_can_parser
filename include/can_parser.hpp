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

using std::vector; using std::map; using std::string; using std::pair; using std::cout; 

class CanParser{
public:
    CanParser();

    vector<pair<string, string>> get_key(int id);
    map<string, Frame> frameset_;
    map<int, vector<pair<string, string>>> find_id_to_get_two_name;
};

CanParser::CanParser() {
    std::cout << "CanParser constructing\n";
    frameset_ = load_yaml("/home/docker/ws/src/nturt_can_parser/doc/can.yaml");
    for (auto it = frameset_.begin(); it != frameset_.end(); it++) {
        std::cout << it->second << "\n";
        for (auto it_ = it->second.datavector_.begin(); it_ != it->second.datavector_.end(); it_++) {
            find_id_to_get_two_name[it->second.id_].push_back(pair<string, string>(it->second.name_, it_->name_));
            std::cout << *it_ << "\n";
        }
    }
    std::cout << "CanParser constructed\n";
}

vector<pair<string, string>> CanParser::get_key(int id) {
    //vector<pair<string, string>> _result;
    return find_id_to_get_two_name[id];
    //return _result;
}
#endif // CAN_PARSER_HPP
