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

using std::vector; using std::map; using std::string; using std::pair; using std::cout; 

class CanParser{
public:
    CanParser();

    void print_err_log();
    int init_parser();
    int check_key(int id, string key, string comp);
    int check_key(int id, string key);
    vector<pair<string, string>> get_key(int id);

    map<string, Frame> frameset_;
    map<int, vector<pair<string, string>>> find_id_to_get_two_name;

    unsigned long pow256[8];
    unsigned long pow2[8];
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
    //TODO: add ERR condition
    return find_id_to_get_two_name[id];
}

int CanParser::init_parser() {
  // prepare 2 power chart
  for (int i = 0; i < 8; i++) {
    pow256[i] = pow(256, i);
    pow2[i] = pow(2, i);
  }
  return OK;
}
int CanParser::check_key(int id, std::string key, std::string comp) {
    //TODO: add ERR condition
    return OK;
}
int CanParser::check_key(int id, std::string key) {
    //TODO: add ERR condition
    return OK;
}

void CanParser::print_err_log() {
    //TODO: add ERR condition
    cout << "no error\n";
}

#endif // CAN_PARSER_HPP
