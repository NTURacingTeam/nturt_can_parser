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


void CanParser::map_print() {

    std::cout << "CanParser map printing...\n";
    frameset_ = load_yaml("/home/docker/ws/src/nturt_can_parser/doc/can.yaml");
    for (auto it = frameset_.begin(); it != frameset_.end(); it++) {
        std::cout << it->second << "\n";
        for (auto it_ = it->second.datavector_.begin(); it_ != it->second.datavector_.end(); it_++) {
            std::cout << *it_ << "\n";
        }
    }
    std::cout << "CanParser map printed\n";
}
double CanParser::get_afd(string frame_name, string comp_name) {
    if (flag_[frame_name][comp_name]) {
        flag_[frame_name][comp_name] = false;
        return after_decode[frame_name][comp_name];
    } 
    //if (frameset_[frame_name][])
    return ERR;
}

int CanParser::set_tbe(string frame_name, string comp_name, double val) {
    //TODO: add ERR condition
    to_be_encode[frame_name][comp_name] = val;
    return OK;
}

int CanParser::encode(int id, int *data) {
    cout << "encode\n";
    string curr_frame = find_id_to_get_frame[id];
    for (auto comp : frameset_[curr_frame].datavector_) {
        long long compose = 0;
        double _tbe = to_be_encode[curr_frame][comp.name_];     //TODO: be replaced
        to_be_encode[curr_frame][comp.name_] = 0;     
        //double _tbe = comp.to_be_encode_;                     //TODO: going to replace
        long long __tbe = (_tbe - comp.offset_) / comp.resolution_;
        if (comp.start_byte_ != comp.end_byte_) {
            if (comp.is_little_endian_ == _CP_LITTLE) {
                for (int i = comp.start_byte_; i < comp.end_byte_; i++) {
                  data[i] = (__tbe >> (i - comp.start_byte_) * 8) & _CP_MASK_LAST_8_BIT;
                  // std::cout << "i, data: " << i << "," << data[i] << std::endl;
                }
            }   
            else if (comp.is_little_endian_ == _CP_BIG) {
                for (int i = comp.end_byte_; i > comp.start_byte_; i--) {
                    data[i - 1] = (__tbe >> (comp.end_byte_ - i) * 8) & _CP_MASK_LAST_8_BIT;
                  }
            }     
            //else {
            //      err_log(__func__, "Wrong is_little_endian_");
            //}
        }
        else if (comp.start_byte_ == comp.end_byte_) {
            //forced little is_little_endian_
            unsigned char mask = ~(((~0) << comp.start_bit_) % pow2[comp.end_bit_]);
            data[comp.start_byte_] = (data[comp.start_byte_] & mask) | (__tbe);
        }   
        else {
            cout << "Wrong bit/byte\n";
        }
        //else {
        //    err_log(__func__, "Wrong bit/byte setting");
        //}
    }
    return OK;
}

int CanParser::decode(int id, int *data) {
    cout << "decode" << "\n";
    string curr_frame = find_id_to_get_frame[id]; 
    for (auto comp : frameset_[curr_frame].datavector_) {
        long long compose = 0;
        if (comp.start_byte_!= comp.end_byte_) {
            if (comp.is_little_endian_ == _CP_LITTLE) {
                for (int i = comp.start_byte_; i < comp.end_byte_; i++) {
                    compose |= data[i] << 8 * (i - comp.start_byte_);
                }
            } 
            else if (comp.is_little_endian_ == _CP_BIG) {
                for (int i = comp.end_byte_; i > comp.start_byte_; i--) {
                    compose |= data[i - 1] << 8 * (comp.end_byte_ - i);
                }
            } 
            else {
                return ERR;
            }
            if (comp.is_signed_) {
                int sign_handling_bit = 8 * (comp.end_byte_ - comp.start_byte_) - 1;
                bool negative = (compose >> sign_handling_bit) & 1;
                long long neg_mask;
                if (negative) {
                    neg_mask = ~((int)pow256[comp.end_byte_ - comp.start_byte_] - 1);
                    compose |= neg_mask;
                }
            }
        }  
        else if (comp.start_byte_== comp.end_byte_) {
          compose = (data[comp.start_byte_] >> comp.start_bit_) % pow2[comp.end_bit_];
        } 
        else {
            return ERR;
        }

        after_decode[curr_frame][comp.name_] = ((double)comp.resolution_ * compose) + comp.offset_;
        flag_[curr_frame][comp.name_] = true;

        comp.after_decode_ = ((double)comp.resolution_ * compose) + comp.offset_;   //test
        //comp.flag_ = true;                                                          //test

        cout << curr_frame << ": " << comp.name_ << "\n" << compose << "\n";
    }
    return OK;
}

/*
int CanParser::decode(int id, const boost::array<unsigned char, 8> data) {
  int idata[8];
  for (int i = 0; i < 8; i++) {
    idata[i] = data[i];
  }
  int res = decode(id, idata);
  return res;
}
*/

CanParser::CanParser() {
    std::cout << "CanParser constructing\n";
    frameset_ = load_yaml("/home/docker/ws/src/nturt_can_parser/doc/can.yaml");
    for (auto it = frameset_.begin(); it != frameset_.end(); it++) {
        find_id_to_get_frame[it->second.id_] = it->second.name_;
        for (auto it_ = it->second.datavector_.begin(); it_ != it->second.datavector_.end(); it_++) {
            find_id_to_get_two_name[it->second.id_].push_back(pair<string, string>(it->second.name_, it_->name_));
            to_be_encode[it->second.name_][it_->name_] = 0;
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
