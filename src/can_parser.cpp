#include "can_parser.hpp"

CanParser::CanParser() {
    std::cout << "CanParser constructing\n";
    frameset_ = load_yaml("/home/docker/ws/src/nturt_can_parser/doc/can.yaml");
    for (auto it = frameset_.begin(); it != frameset_.end(); it++) {
        for (auto it_ = it->second->datavector_.begin(); it_ != it->second->datavector_.end(); it_++) {
            find_id_to_get_two_name[it->second->id_].push_back(std::pair<std::string, std::string>(it->second->name_, (*it_)->name_));
            to_be_encode[it->second->id_][(*it_)->name_] = 0;
        }
    }
    std::cout << "CanParser constructed\n";
}

void CanParser::publish(double _dt, void (*publish_callback)(int, const boost::array<u_int8_t, 8>&)){}

FramePtr CanParser::update_frame(int _id, const boost::array<u_int8_t, 8> &_data){}

double CanParser::get_data(std::string _name){}

Frameset CanParser::get_frameset() {}

int CanParser::init_parser() {
    return OK;
}

void CanParser::map_print() {

    std::cout << "CanParser map printing...\n";
    frameset_ = load_yaml("/home/docker/ws/src/nturt_can_parser/doc/can.yaml");
    for (auto it = frameset_.begin(); it != frameset_.end(); it++) {
        std::cout << it->second << "\n";
        for (auto it_ = it->second->datavector_.begin(); it_ != it->second->datavector_.end(); it_++) {
            std::cout << *it_ << "\n";
        }
    }
    std::cout << "CanParser map printed\n";
}

double CanParser::get_afd(int _id, std::string _data_name) {
    if (flag_[_id][_data_name]) {
        flag_[_id][_data_name] = false;
        return after_decode[_id][_data_name];
    } 
    //if (frameset_[frame_name][])
    return ERR;
}

int CanParser::set_tbe(int _id, string _data_name, double _value) {
    //TODO: add ERR condition
    to_be_encode[_id][_data_name] = _value;
    return OK;
}

int CanParser::encode(int _id, int *_data) {
    std::cout << "encode\n";
    for (auto comp : frameset_[_id]->datavector_) {
        long long compose = 0;
        double _tbe = to_be_encode[_id][comp->name_];     //TODO: be replaced
        to_be_encode[_id][comp->name_] = 0;
        //double _tbe = comp->to_be_encode_;                     //TODO: going to replace
        long long __tbe = (_tbe - comp->offset_) / comp->resolution_;
        if (comp->start_byte_ != comp->end_byte_) {
            if (comp->is_little_endian_ == _CP_LITTLE) {
                for (int i = comp->start_byte_; i < comp->end_byte_; i++) {
                    _data[i] = (__tbe >> (i - comp->start_byte_) * 8) & _CP_MASK_LAST_8_BIT;
                    // std::cout << "i, data: " << i << "," << data[i] << std::endl;
                }
            }   
            else if (comp->is_little_endian_ == _CP_BIG) {
                for (int i = comp->end_byte_; i > comp->start_byte_; i--) {
                    _data[i - 1] = (__tbe >> (comp->end_byte_ - i) * 8) & _CP_MASK_LAST_8_BIT;
                }
            }     
            //else {
            //      err_log(__func__, "Wrong is_little_endian_");
            //}
        }
        else if (comp->start_byte_ == comp->end_byte_) {
            //forced little is_little_endian_
            unsigned char mask = ~(((~0) << comp->start_bit_) % pow2[comp->end_bit_]);
            _data[comp->start_byte_] = (_data[comp->start_byte_] & mask) | (__tbe);
        }   
        else {
            std::cout << "Wrong bit/byte\n";
        }
        //else {
        //    err_log(__func__, "Wrong bit/byte setting");
        //}
    }
    return OK;
}

int CanParser::decode(int _id, int *_data) {
    std::cout << "decode" << "\n";
    for (auto comp : frameset_[_id]->datavector_) {
        long long compose = 0;
        if (comp->start_byte_!= comp->end_byte_) {
            if (comp->is_little_endian_ == _CP_LITTLE) {
                for (int i = comp->start_byte_; i < comp->end_byte_; i++) {
                    compose |= _data[i] << 8 * (i - comp->start_byte_);
                }
            } 
            else if (comp->is_little_endian_ == _CP_BIG) {
                for (int i = comp->end_byte_; i > comp->start_byte_; i--) {
                    compose |= _data[i - 1] << 8 * (comp->end_byte_ - i);
                }
            } 
            else {
                return ERR;
            }
            if (comp->is_signed_) {
                int sign_handling_bit = 8 * (comp->end_byte_ - comp->start_byte_) - 1;
                bool negative = (compose >> sign_handling_bit) & 1;
                long long neg_mask;
                if (negative) {
                    neg_mask = ~((int)pow256[comp->end_byte_ - comp->start_byte_] - 1);
                    compose |= neg_mask;
                }
            }
        }  
        else if (comp->start_byte_== comp->end_byte_) {
          compose = (_data[comp->start_byte_] >> comp->start_bit_) % pow2[comp->end_bit_];
        } 
        else {
            return ERR;
        }

        after_decode[_id][comp->name_] = ((double)comp->resolution_ * compose) + comp->offset_;
        flag_[_id][comp->name_] = true;

        comp->after_decode_ = ((double)comp->resolution_ * compose) + comp->offset_;   //test
        //comp->flag_ = true;                                                          //test

        std::cout << frameset_[_id]->name_ << ": " << comp->name_ << "\n" << compose << "\n";
    }
    return OK;
}

vector<pair<string, string>> CanParser::get_key(int id) {
    //TODO: add ERR condition
    return find_id_to_get_two_name[id];
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
    std::cout << "no error\n";
}
