#include "can_parser.hpp"

void CanParser::init(std::string _file) {
    id_frameset_ = load_yaml(_file);
    name_frameset_ = convert_to_name_frame(id_frameset_);
}

bool CanParser::publish(const std::string &_name, const PublishFun publish_fun) const {
    boost::array<uint8_t, 8> data;
    // if frame not found
    if(name_frameset_.find(_name) == name_frameset_.end()) {
        return false;
    }

    // encode data into a frame, then publish it
    return true;
}

void CanParser::periodic_publish(const double &_dt, const PublishFun publish_fun) const {

}

DataPtr CanParser::update_data(const std::string &_name, const double &_value) {
    // if frame not found
    if(dataset_.find(_name) == dataset_.end()) {
        return nullptr;
    }

    dataset_[_name]->last_data_ = _value;
    return dataset_[_name];
}

FramePtr CanParser::update_frame(const int &_id, const boost::array<u_int8_t, 8> &_data) {
    // if frame not found

    auto curr_frame = id_frameset_.find(_id);
    if(curr_frame == id_frameset_.end()) {
        return nullptr;
    }
    else {
        decode(_id, _data);
        /*
        for (auto curr_data = curr_frame->second->dataset_.begin(); curr_data != curr_frame->second->dataset_.end(); curr_data++) {
            decode();
        }
        */
    }

    // decode frame to can data
}

DataPtr CanParser::get_data(const std::string &_name) const {
    // if data not found
    if(dataset_.find(_name) == dataset_.end()) {
        return nullptr;
    }

    return dataset_.at(_name);
}

FramePtr CanParser::get_frame(const int &_id) const {
    // if frame not found
    if(id_frameset_.find(_id) == id_frameset_.end()) {
        return nullptr;
    }

    return id_frameset_.at(_id);
}

FramePtr CanParser::get_frame(const std::string &_name) const {
    // if frame not found
    if(name_frameset_.find(_name) == name_frameset_.end()) {
        return nullptr;
    }

    return name_frameset_.at(_name);
}

Dataset CanParser::get_dataset() const {
    return dataset_;
}

IdFrameset CanParser::get_id_frameset() const {
    return id_frameset_;
}

NameFrameset CanParser::get_name_frameset() const {
    return name_frameset_;
}

void CanParser::map_print() {

    std::cout << "CanParser map printing...\n";
    std::cout << get_string(id_frameset_) << std::endl;
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

int CanParser::set_tbe(int _id, std::string _data_name, double _value) {
    //TODO: add ERR condition
    to_be_encode[_id][_data_name] = _value;
    return OK;
}

int CanParser::encode(int _id, int *_data) {
    std::cout << "encode\n";
    for (auto it = id_frameset_[_id]->dataset_.begin(); it != id_frameset_[_id]->dataset_.end(); it++) {
        long long compose = 0;
        double _tbe = to_be_encode[_id][it->first];     //TODO: be replaced
        to_be_encode[_id][it->first] = 0;
        //double _tbe = comp->to_be_encode_;                     //TODO: going to replace
        long long __tbe = (_tbe - it->second->offset_) / it->second->resolution_;
        if (it->second->start_byte_ != it->second->end_byte_) {
            if (it->second->is_little_endian_ == _CP_LITTLE) {
                for (int i = it->second->start_byte_; i < it->second->end_byte_; i++) {
                    _data[i] = (__tbe >> (i - it->second->start_byte_) * 8) & _CP_MASK_LAST_8_BIT;
                    // std::cout << "i, data: " << i << "," << data[i] << std::endl;
                }
            }   
            else if (it->second->is_little_endian_ == _CP_BIG) {
                for (int i = it->second->end_byte_; i > it->second->start_byte_; i--) {
                    _data[i - 1] = (__tbe >> (it->second->end_byte_ - i) * 8) & _CP_MASK_LAST_8_BIT;
                }
            }     
            //else {
            //      err_log(__func__, "Wrong is_little_endian_");
            //}
        }
        else if (it->second->start_byte_ == it->second->end_byte_) {
            //forced little is_little_endian_
            unsigned char mask = ~(((~0) << it->second->start_bit_) % pow2[it->second->end_bit_]);
            _data[it->second->start_byte_] = (_data[it->second->start_byte_] & mask) | (__tbe);
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
int CanParser::decode(int _id, const boost::array<unsigned char, 8> &_data) {
    std::cout << "decode" << "\n";
    for (auto it = id_frameset_[_id]->dataset_.begin(); it != id_frameset_[_id]->dataset_.end(); it++) {
        long long compose = 0;
        if (it->second->start_byte_ != it->second->end_byte_) {
            if (it->second->is_little_endian_ == _CP_LITTLE) {
                for (int i = it->second->start_byte_; i < it->second->end_byte_; i++) {
                    compose |= _data[i] << 8 * (i - it->second->start_byte_);
                }
            } 
            else if (it->second->is_little_endian_ == _CP_BIG) {
                for (int i = it->second->end_byte_; i > it->second->start_byte_; i--) {
                    compose |= _data[i - 1] << 8 * (it->second->end_byte_ - i);
                }
            } 
            else {
                return ERR;
            }
            if (it->second->is_signed_) {
                int sign_handling_bit = 8 * (it->second->end_byte_ - it->second->start_byte_) - 1;
                bool negative = (compose >> sign_handling_bit) & 1;
                long long neg_mask;
                if (negative) {
                    neg_mask = ~((int)pow256[it->second->end_byte_ - it->second->start_byte_] - 1);
                    compose |= neg_mask;
                }
            }
        }  
        else if (it->second->start_byte_== it->second->end_byte_) {
          compose = (_data[it->second->start_byte_] >> it->second->start_bit_) % pow2[it->second->end_bit_];
        } 
        else {
            return ERR;
        }

        dataset_[it->second->name_]->last_data_ = ((double)it->second->resolution_ * compose) + it->second->offset_;
        flag_[_id][it->second->name_] = true;

        it->second->last_data_ = ((double)it->second->resolution_ * compose) + it->second->offset_; //test
        //comp->flag_ = true; //test

        std::cout << id_frameset_[_id]->name_ << ": " << it->second->name_ << "\n" << compose << "\n";
    }
    return OK;
}
int CanParser::decode(int _id, int *_data) {
    std::cout << "decode" << "\n";
    for (auto it = id_frameset_[_id]->dataset_.begin(); it != id_frameset_[_id]->dataset_.end(); it++) {
        long long compose = 0;
        if (it->second->start_byte_ != it->second->end_byte_) {
            if (it->second->is_little_endian_ == _CP_LITTLE) {
                for (int i = it->second->start_byte_; i < it->second->end_byte_; i++) {
                    compose |= _data[i] << 8 * (i - it->second->start_byte_);
                }
            } 
            else if (it->second->is_little_endian_ == _CP_BIG) {
                for (int i = it->second->end_byte_; i > it->second->start_byte_; i--) {
                    compose |= _data[i - 1] << 8 * (it->second->end_byte_ - i);
                }
            } 
            else {
                return ERR;
            }
            if (it->second->is_signed_) {
                int sign_handling_bit = 8 * (it->second->end_byte_ - it->second->start_byte_) - 1;
                bool negative = (compose >> sign_handling_bit) & 1;
                long long neg_mask;
                if (negative) {
                    neg_mask = ~((int)pow256[it->second->end_byte_ - it->second->start_byte_] - 1);
                    compose |= neg_mask;
                }
            }
        }  
        else if (it->second->start_byte_== it->second->end_byte_) {
          compose = (_data[it->second->start_byte_] >> it->second->start_bit_) % pow2[it->second->end_bit_];
        } 
        else {
            return ERR;
        }

        dataset_[it->second->name_]->last_data_ = ((double)it->second->resolution_ * compose) + it->second->offset_;
        flag_[_id][it->second->name_] = true;

        it->second->last_data_ = ((double)it->second->resolution_ * compose) + it->second->offset_; //test
        //comp->flag_ = true; //test

        std::cout << id_frameset_[_id]->name_ << ": " << it->second->name_ << "\n" << compose << "\n";
    }
    return OK;
}

std::vector<std::pair<std::string, std::string>> CanParser::get_key(int id) {
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
