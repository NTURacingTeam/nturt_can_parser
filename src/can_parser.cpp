#include "can_parser.hpp"

// global variable definition, put in source file to prevent being linked
// 2 to the power of N.
const u_int64_t pow2[8] = {1, 2, 4, 8, 16, 32, 64, 128};
//256 to the power of N.
const u_int64_t pow256[8] = {1, 256, 65536, 16777216, 4294967296, 1099511627776, 281474976710656, 72057594037927936};

void CanParser::init(std::string _file) {
    id_frameset_ = load_yaml(_file);

    // put all frames that need to be periodically published into one
    for(auto it = id_frameset_.begin(); it != id_frameset_.end(); it++) {
        if(it->second->period_ != 0) {
            periodic_publish_frameset_[it->first] = it->second;
        }
    }

    name_frameset_ = convert_to_name_frame(id_frameset_);

    // append the can data in every frame set into one dataset
    for(auto it = name_frameset_.begin(); it != name_frameset_.end(); it++) {
        dataset_.insert(it->second->dataset_.begin(), it->second->dataset_.end());
    }
}

void CanParser::publish(const FramePtr &_frame, const PublishFun publish_fun) const {
    // default raw can data (when not occupied by any data) to 0
    boost::array<uint8_t, 8> data = {0};

    // encode every data in the frame into raw can data, then publish it
    for(auto it = _frame->dataset_.begin(); it != _frame->dataset_.end(); it++) {
        encode(it->second, data);
    }
    publish_fun(_frame, data);
}

bool CanParser::publish(const std::string &_name, const PublishFun publish_fun) const {
    try {
        FramePtr frame = name_frameset_.at(_name);
        publish(frame, publish_fun);
    }
    // if frame not found
    catch(std::out_of_range) {
        return false;
    }

    return true;
}

void CanParser::periodic_publish(const double &_dt, const PublishFun publish_fun) const {
    for(auto it = periodic_publish_frameset_.begin(); it != periodic_publish_frameset_.end(); it++) {
        it->second->dt_ += _dt;
        if(it->second->dt_ >= it->second->period_) {
            publish(it->second, publish_fun);
            // try to catch up the required period by not setting dt to 0
            it->second->dt_ -= it->second->period_;
        }
    }
}

DataPtr CanParser::update_data(const std::string &_name, const double &_value) {
    try {
        DataPtr data = dataset_.at(_name);
        data->last_data_ = _value;
        return data;
    }
    // if data not found
    catch(std::out_of_range) {
        return nullptr;
    }
}

FramePtr CanParser::update_frame(const int &_id, const boost::array<u_int8_t, 8> &_data) {
    try {
        FramePtr frame = id_frameset_.at(_id);
        for(auto it = frame->dataset_.begin(); it != frame->dataset_.end(); it++) {
            decode(it->second, _data);
        }
        return frame;
    }
    // if frame not found
    catch(std::out_of_range) {
        return nullptr;
    }
}

DataPtr CanParser::get_data(const std::string &_name) const {
    try {
        return dataset_.at(_name);
    }
    // if data not found
    catch(std::out_of_range) {
        return nullptr;
    }
}

FramePtr CanParser::get_frame(const int &_id) const {
    try {
        return id_frameset_.at(_id);
    }
    // if frame not found
    catch(std::out_of_range) {
        return nullptr;
    }
}

FramePtr CanParser::get_frame(const std::string &_name) const {
    try {
        return name_frameset_.at(_name);
    }
    // if frame not found
    catch(std::out_of_range) {
        return nullptr;
    }
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

void decode(DataPtr &_data, const boost::array<uint8_t, 8> &_raw_data) {
    // composed data before converting to double
    u_int64_t compose = 0;
    
    // combine multiple bytes/bits into one long long
    // if is byte data
    if(_data->is_byte_) {
        // if is little endian
        // calculated as (first byte) + 256 * (second byte) + 256 ^ 2 *(third byte) ... (calclate 256's power by shifting to left)
        if(_data->is_little_endian_) {
            for(int i = _data->end_byte_ - 1; i >= _data->start_byte_; i--) {
                compose = (compose << 8) + _raw_data[i];
            }
        }
        else {
            for(int i = _data->start_byte_; i < _data->end_byte_; i++) {
                compose = (compose << 8) + _raw_data[i];
            }
        }
    }
    else {
        // calculated by removing the last "end_bit" bit by modding 2 ^ end_bit, then remove first "start_bit" bit by shifting to right start_bit
        compose = (_raw_data[_data->start_byte_] & (pow2[_data->end_bit_] - 1)) >> _data->start_bit_;
    }

    // if is signed data
    if(_data->is_signed_) {
        // most significant bit (1 means negative)
        // claculated by shifting the data to right by it's length (bit) - 1
        bool is_negative;
        if(_data->is_byte_) {
            is_negative = compose >> ((_data->end_byte_ - _data->start_byte_) * 8 - 1);
        }
        else {
            is_negative = compose >> ((_data->end_bit_ - _data->start_bit_) - 1);
        }

        if(is_negative) {
            // if is not 8 byte long, should put negative mask after compose for extending the negative bits (from the data's most significant
            // bit to u_int64_t's most significant bit) for casting u_int64_t to int64_t
            // calculated as 2 ^ it's length (bit) - 1, then not it
            if(_data->end_byte_ - _data->start_byte_ != 8) {
                u_int64_t negative_mask;
                if(_data->is_byte_) {
                    negative_mask = ~ (pow256[_data->end_byte_ - _data->start_byte_] - 1);
                }
                else {
                    negative_mask = ~ (pow2[_data->end_bit_ - _data->start_bit_] - 1);
                }

                // put the negative_mask on
                compose |= negative_mask;
            }
        }

        // cast to signed data
        _data->last_data_ = _data->resolution_ * static_cast<int64_t>(compose) - _data->offset_;
    }
    else {
        _data->last_data_ = _data->resolution_ * compose - _data->offset_;
    }
}

void encode(const DataPtr &_data, boost::array<uint8_t, 8> &_raw_data) {
    // composed data from double, rounded down
    // don't need to consider the sign of the data because implicit cast has already done if for us
    // DOES NOT check for overfolw
    u_int64_t compose = (_data->last_data_ - _data->offset_) / _data->resolution_;

    // if is byte data
    if(_data->is_byte_) {
        // if is little endian
        // calculated as shift it to the right by 8 * N bit, then take mod 256
        if(_data->is_little_endian_) {
            for(int i = 0; i < _data->end_byte_ - _data->start_byte_; i++) {
                _raw_data[_data->start_byte_ + i] = (compose >> (8 * i)) & 255;
            }
        }
        else {
            for(int i = 0; i < _data->end_byte_ - _data->start_byte_; i++) {
                _raw_data[_data->end_byte_ - i - 1] = (compose >> (8 * i)) & 255;
            }
        }
    }
    else {
        // to clear previous data stored in raw data, a mask with the previous raw data in this range (in [start_bit, end_bit)) and all 0
        // outside the range should exlusive or the raw data to make this range all 0 and not effecting data outside the range
        // calculated by removing the last "end_bit" bit by modding 2 ^ end_bit, then remove first "start_bit" bit by shifting to right start_bit
        u_int8_t clear_mask = (_raw_data[_data->start_byte_] & (pow2[_data->end_bit_] - 1)) >> _data->start_bit_;

        // put clear_mask on
        _raw_data[_data->start_byte_] ^= clear_mask;
        // calculated by removing the last "end_bit - start_bit" (length of the data in bit) by modding 2 ^ length, then shift it to left start_bit
        _raw_data[_data->start_byte_] |= (compose & (pow2[_data->end_bit_ - _data->start_bit_] - 1)) << _data->start_bit_;
    }
}
