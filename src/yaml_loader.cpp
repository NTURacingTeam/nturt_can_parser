#include "yaml_loader.hpp"

// global variable definition, put in source file to prevent being linked
// 2 to the power of N.
const u_int64_t pow2[8] = {1, 2, 4, 8, 16, 32, 64, 128};
//256 to the power of N.
const u_int64_t pow256[8] = {1, 256, 65536, 16777216, 4294967296, 1099511627776, 281474976710656, 72057594037927936};

std::string Data::get_string() const {
    std::string string = "\tCAN data: " + name_ + " --------------------" +
        "\n\t\tis_signed: " + (is_signed_ ? "true" : "false") +
        "\n\t\tis_byte: " + (is_byte_ ? "true" : "false") +
        "\n\t\tis_little_endian: " + (is_little_endian_ ? "true" : "false") +
        "\n\t\tstart_byte: " + std::to_string(start_byte_) +
        "\n\t\tend_byte: " + std::to_string(end_byte_) +
        "\n\t\tstart_bit: " + std::to_string(start_bit_) +
        "\n\t\tend_bit: " + std::to_string(end_bit_) +
        "\n\t\tdefault: " + std::to_string(default_) +
        "\n\t\tresolution: " + std::to_string(resolution_) +
        "\n\t\toffset: " + std::to_string(offset_) +
        "\n\t\tlast_data: " + std::to_string(last_data_);
    return string;
}

u_int64_t Data::get_occupied_bit() const {
    u_int64_t occupied_bit = 0;
    // if is byte data
    if(is_byte_) {
        occupied_bit = (pow256[end_byte_ - start_byte_] - 1) << 8 * start_byte_;
    }
    else {
        occupied_bit = (pow2[end_bit_ - start_bit_] - 1) << (8 * start_byte_ + start_bit_);
    }
    return occupied_bit;
}

u_int8_t Data::get_occupied_byte() const {
    u_int8_t occupied_byte = 0;
    // if is byte data
    if(is_byte_) {
        occupied_byte = (pow2[end_byte_ - start_byte_] - 1) << start_byte_;
    }
    else {
        occupied_byte = 1 << start_byte_;
    }
    return occupied_byte;
}

bool YAML::convert<Data>::decode(const Node &_node, Data &_cType) {
    // check if "name" tag exist
    if(_node["name"]) {
        _cType.name_ = _node["name"].as<std::string>();
    }
    else {
        throw std::runtime_error(std::string("Error: \"name\" tag does not exist under some \"dataset\" tag.\n") +
            "\"name\" tag is required for accessing a can data.\n");
    }
    
    // check if "is_signed" tag exist
    if(_node["is_signed"]) {
        _cType.is_signed_ = _node["is_signed"].as<bool>();
    }
    else {
        throw std::runtime_error(std::string("Error: \"is_signed\" tag does not exist in ") + _cType.name_ + "\" can data.\n" +
            "\"is_signed\" tag is required for determining if this data is signed.\n");
    }

    // check if "is_byte" tag exist
    if(_node["is_byte"]) {
        _cType.is_byte_ = _node["is_byte"].as<bool>();
    }
    else {
        throw std::runtime_error(std::string("Error: \"is_byte\" tag does not exist in \"") + _cType.name_ + "\" can data.\n" +
            "\"is_byte\" tag is required for determining if this data is a byte data.\n");
    }

    // check if "start_byte" tag exist
    if(_node["start_byte"]) {
        if(!_cType.is_byte_) {
            throw std::runtime_error(std::string("Error: \"start_byte\" tag exist while \"is_byte\" flag is set to false in \"") +
                _cType.name_ + "\" can data.\n" +
                "\"start_byte\" is only required if \"is_byte\" flag is set to true.\n");
        }
        else {
            int start_byte = _node["start_byte"].as<int>();
            if(start_byte < 0 || start_byte > 7) {
                throw std::range_error(std::string("Error: \"start_byte: ") + std::to_string(start_byte) + "\" in \"" + _cType.name_ +
                    "\" can data is out of range.\n" +
                    "The range for \"start_byte\" is [0, 7].\n");
            }
            else {
                _cType.start_byte_ = start_byte;
            }
        }
    }
    else if(_cType.is_byte_) {
        throw std::runtime_error(std::string("Error: \"start_byte\" tag does not exist while \"is_byte\" flag is set to true in \"") +
            _cType.name_ + "\" can data.\n" +
            "\"start_byte\" tag is required if \"is_byte\" is set to true for determining the position of the can data.\n");
    }

    // check if "byte" tag exist
    if(_node["byte"]) {
        if (_cType.is_byte_) {
            throw std::runtime_error(std::string("Error: \"byte\" tag exist while \"is_byte\" flag is set to true.\n") +
                "\"byte\" tag is only required if \"is_byte\" is set to false.\n");
        }
        else {
            int byte = _node["byte"].as<int>();
            if(byte < 0 || byte > 7) {
                throw std::range_error(std::string("Error: \"byte: ") + std::to_string(byte) + "\" in \"" + _cType.name_ +
                    "\" can data is out of range.\n" +
                    "The range for \"byte\" is [0, 7].\n");
            }
            else {
                _cType.start_byte_ = byte;
            }
        }
    }
    else if(!_cType.is_byte_) {
        throw std::runtime_error(std::string("Error: \"byte\" tag does not exist while \"is_byte\" flag is set to false.\n") +
            "\"byte\" tag is required if \"is_byte\" is set to false for determining the position of the can data.\n");
    }

    // check if "end_byte" tag exist
    if(_node["end_byte"]) {
        if(!_cType.is_byte_) {
            throw std::runtime_error(std::string("Error: \"end_byte\" tag exist while \"is_byte\" flag is set to false in \"") +
                _cType.name_ + "\" can data.\n" +
                "\"end_byte\" is only required if \"is_byte\" flag is set to true.\n");
        }
        else {
            int end_byte = _node["end_byte"].as<int>();
            if(end_byte <= _cType.start_byte_ || end_byte > 8) {
                throw std::range_error(std::string("Error: \"end_byte: ") + std::to_string(end_byte) + "\" in \"" + _cType.name_ +
                    "\" can data is out of range.\n" +
                    "The range for \"end_byte\" is (start_byte, 8].\n");
            }
            else {
                _cType.end_byte_ = end_byte;
            }
        }
    }
    else if (_cType.is_byte_) {
        throw std::runtime_error(std::string("Error: \"end_byte\" tag does not exist while \"is_byte\" flag is set to true in \"") +
            _cType.name_ + "\" can data.\n" +
            "\"end_byte\" tag is required if \"is_byte\" flag is set to true for determining the position of the can data.\n");
    }
    else {
        _cType.end_byte_ = -1; // set end_byte to -1 if not used when is_byte is set to false
    }

    // check if "is_little_endian" tag exist
    if(_node["is_little_endian"]) {
        if (!_cType.is_byte_) {
            throw std::runtime_error(std::string("Error: \"is_little_endian\" tag exist while \"is_byte\" flag is set to false in \"") +
                _cType.name_ + "\" can data.\n" +
                "\"is_little_endian\" is only required if both \"is_byte\" is set to true and the can data is two byte or longer.\n");
        }
        else if(_cType.end_byte_ - _cType.start_byte_ < 2) {
            throw std::runtime_error(std::string("Error: \"is_little_endian\" tag exist while the can data is less than two bit in \"") +
                _cType.name_ + "\" can data.\n" +
                "\"is_little_endian\" is only required if both \"is_byte\" is set to true and the can data is two byte or longer.\n");
        }
        else {
            _cType.is_little_endian_ = _node["is_little_endian"].as<bool>();
        }
    }
    else if(_cType.is_byte_ && (_cType.end_byte_ - _cType.start_byte_ > 2)) {
        throw std::runtime_error(std::string("Error: \"is_little_endian\" tag does not exist while \"is_byte\" flag is set to true in \"") +
            _cType.name_ + "\" can data.\n" +
            " and the can data is two bit or longer.\n" +
            "\"is_little_endian\" is required if both \"is_byte\" is set to true and the can data is two byte or longer.\n");
    }
    else {
        _cType.is_little_endian_ = false; // set is_little_endian to false if not used when is_byte is set to false or data only one byte long
    }

    // check if "start_bit" tag exist
    if(_node["start_bit"]) {
        if(_cType.is_byte_) {
            throw std::runtime_error(std::string("Error: \"start_bit\" tag exist while \"is_byte\" flag is set to true in \"") +
                _cType.name_ + "\" can data.\n" +
                "\"start_bit\" is only required if \"is_byte\" is set to false.\n");
        }
        else {
            int start_bit = _node["start_bit"].as<int>();
            if(start_bit < 0 || start_bit > 7) {
                throw std::range_error(std::string("Error: \"start_bit: ") + std::to_string(start_bit) + "\" in \"" +
                    _cType.name_ + "\" can data is out of range.\n" +
                    "The range for \"start_bit\" is [0, 7].\n");
            }
            else {
                _cType.start_bit_ = start_bit;
            }
        }
    }
    else if(!_cType.is_byte_) {
        throw std::runtime_error(std::string("Error: \"start_bit\" tag does not exist while \"is_byte\" flag is set to false in \"") +
            _cType.name_ + "\" can data.\n" +
            "\"start_bit\" is required if \"is_byte\" is set to true for determining the position of the can data.\n");
    }
    else {
        _cType.start_bit_ = -1; // set start_bit to -1 if not used when is_byte is set to true
    }

    // check if "end_bit" tag exist
    if(_node["end_bit"]) {
        if(_cType.is_byte_) {
            throw std::runtime_error(std::string("Error: \"end_bit\" tag exist while \"is_byte\" flag is set to true in \"") +
                _cType.name_ + "\" can data.\n" +
                "\"end_bit\" is only required if \"is_byte\" is set to false.\n");
        }
        else {
            int end_bit = _node["end_bit"].as<int>();
            if(end_bit <= _cType.start_bit_ || end_bit > 8) {
                throw std::range_error(std::string("Error: \"end_bit: ") + std::to_string(end_bit) + "\" in \"" +
                    _cType.name_ + "\" can data is out of range.\n" +
                    "The range for \"end_bit\" is (start_bit, 8].\n");
            }
            else {
                _cType.end_bit_ = end_bit;
            }
        }
    }
    else if(!_cType.is_byte_) {
        throw std::runtime_error(std::string("Error: \"end_bit\" tag does not exist while \"is_byte\" flag is set to false in \"") +
            _cType.name_ + "\" can data.\n" +
            "\"end_bit\" is required if \"is_byte\" is set to true for determining the position of the can data.\n");
    }
    else {
        _cType.end_bit_ = -1; // set end_bit to -1 if is not used when is_byte is set to true
    }

    // check if "default" tag exist
    _cType.default_ = (_node["default"] ? _node["default"].as<double>() : 0.0); // default default to 0

    // check if "resolution" tag exist
    _cType.resolution_ = (_node["resolution"] ? _node["resolution"].as<double>() : 1.0); // resolution default to 1

    // check if "offset" tag exist
    _cType.offset_ = (_node["offset"] ? _node["offset"].as<double>() : 0.0); // offset default to 0
    
    _cType.last_data_ = _cType.default_; // lase_data default to default

    return true;
}

std::string Frame::get_string() const {
    std::string data;
    for(auto it = dataset_.begin(); it != dataset_.end(); it++) {
        data += '\n' + it->second->get_string();
    }

    std::string string = "CAN frame: " + name_ + " --------------------" +
        "\n\tid: " + (boost::format("%x") % id_).str() +
        "\n\tis_extended_id: " + (is_extended_id_ ? "true" : "false") +
        "\n\tdlc: " + std::to_string(dlc_) +
        "\n\tperiod: " + std::to_string(period_) +
        "\n\tdt: " + std::to_string(dt_) +
        data;
    return string;
}

u_int64_t Frame::get_occupied_bit() const {
    u_int64_t occupied_bit = 0;
    for(auto it = dataset_.begin(); it != dataset_.end(); it ++) {
        u_int64_t data_occupied_bit = it->second->get_occupied_bit();
        if((data_occupied_bit & occupied_bit) != 0) {
            throw std::runtime_error(std::string("Error: There are two can data in can frame \"") + name_ +
                "\" that have overlapping data position.\n");
        }
        else {
            occupied_bit |= data_occupied_bit;
        }
    }
    return occupied_bit;
}

u_int8_t Frame::get_occupied_byte() const {
    u_int8_t occupied_byte = 0;
    for(auto it = dataset_.begin(); it != dataset_.end(); it ++) {
        occupied_byte |= it->second->get_occupied_byte();
    }
    return occupied_byte;
}

int Frame::get_higtest_occupied_byte() const {
    int hightest_occupied_byte = 0;
    u_int64_t occupied_byte = get_occupied_bit();
    while(occupied_byte != 0) {
        hightest_occupied_byte ++;
        occupied_byte >>= 8;
    }
    return hightest_occupied_byte;
}

bool YAML::convert<Frame>::decode(const Node &_node, Frame &_cType) {
    // a 8x8 bit unsigned int for tracking which bit in which byte of a can frame is occupied
    unsigned long long occupied_data = 0;

    // check if "name" tag exist
    if(_node["name"]) {
        _cType.name_ = _node["name"].as<std::string>();
    }
    else {
        throw std::runtime_error(std::string("Error: There is no \"name\" tag under some \"frame\" tag.\n") +
            "\"name\" tag is required for accessing a can frame.\n");
    }

    // check if "id" tag exist
    if(_node["id"]) {
        _cType.id_ = _node["id"].as<unsigned int>();
    }
    else {
        throw std::runtime_error(std::string("Error: There is no \"id\" tag in\"") + _cType.name_ + "\"can frame\n" +
            "\"id\" tag is required for sending/receiving data correctly");
    }

    // check if "is_extended_id" tag exist
    if(_node["is_extended_id"]) {
        bool is_extended_id = _node["is_extended_id"].as<bool>();
        if(!is_extended_id && _cType.id_ > 0b11111111111) {
            throw std::runtime_error(std::string("Error: The length of id is greater than 12 bit while is_extended_id is set to flase in \"") +
                _cType.name_ + "\" can frame.\n" +
                "Please update \"is_extended_id\" accordingly");
        }
        else {
            _cType.is_extended_id_ = is_extended_id;
        }
    }
    else { // is_extended_id default to weather the id is 12 bit or longer
        if(_cType.id_ > 0b11111111111) {
            _cType.is_extended_id_ = true;
        }
        else {
            _cType.is_extended_id_ = false;
        }
    }
    
    // check if "period" tag exist
    _cType.period_ = _cType.period_ = (_node["period"] ? _node["period"].as<double>() : 0); // period default to 0

    _cType.dt_ = 0; // initialize dt to 0

    // check if "dataset" tag exist
    if(_node["dataset"]) {
        // check if "data" tag exist
        if(_node["dataset"]["data"]) {
            for(auto it = _node["dataset"].begin(); it != _node["dataset"].end(); it ++) {
                auto data = std::make_shared<Data>(it->second.as<Data>());
                _cType.dataset_[data->name_] = data;
            }
        }
        else {
            throw std::runtime_error(std::string("Error: \"data\" tag does not exist under \"dataset\" in \"") +  _cType.name_ +
                "\" can frame.\n" +
                "At least one \"data\" tag is rquired for storing can data of this frame.\n");
        }
    }
    else {
        throw std::runtime_error(std::string("Error: \"dataset\" tag does not in \"") + _cType.name_ + "\" can frame.\n" +
            "\"dataset\" tag is required for storing can data of this frame.\n");
    }

    int hightest_occupied_byte = _cType.get_higtest_occupied_byte();
    // check if "dlc" tag exist
    if(_node["dlc"]) {
        int dlc = _node["dlc"].as<int>();
        if(dlc < hightest_occupied_byte) {
            throw std::runtime_error(std::string("Error: The data lenght derived from can data is ") +
                std::to_string(hightest_occupied_byte) + ", which is greater than \"dlc\" (" + std::to_string(dlc) + ") in \"" +
                _cType.name_ + "can frame.\n" +
                "Please update \"dlc\" accordingly.\n");
        }
        else {
            _cType.dlc_ = dlc;
        }
    }
    else {
        _cType.dlc_ = hightest_occupied_byte; // dlc default to the highest occupied byte of the frame
    }

    return true;
}

IdFrameset load_yaml(const std::string &_file) {
    IdFrameset frameset;
    YAML::Node file = YAML::LoadFile(_file);
    // check if "can" tag exist
    if(!file["can"]) {
        throw std::runtime_error(std::string("Error: \"can\" tag dose not exist in \"") + _file + "\" file.\n" +
            "\"can\" tag is required as the root of the can configuration file.\n");
    }
    else {
        YAML::Node can = file["can"];
        for(auto frame_it = can.begin(); frame_it != can.end(); frame_it++) {
            auto frame = std::make_shared<Frame>(frame_it->second.as<Frame>());
            
            // this is pretty stupid to initialize a data after it has been initialized, but custom yaml convert is also stupid
            for(auto data_it = frame->dataset_.begin(); data_it != frame->dataset_.end(); data_it++) {
                data_it->second->frame_ = frame;
            }
            frameset[frame->id_] = frame;
        }
    }
    return frameset;
}

NameFrameset convert_to_name_frame(const IdFrameset &_id_frameset) {
    NameFrameset frameset;
    for(auto it = _id_frameset.begin(); it != _id_frameset.end(); it++) {
        frameset[it->second->name_] = it->second;
    }
    return frameset;
}

Dataset get_dataset(const IdFrameset &_frameset) {
    Dataset dataset;
    for(auto frame_it = _frameset.begin(); frame_it != _frameset.end(); frame_it++) {
        for(auto data_it = frame_it->second->dataset_.begin(); data_it != frame_it->second->dataset_.end(); data_it++) {
            dataset[data_it->first] = data_it->second;
        }
    }
    return dataset;
}

Dataset get_dataset(const NameFrameset &_frameset) {
    Dataset dataset;
    for(auto frame_it = _frameset.begin(); frame_it != _frameset.end(); frame_it++) {
        for(auto data_it = frame_it->second->dataset_.begin(); data_it != frame_it->second->dataset_.end(); data_it++) {
            dataset[data_it->first] = data_it->second;
        }
    }
    return dataset;
}

std::string get_string(const IdFrameset &_frameset) {
    std::string string;
    for(auto it = _frameset.begin(); it != _frameset.end(); it++) {
        if(it == _frameset.begin()) {
            string += it->second->get_string();
        }
        else {
            string += '\n' + it->second->get_string();
        }
    }
    return string;
}

std::string get_string(const NameFrameset &_frameset) {
    std::string string;
    for(auto it = _frameset.begin(); it != _frameset.end(); it++) {
        if(it == _frameset.begin()) {
            string += it->second->get_string();
        }
        else {
            string += '\n' + it->second->get_string();
        }
    }
    return string;
}
