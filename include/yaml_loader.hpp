#ifndef YAML_LAODER_HPP
#define YAML_LAODER_HPP

// std include
#include <iostream>
#include <map>
#include <string>

// boost include
#include <boost/format.hpp>

// yaml include
#include <yaml-cpp/yaml.h>

class Data {
    public:
        std::string name_;
        int start_byte_;
        int end_byte_;
        int start_bit_;
        int end_bit_;
        double default_;
        double resolution_;
        double offset_;
        bool is_signed_;
        bool is_little_endian_;
        std::string get_string() const;
        

        double after_decode_;    // test
        double to_be_encode_;    // test
        bool flag_;              // test


    private:
        double last_data_;
};

class Frame {
    public:
        std::string name_;
        unsigned int id_;
        bool is_extended_id_;
        int dlc_;
        double frequency_;
        std::map<std::string, Data> dataset_;
        std::string get_string() const;
        std::vector<Data> datavector_;
    private:
        double dt_;
};

inline std::ostream& operator<<(std::ostream &_ostream, const Data &_data) {
    return _ostream << _data.get_string();
}

inline std::ostream& operator<<(std::ostream &_ostream, const Frame &_frame) {
    return _ostream << _frame.get_string();
}

namespace YAML {
template<>
struct convert<Frame> {
    static bool decode(const Node &_node, Frame &_cType) {
        _cType.name_ = _node["name"].as<std::string>();
        _cType.id_ = _node["id"].as<unsigned int>();
        _cType.is_extended_id_ = _node["is_extended_id"].as<bool>();
        _cType.dlc_ = _node["dlc"].as<int>();
        _cType.frequency_ = _node["frequency"].as<double>();
        for(const_iterator it = _node["dataset"].begin(); it != _node["dataset"].end(); it++) {
            Data data = it->second.as<Data>();
            //std::cout << data <<std::endl;
            _cType.dataset_[data.name_] = data;
            _cType.datavector_.push_back(data);
        }
        return true;
    }
};

template<>
struct convert<Data> {
    static bool decode(const Node &_node, Data &_cType) {
        _cType.name_ = _node["name"].as<std::string>();
        _cType.start_byte_ = _node["start_byte"].as<int>();
        _cType.end_byte_ = _node["end_byte"].as<int>();
        _cType.start_bit_ = _node["start_bit"].as<int>();
        _cType.end_bit_ = _node["end_bit"].as<int>();
        _cType.default_ = (_node["default"] ? _node["default"].as<double>() : 0);
        _cType.resolution_ = _node["resolution"].as<double>();
        _cType.offset_ = _node["offset"].as<double>();
        _cType.is_signed_ = _node["is_signed"].as<bool>();
        _cType.is_little_endian_ = _node["is_little_endian"].as<bool>();
        _cType.after_decode_ = 0.0;
        _cType.to_be_encode_= 0.0;
        _cType.flag_ = false;
        return true;
    }
};

} // namespace YAML

std::map<std::string, Frame> load_yaml(std::string _file);

#endif // YAML_LAODER_HPP
