#include "yaml_loader.hpp"

std::string Data::get_string() const {
    std::string str = "CAN data: " + name_ + " --------------------" +
                      "\n\tstart_byte: " + std::to_string(start_byte_) +
                      "\n\tend_byte: " + std::to_string(end_byte_) +
                      "\n\tstart_bit: " + std::to_string(start_bit_) +
                      "\n\tend_bit: " + std::to_string(end_bit_) +
                      "\n\tdefault: " + std::to_string(default_) +
                      "\n\tresolution: " + std::to_string(resolution_) +
                      "\n\toffset: " + std::to_string(offset_) +
                      "\n\tis_signed: " + (is_signed_ ? "true" : "false") +
                      "\n\tis_little_endian: " + (is_little_endian_ ? "true" : "false");
    return str;
}

std::string Frame::get_string() const {
    std::string str = "CAN frame: " + name_ + " --------------------" +
                      "\n\tid: " + (boost::format("%x") % id_).str() +
                      "\n\tis_extended_id: " + (is_extended_id_ ? "true" : "false") +
                      "\n\tdlc: " + std::to_string(dlc_) +
                      "\n\tperiod: " + std::to_string(period_);
    return str;
}

std::map<std::string, Frame> load_yaml(std::string _file) {
    std::map<std::string, Frame> frameset;
    YAML::Node can = YAML::LoadFile(_file)["can"];
    for(YAML::const_iterator it = can.begin(); it != can.end(); it++) {
        Frame frame = it->second.as<Frame>();
        frameset[frame.name_] = frame;
    }
    return frameset;
}