#include "yaml_loader.hpp"

std::string Data::get_string() const {
    std::string str = "\tCAN data: " + name_ + " --------------------" +
                      "\n\t\tstart_byte: " + std::to_string(start_byte_) +
                      "\n\t\tend_byte: " + std::to_string(end_byte_) +
                      "\n\t\tstart_bit: " + std::to_string(start_bit_) +
                      "\n\t\tend_bit: " + std::to_string(end_bit_) +
                      "\n\t\tdefault: " + std::to_string(default_) +
                      "\n\t\tresolution: " + std::to_string(resolution_) +
                      "\n\t\toffset: " + std::to_string(offset_) +
                      "\n\t\tis_signed: " + (is_signed_ ? "true" : "false") +
                      "\n\t\tis_little_endian: " + (is_little_endian_ ? "true" : "false");
    return str;
}

std::string Frame::get_string() const {
    std::string str = "CAN frame: " + name_ + " --------------------" +
                      "\n\tid: " + (boost::format("%x") % id_).str() +
                      "\n\tis_extended_id: " + (is_extended_id_ ? "true" : "false") +
                      "\n\tdlc: " + std::to_string(dlc_) +
                      "\n\tfrequency: " + std::to_string(frequency_);
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
