// std include
#include <iostream>

// boost include
#include <boost/array.hpp>

// nturt include
#include "yaml_loader.hpp"
#include "can_parser.hpp"

NameFrameset frame = convert_to_name_frame(load_yaml("/home/docker/ws/src/nturt_can_parser/doc/test.yaml"));
DataPtr data = frame["mcu_command"]->dataset_["torque_command"];
boost::array<u_int8_t, 8> raw_data = {0};

int main() {
    std::cout << frame << std::endl;

    // decode
    raw_data = {0, 128, 0, 0, 0, 0, 0, 0};
    decode(data, raw_data);
    std::cout << "decoded data: " << data->last_data_ << std::endl;

    // encode
    data->last_data_ = -3276.8;
    raw_data = {0};
    encode(data, raw_data);

    std::cout << "encoded data: ";
    for(int i = 0; i < 8; i++) {
        std::cout << static_cast<int>(raw_data[i]) << ' ';
    }
    std::cout << std::endl;

    return 0;
}
