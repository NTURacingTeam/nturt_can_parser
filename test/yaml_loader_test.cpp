#include "yaml_loader.hpp"

int main() {
    std::map<std::string, Frame> frameset = load_yaml("/home/docker/ws/src/nturt_can_parser/doc/can.yaml");
    for(auto frame_it = frameset.begin(); frame_it != frameset.end(); frame_it++) {
            Frame frame = frame_it->second;
            std::cout << frame << std::endl;
            for(auto data_it = frame.dataset_.begin(); data_it != frame.dataset_.end(); data_it++) {
                Data data = data_it->second;
                std::cout << data << std::endl;
            }
    }
    return 0;
}
