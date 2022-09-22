#include <iostream>
#include <memory>
#include "yaml_loader.hpp"

int main() {
    std::map<std::string, std::shared_ptr<Frame>> frameset = load_yaml("/home/docker/ws/src/nturt_can_parser/doc/can.yaml");
    std::string can = get_string(frameset);
    std::cout << can;

    return 0;
}
