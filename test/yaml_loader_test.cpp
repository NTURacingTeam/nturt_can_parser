// std include
#include <iostream>
#include <memory>

// nturt include
#include "yaml_loader.hpp"

int main(int argc, char **argv) {
    if(argc != 2) {
        std::cout << "Usage: <can file>" << std::endl;
        return 1;
    }

    if(argv[1] == "-h" || argv[1] == "--help") {
        std::cout << "Usage: <can file>" << std::endl;
    }
    else {
        Frameset frameset = load_yaml(argv[1]);
        std::string can = get_string(frameset);
        std::cout << can;
    }

    return 0;
}
