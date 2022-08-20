#include "can_parser.hpp"

CanParser::CanParser() {
    frameset_ = load_yaml("/home/docker/ws/src/nturt_can_parser/doc/can.yaml");
}
