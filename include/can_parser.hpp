#ifndef CAN_PARSER_HPP
#define CAN_PARSER_HPP

// std include
#include <iostream>
#include <map>
#include <string>

// can parser include
#include "yaml_loader.hpp"

class CanParser{
    public:
        CanParser();
    private:
        std::map<std::string, Frame> frameset_;
};

#endif // CAN_PARSER_HPP
