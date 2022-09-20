#include <memory>
#include "yaml_loader.hpp"


/**
 * @brief test
 *
 * @return test 1
 */
int main() {
    std::map<std::string, std::shared_ptr<Frame>> frameset = load_yaml("/home/docker/ws/src/nturt_can_parser/doc/can.yaml");
    std::cout << "===========\n";
    for(auto frame_it = frameset.begin(); frame_it != frameset.end(); frame_it++) {
        std::shared_ptr<Frame> frame = frame_it->second;
        //std::cout << frame << std::endl;
        for(auto data_it = frame->dataset_.begin(); data_it != frame->dataset_.end(); data_it++) {
            std::shared_ptr<Data> data = data_it->second;
            //std::cout << data << std::endl;
        }
        //std::cout << "===========\n";
        //std::cout << "cout vector\n";
        for (auto datavector_it = frame->datavector_.begin(); datavector_it != frame->datavector_.end(); datavector_it++) {
            std::cout << *datavector_it << std::endl;
        }
        //std::cout << "vector endl\n";
        //std::cout << "===========\n";

    }
    std::cout << "===========\n";
    std::cout << frameset["front_box_2"]->dataset_["accelerator_level_1"] << std::endl;
    std::cout << frameset["front_box_2"]->datavector_[0] << std::endl;
    std::cout << frameset["front_box_2"]->datavector_[1] << std::endl;
    return 0;
}
