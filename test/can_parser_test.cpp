#include "can_parser.hpp"

int main() {
    CanParser test;
    //std::vector<std::pair<std::string, std::string>> result = test.get_key(123);
    //std::cout << result[0].first << "\n" << result[0].second << "\n";
    auto result = test.get_key(0x080AD092);
    //for (auto it = result.begin(); it != result.end(); it++) {
    //    cout << it->first << "\n";
    //    cout << it->second << "\n";
    //}
    std::cout << "=========\n";
    int data[8] = {100,150,155,60,10,15,20,0};
    test.decode(0x080AD092, data);
    return 0;
}
