#include "cp_can_id.hpp"
#include <NTURT_CAN_Parser.hpp>
#include <iostream>

// This program is to
// - Validate the CAN Parser
// - Demostrate the usage
int main() {
  Parser myparser;
  myparser.init_parser();
  double fwsr = -20, fwsl = 30;
  double afdfwsr = 0, afdfwsl = 0;
  int id = _CAN_CIA;
  int data[8];
  // For example, we may want to encode wheel speed data
  // which we are going to send it to dashboard.
  myparser.set_tbe("CIA", "X", fwsl);
  myparser.set_tbe("CIA", "Y", fwsr);
  myparser.encode(id, data);
  // After this, we can send the "data" to CAN
  for (int d : data) {
    std::cout << "data: " << d << std::endl;
  }
  // Another example, we want to decode data for certain id:
  // Then we can decode the CAN data and get, for
  // example, front wheel speed
  if (myparser.check_key(id, "CIA") == OK) {
    if (myparser.decode(id, data) == OK) {
      afdfwsl = myparser.get_afd("CIA", "X");
      afdfwsr = myparser.get_afd("CIA", "Y");
    }
  }
  std::cout << "Front wheel speed left: " << afdfwsl << ", right: " << afdfwsr
            << std::endl;
  myparser.print_err_log();
  return 0;
}