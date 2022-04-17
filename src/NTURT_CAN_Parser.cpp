#include <NTURT_CAN_Parser.hpp>

int Parser::init_parser() {
  import_rule("/home/ros/nturt_ws/src/nturt_can_parser/rule.csv");
  // prepare 2 power chart
  for (int i = 0; i < 8; i++) {
    pow256[i] = pow(256, i);
    pow2[i] = pow(2, i);
  }
  return OK;
}

int Parser::update_rule(std::string key, std::string comp, int &id,
                        int &bitbyte, int &endian, int &startbyte,
                        int &stopbyte, int &startbit, int &stopbit,
                        double &scale, double &offset) {
  // update rule map
  rule_[key][comp].id = id;
  rule_[key][comp].bitbyte = bitbyte;
  rule_[key][comp].endian = endian;
  rule_[key][comp].startbyte = startbyte;
  rule_[key][comp].stopbyte = stopbyte;
  rule_[key][comp].startbit = startbit;
  rule_[key][comp].stopbit = stopbit;
  rule_[key][comp].scale = scale;
  rule_[key][comp].offset = offset;
  // update id-frame pair
  frame_[id].data_key.push_back(std::pair<std::string, std::string>(key, comp));
  // update data flag
  flag_[key][comp] = 0;
  // update tbe and afd
  tbe[key][comp] = 0;
  afd[key][comp] = 0;
  return OK;
}

int Parser::import_rule(std::string path) {
  std::ifstream file(path);
  if (!file) {
    err_log(__func__, "File open error");
    return ERR;
  }
  std::string buf, word;
  char _key[10], _frame[4], _comp[10];
  while (getline(file, buf)) {
    std::vector<std::string> row;
    std::stringstream str(buf);
    int id, bitbyte, endian, startbyte, stopbyte, startbit, stopbit;
    double scale, offset;
    while (getline(str, word, ',')) {
      row.push_back(word);
    }
    std::string key(row[0]);
    std::string comp(row[1]);
    bitbyte = std::stoi(row[3]);
    endian = std::stoi(row[4]);
    startbyte = std::stoi(row[5]);
    stopbyte = std::stoi(row[6]);
    startbit = std::stoi(row[7]);
    stopbit = std::stoi(row[8]);
    scale = std::stod(row[9]);
    offset = std::stod(row[10]);
    // id need other way to handle
    std::stringstream ss;
    ss << std::hex << row[2];
    ss >> id;
    update_rule(key, comp, id, bitbyte, endian, startbyte, stopbyte, startbit,
                stopbit, scale, offset);
  }
  std::cout << "key, comp: " << frame_[0x040AD091].data_key.at(1).first << ", "
            << frame_[0x040AD091].data_key.at(1).second << std::endl;
  return OK;
}

int Parser::decode(int id, int *data) {
  for (auto keys : frame_[id].data_key) {
    Rule r = rule_[keys.first][keys.second];
    // if stored in byte
    long long compose = 0;
    if (r.bitbyte == _CP_BYTE) {
      if (r.endian == _CP_LITTLE) {
        for (int i = r.startbyte; i < r.stopbyte; i++) {
          compose |= data[i] << 8 * (i - r.startbyte);
        }
      } else if (r.endian == _CP_BIG) {
        for (int i = r.stopbyte; i > r.startbyte; i--) {
          compose |= data[i - 1] << 8 * (r.stopbyte - i);
        }
      } else {
        err_log(__func__, "Wrong endian");
      }
      // handle the signed int stuff.
      int sign_handling_bit = 8 * (r.stopbyte - r.startbyte) - 1;
      bool negative = (compose >> sign_handling_bit) & 1;
      long long neg_mask;
      if (negative) {
        neg_mask = ~((int)pow256[r.stopbyte - r.startbyte] - 1);
        compose |= neg_mask;
      }
      // std::cout << "sign bit: " << sign_handling_bit
      //           << ", negative: " << negative << ", neg mask: " << neg_mask
      //          << ", compose: " << compose << std::endl;
    }
    // if stored in bit
    else if (r.bitbyte == _CP_BIT) {
      // forced little endian
      compose = (data[r.startbyte] >> r.startbit) % pow2[r.stopbit];
    } else {
      err_log(__func__, "Wrong bit/byte setting");
    }
    afd[keys.first][keys.second] = ((double)r.scale * compose) + r.offset;
    flag_[keys.first][keys.second] = true;
  }
  return OK;
}

int Parser::encode(int id, int *data) {
  for (auto keys : frame_[id].data_key) {
    Rule r = rule_[keys.first][keys.second];
    double _tbe = tbe[keys.first][keys.second];
    long long __tbe = (_tbe - r.offset) / r.scale;
    // std::cout << "comp: " << keys.second << " start,stop: " << r.startbyte <<
    // r.stopbyte
    //          << " byte? " << (r.bitbyte == _CP_BYTE) << " tbe " << _tbe <<
    //          std::endl;
    // if stored in byte
    if (r.bitbyte == _CP_BYTE) {
      if (r.endian == _CP_LITTLE) {
        for (int i = r.startbyte; i < r.stopbyte; i++) {
          data[i] = (__tbe >> (i - r.startbyte) * 8) & _CP_MASK_LAST_8_BIT;
          // std::cout << "i, data: " << i << "," << data[i] << std::endl;
        }
      } else if (r.endian == _CP_BIG) {
        for (int i = r.stopbyte; i > r.startbyte; i--) {
          data[i - 1] = (__tbe >> (r.stopbyte - i) * 8) & _CP_MASK_LAST_8_BIT;
          // std::cout << "i, data: " << i - 1 << ", tbe: " << __tbe
          //           << ", shifted: " << (__tbe >> (r.stopbyte - i) * 8)
          //           << ", result: " << data[i - 1] << std::endl;
        }
      } else {
        err_log(__func__, "Wrong endian");
      }
    }
    // if stored in bit
    else if (r.bitbyte == _CP_BIT) {
      // forced little endian
      unsigned char mask = ~(((~0) << r.startbit) % pow2[r.stopbit]);
      data[r.startbyte] = (data[r.startbyte] & mask) | (__tbe);
    } else {
      err_log(__func__, "Wrong bit/byte setting");
    }
  }
  return OK;
}

#ifdef BOOST_ARRAY
int Parser::decode(int id, const boost::array<unsigned char, 8> data) {
  int idata[8];
  for (int i = 0; i < 8; i++) {
    idata[i] = data[i];
  }
  int res = decode(id, idata);
  return res;
}
int Parser::encode(int id, boost::array<unsigned char, 8> &data) {
  int idata[8];
  int res = encode(id, idata);
  for (int i = 0; i < 8; i++) {
    data[i] = idata[i];
  }
  return res;
}
#endif

int Parser::set_tbe(std::string key, std::map<std::string, double> &_tbe) {
  // key check
  if (tbe.find(key) == tbe.end()) {
    err_log(__func__, "No such key");
    return ERR;
  } else {
    tbe[key] = _tbe;
    return OK;
  }
}

int Parser::set_tbe(std::string key, std::string comp, double _tbe) {
  // key check
  if (tbe.find(key) == tbe.end()) {
    err_log(__func__, "No such key");
    return ERR;
  } else {
    if (tbe[key].find(comp) == tbe[key].end()) {
      err_log(__func__, "No such comp");
      return ERR;
    }
    tbe[key][comp] = _tbe;
    return OK;
  }
}

int Parser::get_afd(std::string key, std::map<std::string, double> &_afd) {
  // key check
  if (afd.find(key) == afd.end()) {
    err_log(__func__, "No such key");
    return ERR;
  }
  auto it = _afd.begin();
  int res = OK;
  while (it != _afd.end()) {
    if (flag_[key][it->first]) {
      it->second = afd[key][it->first];
      flag_[key][it->first] = false;
    } else {
      err_log(__func__, "(key,comp) = (" + key + "," + it->first +
                            ") Get data before decode");
      res = ERR;
    }
  }
  return res;
}

double Parser::get_afd(std::string key, std::string comp) {
  // key check
  if (afd.find(key) == afd.end()) {
    err_log(__func__, "No such key");
    return ERR;
  }
  if (afd[key].find(comp) == afd[key].end()) {
    err_log(__func__, "No such comp");
    return ERR;
  }
  int res = OK;
  if (flag_[key][comp]) {
    flag_[key][comp] = false;
    return afd[key][comp];
  } else {
    err_log(__func__,
            "(key,comp) = (" + key + "," + comp + ") Get data before decode");
    res = ERR;
  }
  return res;
}
