/**
 * @file yaml_loader.hpp
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Yaml loader for can parser to load can rules from a yaml file.
 */

#ifndef YAML_LAODER_HPP
#define YAML_LAODER_HPP

// std include
#include <bitset>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

// boost include
#include <boost/format.hpp>

// yaml include
#include <yaml-cpp/yaml.h>

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Struct for storing can data format.
 */
struct Data {
    /// @brief Name of this can data, it will be used as key of the "dataset" map which stores all can data.
    std::string name_;

    /// @brief If this can data is signed.
    bool is_signed_;

    /// @brief If this can data is a byte data.
    bool is_byte_;

    /// @brief If this can data is little endian, i.e. most significant bit first.
    bool is_little_endian_;

    /// @brief Start byte of this can data, the range will be determined as [start_byte, end_byte).
    int start_byte_;

    /// @brief End byte of this can data, the range will be determined as [start_byte, end_byte).
    int end_byte_;

    /// @brief Start bit of this can data, the range will be determined as [start_bit, end_bit).
    int start_bit_;

    /// @brief End bit of this can data, the range will be determined as [start_bit, end_bit).
    int end_bit_;

    /// @brief Default value of this can data if this can data has never been sent/received, default to 0 if not
    ///        specified in the yaml file.
    double default_;

    /// @brief Resolution of this can data, i.e. the change of value of this can data if on least significant bit
    ///        is changed.
    double resolution_;

    /// @brief Offset of this can data, calculated as original value - offset.
    double offset_;

    /// @brief Buffer storing the value of this can data last time it was sent/received.
    double last_data_;
    
    /**
     * @brief Get the string repersentation of can data.
     * @return String representation of can data.
     */
    std::string get_string() const;

    /**
     * @brief Get which bit of the can frame is occupied by this can data.
     * @return Which bit(s) is/are occupied by this can data in the can frame, which is determined as #byte * 8 + #bit.
     */
    std::bitset<64> get_occupied_bit();

    /**
     * @brief Get which byte of this can data is occupied.
     * @param _data Can data whose occupation of a can frame data byte is to be determined.
     * @return Which byte(s) is/are occupied by this can data in the can frame.
     */
    std::bitset<8> get_occupied_byte();
    
    double after_decode_;    // test
    double to_be_encode_;    // test
    bool flag_;              // test
};

typedef std::shared_ptr<Data> DataPtr;

/**
 * @brief Struct for storing can frame format.
 * @author QuantumSpawner jet22854111@gmail.com
 */
struct Frame {
    /// @brief Name of this can frame, it will be used as key of the "frameset" map which stores all can frames.
    std::string name_;

    /// @brief Can id of this frame.
    unsigned int id_;

    /// @brief If the id is extended format, determined automatically when not specified, i.e. true if the id is
    ///        more than 15 bit long.
    bool is_extended_id_;

    /// @brief The number of bytes in this can frame.
    int dlc_;

    /// @brief Period that this can frame is sent, set to 0 to disable sending this can frame.
    /// @todo Change all instalces of frequency to period.
    double period_;

    /// @brief Time difference between the last this frame was sent [s].
    double dt_;

    /// @brief Map storing pointer to can data correspond to this can frame, with key being the name of the can data .
    std::map<std::string, DataPtr> dataset_;

    /// @brief Vector storing pointer ro can data correspond to this can frame.
    std::vector<DataPtr> datavector_;
    
    /**
     * @brief Get the string representation of can frame.
     * @return String representation of can frame.
     */
    std::string get_string() const;

    /**
     * @brief Get which bit of this can frame is occupied.
     * @return Which bit(s) is/are occupied in this can frame, which is determined as #byte * 8 + #bit.
     * @throw std::runtime_error Throw exception when two can data have overlapping data positions.
     */
    std::bitset<64> get_occupied_bit();

    /**
     * @brief Get which byte of this can frame is occupied.
     * @return Which byte(s) is/are occupied in this can frame.
     * @todo Currently no checking will be done if two can data have overlapping data positions.
     */
    std::bitset<8> get_occupied_byte();

    /**
     * @brief Get the higest occupied byte of this can frame.
     * @return Which byte is the hightest occupied byte.
     */
    int get_higtest_occupied_byte();
};

typedef std::shared_ptr<Frame> FramePtr;

/**
 * @brief Operator used for passing the string representsation of can data to ostream.
 * @param _ostream
 * @param _data
 * @return std::ostream&
 */
inline std::ostream& operator<<(std::ostream &_ostream, const Data &_data) {
    return _ostream << _data.get_string();
}

/**
 * @brief Operator used for passing the string representsation of can frame to ostream.
 * @param _ostream
 * @param _frame
 * @return std::ostream&
 */
inline std::ostream& operator<<(std::ostream &_ostream, const Frame &_frame) {
    return _ostream << _frame.get_string();
}

/// @cond YAML
namespace YAML {

/// @brief Class template used to convert yaml node containing can data into C++ can data class
template<>
struct convert<Data> {
    /**
     * @brief Function to convert yaml node containing can data into C++ can data class, which is implemented in yaml-cpp
     *        as "as<Data>()" function.
     * @param[in] _node The yaml node containing can data.
     * @param[out] _cType The reference of can data class, used to store can data.
     * @return true
     * @throw std::runtime_error
     */
    static bool decode(const Node &_node, Data &_cType);
};

/// @brief Class template used to convert yaml node containing can frame into C++ can frame class.
template<>
struct convert<Frame> {
    /**
     * @brief Function to convert yaml node containing can frame into C++ can frame class, which is implemented in yaml-cpp
     *        as "as<Frame>()" function.
     * @param[in] _node The yaml node containing can frame.
     * @param[out] _cType The reference of can frame class, used to store can frame.
     * @return true
     * @throw std::runtime_error
     */
    static bool decode(const Node &_node, Frame &_cType);
};

} // namespace YAML

/// @endcond

typedef std::map<int, FramePtr> Frameset;

/**
 * @brief Function to load a yaml file into a map containing all can frames and their corresponding data.
 * @param[in] _file The path of the yaml file which contains the can rule.
 * @return Frameset, a map storing pointer to can frame, with key being the id of the can frame.
 * @throw std::runtime_error
 */
Frameset load_yaml(std::string _file);

/**
 * @brief Function to print frameset.
 * @param[in] _frameset The frameset to print.
 * @return The string representation of the frameset.
 */
std::string get_string(Frameset _frameset);

#endif // YAML_LAODER_HPP
