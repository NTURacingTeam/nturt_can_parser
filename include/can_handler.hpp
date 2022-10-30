/**
 * @file can_handler.hpp
 * @brief Can handler for handling the conversion of raw can data and desired data.
 * @author QuantumSpawner jet22854111@gmail.com
 */

#ifndef CAN_HANDLER_HPP
#define CAN_HANDLER_HPP

// std include
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

// boost include
#include <boost/array.hpp>

// nturt include
#include "yaml_loader.hpp"

// type definition
/// @brief Function type definition for publishing can frame.
typedef std::function<void(const FramePtr&, const boost::array<uint8_t, 8>&)> PublishFun;

/**
 * @brief Class for handling the conversion of raw can data and desired data, with orther utilities to fufill normal use cases.
 * @author QuantumSpawner jet22854111@gmail.com
 */
class CanHandler{
    public:
        /**
         * @brief Initialize can parser and load yaml file.
         * @param _file Path to the can yaml file.
         */
        void init(std::string _file);

        /**
         * @brief Function to publish frame by it's pointer.
         * @param _frame Pointer to the can frame.
         * @param publish_fun Function to publish the frame, whose arguments are frame pointer and data of the frame.
         */
        void publish(const FramePtr &_frame, const PublishFun publish_fun) const;

        /**
         * @brief Function to publish frame by name.
         * @param[in] _name Name of the can frame to be published.
         * @param[in] publish_fun Function to publish the frame, whose arguments are frame pointer and data of the frame.
         * @return True if the frame is published successfully, or false if not found.
         */
        bool publish(const std::string &_name, const PublishFun publish_fun) const;

        /**
         * @brief Function to publish the periodically published frame.
         * @param[in] _dt The time difference between this and last call of this function.
         * @param[in] publish_fun Function to publish the frame, whose arguments are frame pointer and data of the frame.
         */
        void periodic_publish(const double &_dt, const PublishFun publish_fun) const;

        /**
         * @brief Function to update the can data.
         * @param[in] _name The name of the can data.
         * @param[in] _value The value of the can data.
         * @return Pointer to can data, nullptr if not found.
         */
        DataPtr update_data(const std::string &_name, const double &_value);

        /**
         * @brief Function to update the can data of a frame.
         * @param[in] _id The id of the frame.
         * @param[in] _data The data of the frame.
         * @return Pointer to the updated frame, nullptr if not found.
         */
        FramePtr update_frame(const int &_id, const boost::array<uint8_t, 8> &_data);

        /**
         * @brief Get can data using the data's name.
         * @param[in] _name The name of the can data.
         * @return Pointer to the can data, nullptr if not found.
         */
        DataPtr get_data(const std::string &_name) const;

        /**
         * @brief Get can frame using the frame's id.
         * @param[in] _id Id of the can frame.
         * @return Pointer to the can frame, nullptr if not found.
         */
        FramePtr get_frame(const int &_id) const;

        /**
         * @brief Get frame using the frame's name.
         * @param[in] _name Name of the can frame.
         * @return Pointer to the can frame, nullptr if not found.
         */
        FramePtr get_frame(const std::string &_name) const;
        
        /**
         * @brief Get dataset stroed in the can parser.
         * @return Dataset, a map storing pointer to can data of all frames, with key being the name of the can data.
         */
        Dataset get_dataset() const;

        /**
         * @brief Get id frameset stored in the can parser.
         * @return Frameset, a map storing pointer to can frame, with key being the id of the can frame.
         */
        IdFrameset get_id_frameset() const;

        /**
         * @brief Get name frameset stored in the can parser.
         * @return Frameset, a map storing pointer to can frame, with key being the name of the can frame.
         */
        NameFrameset get_name_frameset() const;

    private:
        /// @brief Map storing pointer to can frame, with key being the id of the can frame.
        IdFrameset id_frameset_;

        /// @brief Id frameset that stores only frames that have to be periodically published.
        IdFrameset periodic_publish_frameset_;

        /// @brief Map storing pointer to can frame, with key being the name of the can frame.
        NameFrameset name_frameset_;

        /// @brief Map storing pointer to can data, with key being the name of the can data.
        Dataset dataset_;
};

/**
 * @brief Function to decode raw can data from a frame into desired data according to the data's configuration.
 * 
 * The data is calculated by \f$\text{resolution}\times\text{raw data}-\text{offset}\f$.
 * @param[out] _data Pointer to can data where last_data is modified by the decoded data form raw_data.
 * @param[in] _raw_data Raw can data to be decoded to can data.
 */
void decode(DataPtr &_data, const boost::array<uint8_t, 8> &_raw_data);

/**
 * @brief Function to encode data into raw can data according to the data's configuration.
 * 
 * The raw data is calculated by \f$(\text{data}+\text{offset})/\text{resolution}\f$, rounded down.
 * @note DOSE NOT check for overfolw.
 * @param[in] _data Pointer to can data where last_data to be encoded to raw can data.
 * @param[out] _raw_data Raw can data modified by last_data in can data.
 */
void encode(const DataPtr &_data, boost::array<uint8_t, 8> &_raw_data);

#endif // CAN_HANDLER_HPP
