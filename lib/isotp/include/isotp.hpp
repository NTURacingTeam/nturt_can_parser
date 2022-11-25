#ifndef ISOTP_HPP
#define ISOTP_HPP

// std include
#include <assert.h>
#include <string.h>
#include <stdint.h>

// isotp include
#include "isotp_defines.hpp"
#include "isotp_config.hpp"

class IsoTp {
    public:
        /**
         * @brief Constructor.
         * 
         * @param[in] send_id The can frame id for sending isotp data.
         * @param[in] send_buf_size Send buffer size.
         * @param[in] recveive_buf_size Recieve buffer size.
         * @param[in] send_can_fun Function for sending can frame.
         * @param[in] get_milli_fun Function to get milliseconds.
         * @param[in] log_error_fun Function for logging error. Default to not logging.
         */
        IsoTp(uint32_t _send_id, uint16_t _send_buf_size, uint16_t _receive_buf_size, SendCanFun _send_can_fun,
            GetMilliFun _get_milli_fun, LogErrorFun _log_error_fun);

        /// @brief Deconstructor.
        ~IsoTp();

        /**
         * @brief Function that should be called every time to update isotp.
         * 
         * Call this function periodically to handle timeouts, send consecutive frames, etc.
         */
        void update();

        /**
         * @brief Sends isotp frames via can, using the id set in the constructor.
         *
         * Single-frame messages will be sent immediately when calling this function.
         * Multi-frame messages will be sent consecutively when calling update.
         *
         * @param[in] payload The payload to be sent. (Up to 4095 bytes).
         * @param[in] size The size of the payload to be sent.
         * @return Possible return values:
         *  - @code ISOTP_RET_OVERFLOW @endcode
         *  - @code ISOTP_RET_INPROGRESS @endcode
         *  - @code ISOTP_RET_OK @endcode
         *  - The return value of the user @code SendCanFun @endcode.
         */
        int send(const uint8_t *payload, const uint16_t size);

        /// @brief See @link send @endlink, with the exception that this function is used only for functional addressing.
        int send_with_id(const uint32_t id, const uint8_t *payload, const uint16_t size);

        /**
         * @brief Handles incoming isotp can messages.
         *
         * @param[in] data The data received via can.
         * @param[in] len The length of the data received.
         */
        void on_can_message(const uint8_t *data, const uint8_t len);

        /**
         * @brief Receives and parses the received data and copies the parsed data in to the internal buffer.
         * 
         * @param[out] payload A pointer to an area in memory where the raw data is copied from.
         * @param[in] payload_size The size of the received (raw) can data.
         * @param[out] out_size Reference to a variable which will contain the size of the actual (parsed) data.
         * @return Possible return values:
         *  - @code ISOTP_RET_OK @endcode
         *  - @code ISOTP_RET_NO_DATA @endcode
         */
        int receive(uint8_t *payload, const uint16_t payload_size, uint16_t &out_size);

    private:
        // sender paramters
        /// @brief used to reply consecutive frame
        uint32_t send_arbitration_id;

        // message buffer
        uint8_t *send_buffer;

        uint16_t send_buf_size;

        uint16_t send_size = 0;

        uint16_t send_offset = 0;

        // multi-frame flags
        uint8_t send_sn = 0;

        /// @brief Remaining block size.
        uint16_t send_bs_remain = 0;

        /// @brief Separation Time between consecutive frames, unit millis.
        uint8_t send_st_min = 0;

        /// @brief Maximum number of FC.Wait frame transmissions.
        uint8_t send_wtf_count = 0;

        /// @brief Last time send consecutive frame.
        uint32_t send_timer_st = 0;

        /// @brief Time until reception of the next FlowControl N_PDU start at sending FF, CF, receive FC end at receive FC.
        uint32_t send_timer_bs = 0;

        int send_protocol_result = 0;

        uint8_t send_status = ISOTP_RECEIVE_STATUS_IDLE;

        // receiver paramters
        uint32_t receive_arbitration_id = 0;

        // message buffer
        uint8_t *receive_buffer;

        uint16_t receive_buf_size;

        uint16_t receive_size = 0;

        uint16_t receive_offset = 0;

        // multi-frame control
        uint8_t receive_sn = 0;
        
        /// @brief Maximum number of FC.Wait frame transmissions.
        uint8_t receive_bs_count = 0;

        /// @brief Time until transmission of the next ConsecutiveFrame N_PDU start at sending FC, receive CF  end at receive FC.
        uint32_t receive_timer_cr = 0;

        int receive_protocol_result = 0;

        uint8_t receive_status = ISOTP_SEND_STATUS_IDLE;

        // user defined function
        /// @brief Function for sending can frame.
        SendCanFun send_can_fun;

        /// @brief Function for getting milliseconds.
        GetMilliFun get_milli_fun;

        /// @brief Function for logging error.
        LogErrorFun log_error_fun;

        /// @brief Function for converting microsecond to st_min.
        uint8_t ms_to_st_min(uint8_t ms);

        /// @brief Function for converting st_min to microsecond.
        uint8_t st_min_to_ms(uint8_t st_min);

        int send_flow_control(uint8_t flow_status, uint8_t block_size, uint8_t st_min_ms);

        int send_single_frame(uint32_t id);

        int send_first_frame(uint32_t id);

        int send_consecutive_frame();

        int receive_single_frame(IsoTpCanMessage *message, uint8_t len);

        int receive_first_frame(IsoTpCanMessage *message, uint8_t len);

        int receive_consecutive_frame(IsoTpCanMessage *message, uint8_t len);

        int receive_flow_control_frame(IsoTpCanMessage *message, uint8_t len);
};

#endif // ISOTP_HPP
