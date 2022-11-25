#ifndef ISOTP_DEFINES_HPP
#define ISOTP_DEFINES_HPP

// std include
#include <functional>
#include <stdint.h>

// isotp return type
#define ISOTP_RET_OK                        0
#define ISOTP_RET_ERROR                    -1
#define ISOTP_RET_INPROGRESS               -2
#define ISOTP_RET_OVERFLOW                 -3
#define ISOTP_RET_WRONG_SN                 -4
#define ISOTP_RET_NO_DATA                  -5
#define ISOTP_RET_TIMEOUT                  -6
#define ISOTP_RET_LENGTH                   -7

// network layer resault code
#define ISOTP_PROTOCOL_RESULT_OK            0
#define ISOTP_PROTOCOL_RESULT_TIMEOUT_A    -1
#define ISOTP_PROTOCOL_RESULT_TIMEOUT_BS   -2
#define ISOTP_PROTOCOL_RESULT_TIMEOUT_CR   -3
#define ISOTP_PROTOCOL_RESULT_WRONG_SN     -4
#define ISOTP_PROTOCOL_RESULT_INVALID_FS   -5
#define ISOTP_PROTOCOL_RESULT_UNEXP_PDU    -6
#define ISOTP_PROTOCOL_RESULT_WFT_OVRN     -7
#define ISOTP_PROTOCOL_RESULT_BUFFER_OVFLW -8
#define ISOTP_PROTOCOL_RESULT_ERROR        -9

/// @brief Invalid bs.
#define ISOTP_INVALID_BS 0xFFFF

/// @brief Return logic true if 'a' is after 'b'.
#define IsoTpTimeAfter(a,b) ((int32_t)((int32_t)(b) - (int32_t)(a)) < 0)

// type definition
/// @brief Function type definition for sending can frame.
typedef std::function<int(const uint32_t, const uint8_t*, const uint8_t)> SendCanFun;

/// @brief Function type definition for getting milliseconds.
typedef std::function<uint64_t(void)> GetMilliFun;

/// @brief Function type definition for logging error.
typedef std::function<void(const char*)> LogErrorFun;

/// @brief IsoTp sender status.
enum IsoTpSendStatusTypes {
    ISOTP_SEND_STATUS_IDLE,
    ISOTP_SEND_STATUS_INPROGRESS,
    ISOTP_SEND_STATUS_ERROR,
};

/// @brief IsoTp receiver status.
enum IsoTpReceiveStatusTypes{
    ISOTP_RECEIVE_STATUS_IDLE,
    ISOTP_RECEIVE_STATUS_INPROGRESS,
    ISOTP_RECEIVE_STATUS_FULL,
};

/// @brief Protocol Control Information (PCI) types, for identifying each frame of an isotp message.
enum IsoTpProtocolControlInformation {
    ISOTP_PCI_TYPE_SINGLE             = 0x0,
    ISOTP_PCI_TYPE_FIRST_FRAME        = 0x1,
    TSOTP_PCI_TYPE_CONSECUTIVE_FRAME  = 0x2,
    ISOTP_PCI_TYPE_FLOW_CONTROL_FRAME = 0x3
};

/// @brief Protocol Control Information (PCI) flow control identifiers.
enum IsoTpFlowStatus {
    PCI_FLOW_STATUS_CONTINUE = 0x0,
    PCI_FLOW_STATUS_WAIT     = 0x1,
    PCI_FLOW_STATUS_OVERFLOW = 0x2
};

// can frame defination
/// @brief Common frame with only pci information.
struct IsoTpPciType {
    uint8_t reserve_1:4;
    uint8_t type:4;
    uint8_t reserve_2[7];
};

/**
 * @brief Single frame.
 * 
 * +-------------------------+-----+
 * | byte #0                 | ... |
 * +-------------------------+-----+
 * | nibble #0   | nibble #1 | ... |
 * +-------------+-----------+ ... +
 * | PCIType = 0 | SF_DL     | ... |
 * +-------------+-----------+-----+
 */
struct IsoTpSingleFrame {
    uint8_t SF_DL:4;
    uint8_t type:4;
    uint8_t data[7];
};

/**
 * @brief First frame.
 * 
 * +-------------------------+-----------------------+-----+
 * | byte #0                 | byte #1               | ... |
 * +-------------------------+-----------+-----------+-----+
 * | nibble #0   | nibble #1 | nibble #2 | nibble #3 | ... |
 * +-------------+-----------+-----------+-----------+-----+
 * | PCIType = 1 | FF_DL                             | ... |
 * +-------------+-----------+-----------------------+-----+
 */
struct IsoTpFirstFrame {
    uint8_t FF_DL_high:4;
    uint8_t type:4;
    uint8_t FF_DL_low;
    uint8_t data[6];
};

/**
 * @brief Consecutive frame.
 *
 * +-------------------------+-----+
 * | byte #0                 | ... |
 * +-------------------------+-----+
 * | nibble #0   | nibble #1 | ... |
 * +-------------+-----------+ ... +
 * | PCIType = 0 | SN        | ... |
 * +-------------+-----------+-----+
 */
struct IsoTpConsecutiveFrame {
    uint8_t SN:4;
    uint8_t type:4;
    uint8_t data[7];
};

/**
 * @brief Flow control frame.
 * 
 * +-------------------------+-----------------------+-----------------------+-----+
 * | byte #0                 | byte #1               | byte #2               | ... |
 * +-------------------------+-----------+-----------+-----------+-----------+-----+
 * | nibble #0   | nibble #1 | nibble #2 | nibble #3 | nibble #4 | nibble #5 | ... |
 * +-------------+-----------+-----------+-----------+-----------+-----------+-----+
 * | PCIType = 1 | FS        | BS                    | STmin                 | ... |
 * +-------------+-----------+-----------------------+-----------------------+-----+
 */
struct IsoTpFlowControl {
    uint8_t FS:4;
    uint8_t type:4;
    uint8_t BS;
    uint8_t STmin;
    uint8_t reserve[5];
};

/// @brief Frame without specific structure.
struct IsoTpDataArray {
    uint8_t ptr[8];
};

/// @brief Can frame used in isotp. Using union to parse into different types of frame.
struct IsoTpCanMessage {
    union {
        IsoTpPciType common;
        IsoTpSingleFrame single_frame;
        IsoTpFirstFrame first_frame;
        IsoTpConsecutiveFrame consecutive_frame;
        IsoTpFlowControl flow_control;
        IsoTpDataArray data_array;
    } as;
};

#endif // ISOTP_DEFINES_HPP
