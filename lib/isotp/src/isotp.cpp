#include "isotp.hpp"

IsoTp::IsoTp(uint32_t _send_id, uint16_t _send_buf_size, uint16_t _receive_buf_size, SendCanFun _send_can_fun,
    GetMilliFun _get_milli_fun, LogErrorFun _log_error_fun = [](const char*) {}) :
    send_arbitration_id(_send_id), send_buf_size(_send_buf_size), receive_buf_size(_receive_buf_size),
    log_error_fun(_log_error_fun), send_can_fun(_send_can_fun), get_milli_fun(_get_milli_fun),
    send_buffer(new uint8_t(_send_buf_size)), receive_buffer(new uint8_t(_receive_buf_size)) {
    
}

IsoTp::~IsoTp() {
    delete send_buffer;
    delete receive_buffer;
}

void IsoTp::update() {
    int ret;

    /* only polling when operation in progress */
    if (ISOTP_SEND_STATUS_INPROGRESS == send_status) {

        /* continue send data */
        if (/* send data if bs_remain is invalid or bs_remain large than zero */
        (ISOTP_INVALID_BS == send_bs_remain || send_bs_remain > 0) &&
        /* and if st_min is zero or go beyond interval time */
        (0 == send_st_min || (0 != send_st_min && IsoTpTimeAfter(get_milli_fun(), send_timer_st)))) {
            ret = send_consecutive_frame();
            if (ISOTP_RET_OK == ret) {
                if (ISOTP_INVALID_BS != send_bs_remain) {
                    send_bs_remain -= 1;
                }
                send_timer_bs = get_milli_fun() + ISO_TP_DEFAULT_RESPONSE_TIMEOUT;
                send_timer_st = get_milli_fun() + send_st_min;

                /* check if send finish */
                if (send_offset >= send_size) {
                    send_status = ISOTP_SEND_STATUS_IDLE;
                }
            } else {
                send_status = ISOTP_SEND_STATUS_ERROR;
            }
        }

        /* check timeout */
        if (IsoTpTimeAfter(get_milli_fun(), send_timer_bs)) {
            send_protocol_result = ISOTP_PROTOCOL_RESULT_TIMEOUT_BS;
            send_status = ISOTP_SEND_STATUS_ERROR;
        }
    }

    /* only polling when operation in progress */
    if (ISOTP_RECEIVE_STATUS_INPROGRESS == receive_status) {
        
        /* check timeout */
        if (IsoTpTimeAfter(get_milli_fun(), receive_timer_cr)) {
            receive_protocol_result = ISOTP_PROTOCOL_RESULT_TIMEOUT_CR;
            receive_status = ISOTP_RECEIVE_STATUS_IDLE;
        }
    }
}

int IsoTp::send(const uint8_t *payload, const uint16_t size) {
    return send_with_id(send_arbitration_id, payload, size);
}

int IsoTp::send_with_id(const uint32_t id, const uint8_t *payload, const uint16_t size) {
    int ret;

    if (size > send_buf_size) {
        log_error_fun("Message size too large. Increase ISO_TP_MAX_MESSAGE_SIZE to set a larger buffer\n");
        return ISOTP_RET_OVERFLOW;
    }

    if (ISOTP_SEND_STATUS_INPROGRESS == send_status) {
        log_error_fun("Abort previous message, transmission in progress.\n");
        return ISOTP_RET_INPROGRESS;
    }

    /* copy into local buffer */
    send_size = size;
    send_offset = 0;
    (void) memcpy(send_buffer, payload, size);

    if (send_size < 8) {
        /* send single frame */
        ret = send_single_frame(id);
    } else {
        /* send multi-frame */
        ret = send_first_frame(id);

        /* init multi-frame control flags */
        if (ISOTP_RET_OK == ret) {
            send_bs_remain = 0;
            send_st_min = 0;
            send_wtf_count = 0;
            send_timer_st = get_milli_fun();
            send_timer_bs = get_milli_fun() + ISO_TP_DEFAULT_RESPONSE_TIMEOUT;
            send_protocol_result = ISOTP_PROTOCOL_RESULT_OK;
            send_status = ISOTP_SEND_STATUS_INPROGRESS;
        }
    }

    return ret;
}

void IsoTp::on_can_message(const uint8_t *data, const uint8_t len) {
    IsoTpCanMessage message;
    int ret;
    
    if (len < 2 || len > 8) {
        return;
    }

    memcpy(message.as.data_array.ptr, data, len);
    memset(message.as.data_array.ptr + len, 0, sizeof(message.as.data_array.ptr) - len);

    switch (message.as.common.type) {
        case ISOTP_PCI_TYPE_SINGLE: {
            /* update protocol result */
            if (ISOTP_RECEIVE_STATUS_INPROGRESS == receive_status) {
                receive_protocol_result = ISOTP_PROTOCOL_RESULT_UNEXP_PDU;
            } else {
                receive_protocol_result = ISOTP_PROTOCOL_RESULT_OK;
            }

            /* handle message */
            ret = receive_single_frame(&message, len);
            
            if (ISOTP_RET_OK == ret) {
                /* change status */
                receive_status = ISOTP_RECEIVE_STATUS_FULL;
            }
            break;
        }
        case ISOTP_PCI_TYPE_FIRST_FRAME: {
            /* update protocol result */
            if (ISOTP_RECEIVE_STATUS_INPROGRESS == receive_status) {
                receive_protocol_result = ISOTP_PROTOCOL_RESULT_UNEXP_PDU;
            } else {
                receive_protocol_result = ISOTP_PROTOCOL_RESULT_OK;
            }

            /* handle message */
            ret = receive_first_frame(&message, len);

            /* if overflow happened */
            if (ISOTP_RET_OVERFLOW == ret) {
                /* update protocol result */
                receive_protocol_result = ISOTP_PROTOCOL_RESULT_BUFFER_OVFLW;
                /* change status */
                receive_status = ISOTP_RECEIVE_STATUS_IDLE;
                /* send error message */
                send_flow_control(PCI_FLOW_STATUS_OVERFLOW, 0, 0);
                break;
            }

            /* if receive successful */
            if (ISOTP_RET_OK == ret) {
                /* change status */
                receive_status = ISOTP_RECEIVE_STATUS_INPROGRESS;
                /* send fc frame */
                receive_bs_count = ISO_TP_DEFAULT_BLOCK_SIZE;
                send_flow_control(PCI_FLOW_STATUS_CONTINUE, receive_bs_count, ISO_TP_DEFAULT_ST_MIN);
                /* refresh timer cs */
                receive_timer_cr = get_milli_fun() + ISO_TP_DEFAULT_RESPONSE_TIMEOUT;
            }
            
            break;
        }
        case TSOTP_PCI_TYPE_CONSECUTIVE_FRAME: {
            /* check if in receiving status */
            if (ISOTP_RECEIVE_STATUS_INPROGRESS != receive_status) {
                receive_protocol_result = ISOTP_PROTOCOL_RESULT_UNEXP_PDU;
                break;
            }

            /* handle message */
            ret = receive_consecutive_frame(&message, len);

            /* if wrong sn */
            if (ISOTP_RET_WRONG_SN == ret) {
                receive_protocol_result = ISOTP_PROTOCOL_RESULT_WRONG_SN;
                receive_status = ISOTP_RECEIVE_STATUS_IDLE;
                break;
            }

            /* if success */
            if (ISOTP_RET_OK == ret) {
                /* refresh timer cs */
                receive_timer_cr = get_milli_fun() + ISO_TP_DEFAULT_RESPONSE_TIMEOUT;
                
                /* receive finished */
                if (receive_offset >= receive_size) {
                    receive_status = ISOTP_RECEIVE_STATUS_FULL;
                } else {
                    /* send fc when bs reaches limit */
                    if (0 == --receive_bs_count) {
                        receive_bs_count = ISO_TP_DEFAULT_BLOCK_SIZE;
                        send_flow_control(PCI_FLOW_STATUS_CONTINUE, receive_bs_count, ISO_TP_DEFAULT_ST_MIN);
                    }
                }
            }
            
            break;
        }
        case ISOTP_PCI_TYPE_FLOW_CONTROL_FRAME:
            /* handle fc frame only when sending in progress  */
            if (ISOTP_SEND_STATUS_INPROGRESS != send_status) {
                break;
            }

            /* handle message */
            ret = receive_flow_control_frame(&message, len);
            
            if (ISOTP_RET_OK == ret) {
                /* refresh bs timer */
                send_timer_bs = get_milli_fun() + ISO_TP_DEFAULT_RESPONSE_TIMEOUT;

                /* overflow */
                if (PCI_FLOW_STATUS_OVERFLOW == message.as.flow_control.FS) {
                    send_protocol_result = ISOTP_PROTOCOL_RESULT_BUFFER_OVFLW;
                    send_status = ISOTP_SEND_STATUS_ERROR;
                }

                /* wait */
                else if (PCI_FLOW_STATUS_WAIT == message.as.flow_control.FS) {
                    send_wtf_count += 1;
                    /* wait exceed allowed count */
                    if (send_wtf_count > ISO_TP_MAX_WFT_NUMBER) {
                        send_protocol_result = ISOTP_PROTOCOL_RESULT_WFT_OVRN;
                        send_status = ISOTP_SEND_STATUS_ERROR;
                    }
                }

                /* permit send */
                else if (PCI_FLOW_STATUS_CONTINUE == message.as.flow_control.FS) {
                    if (0 == message.as.flow_control.BS) {
                        send_bs_remain = ISOTP_INVALID_BS;
                    } else {
                        send_bs_remain = message.as.flow_control.BS;
                    }
                    send_st_min = st_min_to_ms(message.as.flow_control.STmin);
                    send_wtf_count = 0;
                }
            }
            break;
        default:
            break;
    };
}

int IsoTp::receive(uint8_t *payload, const uint16_t payload_size, uint16_t &out_size) {
    uint16_t copylen;
    
    if (ISOTP_RECEIVE_STATUS_FULL != receive_status) {
        return ISOTP_RET_NO_DATA;
    }

    copylen = receive_size;
    if (copylen > payload_size) {
        copylen = payload_size;
    }

    memcpy(payload, receive_buffer, copylen);
    out_size = copylen;

    receive_status = ISOTP_RECEIVE_STATUS_IDLE;

    return ISOTP_RET_OK;
}

uint8_t IsoTp::ms_to_st_min(uint8_t ms) {
    uint8_t st_min;

    st_min = ms;
    if (st_min > 0x7F) {
        st_min = 0x7F;
    }

    return st_min;
}

uint8_t IsoTp::st_min_to_ms(uint8_t st_min) {
    uint8_t ms;
    
    if (st_min >= 0xF1 && st_min <= 0xF9) {
        ms = 1;
    } else if (st_min <= 0x7F) {
        ms = st_min;
    } else {
        ms = 0;
    }

    return ms;
}

int IsoTp::send_flow_control(uint8_t flow_status, uint8_t block_size, uint8_t st_min_ms) {

    IsoTpCanMessage message;
    int ret;

    /* setup message  */
    message.as.flow_control.type = ISOTP_PCI_TYPE_FLOW_CONTROL_FRAME;
    message.as.flow_control.FS = flow_status;
    message.as.flow_control.BS = block_size;
    message.as.flow_control.STmin = ms_to_st_min(st_min_ms);

    /* send message */
#ifdef ISO_TP_FRAME_PADDING
    (void) memset(message.as.flow_control.reserve, 0, sizeof(message.as.flow_control.reserve));
    ret = send_can_fun(send_arbitration_id, message.as.data_array.ptr, sizeof(message));
#else    
    ret = send_can_fun(send_arbitration_id, message.as.data_array.ptr, 3);
#endif

    return ret;
}

int IsoTp::send_single_frame(uint32_t id) {

    IsoTpCanMessage message;
    int ret;

    /* multi frame message length must greater than 7  */
    assert(send_size <= 7);

    /* setup message  */
    message.as.single_frame.type = ISOTP_PCI_TYPE_SINGLE;
    message.as.single_frame.SF_DL = (uint8_t) send_size;
    (void) memcpy(message.as.single_frame.data, send_buffer, send_size);

    /* send message */
#ifdef ISO_TP_FRAME_PADDING
    (void) memset(message.as.single_frame.data + send_size, 0, sizeof(message.as.single_frame.data) - send_size);
    ret = send_can_fun(id, message.as.data_array.ptr, sizeof(message));
#else
    ret = send_can_fun(id, message.as.data_array.ptr, send_size + 1);
#endif

    return ret;
}

int IsoTp::send_first_frame(uint32_t id) {
    
    IsoTpCanMessage message;
    int ret;

    /* multi frame message length must greater than 7  */
    assert(send_size > 7);

    /* setup message  */
    message.as.first_frame.type = ISOTP_PCI_TYPE_FIRST_FRAME;
    message.as.first_frame.FF_DL_low = (uint8_t) send_size;
    message.as.first_frame.FF_DL_high = (uint8_t) (0x0F & (send_size >> 8));
    (void) memcpy(message.as.first_frame.data, send_buffer, sizeof(message.as.first_frame.data));

    /* send message */
    ret = send_can_fun(id, message.as.data_array.ptr, sizeof(message));
    if (ISOTP_RET_OK == ret) {
        send_offset += sizeof(message.as.first_frame.data);
        send_sn = 1;
    }

    return ret;
}

int IsoTp::send_consecutive_frame() {
    
    IsoTpCanMessage message;
    uint16_t data_length;
    int ret;

    /* multi frame message length must greater than 7  */
    assert(send_size > 7);

    /* setup message  */
    message.as.consecutive_frame.type = TSOTP_PCI_TYPE_CONSECUTIVE_FRAME;
    message.as.consecutive_frame.SN = send_sn;
    data_length = send_size - send_offset;
    if (data_length > sizeof(message.as.consecutive_frame.data)) {
        data_length = sizeof(message.as.consecutive_frame.data);
    }
    (void) memcpy(message.as.consecutive_frame.data, send_buffer + send_offset, data_length);

    /* send message */
#ifdef ISO_TP_FRAME_PADDING
    (void) memset(message.as.consecutive_frame.data + data_length, 0, sizeof(message.as.consecutive_frame.data) - data_length);
    ret = send_can_fun(send_arbitration_id, message.as.data_array.ptr, sizeof(message));
#else
    ret = send_can_fun(send_arbitration_id,
            message.as.data_array.ptr,
            data_length + 1);
#endif
    if (ISOTP_RET_OK == ret) {
        send_offset += data_length;
        if (++(send_sn) > 0x0F) {
            send_sn = 0;
        }
    }
    
    return ret;
}

int IsoTp::receive_single_frame(IsoTpCanMessage *message, uint8_t len) {
    /* check data length */
    if ((0 == message->as.single_frame.SF_DL) || (message->as.single_frame.SF_DL > (len - 1))) {
        log_error_fun("Single-frame length too small.");
        return ISOTP_RET_LENGTH;
    }

    /* copying data */
    (void) memcpy(receive_buffer, message->as.single_frame.data, message->as.single_frame.SF_DL);
    receive_size = message->as.single_frame.SF_DL;
    
    return ISOTP_RET_OK;
}

int IsoTp::receive_first_frame(IsoTpCanMessage *message, uint8_t len) {
    uint16_t payload_length;

    if (8 != len) {
        log_error_fun("First frame should be 8 bytes in length.");
        return ISOTP_RET_LENGTH;
    }

    /* check data length */
    payload_length = message->as.first_frame.FF_DL_high;
    payload_length = (payload_length << 8) + message->as.first_frame.FF_DL_low;

    /* should not use multiple frame transmition */
    if (payload_length <= 7) {
        log_error_fun("Should not use multiple frame transmission.");
        return ISOTP_RET_LENGTH;
    }
    
    if (payload_length > receive_buf_size) {
        log_error_fun("Multi-frame response too large for receiving buffer.");
        return ISOTP_RET_OVERFLOW;
    }
    
    /* copying data */
    (void) memcpy(receive_buffer, message->as.first_frame.data, sizeof(message->as.first_frame.data));
    receive_size = payload_length;
    receive_offset = sizeof(message->as.first_frame.data);
    receive_sn = 1;

    return ISOTP_RET_OK;
}

int IsoTp::receive_consecutive_frame(IsoTpCanMessage *message, uint8_t len) {
    uint16_t remaining_bytes;
    
    /* check sn */
    if (receive_sn != message->as.consecutive_frame.SN) {
        return ISOTP_RET_WRONG_SN;
    }

    /* check data length */
    remaining_bytes = receive_size - receive_offset;
    if (remaining_bytes > sizeof(message->as.consecutive_frame.data)) {
        remaining_bytes = sizeof(message->as.consecutive_frame.data);
    }
    if (remaining_bytes > len - 1) {
        log_error_fun("Consecutive frame too short.");
        return ISOTP_RET_LENGTH;
    }

    /* copying data */
    (void) memcpy(receive_buffer + receive_offset, message->as.consecutive_frame.data, remaining_bytes);

    receive_offset += remaining_bytes;
    if (++(receive_sn) > 0x0F) {
        receive_sn = 0;
    }

    return ISOTP_RET_OK;
}

int IsoTp::receive_flow_control_frame(IsoTpCanMessage *message, uint8_t len) {
    /* check message length */
    if (len < 3) {
        log_error_fun("Flow control frame too short.");
        return ISOTP_RET_LENGTH;
    }

    return ISOTP_RET_OK;
}
