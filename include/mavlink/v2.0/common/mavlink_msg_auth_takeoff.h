#pragma once
// MESSAGE AUTH_TAKEOFF PACKING

#define MAVLINK_MSG_ID_AUTH_TAKEOFF 12921


typedef struct __mavlink_auth_takeoff_t {
 uint8_t status; /*<  */
} mavlink_auth_takeoff_t;

#define MAVLINK_MSG_ID_AUTH_TAKEOFF_LEN 1
#define MAVLINK_MSG_ID_AUTH_TAKEOFF_MIN_LEN 1
#define MAVLINK_MSG_ID_12921_LEN 1
#define MAVLINK_MSG_ID_12921_MIN_LEN 1

#define MAVLINK_MSG_ID_AUTH_TAKEOFF_CRC 113
#define MAVLINK_MSG_ID_12921_CRC 113



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AUTH_TAKEOFF { \
    12921, \
    "AUTH_TAKEOFF", \
    1, \
    {  { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_auth_takeoff_t, status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AUTH_TAKEOFF { \
    "AUTH_TAKEOFF", \
    1, \
    {  { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_auth_takeoff_t, status) }, \
         } \
}
#endif

/**
 * @brief Pack a auth_takeoff message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param status  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_auth_takeoff_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AUTH_TAKEOFF_LEN];
    _mav_put_uint8_t(buf, 0, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AUTH_TAKEOFF_LEN);
#else
    mavlink_auth_takeoff_t packet;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AUTH_TAKEOFF_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AUTH_TAKEOFF;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AUTH_TAKEOFF_MIN_LEN, MAVLINK_MSG_ID_AUTH_TAKEOFF_LEN, MAVLINK_MSG_ID_AUTH_TAKEOFF_CRC);
}

/**
 * @brief Pack a auth_takeoff message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param status  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_auth_takeoff_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AUTH_TAKEOFF_LEN];
    _mav_put_uint8_t(buf, 0, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AUTH_TAKEOFF_LEN);
#else
    mavlink_auth_takeoff_t packet;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AUTH_TAKEOFF_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AUTH_TAKEOFF;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AUTH_TAKEOFF_MIN_LEN, MAVLINK_MSG_ID_AUTH_TAKEOFF_LEN, MAVLINK_MSG_ID_AUTH_TAKEOFF_CRC);
}

/**
 * @brief Encode a auth_takeoff struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param auth_takeoff C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_auth_takeoff_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_auth_takeoff_t* auth_takeoff)
{
    return mavlink_msg_auth_takeoff_pack(system_id, component_id, msg, auth_takeoff->status);
}

/**
 * @brief Encode a auth_takeoff struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param auth_takeoff C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_auth_takeoff_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_auth_takeoff_t* auth_takeoff)
{
    return mavlink_msg_auth_takeoff_pack_chan(system_id, component_id, chan, msg, auth_takeoff->status);
}

/**
 * @brief Send a auth_takeoff message
 * @param chan MAVLink channel to send the message
 *
 * @param status  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_auth_takeoff_send(mavlink_channel_t chan, uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AUTH_TAKEOFF_LEN];
    _mav_put_uint8_t(buf, 0, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTH_TAKEOFF, buf, MAVLINK_MSG_ID_AUTH_TAKEOFF_MIN_LEN, MAVLINK_MSG_ID_AUTH_TAKEOFF_LEN, MAVLINK_MSG_ID_AUTH_TAKEOFF_CRC);
#else
    mavlink_auth_takeoff_t packet;
    packet.status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTH_TAKEOFF, (const char *)&packet, MAVLINK_MSG_ID_AUTH_TAKEOFF_MIN_LEN, MAVLINK_MSG_ID_AUTH_TAKEOFF_LEN, MAVLINK_MSG_ID_AUTH_TAKEOFF_CRC);
#endif
}

/**
 * @brief Send a auth_takeoff message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_auth_takeoff_send_struct(mavlink_channel_t chan, const mavlink_auth_takeoff_t* auth_takeoff)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_auth_takeoff_send(chan, auth_takeoff->status);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTH_TAKEOFF, (const char *)auth_takeoff, MAVLINK_MSG_ID_AUTH_TAKEOFF_MIN_LEN, MAVLINK_MSG_ID_AUTH_TAKEOFF_LEN, MAVLINK_MSG_ID_AUTH_TAKEOFF_CRC);
#endif
}

#if MAVLINK_MSG_ID_AUTH_TAKEOFF_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_auth_takeoff_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTH_TAKEOFF, buf, MAVLINK_MSG_ID_AUTH_TAKEOFF_MIN_LEN, MAVLINK_MSG_ID_AUTH_TAKEOFF_LEN, MAVLINK_MSG_ID_AUTH_TAKEOFF_CRC);
#else
    mavlink_auth_takeoff_t *packet = (mavlink_auth_takeoff_t *)msgbuf;
    packet->status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTH_TAKEOFF, (const char *)packet, MAVLINK_MSG_ID_AUTH_TAKEOFF_MIN_LEN, MAVLINK_MSG_ID_AUTH_TAKEOFF_LEN, MAVLINK_MSG_ID_AUTH_TAKEOFF_CRC);
#endif
}
#endif

#endif

// MESSAGE AUTH_TAKEOFF UNPACKING


/**
 * @brief Get field status from auth_takeoff message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_auth_takeoff_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a auth_takeoff message into a struct
 *
 * @param msg The message to decode
 * @param auth_takeoff C-struct to decode the message contents into
 */
static inline void mavlink_msg_auth_takeoff_decode(const mavlink_message_t* msg, mavlink_auth_takeoff_t* auth_takeoff)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    auth_takeoff->status = mavlink_msg_auth_takeoff_get_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AUTH_TAKEOFF_LEN? msg->len : MAVLINK_MSG_ID_AUTH_TAKEOFF_LEN;
        memset(auth_takeoff, 0, MAVLINK_MSG_ID_AUTH_TAKEOFF_LEN);
    memcpy(auth_takeoff, _MAV_PAYLOAD(msg), len);
#endif
}
