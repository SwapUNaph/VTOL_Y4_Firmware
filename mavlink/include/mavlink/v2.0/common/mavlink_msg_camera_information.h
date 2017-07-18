#pragma once
// MESSAGE CAMERA_INFORMATION PACKING

#define MAVLINK_MSG_ID_CAMERA_INFORMATION 259

MAVPACKED(
typedef struct __mavlink_camera_information_t {
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 uint32_t firmware_version; /*< Version of the camera firmware*/
 float focal_length; /*< Focal length in mm*/
 float sensor_size_h; /*< Image sensor size horizontal in mm*/
 float sensor_size_v; /*< Image sensor size vertical in mm*/
 uint16_t resolution_h; /*< Image resolution in pixels horizontal*/
 uint16_t resolution_v; /*< Image resolution in pixels vertical*/
 uint8_t camera_id; /*< Camera ID (1 for first, 2 for second, etc.)*/
 uint8_t camera_count; /*< Number of cameras*/
 uint8_t vendor_name[32]; /*< Name of the camera vendor*/
 uint8_t model_name[32]; /*< Name of the camera model*/
 uint8_t lens_id; /*< Reserved for a lens ID*/
}) mavlink_camera_information_t;

#define MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN 91
#define MAVLINK_MSG_ID_CAMERA_INFORMATION_MIN_LEN 91
#define MAVLINK_MSG_ID_259_LEN 91
#define MAVLINK_MSG_ID_259_MIN_LEN 91

#define MAVLINK_MSG_ID_CAMERA_INFORMATION_CRC 49
#define MAVLINK_MSG_ID_259_CRC 49

#define MAVLINK_MSG_CAMERA_INFORMATION_FIELD_VENDOR_NAME_LEN 32
#define MAVLINK_MSG_CAMERA_INFORMATION_FIELD_MODEL_NAME_LEN 32

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CAMERA_INFORMATION { \
    259, \
    "CAMERA_INFORMATION", \
    12, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_camera_information_t, time_boot_ms) }, \
         { "firmware_version", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_camera_information_t, firmware_version) }, \
         { "focal_length", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_camera_information_t, focal_length) }, \
         { "sensor_size_h", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_camera_information_t, sensor_size_h) }, \
         { "sensor_size_v", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_camera_information_t, sensor_size_v) }, \
         { "resolution_h", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_camera_information_t, resolution_h) }, \
         { "resolution_v", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_camera_information_t, resolution_v) }, \
         { "camera_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_camera_information_t, camera_id) }, \
         { "camera_count", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_camera_information_t, camera_count) }, \
         { "vendor_name", NULL, MAVLINK_TYPE_UINT8_T, 32, 26, offsetof(mavlink_camera_information_t, vendor_name) }, \
         { "model_name", NULL, MAVLINK_TYPE_UINT8_T, 32, 58, offsetof(mavlink_camera_information_t, model_name) }, \
         { "lens_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 90, offsetof(mavlink_camera_information_t, lens_id) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CAMERA_INFORMATION { \
    "CAMERA_INFORMATION", \
    12, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_camera_information_t, time_boot_ms) }, \
         { "firmware_version", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_camera_information_t, firmware_version) }, \
         { "focal_length", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_camera_information_t, focal_length) }, \
         { "sensor_size_h", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_camera_information_t, sensor_size_h) }, \
         { "sensor_size_v", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_camera_information_t, sensor_size_v) }, \
         { "resolution_h", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_camera_information_t, resolution_h) }, \
         { "resolution_v", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_camera_information_t, resolution_v) }, \
         { "camera_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_camera_information_t, camera_id) }, \
         { "camera_count", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_camera_information_t, camera_count) }, \
         { "vendor_name", NULL, MAVLINK_TYPE_UINT8_T, 32, 26, offsetof(mavlink_camera_information_t, vendor_name) }, \
         { "model_name", NULL, MAVLINK_TYPE_UINT8_T, 32, 58, offsetof(mavlink_camera_information_t, model_name) }, \
         { "lens_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 90, offsetof(mavlink_camera_information_t, lens_id) }, \
         } \
}
#endif

/**
 * @brief Pack a camera_information message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param camera_id Camera ID (1 for first, 2 for second, etc.)
 * @param camera_count Number of cameras
 * @param vendor_name Name of the camera vendor
 * @param model_name Name of the camera model
 * @param firmware_version Version of the camera firmware
 * @param focal_length Focal length in mm
 * @param sensor_size_h Image sensor size horizontal in mm
 * @param sensor_size_v Image sensor size vertical in mm
 * @param resolution_h Image resolution in pixels horizontal
 * @param resolution_v Image resolution in pixels vertical
 * @param lens_id Reserved for a lens ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_information_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t camera_id, uint8_t camera_count, const uint8_t *vendor_name, const uint8_t *model_name, uint32_t firmware_version, float focal_length, float sensor_size_h, float sensor_size_v, uint16_t resolution_h, uint16_t resolution_v, uint8_t lens_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, firmware_version);
    _mav_put_float(buf, 8, focal_length);
    _mav_put_float(buf, 12, sensor_size_h);
    _mav_put_float(buf, 16, sensor_size_v);
    _mav_put_uint16_t(buf, 20, resolution_h);
    _mav_put_uint16_t(buf, 22, resolution_v);
    _mav_put_uint8_t(buf, 24, camera_id);
    _mav_put_uint8_t(buf, 25, camera_count);
    _mav_put_uint8_t(buf, 90, lens_id);
    _mav_put_uint8_t_array(buf, 26, vendor_name, 32);
    _mav_put_uint8_t_array(buf, 58, model_name, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN);
#else
    mavlink_camera_information_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.firmware_version = firmware_version;
    packet.focal_length = focal_length;
    packet.sensor_size_h = sensor_size_h;
    packet.sensor_size_v = sensor_size_v;
    packet.resolution_h = resolution_h;
    packet.resolution_v = resolution_v;
    packet.camera_id = camera_id;
    packet.camera_count = camera_count;
    packet.lens_id = lens_id;
    mav_array_memcpy(packet.vendor_name, vendor_name, sizeof(uint8_t)*32);
    mav_array_memcpy(packet.model_name, model_name, sizeof(uint8_t)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_INFORMATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAMERA_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_CRC);
}

/**
 * @brief Pack a camera_information message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param camera_id Camera ID (1 for first, 2 for second, etc.)
 * @param camera_count Number of cameras
 * @param vendor_name Name of the camera vendor
 * @param model_name Name of the camera model
 * @param firmware_version Version of the camera firmware
 * @param focal_length Focal length in mm
 * @param sensor_size_h Image sensor size horizontal in mm
 * @param sensor_size_v Image sensor size vertical in mm
 * @param resolution_h Image resolution in pixels horizontal
 * @param resolution_v Image resolution in pixels vertical
 * @param lens_id Reserved for a lens ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_information_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint8_t camera_id,uint8_t camera_count,const uint8_t *vendor_name,const uint8_t *model_name,uint32_t firmware_version,float focal_length,float sensor_size_h,float sensor_size_v,uint16_t resolution_h,uint16_t resolution_v,uint8_t lens_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, firmware_version);
    _mav_put_float(buf, 8, focal_length);
    _mav_put_float(buf, 12, sensor_size_h);
    _mav_put_float(buf, 16, sensor_size_v);
    _mav_put_uint16_t(buf, 20, resolution_h);
    _mav_put_uint16_t(buf, 22, resolution_v);
    _mav_put_uint8_t(buf, 24, camera_id);
    _mav_put_uint8_t(buf, 25, camera_count);
    _mav_put_uint8_t(buf, 90, lens_id);
    _mav_put_uint8_t_array(buf, 26, vendor_name, 32);
    _mav_put_uint8_t_array(buf, 58, model_name, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN);
#else
    mavlink_camera_information_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.firmware_version = firmware_version;
    packet.focal_length = focal_length;
    packet.sensor_size_h = sensor_size_h;
    packet.sensor_size_v = sensor_size_v;
    packet.resolution_h = resolution_h;
    packet.resolution_v = resolution_v;
    packet.camera_id = camera_id;
    packet.camera_count = camera_count;
    packet.lens_id = lens_id;
    mav_array_memcpy(packet.vendor_name, vendor_name, sizeof(uint8_t)*32);
    mav_array_memcpy(packet.model_name, model_name, sizeof(uint8_t)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_INFORMATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAMERA_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_CRC);
}

/**
 * @brief Encode a camera_information struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param camera_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_information_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_camera_information_t* camera_information)
{
    return mavlink_msg_camera_information_pack(system_id, component_id, msg, camera_information->time_boot_ms, camera_information->camera_id, camera_information->camera_count, camera_information->vendor_name, camera_information->model_name, camera_information->firmware_version, camera_information->focal_length, camera_information->sensor_size_h, camera_information->sensor_size_v, camera_information->resolution_h, camera_information->resolution_v, camera_information->lens_id);
}

/**
 * @brief Encode a camera_information struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param camera_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_information_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_camera_information_t* camera_information)
{
    return mavlink_msg_camera_information_pack_chan(system_id, component_id, chan, msg, camera_information->time_boot_ms, camera_information->camera_id, camera_information->camera_count, camera_information->vendor_name, camera_information->model_name, camera_information->firmware_version, camera_information->focal_length, camera_information->sensor_size_h, camera_information->sensor_size_v, camera_information->resolution_h, camera_information->resolution_v, camera_information->lens_id);
}

/**
 * @brief Send a camera_information message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param camera_id Camera ID (1 for first, 2 for second, etc.)
 * @param camera_count Number of cameras
 * @param vendor_name Name of the camera vendor
 * @param model_name Name of the camera model
 * @param firmware_version Version of the camera firmware
 * @param focal_length Focal length in mm
 * @param sensor_size_h Image sensor size horizontal in mm
 * @param sensor_size_v Image sensor size vertical in mm
 * @param resolution_h Image resolution in pixels horizontal
 * @param resolution_v Image resolution in pixels vertical
 * @param lens_id Reserved for a lens ID
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_camera_information_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t camera_id, uint8_t camera_count, const uint8_t *vendor_name, const uint8_t *model_name, uint32_t firmware_version, float focal_length, float sensor_size_h, float sensor_size_v, uint16_t resolution_h, uint16_t resolution_v, uint8_t lens_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, firmware_version);
    _mav_put_float(buf, 8, focal_length);
    _mav_put_float(buf, 12, sensor_size_h);
    _mav_put_float(buf, 16, sensor_size_v);
    _mav_put_uint16_t(buf, 20, resolution_h);
    _mav_put_uint16_t(buf, 22, resolution_v);
    _mav_put_uint8_t(buf, 24, camera_id);
    _mav_put_uint8_t(buf, 25, camera_count);
    _mav_put_uint8_t(buf, 90, lens_id);
    _mav_put_uint8_t_array(buf, 26, vendor_name, 32);
    _mav_put_uint8_t_array(buf, 58, model_name, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_INFORMATION, buf, MAVLINK_MSG_ID_CAMERA_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_CRC);
#else
    mavlink_camera_information_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.firmware_version = firmware_version;
    packet.focal_length = focal_length;
    packet.sensor_size_h = sensor_size_h;
    packet.sensor_size_v = sensor_size_v;
    packet.resolution_h = resolution_h;
    packet.resolution_v = resolution_v;
    packet.camera_id = camera_id;
    packet.camera_count = camera_count;
    packet.lens_id = lens_id;
    mav_array_memcpy(packet.vendor_name, vendor_name, sizeof(uint8_t)*32);
    mav_array_memcpy(packet.model_name, model_name, sizeof(uint8_t)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_INFORMATION, (const char *)&packet, MAVLINK_MSG_ID_CAMERA_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_CRC);
#endif
}

/**
 * @brief Send a camera_information message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_camera_information_send_struct(mavlink_channel_t chan, const mavlink_camera_information_t* camera_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_camera_information_send(chan, camera_information->time_boot_ms, camera_information->camera_id, camera_information->camera_count, camera_information->vendor_name, camera_information->model_name, camera_information->firmware_version, camera_information->focal_length, camera_information->sensor_size_h, camera_information->sensor_size_v, camera_information->resolution_h, camera_information->resolution_v, camera_information->lens_id);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_INFORMATION, (const char *)camera_information, MAVLINK_MSG_ID_CAMERA_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_camera_information_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t camera_id, uint8_t camera_count, const uint8_t *vendor_name, const uint8_t *model_name, uint32_t firmware_version, float focal_length, float sensor_size_h, float sensor_size_v, uint16_t resolution_h, uint16_t resolution_v, uint8_t lens_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, firmware_version);
    _mav_put_float(buf, 8, focal_length);
    _mav_put_float(buf, 12, sensor_size_h);
    _mav_put_float(buf, 16, sensor_size_v);
    _mav_put_uint16_t(buf, 20, resolution_h);
    _mav_put_uint16_t(buf, 22, resolution_v);
    _mav_put_uint8_t(buf, 24, camera_id);
    _mav_put_uint8_t(buf, 25, camera_count);
    _mav_put_uint8_t(buf, 90, lens_id);
    _mav_put_uint8_t_array(buf, 26, vendor_name, 32);
    _mav_put_uint8_t_array(buf, 58, model_name, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_INFORMATION, buf, MAVLINK_MSG_ID_CAMERA_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_CRC);
#else
    mavlink_camera_information_t *packet = (mavlink_camera_information_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->firmware_version = firmware_version;
    packet->focal_length = focal_length;
    packet->sensor_size_h = sensor_size_h;
    packet->sensor_size_v = sensor_size_v;
    packet->resolution_h = resolution_h;
    packet->resolution_v = resolution_v;
    packet->camera_id = camera_id;
    packet->camera_count = camera_count;
    packet->lens_id = lens_id;
    mav_array_memcpy(packet->vendor_name, vendor_name, sizeof(uint8_t)*32);
    mav_array_memcpy(packet->model_name, model_name, sizeof(uint8_t)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_INFORMATION, (const char *)packet, MAVLINK_MSG_ID_CAMERA_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN, MAVLINK_MSG_ID_CAMERA_INFORMATION_CRC);
#endif
}
#endif

#endif

// MESSAGE CAMERA_INFORMATION UNPACKING


/**
 * @brief Get field time_boot_ms from camera_information message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_camera_information_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field camera_id from camera_information message
 *
 * @return Camera ID (1 for first, 2 for second, etc.)
 */
static inline uint8_t mavlink_msg_camera_information_get_camera_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field camera_count from camera_information message
 *
 * @return Number of cameras
 */
static inline uint8_t mavlink_msg_camera_information_get_camera_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field vendor_name from camera_information message
 *
 * @return Name of the camera vendor
 */
static inline uint16_t mavlink_msg_camera_information_get_vendor_name(const mavlink_message_t* msg, uint8_t *vendor_name)
{
    return _MAV_RETURN_uint8_t_array(msg, vendor_name, 32,  26);
}

/**
 * @brief Get field model_name from camera_information message
 *
 * @return Name of the camera model
 */
static inline uint16_t mavlink_msg_camera_information_get_model_name(const mavlink_message_t* msg, uint8_t *model_name)
{
    return _MAV_RETURN_uint8_t_array(msg, model_name, 32,  58);
}

/**
 * @brief Get field firmware_version from camera_information message
 *
 * @return Version of the camera firmware
 */
static inline uint32_t mavlink_msg_camera_information_get_firmware_version(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field focal_length from camera_information message
 *
 * @return Focal length in mm
 */
static inline float mavlink_msg_camera_information_get_focal_length(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field sensor_size_h from camera_information message
 *
 * @return Image sensor size horizontal in mm
 */
static inline float mavlink_msg_camera_information_get_sensor_size_h(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field sensor_size_v from camera_information message
 *
 * @return Image sensor size vertical in mm
 */
static inline float mavlink_msg_camera_information_get_sensor_size_v(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field resolution_h from camera_information message
 *
 * @return Image resolution in pixels horizontal
 */
static inline uint16_t mavlink_msg_camera_information_get_resolution_h(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field resolution_v from camera_information message
 *
 * @return Image resolution in pixels vertical
 */
static inline uint16_t mavlink_msg_camera_information_get_resolution_v(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Get field lens_id from camera_information message
 *
 * @return Reserved for a lens ID
 */
static inline uint8_t mavlink_msg_camera_information_get_lens_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  90);
}

/**
 * @brief Decode a camera_information message into a struct
 *
 * @param msg The message to decode
 * @param camera_information C-struct to decode the message contents into
 */
static inline void mavlink_msg_camera_information_decode(const mavlink_message_t* msg, mavlink_camera_information_t* camera_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    camera_information->time_boot_ms = mavlink_msg_camera_information_get_time_boot_ms(msg);
    camera_information->firmware_version = mavlink_msg_camera_information_get_firmware_version(msg);
    camera_information->focal_length = mavlink_msg_camera_information_get_focal_length(msg);
    camera_information->sensor_size_h = mavlink_msg_camera_information_get_sensor_size_h(msg);
    camera_information->sensor_size_v = mavlink_msg_camera_information_get_sensor_size_v(msg);
    camera_information->resolution_h = mavlink_msg_camera_information_get_resolution_h(msg);
    camera_information->resolution_v = mavlink_msg_camera_information_get_resolution_v(msg);
    camera_information->camera_id = mavlink_msg_camera_information_get_camera_id(msg);
    camera_information->camera_count = mavlink_msg_camera_information_get_camera_count(msg);
    mavlink_msg_camera_information_get_vendor_name(msg, camera_information->vendor_name);
    mavlink_msg_camera_information_get_model_name(msg, camera_information->model_name);
    camera_information->lens_id = mavlink_msg_camera_information_get_lens_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN? msg->len : MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN;
        memset(camera_information, 0, MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN);
    memcpy(camera_information, _MAV_PAYLOAD(msg), len);
#endif
}
