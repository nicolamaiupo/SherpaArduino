// MESSAGE COMMANDS PACKING

#define MAVLINK_MSG_ID_COMMANDS 10

typedef struct __mavlink_commands_t
{
 uint32_t TStamp; ///< TStamp
 uint16_t LSX; ///< Left Stick X axis
 uint16_t LSY; ///< LSY
 uint16_t RSX; ///< RSX
 uint16_t RSY; ///< RSY
 uint16_t BSX; ///< BSX
 uint16_t BSY; ///< BSY
 uint16_t Buttons; ///< Buttons
} mavlink_commands_t;

#define MAVLINK_MSG_ID_COMMANDS_LEN 18
#define MAVLINK_MSG_ID_10_LEN 18

#define MAVLINK_MSG_ID_COMMANDS_CRC 152
#define MAVLINK_MSG_ID_10_CRC 152



#define MAVLINK_MESSAGE_INFO_COMMANDS { \
	"COMMANDS", \
	8, \
	{  { "TStamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_commands_t, TStamp) }, \
         { "LSX", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_commands_t, LSX) }, \
         { "LSY", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_commands_t, LSY) }, \
         { "RSX", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_commands_t, RSX) }, \
         { "RSY", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_commands_t, RSY) }, \
         { "BSX", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_commands_t, BSX) }, \
         { "BSY", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_commands_t, BSY) }, \
         { "Buttons", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_commands_t, Buttons) }, \
         } \
}


/**
 * @brief Pack a commands message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param LSX Left Stick X axis
 * @param LSY LSY
 * @param RSX RSX
 * @param RSY RSY
 * @param BSX BSX
 * @param BSY BSY
 * @param Buttons Buttons
 * @param TStamp TStamp
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_commands_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t LSX, uint16_t LSY, uint16_t RSX, uint16_t RSY, uint16_t BSX, uint16_t BSY, uint16_t Buttons, uint32_t TStamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_COMMANDS_LEN];
	_mav_put_uint32_t(buf, 0, TStamp);
	_mav_put_uint16_t(buf, 4, LSX);
	_mav_put_uint16_t(buf, 6, LSY);
	_mav_put_uint16_t(buf, 8, RSX);
	_mav_put_uint16_t(buf, 10, RSY);
	_mav_put_uint16_t(buf, 12, BSX);
	_mav_put_uint16_t(buf, 14, BSY);
	_mav_put_uint16_t(buf, 16, Buttons);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMMANDS_LEN);
#else
	mavlink_commands_t packet;
	packet.TStamp = TStamp;
	packet.LSX = LSX;
	packet.LSY = LSY;
	packet.RSX = RSX;
	packet.RSY = RSY;
	packet.BSX = BSX;
	packet.BSY = BSY;
	packet.Buttons = Buttons;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMANDS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_COMMANDS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_COMMANDS_LEN, MAVLINK_MSG_ID_COMMANDS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_COMMANDS_LEN);
#endif
}

/**
 * @brief Pack a commands message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param LSX Left Stick X axis
 * @param LSY LSY
 * @param RSX RSX
 * @param RSY RSY
 * @param BSX BSX
 * @param BSY BSY
 * @param Buttons Buttons
 * @param TStamp TStamp
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_commands_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t LSX,uint16_t LSY,uint16_t RSX,uint16_t RSY,uint16_t BSX,uint16_t BSY,uint16_t Buttons,uint32_t TStamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_COMMANDS_LEN];
	_mav_put_uint32_t(buf, 0, TStamp);
	_mav_put_uint16_t(buf, 4, LSX);
	_mav_put_uint16_t(buf, 6, LSY);
	_mav_put_uint16_t(buf, 8, RSX);
	_mav_put_uint16_t(buf, 10, RSY);
	_mav_put_uint16_t(buf, 12, BSX);
	_mav_put_uint16_t(buf, 14, BSY);
	_mav_put_uint16_t(buf, 16, Buttons);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMMANDS_LEN);
#else
	mavlink_commands_t packet;
	packet.TStamp = TStamp;
	packet.LSX = LSX;
	packet.LSY = LSY;
	packet.RSX = RSX;
	packet.RSY = RSY;
	packet.BSX = BSX;
	packet.BSY = BSY;
	packet.Buttons = Buttons;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMANDS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_COMMANDS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_COMMANDS_LEN, MAVLINK_MSG_ID_COMMANDS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_COMMANDS_LEN);
#endif
}

/**
 * @brief Encode a commands struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param commands C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_commands_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_commands_t* commands)
{
	return mavlink_msg_commands_pack(system_id, component_id, msg, commands->LSX, commands->LSY, commands->RSX, commands->RSY, commands->BSX, commands->BSY, commands->Buttons, commands->TStamp);
}

/**
 * @brief Encode a commands struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param commands C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_commands_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_commands_t* commands)
{
	return mavlink_msg_commands_pack_chan(system_id, component_id, chan, msg, commands->LSX, commands->LSY, commands->RSX, commands->RSY, commands->BSX, commands->BSY, commands->Buttons, commands->TStamp);
}

/**
 * @brief Send a commands message
 * @param chan MAVLink channel to send the message
 *
 * @param LSX Left Stick X axis
 * @param LSY LSY
 * @param RSX RSX
 * @param RSY RSY
 * @param BSX BSX
 * @param BSY BSY
 * @param Buttons Buttons
 * @param TStamp TStamp
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_commands_send(mavlink_channel_t chan, uint16_t LSX, uint16_t LSY, uint16_t RSX, uint16_t RSY, uint16_t BSX, uint16_t BSY, uint16_t Buttons, uint32_t TStamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_COMMANDS_LEN];
	_mav_put_uint32_t(buf, 0, TStamp);
	_mav_put_uint16_t(buf, 4, LSX);
	_mav_put_uint16_t(buf, 6, LSY);
	_mav_put_uint16_t(buf, 8, RSX);
	_mav_put_uint16_t(buf, 10, RSY);
	_mav_put_uint16_t(buf, 12, BSX);
	_mav_put_uint16_t(buf, 14, BSY);
	_mav_put_uint16_t(buf, 16, Buttons);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMANDS, buf, MAVLINK_MSG_ID_COMMANDS_LEN, MAVLINK_MSG_ID_COMMANDS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMANDS, buf, MAVLINK_MSG_ID_COMMANDS_LEN);
#endif
#else
	mavlink_commands_t packet;
	packet.TStamp = TStamp;
	packet.LSX = LSX;
	packet.LSY = LSY;
	packet.RSX = RSX;
	packet.RSY = RSY;
	packet.BSX = BSX;
	packet.BSY = BSY;
	packet.Buttons = Buttons;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMANDS, (const char *)&packet, MAVLINK_MSG_ID_COMMANDS_LEN, MAVLINK_MSG_ID_COMMANDS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMANDS, (const char *)&packet, MAVLINK_MSG_ID_COMMANDS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_COMMANDS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_commands_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t LSX, uint16_t LSY, uint16_t RSX, uint16_t RSY, uint16_t BSX, uint16_t BSY, uint16_t Buttons, uint32_t TStamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, TStamp);
	_mav_put_uint16_t(buf, 4, LSX);
	_mav_put_uint16_t(buf, 6, LSY);
	_mav_put_uint16_t(buf, 8, RSX);
	_mav_put_uint16_t(buf, 10, RSY);
	_mav_put_uint16_t(buf, 12, BSX);
	_mav_put_uint16_t(buf, 14, BSY);
	_mav_put_uint16_t(buf, 16, Buttons);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMANDS, buf, MAVLINK_MSG_ID_COMMANDS_LEN, MAVLINK_MSG_ID_COMMANDS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMANDS, buf, MAVLINK_MSG_ID_COMMANDS_LEN);
#endif
#else
	mavlink_commands_t *packet = (mavlink_commands_t *)msgbuf;
	packet->TStamp = TStamp;
	packet->LSX = LSX;
	packet->LSY = LSY;
	packet->RSX = RSX;
	packet->RSY = RSY;
	packet->BSX = BSX;
	packet->BSY = BSY;
	packet->Buttons = Buttons;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMANDS, (const char *)packet, MAVLINK_MSG_ID_COMMANDS_LEN, MAVLINK_MSG_ID_COMMANDS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMANDS, (const char *)packet, MAVLINK_MSG_ID_COMMANDS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE COMMANDS UNPACKING


/**
 * @brief Get field LSX from commands message
 *
 * @return Left Stick X axis
 */
static inline uint16_t mavlink_msg_commands_get_LSX(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field LSY from commands message
 *
 * @return LSY
 */
static inline uint16_t mavlink_msg_commands_get_LSY(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field RSX from commands message
 *
 * @return RSX
 */
static inline uint16_t mavlink_msg_commands_get_RSX(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field RSY from commands message
 *
 * @return RSY
 */
static inline uint16_t mavlink_msg_commands_get_RSY(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field BSX from commands message
 *
 * @return BSX
 */
static inline uint16_t mavlink_msg_commands_get_BSX(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field BSY from commands message
 *
 * @return BSY
 */
static inline uint16_t mavlink_msg_commands_get_BSY(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Get field Buttons from commands message
 *
 * @return Buttons
 */
static inline uint16_t mavlink_msg_commands_get_Buttons(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field TStamp from commands message
 *
 * @return TStamp
 */
static inline uint32_t mavlink_msg_commands_get_TStamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a commands message into a struct
 *
 * @param msg The message to decode
 * @param commands C-struct to decode the message contents into
 */
static inline void mavlink_msg_commands_decode(const mavlink_message_t* msg, mavlink_commands_t* commands)
{
#if MAVLINK_NEED_BYTE_SWAP
	commands->TStamp = mavlink_msg_commands_get_TStamp(msg);
	commands->LSX = mavlink_msg_commands_get_LSX(msg);
	commands->LSY = mavlink_msg_commands_get_LSY(msg);
	commands->RSX = mavlink_msg_commands_get_RSX(msg);
	commands->RSY = mavlink_msg_commands_get_RSY(msg);
	commands->BSX = mavlink_msg_commands_get_BSX(msg);
	commands->BSY = mavlink_msg_commands_get_BSY(msg);
	commands->Buttons = mavlink_msg_commands_get_Buttons(msg);
#else
	memcpy(commands, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_COMMANDS_LEN);
#endif
}
