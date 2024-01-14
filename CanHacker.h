/*
 * CanHacker.h
 *
 *      Author: Dmitry
 */

#ifndef CANHACKER_H_
#define CANHACKER_H_

#ifndef USE_ESP32_INTERNAL_CAN
#include <can.h>
#include <mcp2515.h>
#else
#include <CAN.h>
#include <stdint.h>
#endif

#define CAN_MIN_DLEN 1
#define HEX_PER_BYTE 2
#define MIN_MESSAGE_DATA_HEX_LENGTH CAN_MIN_DLEN * HEX_PER_BYTE
#define MAX_MESSAGE_DATA_HEX_LENGTH CAN_MAX_DLEN * HEX_PER_BYTE
#define MIN_MESSAGE_LENGTH 5

#define CANHACKER_CMD_MAX_LENGTH 26

#define CANHACKER_SERIAL_RESPONSE     "N0001\r"
#define CANHACKER_SW_VERSION_RESPONSE "v0107\r"
#define CANHACKER_VERSION_RESPONSE    "V1010\r"

#ifdef USE_ESP32_INTERNAL_CAN
#define CAN_MAX_DLEN 8

/* special address description flags for the CAN_ID */
#define CAN_EFF_FLAG 0x80000000UL /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000UL /* remote transmission request */
#define CAN_ERR_FLAG 0x20000000UL /* error message frame */

/* valid bits in CAN ID for frame formats */
#define CAN_SFF_MASK 0x000007FFUL /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFUL /* extended frame format (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFFUL /* omit EFF, RTR, ERR flags */

#define CAN_10KBPS		10E3
#define CAN_20KBPS		20E3
#define CAN_40KBPS		40E3
#define CAN_50KBPS		50E3
#define CAN_80KBPS		80E3
#define CAN_100KBPS		100E3
#define CAN_125KBPS		125E3
#define CAN_200KBPS		200E3
#define CAN_250KBPS		250E3
#define CAN_500KBPS		500E3
#define CAN_1000KBPS	1000E3

typedef unsigned char __u8;
typedef unsigned short __u16;
typedef unsigned long __u32;

typedef __u32 canid_t;
typedef long CAN_SPEED;

struct can_frame {
    canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
    __u8    can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
    __u8    data[CAN_MAX_DLEN] __attribute__((aligned(8)));
};
#endif

class CanHacker {
    public:
        enum ERROR {
            ERROR_OK,
            ERROR_CONNECTED,
            ERROR_NOT_CONNECTED,
            ERROR_UNKNOWN_COMMAND,
            ERROR_INVALID_COMMAND,
            ERROR_ERROR_FRAME_NOT_SUPPORTED,
            ERROR_BUFFER_OVERFLOW,
            ERROR_SERIAL_TX_OVERRUN,
            ERROR_LISTEN_ONLY,
#ifndef USE_ESP32_INTERNAL_CAN
            ERROR_MCP2515_INIT,
            ERROR_MCP2515_INIT_CONFIG,
            ERROR_MCP2515_INIT_BITRATE,
            ERROR_MCP2515_INIT_SET_MODE,
            ERROR_MCP2515_SEND,
            ERROR_MCP2515_READ,
            ERROR_MCP2515_FILTER,
            ERROR_MCP2515_ERRIF,
            ERROR_MCP2515_MERRF
#else
			ERROR_ESP32_CAN_INIT_BITRATE,
			ERROR_ESP32_CAN_SEND
#endif
        };

#ifndef USE_ESP32_INTERNAL_CAN
        CanHacker(Stream *stream, Stream *debugStream, uint8_t cs);
#else
		CanHacker(Stream *stream, Stream *debugStream);
#endif
        ~CanHacker();
#ifndef USE_ESP32_INTERNAL_CAN
        void setClock(const CAN_CLOCK clock);
#endif
        ERROR receiveCommand(const char *buffer, const int length);
        ERROR receiveCanFrame(const struct can_frame *frame);
        ERROR sendFrame(const struct can_frame *);
        ERROR enableLoopback();
        ERROR disableLoopback();
        ERROR pollReceiveCan();
#ifndef USE_ESP32_INTERNAL_CAN
        ERROR receiveCan(const MCP2515::RXBn rxBuffer);
        MCP2515 *getMcp2515();
#else
		ERROR receiveCan();
#endif
        ERROR processInterrupt();
        Stream *getInterfaceStream();

    private:

        static const char CR  = '\r';
        static const char BEL = 7;
        static const uint16_t TIMESTAMP_LIMIT = 0xEA60;
#ifndef USE_ESP32_INTERNAL_CAN
        CAN_CLOCK canClock = MCP_16MHZ;
#endif
        bool _timestampEnabled = false;
        bool _listenOnly = false;
        bool _loopback = false;
        uint8_t _cs;
#ifndef USE_ESP32_INTERNAL_CAN
        MCP2515 *mcp2515;
#endif
        CAN_SPEED bitrate;
        bool _isConnected = false;
        Stream *_stream;
        Stream *_debugStream;

        enum /*class*/ COMMAND : char {
            COMMAND_SET_BITRATE    = 'S', // set CAN bit rate
            COMMAND_SET_BTR        = 's', // set CAN bit rate via
            COMMAND_OPEN_CAN_CHAN  = 'O', // open CAN channel
            COMMAND_CLOSE_CAN_CHAN = 'C', // close CAN channel
            COMMAND_SEND_11BIT_ID  = 't', // send CAN message with 11bit ID
            COMMAND_SEND_29BIT_ID  = 'T', // send CAN message with 29bit ID
            COMMAND_SEND_R11BIT_ID = 'r', // send CAN remote message with 11bit ID
            COMMAND_SEND_R29BIT_ID = 'R', // send CAN remote message with 29bit ID
            COMMAND_READ_STATUS    = 'F', // read status flag byte
            COMMAND_SET_ACR        = 'M', // set Acceptance Code Register
            COMMAND_SET_AMR        = 'm', // set Acceptance Mask Register
            COMMAND_GET_VERSION    = 'V', // get hardware and software version
            COMMAND_GET_SW_VERSION = 'v', // get software version only
            COMMAND_GET_SERIAL     = 'N', // get device serial number
            COMMAND_TIME_STAMP     = 'Z', // toggle time stamp setting
            COMMAND_READ_ECR       = 'E', // read Error Capture Register
            COMMAND_READ_ALCR      = 'A', // read Arbritation Lost Capture Register
            COMMAND_READ_REG       = 'G', // read register conten from SJA1000
            COMMAND_WRITE_REG      = 'W', // write register content to SJA1000
            COMMAND_LISTEN_ONLY    = 'L'  // switch to listen only mode
        };

        ERROR parseTransmit(const char *buffer, int length, struct can_frame *frame);
        ERROR createTransmit(const struct can_frame *frame, char *buffer, const int length);

        uint16_t getTimestamp();
        ERROR setFilter(const uint32_t filter);
        ERROR setFilterMask(const uint32_t mask);

        ERROR connectCan();
        ERROR disconnectCan();
        bool isConnected();
        ERROR writeCan(const struct can_frame *);
        ERROR writeStream(const char character);
        ERROR writeStream(const char *buffer);
        ERROR writeDebugStream(const char character);
        ERROR writeDebugStream(const char *buffer);
        ERROR writeDebugStream(const int buffer);
        ERROR writeDebugStream(const uint8_t *buffer, size_t size);
        ERROR writeDebugStream(const __FlashStringHelper *ifsh);

        ERROR receiveSetBitrateCommand(const char *buffer, const int length);
        ERROR receiveTransmitCommand(const char *buffer, const int length);
        ERROR receiveTimestampCommand(const char *buffer, const int length);
        ERROR receiveCloseCommand(const char *buffer, const int length);
        ERROR receiveOpenCommand(const char *buffer, const int length);
        ERROR receiveListenOnlyCommand(const char *buffer, const int length);
        ERROR receiveSetAcrCommand(const char *buffer, const int length);
        ERROR receiveSetAmrCommand(const char *buffer, const int length);
};

#endif /* CANHACKER_H_ */
