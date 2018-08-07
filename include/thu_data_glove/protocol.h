#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <QtCore/QByteArray>

#define GLOVE_PACKAGE_HEADER	0xA55Au
#define GLOVE_PACKAGE_EOP	0xEDu

#define GLOVE_DATALOAD_INDICATOR_CONTROL	0x22u
#define GLOVE_DATALOAD_INDICATOR_DATA	0x37u
#define GLOVE_DATALOAD_DATATYPE_SEARCH	0x01u
#define GLOVE_DATALOAD_DATATYPE_SEARCH_RET	0x02u
#define GLOVE_DATALOAD_DATATYPE_CONNECT	0x03u
#define GLOVE_DATALOAD_DATATYPE_CONNECT_RET	0x04u
#define GLOVE_DATALOAD_DATATYPE_DISCONNECT	0x09u
#define GLOVE_DATALOAD_DATATYPE_DISCONNECT_RET	0x0Au
#define GLOVE_DATALOAD_DATATYPE_DATA	0x64u

#define GLOVE_UDP_PORT  12378u

// ---- special structures ---------------- //
// 1. 1. 168. 192
typedef union {
	struct {
		uint8_t byte[4];
	};
	uint32_t ip;
} __attribute__((packed)) ip_addr;

typedef struct {
	uint8_t id[12];
} __attribute__((packed)) dev_id;

// ---- package frame defines here -------- //
typedef struct {
	uint16_t header = GLOVE_PACKAGE_HEADER;
	uint16_t length = 0;
	uint8_t dataload[];
	//uint8_t chksum = 0; // chksum of data
	//uint8_t eop = GLOVE_PACKAGE_EOP;
} __attribute__((packed)) package_frame;

// ---- dataload defines here ------------- //
typedef struct {
	uint8_t indicator = GLOVE_DATALOAD_INDICATOR_CONTROL;
	uint8_t data_type = GLOVE_DATALOAD_DATATYPE_SEARCH;
	ip_addr ip; // ip of this computer
	uint16_t udp_port; // port of this computer
} __attribute__((packed)) cmd_glove_search;

typedef struct {
	uint8_t indicator = GLOVE_DATALOAD_INDICATOR_CONTROL;
	uint8_t data_type = GLOVE_DATALOAD_DATATYPE_SEARCH_RET;
	uint16_t dev_type = 0x00;
	uint16_t dev_version = 0x00;
	uint16_t fw_version = 0x00;
	dev_id id;
	uint8_t dev_name_len;
	char name[];
} __attribute__((packed)) response_glove_search;

typedef struct {
	uint8_t indicator = GLOVE_DATALOAD_INDICATOR_CONTROL;
	uint8_t data_type = GLOVE_DATALOAD_DATATYPE_CONNECT;
	dev_id id;
} __attribute__((packed)) cmd_glove_connect;

typedef struct {
	uint8_t indicator = GLOVE_DATALOAD_INDICATOR_CONTROL;
	uint8_t data_type = GLOVE_DATALOAD_DATATYPE_CONNECT_RET;
} __attribute__((packed)) response_glove_connect;

typedef struct {
	uint8_t indicator = GLOVE_DATALOAD_INDICATOR_CONTROL;
	uint8_t data_type = GLOVE_DATALOAD_DATATYPE_DISCONNECT;
} __attribute__((packed)) cmd_glove_disconnect;

typedef struct {
	uint8_t indicator = GLOVE_DATALOAD_INDICATOR_CONTROL;
	uint8_t data_type = GLOVE_DATALOAD_DATATYPE_DISCONNECT_RET;
} __attribute__((packed)) response_glove_disconnect;

typedef struct {
	uint8_t indicator = GLOVE_DATALOAD_INDICATOR_DATA;
	uint8_t data_type = GLOVE_DATALOAD_DATATYPE_DATA;
	int16_t sensor_data[18*12];
	uint32_t time;
} __attribute__((packed)) response_glove_data;

// ---- functions goes here -------------- //

// dataload to package frame
QByteArray pack(const QByteArray &data);
QByteArray pack(const uint8_t *data, size_t len);

// package frame to dataload, will remove unpacked data if completed
QByteArray unpack(QByteArray &data, bool *complete_package=nullptr);

// pack cmd_glove_search
QByteArray cmd_glove_search_packer(ip_addr self_ip);

// pack cmd_glove_connect
QByteArray cmd_glove_connect_packer(dev_id id);

// pack cmd_glove_disconnect
QByteArray cmd_glove_disconnect_packer();

#endif // PROTOCOL_H
