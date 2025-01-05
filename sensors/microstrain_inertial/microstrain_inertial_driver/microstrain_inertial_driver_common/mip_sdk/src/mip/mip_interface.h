#pragma once

#include <stdint.h>
#include <stddef.h>

#include "mip_parser.h"
#include "mip_cmdqueue.h"
#include "mip_dispatch.h"

#ifdef __cplusplus
namespace mip{
namespace C {
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_c
///@{
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_interface_c  Mip Interface [C]
///
///@brief High-level C functions for controlling a MIP device.
///
/// This module contains functions and classes for communicating with a
/// MIP device in C.
///
///
///@li Sending commands
///@li Receiving Data
///
///@{

struct mip_interface;

// Documentation is in source file.
typedef bool (*mip_send_callback)(struct mip_interface* device, const uint8_t* data, size_t length);
typedef bool (*mip_recv_callback)(struct mip_interface* device, uint8_t* buffer, size_t max_length, mip_timeout wait_time, size_t* length_out, mip_timestamp* timestamp_out);
typedef bool (*mip_update_callback)(struct mip_interface* device, mip_timeout timeout);


////////////////////////////////////////////////////////////////////////////////
///@brief State of the interface for communicating with a MIP device.
///
typedef struct mip_interface
{
    mip_parser          _parser;          ///<@private MIP Parser for incoming MIP packets.
    mip_cmd_queue       _queue;           ///<@private Queue for checking command replies.
    mip_dispatcher      _dispatcher;      ///<@private Dispatcher for data callbacks.
    unsigned int        _max_update_pkts; ///<@private Max number of MIP packets to parse at once.
    mip_send_callback   _send_callback;   ///<@private Optional function which is called to send raw bytes to the device.
    mip_recv_callback   _recv_callback;   ///<@private Optional function which is called to receive raw bytes from the device.
    mip_update_callback _update_callback; ///<@private Optional function to call during updates.
    void*               _user_pointer;    ///<@private Optional user-specified data pointer.
} mip_interface;


void mip_interface_init(
    mip_interface* device, uint8_t* parse_buffer, size_t parse_buffer_size,
    mip_timeout parse_timeout, mip_timeout base_reply_timeout,
    mip_send_callback send, mip_recv_callback recv,
    mip_update_callback update, void* user_pointer
);

//
// Communications
//

bool mip_interface_send_to_device(mip_interface* device, const uint8_t* data, size_t length);
bool mip_interface_recv_from_device(mip_interface* device, uint8_t* buffer, size_t max_length, mip_timeout timeout, size_t* length_out, mip_timestamp* now);
bool mip_interface_update(mip_interface* device, mip_timeout wait_time);

bool mip_interface_default_update(mip_interface* device, mip_timeout wait_time);
size_t mip_interface_receive_bytes(mip_interface* device, const uint8_t* data, size_t length, mip_timestamp timestamp);
void mip_interface_process_unparsed_packets(mip_interface* device);
bool mip_interface_parse_callback(void* device, const mip_packet* packet, mip_timestamp timestamp);
void mip_interface_receive_packet(mip_interface* device, const mip_packet* packet, mip_timestamp timestamp);

//
// Commands
//

enum mip_cmd_result mip_interface_wait_for_reply(mip_interface* device, mip_pending_cmd* cmd);
enum mip_cmd_result mip_interface_run_command(mip_interface* device, uint8_t descriptor_set, uint8_t field_descriptor, const uint8_t* payload, uint8_t payload_length);
enum mip_cmd_result mip_interface_run_command_with_response(mip_interface* device, uint8_t descriptor_set, uint8_t field_descriptor, const uint8_t* payload, uint8_t payload_length, uint8_t response_descriptor, uint8_t* response_data, uint8_t* response_length_inout);
enum mip_cmd_result mip_interface_run_command_packet(mip_interface* device, const mip_packet* packet, mip_pending_cmd* cmd);

bool mip_interface_start_command_packet(mip_interface* device, const mip_packet* packet, mip_pending_cmd* cmd);

//
// Data Callbacks
//

void mip_interface_register_packet_callback(mip_interface* device, mip_dispatch_handler* handler, uint8_t descriptor_set, bool after_fields, mip_dispatch_packet_callback callback, void* user_data);
void mip_interface_register_field_callback(mip_interface* device, mip_dispatch_handler* handler, uint8_t descriptor_set, uint8_t field_descriptor, mip_dispatch_field_callback callback, void* user_data);
void mip_interface_register_extractor(mip_interface* device, mip_dispatch_handler* handler, uint8_t descriptor_set, uint8_t field_descriptor, mip_dispatch_extractor callback, void* field_ptr);

//
// Accessors
//

void mip_interface_set_recv_function(mip_interface* device, mip_recv_callback function);
void mip_interface_set_send_function(mip_interface* device, mip_send_callback function);
void mip_interface_set_update_function(mip_interface* device, mip_update_callback function);
void mip_interface_set_user_pointer(mip_interface* device, void* pointer);

void mip_interface_set_max_packets_per_update(mip_interface* device, unsigned int max_packets);
unsigned int mip_interface_max_packets_per_update(const mip_interface* device);

mip_recv_callback   mip_interface_recv_function(const mip_interface* device);
mip_send_callback   mip_interface_send_function(const mip_interface* device);
mip_update_callback mip_interface_update_function(const mip_interface* device);
void*               mip_interface_user_pointer(const mip_interface* device);

mip_parser*    mip_interface_parser(mip_interface* device);
mip_cmd_queue* mip_interface_cmd_queue(mip_interface* device);

///@}
///@}

#ifdef __cplusplus
} // namespace mip
} // namespace C
} // extern "C"
#endif
