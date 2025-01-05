#pragma once

#include "common.h"
#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {

#endif // __cplusplus
struct mip_interface;
struct mip_serializer;
struct mip_field;

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands_c  MIP Commands [C]
///@{
///@defgroup system_commands_c  System Commands [C]
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    MIP_SYSTEM_CMD_DESC_SET                  = 0x7F,
    
    MIP_CMD_DESC_MIPNET_ENUMERATE            = 0x01,
    MIP_CMD_DESC_SYSTEM_COM_MODE             = 0x10,
    MIP_CMD_DESC_SYSTEM_HARDWARE_CONTROL     = 0x11,
    MIP_CMD_DESC_SYSTEM_HARDWARE_CONTROL_2   = 0x12,
    
    MIP_REPLY_DESC_SYSTEM_COM_MODE           = 0x90,
    MIP_REPLY_DESC_SYSTEM_HARDWARE_CONTROL   = 0x91,
    MIP_REPLY_DESC_SYSTEM_HARDWARE_CONTROL_2 = 0x92,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

enum { MIP_SYSTEM_COMMAND_COMM_MODE_PASSTHRU = 0x00 };
enum { MIP_SYSTEM_COMMAND_COMM_MODE_NORMAL = 0x01 };
enum { MIP_SYSTEM_COMMAND_COMM_MODE_IMU = 0x02 };
enum { MIP_SYSTEM_COMMAND_COMM_MODE_GPS = 0x03 };

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup c_system_comm_mode  (0x7F,0x10) Comm Mode [C]
/// Advanced specialized communication modes.
/// 
/// This command allows the user to communicate directly with various subsystems which may be present in MIP devices (i.e. IMU, GNSS, etc.)
/// Please see the specific device's user manual for possible modes.
/// 
/// This command responds with an ACK/NACK just prior to switching to the new protocol.
/// For all functions except 0x01 (use new settings), the new communications mode value is ignored.
/// 
/// 
///
///@{

struct mip_system_comm_mode_command
{
    mip_function_selector function;
    uint8_t mode;
    
};
typedef struct mip_system_comm_mode_command mip_system_comm_mode_command;
void insert_mip_system_comm_mode_command(struct mip_serializer* serializer, const mip_system_comm_mode_command* self);
void extract_mip_system_comm_mode_command(struct mip_serializer* serializer, mip_system_comm_mode_command* self);

struct mip_system_comm_mode_response
{
    uint8_t mode;
    
};
typedef struct mip_system_comm_mode_response mip_system_comm_mode_response;
void insert_mip_system_comm_mode_response(struct mip_serializer* serializer, const mip_system_comm_mode_response* self);
void extract_mip_system_comm_mode_response(struct mip_serializer* serializer, mip_system_comm_mode_response* self);

mip_cmd_result mip_system_write_comm_mode(struct mip_interface* device, uint8_t mode);
mip_cmd_result mip_system_read_comm_mode(struct mip_interface* device, uint8_t* mode_out);
mip_cmd_result mip_system_default_comm_mode(struct mip_interface* device);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
} // namespace C
} // namespace mip
} // extern "C"
#endif // __cplusplus

