#pragma once

#include "common.h"
#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

namespace mip {
class Serializer;

namespace C {
struct mip_interface;
} // namespace C

namespace commands_system {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands_cpp  MIP Commands [CPP]
///@{
///@defgroup system_commands_cpp  System Commands [CPP]
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    DESCRIPTOR_SET           = 0x7F,
    
    CMD_ENUMERATE            = 0x01,
    CMD_COM_MODE             = 0x10,
    CMD_HARDWARE_CONTROL     = 0x11,
    CMD_HARDWARE_CONTROL_2   = 0x12,
    
    REPLY_COM_MODE           = 0x90,
    REPLY_HARDWARE_CONTROL   = 0x91,
    REPLY_HARDWARE_CONTROL_2 = 0x92,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

static constexpr const uint8_t MIP_SYSTEM_COMMAND_COMM_MODE_PASSTHRU = 0x00;
static constexpr const uint8_t MIP_SYSTEM_COMMAND_COMM_MODE_NORMAL = 0x01;
static constexpr const uint8_t MIP_SYSTEM_COMMAND_COMM_MODE_IMU = 0x02;
static constexpr const uint8_t MIP_SYSTEM_COMMAND_COMM_MODE_GPS = 0x03;

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_system_comm_mode  (0x7F,0x10) Comm Mode [CPP]
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

struct CommMode
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t mode = 0;
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_system::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_system::CMD_COM_MODE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "CommMode";
    static constexpr const char* DOC_NAME = "CommMode";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8001;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x0000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x0000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(mode);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(mode));
    }
    
    static CommMode create_sld_all(::mip::FunctionSelector function)
    {
        CommMode cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_system::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_system::REPLY_COM_MODE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "CommMode::Response";
        static constexpr const char* DOC_NAME = "CommMode Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        uint8_t mode = 0;
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(mode));
        }
    };
};
void insert(Serializer& serializer, const CommMode& self);
void extract(Serializer& serializer, CommMode& self);

void insert(Serializer& serializer, const CommMode::Response& self);
void extract(Serializer& serializer, CommMode::Response& self);

TypedResult<CommMode> writeCommMode(C::mip_interface& device, uint8_t mode);
TypedResult<CommMode> readCommMode(C::mip_interface& device, uint8_t* modeOut);
TypedResult<CommMode> defaultCommMode(C::mip_interface& device);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace commands_system
} // namespace mip

