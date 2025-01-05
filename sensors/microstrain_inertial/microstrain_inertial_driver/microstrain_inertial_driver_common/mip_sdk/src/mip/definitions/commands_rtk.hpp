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

namespace commands_rtk {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands_cpp  MIP Commands [CPP]
///@{
///@defgroup rtk_commands_cpp  Rtk Commands [CPP]
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    DESCRIPTOR_SET                   = 0x0F,
    
    CMD_GET_STATUS_FLAGS             = 0x01,
    CMD_GET_IMEI                     = 0x02,
    CMD_GET_IMSI                     = 0x03,
    CMD_GET_ICCID                    = 0x04,
    CMD_GET_RSSI                     = 0x05,
    CMD_CONNECTED_DEVICE_TYPE        = 0x06,
    CMD_GET_ACT_CODE                 = 0x07,
    CMD_GET_MODEM_FIRMWARE_VERSION   = 0x08,
    CMD_SERVICE_STATUS               = 0x0A,
    CMD_PROD_ERASE_STORAGE           = 0x20,
    CMD_CONTROL                      = 0x21,
    CMD_MODEM_HARD_RESET             = 0x22,
    
    REPLY_GET_STATUS_FLAGS           = 0x81,
    REPLY_GET_IMEI                   = 0x82,
    REPLY_GET_IMSI                   = 0x83,
    REPLY_GET_ICCID                  = 0x84,
    REPLY_CONNECTED_DEVICE_TYPE      = 0x86,
    REPLY_GET_ACT_CODE               = 0x87,
    REPLY_GET_MODEM_FIRMWARE_VERSION = 0x88,
    REPLY_GET_RSSI                   = 0x85,
    REPLY_SERVICE_STATUS             = 0x8A,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

enum class MediaSelector : uint8_t
{
    MEDIA_EXTERNALFLASH = 0,  ///<  
    MEDIA_SD            = 1,  ///<  
};

enum class LedAction : uint8_t
{
    LED_NONE    = 0,  ///<  
    LED_FLASH   = 1,  ///<  
    LED_PULSATE = 2,  ///<  
};


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_get_status_flags  (0x0F,0x01) Get Status Flags [CPP]
///
///@{

struct GetStatusFlags
{
    struct StatusFlagsLegacy : Bitfield<StatusFlagsLegacy>
    {
        enum _enumType : uint32_t
        {
            NONE                 = 0x00000000,
            CONTROLLERSTATE      = 0x00000007,  ///<  
            PLATFORMSTATE        = 0x000000F8,  ///<  
            CONTROLLERSTATUSCODE = 0x00000700,  ///<  
            PLATFORMSTATUSCODE   = 0x00003800,  ///<  
            RESETCODE            = 0x0000C000,  ///<  
            SIGNALQUALITY        = 0x000F0000,  ///<  
            RESERVED             = 0xFFF00000,  ///<  
            RSSI                 = 0x03F00000,  ///<  
            RSRP                 = 0x0C000000,  ///<  
            RSRQ                 = 0x30000000,  ///<  
            SINR                 = 0xC0000000,  ///<  
            ALL                  = 0xFFFFFFFF,
        };
        uint32_t value = NONE;
        
        StatusFlagsLegacy() : value(NONE) {}
        StatusFlagsLegacy(int val) : value((uint32_t)val) {}
        operator uint32_t() const { return value; }
        StatusFlagsLegacy& operator=(uint32_t val) { value = val; return *this; }
        StatusFlagsLegacy& operator=(int val) { value = uint32_t(val); return *this; }
        StatusFlagsLegacy& operator|=(uint32_t val) { return *this = value | val; }
        StatusFlagsLegacy& operator&=(uint32_t val) { return *this = value & val; }
        
        uint32_t controllerstate() const { return (value & CONTROLLERSTATE) >> 0; }
        void controllerstate(uint32_t val) { value = (value & ~CONTROLLERSTATE) | (val << 0); }
        uint32_t platformstate() const { return (value & PLATFORMSTATE) >> 3; }
        void platformstate(uint32_t val) { value = (value & ~PLATFORMSTATE) | (val << 3); }
        uint32_t controllerstatuscode() const { return (value & CONTROLLERSTATUSCODE) >> 8; }
        void controllerstatuscode(uint32_t val) { value = (value & ~CONTROLLERSTATUSCODE) | (val << 8); }
        uint32_t platformstatuscode() const { return (value & PLATFORMSTATUSCODE) >> 11; }
        void platformstatuscode(uint32_t val) { value = (value & ~PLATFORMSTATUSCODE) | (val << 11); }
        uint32_t resetcode() const { return (value & RESETCODE) >> 14; }
        void resetcode(uint32_t val) { value = (value & ~RESETCODE) | (val << 14); }
        uint32_t signalquality() const { return (value & SIGNALQUALITY) >> 16; }
        void signalquality(uint32_t val) { value = (value & ~SIGNALQUALITY) | (val << 16); }
        uint32_t reserved() const { return (value & RESERVED) >> 20; }
        void reserved(uint32_t val) { value = (value & ~RESERVED) | (val << 20); }
        uint32_t rssi() const { return (value & RSSI) >> 20; }
        void rssi(uint32_t val) { value = (value & ~RSSI) | (val << 20); }
        uint32_t rsrp() const { return (value & RSRP) >> 26; }
        void rsrp(uint32_t val) { value = (value & ~RSRP) | (val << 26); }
        uint32_t rsrq() const { return (value & RSRQ) >> 28; }
        void rsrq(uint32_t val) { value = (value & ~RSRQ) | (val << 28); }
        uint32_t sinr() const { return (value & SINR) >> 30; }
        void sinr(uint32_t val) { value = (value & ~SINR) | (val << 30); }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    
    struct StatusFlags : Bitfield<StatusFlags>
    {
        enum _enumType : uint32_t
        {
            NONE                    = 0x00000000,
            MODEM_STATE             = 0x0000000F,  ///<  
            CONNECTION_TYPE         = 0x000000F0,  ///<  
            RSSI                    = 0x0000FF00,  ///<  
            SIGNAL_QUALITY          = 0x000F0000,  ///<  
            TOWER_CHANGE_INDICATOR  = 0x00F00000,  ///<  
            NMEA_TIMEOUT            = 0x01000000,  ///<  
            SERVER_TIMEOUT          = 0x02000000,  ///<  
            CORRECTIONS_TIMEOUT     = 0x04000000,  ///<  
            DEVICE_OUT_OF_RANGE     = 0x08000000,  ///<  
            CORRECTIONS_UNAVAILABLE = 0x10000000,  ///<  
            RESERVED                = 0x20000000,  ///<  
            VERSION                 = 0xC0000000,  ///<  
            ALL                     = 0xFFFFFFFF,
        };
        uint32_t value = NONE;
        
        StatusFlags() : value(NONE) {}
        StatusFlags(int val) : value((uint32_t)val) {}
        operator uint32_t() const { return value; }
        StatusFlags& operator=(uint32_t val) { value = val; return *this; }
        StatusFlags& operator=(int val) { value = uint32_t(val); return *this; }
        StatusFlags& operator|=(uint32_t val) { return *this = value | val; }
        StatusFlags& operator&=(uint32_t val) { return *this = value & val; }
        
        uint32_t modemState() const { return (value & MODEM_STATE) >> 0; }
        void modemState(uint32_t val) { value = (value & ~MODEM_STATE) | (val << 0); }
        uint32_t connectionType() const { return (value & CONNECTION_TYPE) >> 4; }
        void connectionType(uint32_t val) { value = (value & ~CONNECTION_TYPE) | (val << 4); }
        uint32_t rssi() const { return (value & RSSI) >> 8; }
        void rssi(uint32_t val) { value = (value & ~RSSI) | (val << 8); }
        uint32_t signalQuality() const { return (value & SIGNAL_QUALITY) >> 16; }
        void signalQuality(uint32_t val) { value = (value & ~SIGNAL_QUALITY) | (val << 16); }
        uint32_t towerChangeIndicator() const { return (value & TOWER_CHANGE_INDICATOR) >> 20; }
        void towerChangeIndicator(uint32_t val) { value = (value & ~TOWER_CHANGE_INDICATOR) | (val << 20); }
        bool nmeaTimeout() const { return (value & NMEA_TIMEOUT) > 0; }
        void nmeaTimeout(bool val) { if(val) value |= NMEA_TIMEOUT; else value &= ~NMEA_TIMEOUT; }
        bool serverTimeout() const { return (value & SERVER_TIMEOUT) > 0; }
        void serverTimeout(bool val) { if(val) value |= SERVER_TIMEOUT; else value &= ~SERVER_TIMEOUT; }
        bool correctionsTimeout() const { return (value & CORRECTIONS_TIMEOUT) > 0; }
        void correctionsTimeout(bool val) { if(val) value |= CORRECTIONS_TIMEOUT; else value &= ~CORRECTIONS_TIMEOUT; }
        bool deviceOutOfRange() const { return (value & DEVICE_OUT_OF_RANGE) > 0; }
        void deviceOutOfRange(bool val) { if(val) value |= DEVICE_OUT_OF_RANGE; else value &= ~DEVICE_OUT_OF_RANGE; }
        bool correctionsUnavailable() const { return (value & CORRECTIONS_UNAVAILABLE) > 0; }
        void correctionsUnavailable(bool val) { if(val) value |= CORRECTIONS_UNAVAILABLE; else value &= ~CORRECTIONS_UNAVAILABLE; }
        bool reserved() const { return (value & RESERVED) > 0; }
        void reserved(bool val) { if(val) value |= RESERVED; else value &= ~RESERVED; }
        uint32_t version() const { return (value & VERSION) >> 30; }
        void version(uint32_t val) { value = (value & ~VERSION) | (val << 30); }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_GET_STATUS_FLAGS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GetStatusFlags";
    static constexpr const char* DOC_NAME = "Get RTK Device Status Flags";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple();
    }
    
    auto as_tuple()
    {
        return std::make_tuple();
    }
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_GET_STATUS_FLAGS;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GetStatusFlags::Response";
        static constexpr const char* DOC_NAME = "Get RTK Device Status Flags Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        StatusFlags flags; ///< Model number dependent. See above structures.
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(flags));
        }
    };
};
void insert(Serializer& serializer, const GetStatusFlags& self);
void extract(Serializer& serializer, GetStatusFlags& self);

void insert(Serializer& serializer, const GetStatusFlags::Response& self);
void extract(Serializer& serializer, GetStatusFlags::Response& self);

TypedResult<GetStatusFlags> getStatusFlags(C::mip_interface& device, GetStatusFlags::StatusFlags* flagsOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_get_imei  (0x0F,0x02) Get Imei [CPP]
///
///@{

struct GetImei
{
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_GET_IMEI;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GetImei";
    static constexpr const char* DOC_NAME = "Get RTK Device IMEI (International Mobile Equipment Identifier)";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple();
    }
    
    auto as_tuple()
    {
        return std::make_tuple();
    }
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_GET_IMEI;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GetImei::Response";
        static constexpr const char* DOC_NAME = "Get RTK Device IMEI (International Mobile Equipment Identifier) Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        char IMEI[32] = {0};
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(IMEI));
        }
    };
};
void insert(Serializer& serializer, const GetImei& self);
void extract(Serializer& serializer, GetImei& self);

void insert(Serializer& serializer, const GetImei::Response& self);
void extract(Serializer& serializer, GetImei::Response& self);

TypedResult<GetImei> getImei(C::mip_interface& device, char* imeiOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_get_imsi  (0x0F,0x03) Get Imsi [CPP]
///
///@{

struct GetImsi
{
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_GET_IMSI;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GetImsi";
    static constexpr const char* DOC_NAME = "Get RTK Device IMSI (International Mobile Subscriber Identifier)";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple();
    }
    
    auto as_tuple()
    {
        return std::make_tuple();
    }
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_GET_IMSI;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GetImsi::Response";
        static constexpr const char* DOC_NAME = "Get RTK Device IMSI (International Mobile Subscriber Identifier) Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        char IMSI[32] = {0};
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(IMSI));
        }
    };
};
void insert(Serializer& serializer, const GetImsi& self);
void extract(Serializer& serializer, GetImsi& self);

void insert(Serializer& serializer, const GetImsi::Response& self);
void extract(Serializer& serializer, GetImsi::Response& self);

TypedResult<GetImsi> getImsi(C::mip_interface& device, char* imsiOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_get_iccid  (0x0F,0x04) Get Iccid [CPP]
///
///@{

struct GetIccid
{
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_GET_ICCID;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GetIccid";
    static constexpr const char* DOC_NAME = "Get RTK Device ICCID (Integrated Circuit Card Identification [SIM Number])";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple();
    }
    
    auto as_tuple()
    {
        return std::make_tuple();
    }
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_GET_ICCID;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GetIccid::Response";
        static constexpr const char* DOC_NAME = "Get RTK Device ICCID (Integrated Circuit Card Identification [SIM Number]) Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        char ICCID[32] = {0};
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(ICCID));
        }
    };
};
void insert(Serializer& serializer, const GetIccid& self);
void extract(Serializer& serializer, GetIccid& self);

void insert(Serializer& serializer, const GetIccid::Response& self);
void extract(Serializer& serializer, GetIccid::Response& self);

TypedResult<GetIccid> getIccid(C::mip_interface& device, char* iccidOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_connected_device_type  (0x0F,0x06) Connected Device Type [CPP]
///
///@{

struct ConnectedDeviceType
{
    enum class Type : uint8_t
    {
        GENERIC = 0,  ///<  
        GQ7     = 1,  ///<  
    };
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Type devType = static_cast<Type>(0);
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_CONNECTED_DEVICE_TYPE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ConnectedDeviceType";
    static constexpr const char* DOC_NAME = "Configure or read the type of the connected device";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8001;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(devType);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(devType));
    }
    
    static ConnectedDeviceType create_sld_all(::mip::FunctionSelector function)
    {
        ConnectedDeviceType cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_CONNECTED_DEVICE_TYPE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "ConnectedDeviceType::Response";
        static constexpr const char* DOC_NAME = "Configure or read the type of the connected device Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        Type devType = static_cast<Type>(0);
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(devType));
        }
    };
};
void insert(Serializer& serializer, const ConnectedDeviceType& self);
void extract(Serializer& serializer, ConnectedDeviceType& self);

void insert(Serializer& serializer, const ConnectedDeviceType::Response& self);
void extract(Serializer& serializer, ConnectedDeviceType::Response& self);

TypedResult<ConnectedDeviceType> writeConnectedDeviceType(C::mip_interface& device, ConnectedDeviceType::Type devtype);
TypedResult<ConnectedDeviceType> readConnectedDeviceType(C::mip_interface& device, ConnectedDeviceType::Type* devtypeOut);
TypedResult<ConnectedDeviceType> saveConnectedDeviceType(C::mip_interface& device);
TypedResult<ConnectedDeviceType> loadConnectedDeviceType(C::mip_interface& device);
TypedResult<ConnectedDeviceType> defaultConnectedDeviceType(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_get_act_code  (0x0F,0x07) Get Act Code [CPP]
///
///@{

struct GetActCode
{
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_GET_ACT_CODE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GetActCode";
    static constexpr const char* DOC_NAME = "Get RTK Device Activation Code";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple();
    }
    
    auto as_tuple()
    {
        return std::make_tuple();
    }
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_GET_ACT_CODE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GetActCode::Response";
        static constexpr const char* DOC_NAME = "Get RTK Device Activation Code Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        char ActivationCode[32] = {0};
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(ActivationCode));
        }
    };
};
void insert(Serializer& serializer, const GetActCode& self);
void extract(Serializer& serializer, GetActCode& self);

void insert(Serializer& serializer, const GetActCode::Response& self);
void extract(Serializer& serializer, GetActCode::Response& self);

TypedResult<GetActCode> getActCode(C::mip_interface& device, char* activationcodeOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_get_modem_firmware_version  (0x0F,0x08) Get Modem Firmware Version [CPP]
///
///@{

struct GetModemFirmwareVersion
{
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_GET_MODEM_FIRMWARE_VERSION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GetModemFirmwareVersion";
    static constexpr const char* DOC_NAME = "Get RTK Device's Cell Modem Firmware version number";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple();
    }
    
    auto as_tuple()
    {
        return std::make_tuple();
    }
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_GET_MODEM_FIRMWARE_VERSION;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GetModemFirmwareVersion::Response";
        static constexpr const char* DOC_NAME = "Get RTK Device's Cell Modem Firmware version number Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        char ModemFirmwareVersion[32] = {0};
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(ModemFirmwareVersion));
        }
    };
};
void insert(Serializer& serializer, const GetModemFirmwareVersion& self);
void extract(Serializer& serializer, GetModemFirmwareVersion& self);

void insert(Serializer& serializer, const GetModemFirmwareVersion::Response& self);
void extract(Serializer& serializer, GetModemFirmwareVersion::Response& self);

TypedResult<GetModemFirmwareVersion> getModemFirmwareVersion(C::mip_interface& device, char* modemfirmwareversionOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_get_rssi  (0x0F,0x05) Get Rssi [CPP]
/// Get the RSSI and connected/disconnected status of modem
///
///@{

struct GetRssi
{
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_GET_RSSI;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GetRssi";
    static constexpr const char* DOC_NAME = "GetRssi";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple();
    }
    
    auto as_tuple()
    {
        return std::make_tuple();
    }
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_GET_RSSI;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GetRssi::Response";
        static constexpr const char* DOC_NAME = "GetRssi Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        bool valid = 0;
        int32_t rssi = 0;
        int32_t signalQuality = 0;
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(valid),std::ref(rssi),std::ref(signalQuality));
        }
    };
};
void insert(Serializer& serializer, const GetRssi& self);
void extract(Serializer& serializer, GetRssi& self);

void insert(Serializer& serializer, const GetRssi::Response& self);
void extract(Serializer& serializer, GetRssi::Response& self);

TypedResult<GetRssi> getRssi(C::mip_interface& device, bool* validOut, int32_t* rssiOut, int32_t* signalqualityOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_service_status  (0x0F,0x0A) Service Status [CPP]
/// The 3DMRTK will send this message to the server to indicate that the connection should remain open. The Server will respond with information and status.
///
///@{

struct ServiceStatus
{
    struct ServiceFlags : Bitfield<ServiceFlags>
    {
        enum _enumType : uint8_t
        {
            NONE                    = 0x00,
            THROTTLE                = 0x01,  ///<  
            CORRECTIONS_UNAVAILABLE = 0x02,  ///<  
            RESERVED                = 0xFC,  ///<  
            ALL                     = 0xFF,
        };
        uint8_t value = NONE;
        
        ServiceFlags() : value(NONE) {}
        ServiceFlags(int val) : value((uint8_t)val) {}
        operator uint8_t() const { return value; }
        ServiceFlags& operator=(uint8_t val) { value = val; return *this; }
        ServiceFlags& operator=(int val) { value = uint8_t(val); return *this; }
        ServiceFlags& operator|=(uint8_t val) { return *this = value | val; }
        ServiceFlags& operator&=(uint8_t val) { return *this = value & val; }
        
        bool throttle() const { return (value & THROTTLE) > 0; }
        void throttle(bool val) { if(val) value |= THROTTLE; else value &= ~THROTTLE; }
        bool correctionsUnavailable() const { return (value & CORRECTIONS_UNAVAILABLE) > 0; }
        void correctionsUnavailable(bool val) { if(val) value |= CORRECTIONS_UNAVAILABLE; else value &= ~CORRECTIONS_UNAVAILABLE; }
        uint8_t reserved() const { return (value & RESERVED) >> 2; }
        void reserved(uint8_t val) { value = (value & ~RESERVED) | (val << 2); }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    
    uint32_t reserved1 = 0;
    uint32_t reserved2 = 0;
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_SERVICE_STATUS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ServiceStatus";
    static constexpr const char* DOC_NAME = "ServiceStatus";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(reserved1,reserved2);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(reserved1),std::ref(reserved2));
    }
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::REPLY_SERVICE_STATUS;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "ServiceStatus::Response";
        static constexpr const char* DOC_NAME = "ServiceStatus Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        ServiceFlags flags;
        uint32_t receivedBytes = 0;
        uint32_t lastBytes = 0;
        uint64_t lastBytesTime = 0;
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(flags),std::ref(receivedBytes),std::ref(lastBytes),std::ref(lastBytesTime));
        }
    };
};
void insert(Serializer& serializer, const ServiceStatus& self);
void extract(Serializer& serializer, ServiceStatus& self);

void insert(Serializer& serializer, const ServiceStatus::Response& self);
void extract(Serializer& serializer, ServiceStatus::Response& self);

TypedResult<ServiceStatus> serviceStatus(C::mip_interface& device, uint32_t reserved1, uint32_t reserved2, ServiceStatus::ServiceFlags* flagsOut, uint32_t* receivedbytesOut, uint32_t* lastbytesOut, uint64_t* lastbytestimeOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_prod_erase_storage  (0x0F,0x20) Prod Erase Storage [CPP]
/// This command will erase the selected media to a raw and uninitialized state. ALL DATA WILL BE LOST.
/// This command is only available in calibration mode.
///
///@{

struct ProdEraseStorage
{
    MediaSelector media = static_cast<MediaSelector>(0);
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_PROD_ERASE_STORAGE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ProdEraseStorage";
    static constexpr const char* DOC_NAME = "ProdEraseStorage";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(media);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(media));
    }
    typedef void Response;
};
void insert(Serializer& serializer, const ProdEraseStorage& self);
void extract(Serializer& serializer, ProdEraseStorage& self);

TypedResult<ProdEraseStorage> prodEraseStorage(C::mip_interface& device, MediaSelector media);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_led_control  (0x0F,0x21) Led Control [CPP]
/// This command allows direct control of the LED on the 3DM RTK. This command is only available in calibration mode or Production Test Mode.
///
///@{

struct LedControl
{
    uint8_t primaryColor[3] = {0};
    uint8_t altColor[3] = {0};
    LedAction act = static_cast<LedAction>(0);
    uint32_t period = 0;
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_CONTROL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "LedControl";
    static constexpr const char* DOC_NAME = "LedControl";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(primaryColor,altColor,act,period);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(primaryColor),std::ref(altColor),std::ref(act),std::ref(period));
    }
    typedef void Response;
};
void insert(Serializer& serializer, const LedControl& self);
void extract(Serializer& serializer, LedControl& self);

TypedResult<LedControl> ledControl(C::mip_interface& device, const uint8_t* primarycolor, const uint8_t* altcolor, LedAction act, uint32_t period);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_rtk_modem_hard_reset  (0x0F,0x22) Modem Hard Reset [CPP]
/// This command will clear the modem flash.  THIS MUST NOT BE DONE OFTEN AS IT CAN DAMAGE THE FLASH!
/// This command is only available in calibration mode.
///
///@{

struct ModemHardReset
{
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_rtk::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_rtk::CMD_MODEM_HARD_RESET;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ModemHardReset";
    static constexpr const char* DOC_NAME = "ModemHardReset";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple();
    }
    
    auto as_tuple()
    {
        return std::make_tuple();
    }
    typedef void Response;
};
void insert(Serializer& serializer, const ModemHardReset& self);
void extract(Serializer& serializer, ModemHardReset& self);

TypedResult<ModemHardReset> modemHardReset(C::mip_interface& device);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace commands_rtk
} // namespace mip

