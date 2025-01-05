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
///@defgroup rtk_commands_c  Rtk Commands [C]
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    MIP_RTK_CMD_DESC_SET                          = 0x0F,
    
    MIP_CMD_DESC_RTK_GET_STATUS_FLAGS             = 0x01,
    MIP_CMD_DESC_RTK_GET_IMEI                     = 0x02,
    MIP_CMD_DESC_RTK_GET_IMSI                     = 0x03,
    MIP_CMD_DESC_RTK_GET_ICCID                    = 0x04,
    MIP_CMD_DESC_RTK_GET_RSSI                     = 0x05,
    MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE        = 0x06,
    MIP_CMD_DESC_RTK_GET_ACT_CODE                 = 0x07,
    MIP_CMD_DESC_RTK_GET_MODEM_FIRMWARE_VERSION   = 0x08,
    MIP_CMD_DESC_RTK_SERVICE_STATUS               = 0x0A,
    MIP_CMD_DESC_RTK_PROD_ERASE_STORAGE           = 0x20,
    MIP_CMD_DESC_LED_CONTROL                      = 0x21,
    MIP_CMD_DESC_RTK_MODEM_HARD_RESET             = 0x22,
    
    MIP_REPLY_DESC_RTK_GET_STATUS_FLAGS           = 0x81,
    MIP_REPLY_DESC_RTK_GET_IMEI                   = 0x82,
    MIP_REPLY_DESC_RTK_GET_IMSI                   = 0x83,
    MIP_REPLY_DESC_RTK_GET_ICCID                  = 0x84,
    MIP_REPLY_DESC_RTK_CONNECTED_DEVICE_TYPE      = 0x86,
    MIP_REPLY_DESC_RTK_GET_ACT_CODE               = 0x87,
    MIP_REPLY_DESC_RTK_GET_MODEM_FIRMWARE_VERSION = 0x88,
    MIP_REPLY_DESC_RTK_GET_RSSI                   = 0x85,
    MIP_REPLY_DESC_RTK_SERVICE_STATUS             = 0x8A,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

typedef uint8_t mip_media_selector;
static const mip_media_selector MIP_MEDIA_SELECTOR_MEDIA_EXTERNALFLASH = 0; ///<  
static const mip_media_selector MIP_MEDIA_SELECTOR_MEDIA_SD            = 1; ///<  

void insert_mip_media_selector(struct mip_serializer* serializer, const mip_media_selector self);
void extract_mip_media_selector(struct mip_serializer* serializer, mip_media_selector* self);

typedef uint8_t mip_led_action;
static const mip_led_action MIP_LED_ACTION_LED_NONE    = 0; ///<  
static const mip_led_action MIP_LED_ACTION_LED_FLASH   = 1; ///<  
static const mip_led_action MIP_LED_ACTION_LED_PULSATE = 2; ///<  

void insert_mip_led_action(struct mip_serializer* serializer, const mip_led_action self);
void extract_mip_led_action(struct mip_serializer* serializer, mip_led_action* self);


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup c_rtk_get_status_flags  (0x0F,0x01) Get Status Flags [C]
///
///@{

typedef uint32_t mip_rtk_get_status_flags_command_status_flags_legacy;
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_NONE                 = 0x00000000;
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_CONTROLLERSTATE      = 0x00000007; ///<  
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_PLATFORMSTATE        = 0x000000F8; ///<  
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_CONTROLLERSTATUSCODE = 0x00000700; ///<  
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_PLATFORMSTATUSCODE   = 0x00003800; ///<  
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_RESETCODE            = 0x0000C000; ///<  
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_SIGNALQUALITY        = 0x000F0000; ///<  
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_RESERVED             = 0xFFF00000; ///<  
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_RSSI                 = 0x03F00000; ///<  
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_RSRP                 = 0x0C000000; ///<  
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_RSRQ                 = 0x30000000; ///<  
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_SINR                 = 0xC0000000; ///<  
static const mip_rtk_get_status_flags_command_status_flags_legacy MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_LEGACY_ALL                  = 0xFFFFFFFF;

typedef uint32_t mip_rtk_get_status_flags_command_status_flags;
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_NONE                    = 0x00000000;
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_MODEM_STATE             = 0x0000000F; ///<  
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_CONNECTION_TYPE         = 0x000000F0; ///<  
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_RSSI                    = 0x0000FF00; ///<  
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_SIGNAL_QUALITY          = 0x000F0000; ///<  
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_TOWER_CHANGE_INDICATOR  = 0x00F00000; ///<  
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_NMEA_TIMEOUT            = 0x01000000; ///<  
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_SERVER_TIMEOUT          = 0x02000000; ///<  
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_CORRECTIONS_TIMEOUT     = 0x04000000; ///<  
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_DEVICE_OUT_OF_RANGE     = 0x08000000; ///<  
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_CORRECTIONS_UNAVAILABLE = 0x10000000; ///<  
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_RESERVED                = 0x20000000; ///<  
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_VERSION                 = 0xC0000000; ///<  
static const mip_rtk_get_status_flags_command_status_flags MIP_RTK_GET_STATUS_FLAGS_COMMAND_STATUS_FLAGS_ALL                     = 0xFFFFFFFF;

void insert_mip_rtk_get_status_flags_command_status_flags_legacy(struct mip_serializer* serializer, const mip_rtk_get_status_flags_command_status_flags_legacy self);
void extract_mip_rtk_get_status_flags_command_status_flags_legacy(struct mip_serializer* serializer, mip_rtk_get_status_flags_command_status_flags_legacy* self);

void insert_mip_rtk_get_status_flags_command_status_flags(struct mip_serializer* serializer, const mip_rtk_get_status_flags_command_status_flags self);
void extract_mip_rtk_get_status_flags_command_status_flags(struct mip_serializer* serializer, mip_rtk_get_status_flags_command_status_flags* self);

struct mip_rtk_get_status_flags_response
{
    mip_rtk_get_status_flags_command_status_flags flags; ///< Model number dependent. See above structures.
    
};
typedef struct mip_rtk_get_status_flags_response mip_rtk_get_status_flags_response;
void insert_mip_rtk_get_status_flags_response(struct mip_serializer* serializer, const mip_rtk_get_status_flags_response* self);
void extract_mip_rtk_get_status_flags_response(struct mip_serializer* serializer, mip_rtk_get_status_flags_response* self);

mip_cmd_result mip_rtk_get_status_flags(struct mip_interface* device, mip_rtk_get_status_flags_command_status_flags* flags_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_rtk_get_imei  (0x0F,0x02) Get Imei [C]
///
///@{

struct mip_rtk_get_imei_response
{
    char IMEI[32];
    
};
typedef struct mip_rtk_get_imei_response mip_rtk_get_imei_response;
void insert_mip_rtk_get_imei_response(struct mip_serializer* serializer, const mip_rtk_get_imei_response* self);
void extract_mip_rtk_get_imei_response(struct mip_serializer* serializer, mip_rtk_get_imei_response* self);

mip_cmd_result mip_rtk_get_imei(struct mip_interface* device, char* imei_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_rtk_get_imsi  (0x0F,0x03) Get Imsi [C]
///
///@{

struct mip_rtk_get_imsi_response
{
    char IMSI[32];
    
};
typedef struct mip_rtk_get_imsi_response mip_rtk_get_imsi_response;
void insert_mip_rtk_get_imsi_response(struct mip_serializer* serializer, const mip_rtk_get_imsi_response* self);
void extract_mip_rtk_get_imsi_response(struct mip_serializer* serializer, mip_rtk_get_imsi_response* self);

mip_cmd_result mip_rtk_get_imsi(struct mip_interface* device, char* imsi_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_rtk_get_iccid  (0x0F,0x04) Get Iccid [C]
///
///@{

struct mip_rtk_get_iccid_response
{
    char ICCID[32];
    
};
typedef struct mip_rtk_get_iccid_response mip_rtk_get_iccid_response;
void insert_mip_rtk_get_iccid_response(struct mip_serializer* serializer, const mip_rtk_get_iccid_response* self);
void extract_mip_rtk_get_iccid_response(struct mip_serializer* serializer, mip_rtk_get_iccid_response* self);

mip_cmd_result mip_rtk_get_iccid(struct mip_interface* device, char* iccid_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_rtk_connected_device_type  (0x0F,0x06) Connected Device Type [C]
///
///@{

typedef uint8_t mip_rtk_connected_device_type_command_type;
static const mip_rtk_connected_device_type_command_type MIP_RTK_CONNECTED_DEVICE_TYPE_COMMAND_TYPE_GENERIC = 0; ///<  
static const mip_rtk_connected_device_type_command_type MIP_RTK_CONNECTED_DEVICE_TYPE_COMMAND_TYPE_GQ7     = 1; ///<  

struct mip_rtk_connected_device_type_command
{
    mip_function_selector function;
    mip_rtk_connected_device_type_command_type devType;
    
};
typedef struct mip_rtk_connected_device_type_command mip_rtk_connected_device_type_command;
void insert_mip_rtk_connected_device_type_command(struct mip_serializer* serializer, const mip_rtk_connected_device_type_command* self);
void extract_mip_rtk_connected_device_type_command(struct mip_serializer* serializer, mip_rtk_connected_device_type_command* self);

void insert_mip_rtk_connected_device_type_command_type(struct mip_serializer* serializer, const mip_rtk_connected_device_type_command_type self);
void extract_mip_rtk_connected_device_type_command_type(struct mip_serializer* serializer, mip_rtk_connected_device_type_command_type* self);

struct mip_rtk_connected_device_type_response
{
    mip_rtk_connected_device_type_command_type devType;
    
};
typedef struct mip_rtk_connected_device_type_response mip_rtk_connected_device_type_response;
void insert_mip_rtk_connected_device_type_response(struct mip_serializer* serializer, const mip_rtk_connected_device_type_response* self);
void extract_mip_rtk_connected_device_type_response(struct mip_serializer* serializer, mip_rtk_connected_device_type_response* self);

mip_cmd_result mip_rtk_write_connected_device_type(struct mip_interface* device, mip_rtk_connected_device_type_command_type dev_type);
mip_cmd_result mip_rtk_read_connected_device_type(struct mip_interface* device, mip_rtk_connected_device_type_command_type* dev_type_out);
mip_cmd_result mip_rtk_save_connected_device_type(struct mip_interface* device);
mip_cmd_result mip_rtk_load_connected_device_type(struct mip_interface* device);
mip_cmd_result mip_rtk_default_connected_device_type(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_rtk_get_act_code  (0x0F,0x07) Get Act Code [C]
///
///@{

struct mip_rtk_get_act_code_response
{
    char ActivationCode[32];
    
};
typedef struct mip_rtk_get_act_code_response mip_rtk_get_act_code_response;
void insert_mip_rtk_get_act_code_response(struct mip_serializer* serializer, const mip_rtk_get_act_code_response* self);
void extract_mip_rtk_get_act_code_response(struct mip_serializer* serializer, mip_rtk_get_act_code_response* self);

mip_cmd_result mip_rtk_get_act_code(struct mip_interface* device, char* activation_code_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_rtk_get_modem_firmware_version  (0x0F,0x08) Get Modem Firmware Version [C]
///
///@{

struct mip_rtk_get_modem_firmware_version_response
{
    char ModemFirmwareVersion[32];
    
};
typedef struct mip_rtk_get_modem_firmware_version_response mip_rtk_get_modem_firmware_version_response;
void insert_mip_rtk_get_modem_firmware_version_response(struct mip_serializer* serializer, const mip_rtk_get_modem_firmware_version_response* self);
void extract_mip_rtk_get_modem_firmware_version_response(struct mip_serializer* serializer, mip_rtk_get_modem_firmware_version_response* self);

mip_cmd_result mip_rtk_get_modem_firmware_version(struct mip_interface* device, char* modem_firmware_version_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_rtk_get_rssi  (0x0F,0x05) Get Rssi [C]
/// Get the RSSI and connected/disconnected status of modem
///
///@{

struct mip_rtk_get_rssi_response
{
    bool valid;
    int32_t rssi;
    int32_t signalQuality;
    
};
typedef struct mip_rtk_get_rssi_response mip_rtk_get_rssi_response;
void insert_mip_rtk_get_rssi_response(struct mip_serializer* serializer, const mip_rtk_get_rssi_response* self);
void extract_mip_rtk_get_rssi_response(struct mip_serializer* serializer, mip_rtk_get_rssi_response* self);

mip_cmd_result mip_rtk_get_rssi(struct mip_interface* device, bool* valid_out, int32_t* rssi_out, int32_t* signal_quality_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_rtk_service_status  (0x0F,0x0A) Service Status [C]
/// The 3DMRTK will send this message to the server to indicate that the connection should remain open. The Server will respond with information and status.
///
///@{

typedef uint8_t mip_rtk_service_status_command_service_flags;
static const mip_rtk_service_status_command_service_flags MIP_RTK_SERVICE_STATUS_COMMAND_SERVICE_FLAGS_NONE                    = 0x00;
static const mip_rtk_service_status_command_service_flags MIP_RTK_SERVICE_STATUS_COMMAND_SERVICE_FLAGS_THROTTLE                = 0x01; ///<  
static const mip_rtk_service_status_command_service_flags MIP_RTK_SERVICE_STATUS_COMMAND_SERVICE_FLAGS_CORRECTIONS_UNAVAILABLE = 0x02; ///<  
static const mip_rtk_service_status_command_service_flags MIP_RTK_SERVICE_STATUS_COMMAND_SERVICE_FLAGS_RESERVED                = 0xFC; ///<  
static const mip_rtk_service_status_command_service_flags MIP_RTK_SERVICE_STATUS_COMMAND_SERVICE_FLAGS_ALL                     = 0xFF;

struct mip_rtk_service_status_command
{
    uint32_t reserved1;
    uint32_t reserved2;
    
};
typedef struct mip_rtk_service_status_command mip_rtk_service_status_command;
void insert_mip_rtk_service_status_command(struct mip_serializer* serializer, const mip_rtk_service_status_command* self);
void extract_mip_rtk_service_status_command(struct mip_serializer* serializer, mip_rtk_service_status_command* self);

void insert_mip_rtk_service_status_command_service_flags(struct mip_serializer* serializer, const mip_rtk_service_status_command_service_flags self);
void extract_mip_rtk_service_status_command_service_flags(struct mip_serializer* serializer, mip_rtk_service_status_command_service_flags* self);

struct mip_rtk_service_status_response
{
    mip_rtk_service_status_command_service_flags flags;
    uint32_t receivedBytes;
    uint32_t lastBytes;
    uint64_t lastBytesTime;
    
};
typedef struct mip_rtk_service_status_response mip_rtk_service_status_response;
void insert_mip_rtk_service_status_response(struct mip_serializer* serializer, const mip_rtk_service_status_response* self);
void extract_mip_rtk_service_status_response(struct mip_serializer* serializer, mip_rtk_service_status_response* self);

mip_cmd_result mip_rtk_service_status(struct mip_interface* device, uint32_t reserved1, uint32_t reserved2, mip_rtk_service_status_command_service_flags* flags_out, uint32_t* received_bytes_out, uint32_t* last_bytes_out, uint64_t* last_bytes_time_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_rtk_prod_erase_storage  (0x0F,0x20) Prod Erase Storage [C]
/// This command will erase the selected media to a raw and uninitialized state. ALL DATA WILL BE LOST.
/// This command is only available in calibration mode.
///
///@{

struct mip_rtk_prod_erase_storage_command
{
    mip_media_selector media;
    
};
typedef struct mip_rtk_prod_erase_storage_command mip_rtk_prod_erase_storage_command;
void insert_mip_rtk_prod_erase_storage_command(struct mip_serializer* serializer, const mip_rtk_prod_erase_storage_command* self);
void extract_mip_rtk_prod_erase_storage_command(struct mip_serializer* serializer, mip_rtk_prod_erase_storage_command* self);

mip_cmd_result mip_rtk_prod_erase_storage(struct mip_interface* device, mip_media_selector media);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_rtk_led_control  (0x0F,0x21) Led Control [C]
/// This command allows direct control of the LED on the 3DM RTK. This command is only available in calibration mode or Production Test Mode.
///
///@{

struct mip_rtk_led_control_command
{
    uint8_t primaryColor[3];
    uint8_t altColor[3];
    mip_led_action act;
    uint32_t period;
    
};
typedef struct mip_rtk_led_control_command mip_rtk_led_control_command;
void insert_mip_rtk_led_control_command(struct mip_serializer* serializer, const mip_rtk_led_control_command* self);
void extract_mip_rtk_led_control_command(struct mip_serializer* serializer, mip_rtk_led_control_command* self);

mip_cmd_result mip_rtk_led_control(struct mip_interface* device, const uint8_t* primary_color, const uint8_t* alt_color, mip_led_action act, uint32_t period);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_rtk_modem_hard_reset  (0x0F,0x22) Modem Hard Reset [C]
/// This command will clear the modem flash.  THIS MUST NOT BE DONE OFTEN AS IT CAN DAMAGE THE FLASH!
/// This command is only available in calibration mode.
///
///@{

mip_cmd_result mip_rtk_modem_hard_reset(struct mip_interface* device);

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

