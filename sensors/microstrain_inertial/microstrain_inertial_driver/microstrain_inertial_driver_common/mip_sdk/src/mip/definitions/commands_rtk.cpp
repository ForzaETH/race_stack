
#include "commands_rtk.hpp"

#include "../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mip {
class Serializer;

namespace C {
struct mip_interface;
} // namespace C

namespace commands_rtk {

using ::mip::insert;
using ::mip::extract;
using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(Serializer& serializer, const GetStatusFlags& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, GetStatusFlags& self)
{
    (void)serializer;
    (void)self;
}

void insert(Serializer& serializer, const GetStatusFlags::Response& self)
{
    insert(serializer, self.flags);
    
}
void extract(Serializer& serializer, GetStatusFlags::Response& self)
{
    extract(serializer, self.flags);
    
}

TypedResult<GetStatusFlags> getStatusFlags(C::mip_interface& device, GetStatusFlags::StatusFlags* flagsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<GetStatusFlags> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_STATUS_FLAGS, NULL, 0, REPLY_GET_STATUS_FLAGS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(flagsOut);
        extract(deserializer, *flagsOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const GetImei& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, GetImei& self)
{
    (void)serializer;
    (void)self;
}

void insert(Serializer& serializer, const GetImei::Response& self)
{
    for(unsigned int i=0; i < 32; i++)
        insert(serializer, self.IMEI[i]);
    
}
void extract(Serializer& serializer, GetImei::Response& self)
{
    for(unsigned int i=0; i < 32; i++)
        extract(serializer, self.IMEI[i]);
    
}

TypedResult<GetImei> getImei(C::mip_interface& device, char* imeiOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<GetImei> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_IMEI, NULL, 0, REPLY_GET_IMEI, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(imeiOut || (32 == 0));
        for(unsigned int i=0; i < 32; i++)
            extract(deserializer, imeiOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const GetImsi& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, GetImsi& self)
{
    (void)serializer;
    (void)self;
}

void insert(Serializer& serializer, const GetImsi::Response& self)
{
    for(unsigned int i=0; i < 32; i++)
        insert(serializer, self.IMSI[i]);
    
}
void extract(Serializer& serializer, GetImsi::Response& self)
{
    for(unsigned int i=0; i < 32; i++)
        extract(serializer, self.IMSI[i]);
    
}

TypedResult<GetImsi> getImsi(C::mip_interface& device, char* imsiOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<GetImsi> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_IMSI, NULL, 0, REPLY_GET_IMSI, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(imsiOut || (32 == 0));
        for(unsigned int i=0; i < 32; i++)
            extract(deserializer, imsiOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const GetIccid& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, GetIccid& self)
{
    (void)serializer;
    (void)self;
}

void insert(Serializer& serializer, const GetIccid::Response& self)
{
    for(unsigned int i=0; i < 32; i++)
        insert(serializer, self.ICCID[i]);
    
}
void extract(Serializer& serializer, GetIccid::Response& self)
{
    for(unsigned int i=0; i < 32; i++)
        extract(serializer, self.ICCID[i]);
    
}

TypedResult<GetIccid> getIccid(C::mip_interface& device, char* iccidOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<GetIccid> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_ICCID, NULL, 0, REPLY_GET_ICCID, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(iccidOut || (32 == 0));
        for(unsigned int i=0; i < 32; i++)
            extract(deserializer, iccidOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const ConnectedDeviceType& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.devType);
        
    }
}
void extract(Serializer& serializer, ConnectedDeviceType& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.devType);
        
    }
}

void insert(Serializer& serializer, const ConnectedDeviceType::Response& self)
{
    insert(serializer, self.devType);
    
}
void extract(Serializer& serializer, ConnectedDeviceType::Response& self)
{
    extract(serializer, self.devType);
    
}

TypedResult<ConnectedDeviceType> writeConnectedDeviceType(C::mip_interface& device, ConnectedDeviceType::Type devtype)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, devtype);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONNECTED_DEVICE_TYPE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<ConnectedDeviceType> readConnectedDeviceType(C::mip_interface& device, ConnectedDeviceType::Type* devtypeOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<ConnectedDeviceType> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_CONNECTED_DEVICE_TYPE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_CONNECTED_DEVICE_TYPE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(devtypeOut);
        extract(deserializer, *devtypeOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<ConnectedDeviceType> saveConnectedDeviceType(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONNECTED_DEVICE_TYPE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<ConnectedDeviceType> loadConnectedDeviceType(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONNECTED_DEVICE_TYPE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<ConnectedDeviceType> defaultConnectedDeviceType(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONNECTED_DEVICE_TYPE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const GetActCode& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, GetActCode& self)
{
    (void)serializer;
    (void)self;
}

void insert(Serializer& serializer, const GetActCode::Response& self)
{
    for(unsigned int i=0; i < 32; i++)
        insert(serializer, self.ActivationCode[i]);
    
}
void extract(Serializer& serializer, GetActCode::Response& self)
{
    for(unsigned int i=0; i < 32; i++)
        extract(serializer, self.ActivationCode[i]);
    
}

TypedResult<GetActCode> getActCode(C::mip_interface& device, char* activationcodeOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<GetActCode> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_ACT_CODE, NULL, 0, REPLY_GET_ACT_CODE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(activationcodeOut || (32 == 0));
        for(unsigned int i=0; i < 32; i++)
            extract(deserializer, activationcodeOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const GetModemFirmwareVersion& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, GetModemFirmwareVersion& self)
{
    (void)serializer;
    (void)self;
}

void insert(Serializer& serializer, const GetModemFirmwareVersion::Response& self)
{
    for(unsigned int i=0; i < 32; i++)
        insert(serializer, self.ModemFirmwareVersion[i]);
    
}
void extract(Serializer& serializer, GetModemFirmwareVersion::Response& self)
{
    for(unsigned int i=0; i < 32; i++)
        extract(serializer, self.ModemFirmwareVersion[i]);
    
}

TypedResult<GetModemFirmwareVersion> getModemFirmwareVersion(C::mip_interface& device, char* modemfirmwareversionOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<GetModemFirmwareVersion> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_MODEM_FIRMWARE_VERSION, NULL, 0, REPLY_GET_MODEM_FIRMWARE_VERSION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(modemfirmwareversionOut || (32 == 0));
        for(unsigned int i=0; i < 32; i++)
            extract(deserializer, modemfirmwareversionOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const GetRssi& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, GetRssi& self)
{
    (void)serializer;
    (void)self;
}

void insert(Serializer& serializer, const GetRssi::Response& self)
{
    insert(serializer, self.valid);
    
    insert(serializer, self.rssi);
    
    insert(serializer, self.signalQuality);
    
}
void extract(Serializer& serializer, GetRssi::Response& self)
{
    extract(serializer, self.valid);
    
    extract(serializer, self.rssi);
    
    extract(serializer, self.signalQuality);
    
}

TypedResult<GetRssi> getRssi(C::mip_interface& device, bool* validOut, int32_t* rssiOut, int32_t* signalqualityOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<GetRssi> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_RSSI, NULL, 0, REPLY_GET_RSSI, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(validOut);
        extract(deserializer, *validOut);
        
        assert(rssiOut);
        extract(deserializer, *rssiOut);
        
        assert(signalqualityOut);
        extract(deserializer, *signalqualityOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const ServiceStatus& self)
{
    insert(serializer, self.reserved1);
    
    insert(serializer, self.reserved2);
    
}
void extract(Serializer& serializer, ServiceStatus& self)
{
    extract(serializer, self.reserved1);
    
    extract(serializer, self.reserved2);
    
}

void insert(Serializer& serializer, const ServiceStatus::Response& self)
{
    insert(serializer, self.flags);
    
    insert(serializer, self.receivedBytes);
    
    insert(serializer, self.lastBytes);
    
    insert(serializer, self.lastBytesTime);
    
}
void extract(Serializer& serializer, ServiceStatus::Response& self)
{
    extract(serializer, self.flags);
    
    extract(serializer, self.receivedBytes);
    
    extract(serializer, self.lastBytes);
    
    extract(serializer, self.lastBytesTime);
    
}

TypedResult<ServiceStatus> serviceStatus(C::mip_interface& device, uint32_t reserved1, uint32_t reserved2, ServiceStatus::ServiceFlags* flagsOut, uint32_t* receivedbytesOut, uint32_t* lastbytesOut, uint64_t* lastbytestimeOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, reserved1);
    
    insert(serializer, reserved2);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<ServiceStatus> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SERVICE_STATUS, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_SERVICE_STATUS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(flagsOut);
        extract(deserializer, *flagsOut);
        
        assert(receivedbytesOut);
        extract(deserializer, *receivedbytesOut);
        
        assert(lastbytesOut);
        extract(deserializer, *lastbytesOut);
        
        assert(lastbytestimeOut);
        extract(deserializer, *lastbytestimeOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const ProdEraseStorage& self)
{
    insert(serializer, self.media);
    
}
void extract(Serializer& serializer, ProdEraseStorage& self)
{
    extract(serializer, self.media);
    
}

TypedResult<ProdEraseStorage> prodEraseStorage(C::mip_interface& device, MediaSelector media)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, media);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PROD_ERASE_STORAGE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const LedControl& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.primaryColor[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.altColor[i]);
    
    insert(serializer, self.act);
    
    insert(serializer, self.period);
    
}
void extract(Serializer& serializer, LedControl& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.primaryColor[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.altColor[i]);
    
    extract(serializer, self.act);
    
    extract(serializer, self.period);
    
}

TypedResult<LedControl> ledControl(C::mip_interface& device, const uint8_t* primarycolor, const uint8_t* altcolor, LedAction act, uint32_t period)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    assert(primarycolor || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, primarycolor[i]);
    
    assert(altcolor || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, altcolor[i]);
    
    insert(serializer, act);
    
    insert(serializer, period);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const ModemHardReset& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, ModemHardReset& self)
{
    (void)serializer;
    (void)self;
}

TypedResult<ModemHardReset> modemHardReset(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MODEM_HARD_RESET, NULL, 0);
}

} // namespace commands_rtk
} // namespace mip

