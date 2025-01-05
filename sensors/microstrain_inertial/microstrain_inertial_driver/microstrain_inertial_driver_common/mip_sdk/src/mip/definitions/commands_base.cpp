
#include "commands_base.hpp"

#include "../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mip {
class Serializer;

namespace C {
struct mip_interface;
} // namespace C

namespace commands_base {

using ::mip::insert;
using ::mip::extract;
using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

void insert(Serializer& serializer, const BaseDeviceInfo& self)
{
    insert(serializer, self.firmware_version);
    
    for(unsigned int i=0; i < 16; i++)
        insert(serializer, self.model_name[i]);
    
    for(unsigned int i=0; i < 16; i++)
        insert(serializer, self.model_number[i]);
    
    for(unsigned int i=0; i < 16; i++)
        insert(serializer, self.serial_number[i]);
    
    for(unsigned int i=0; i < 16; i++)
        insert(serializer, self.lot_number[i]);
    
    for(unsigned int i=0; i < 16; i++)
        insert(serializer, self.device_options[i]);
    
}
void extract(Serializer& serializer, BaseDeviceInfo& self)
{
    extract(serializer, self.firmware_version);
    
    for(unsigned int i=0; i < 16; i++)
        extract(serializer, self.model_name[i]);
    
    for(unsigned int i=0; i < 16; i++)
        extract(serializer, self.model_number[i]);
    
    for(unsigned int i=0; i < 16; i++)
        extract(serializer, self.serial_number[i]);
    
    for(unsigned int i=0; i < 16; i++)
        extract(serializer, self.lot_number[i]);
    
    for(unsigned int i=0; i < 16; i++)
        extract(serializer, self.device_options[i]);
    
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(Serializer& serializer, const Ping& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, Ping& self)
{
    (void)serializer;
    (void)self;
}

TypedResult<Ping> ping(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PING, NULL, 0);
}
void insert(Serializer& serializer, const SetIdle& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, SetIdle& self)
{
    (void)serializer;
    (void)self;
}

TypedResult<SetIdle> setIdle(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SET_TO_IDLE, NULL, 0);
}
void insert(Serializer& serializer, const GetDeviceInfo& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, GetDeviceInfo& self)
{
    (void)serializer;
    (void)self;
}

void insert(Serializer& serializer, const GetDeviceInfo::Response& self)
{
    insert(serializer, self.device_info);
    
}
void extract(Serializer& serializer, GetDeviceInfo::Response& self)
{
    extract(serializer, self.device_info);
    
}

TypedResult<GetDeviceInfo> getDeviceInfo(C::mip_interface& device, BaseDeviceInfo* deviceInfoOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<GetDeviceInfo> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_DEVICE_INFO, NULL, 0, REPLY_DEVICE_INFO, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(deviceInfoOut);
        extract(deserializer, *deviceInfoOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const GetDeviceDescriptors& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, GetDeviceDescriptors& self)
{
    (void)serializer;
    (void)self;
}

void insert(Serializer& serializer, const GetDeviceDescriptors::Response& self)
{
    for(unsigned int i=0; i < self.descriptors_count; i++)
        insert(serializer, self.descriptors[i]);
    
}
void extract(Serializer& serializer, GetDeviceDescriptors::Response& self)
{
    for(self.descriptors_count = 0; (self.descriptors_count < sizeof(self.descriptors)/sizeof(self.descriptors[0])) && (serializer.remaining() > 0); (self.descriptors_count)++)
        extract(serializer, self.descriptors[self.descriptors_count]);
    
}

TypedResult<GetDeviceDescriptors> getDeviceDescriptors(C::mip_interface& device, uint16_t* descriptorsOut, size_t descriptorsOutMax, uint8_t* descriptorsOutCount)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<GetDeviceDescriptors> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_DEVICE_DESCRIPTORS, NULL, 0, REPLY_DEVICE_DESCRIPTORS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        for(*descriptorsOutCount = 0; (*descriptorsOutCount < descriptorsOutMax) && (deserializer.remaining() > 0); (*descriptorsOutCount)++)
            extract(deserializer, descriptorsOut[*descriptorsOutCount]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const BuiltInTest& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, BuiltInTest& self)
{
    (void)serializer;
    (void)self;
}

void insert(Serializer& serializer, const BuiltInTest::Response& self)
{
    insert(serializer, self.result);
    
}
void extract(Serializer& serializer, BuiltInTest::Response& self)
{
    extract(serializer, self.result);
    
}

TypedResult<BuiltInTest> builtInTest(C::mip_interface& device, uint32_t* resultOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<BuiltInTest> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_BUILT_IN_TEST, NULL, 0, REPLY_BUILT_IN_TEST, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(resultOut);
        extract(deserializer, *resultOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const Resume& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, Resume& self)
{
    (void)serializer;
    (void)self;
}

TypedResult<Resume> resume(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RESUME, NULL, 0);
}
void insert(Serializer& serializer, const GetExtendedDescriptors& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, GetExtendedDescriptors& self)
{
    (void)serializer;
    (void)self;
}

void insert(Serializer& serializer, const GetExtendedDescriptors::Response& self)
{
    for(unsigned int i=0; i < self.descriptors_count; i++)
        insert(serializer, self.descriptors[i]);
    
}
void extract(Serializer& serializer, GetExtendedDescriptors::Response& self)
{
    for(self.descriptors_count = 0; (self.descriptors_count < sizeof(self.descriptors)/sizeof(self.descriptors[0])) && (serializer.remaining() > 0); (self.descriptors_count)++)
        extract(serializer, self.descriptors[self.descriptors_count]);
    
}

TypedResult<GetExtendedDescriptors> getExtendedDescriptors(C::mip_interface& device, uint16_t* descriptorsOut, size_t descriptorsOutMax, uint8_t* descriptorsOutCount)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<GetExtendedDescriptors> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_EXTENDED_DESCRIPTORS, NULL, 0, REPLY_GET_EXTENDED_DESCRIPTORS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        for(*descriptorsOutCount = 0; (*descriptorsOutCount < descriptorsOutMax) && (deserializer.remaining() > 0); (*descriptorsOutCount)++)
            extract(deserializer, descriptorsOut[*descriptorsOutCount]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const ContinuousBit& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, ContinuousBit& self)
{
    (void)serializer;
    (void)self;
}

void insert(Serializer& serializer, const ContinuousBit::Response& self)
{
    for(unsigned int i=0; i < 16; i++)
        insert(serializer, self.result[i]);
    
}
void extract(Serializer& serializer, ContinuousBit::Response& self)
{
    for(unsigned int i=0; i < 16; i++)
        extract(serializer, self.result[i]);
    
}

TypedResult<ContinuousBit> continuousBit(C::mip_interface& device, uint8_t* resultOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<ContinuousBit> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_CONTINUOUS_BIT, NULL, 0, REPLY_CONTINUOUS_BIT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(resultOut || (16 == 0));
        for(unsigned int i=0; i < 16; i++)
            extract(deserializer, resultOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const CommSpeed& self)
{
    insert(serializer, self.function);
    
    insert(serializer, self.port);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.baud);
        
    }
}
void extract(Serializer& serializer, CommSpeed& self)
{
    extract(serializer, self.function);
    
    extract(serializer, self.port);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.baud);
        
    }
}

void insert(Serializer& serializer, const CommSpeed::Response& self)
{
    insert(serializer, self.port);
    
    insert(serializer, self.baud);
    
}
void extract(Serializer& serializer, CommSpeed::Response& self)
{
    extract(serializer, self.port);
    
    extract(serializer, self.baud);
    
}

TypedResult<CommSpeed> writeCommSpeed(C::mip_interface& device, uint8_t port, uint32_t baud)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, port);
    
    insert(serializer, baud);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COMM_SPEED, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<CommSpeed> readCommSpeed(C::mip_interface& device, uint8_t port, uint32_t* baudOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, port);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<CommSpeed> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_COMM_SPEED, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_COMM_SPEED, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, port);
        
        assert(baudOut);
        extract(deserializer, *baudOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<CommSpeed> saveCommSpeed(C::mip_interface& device, uint8_t port)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, port);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COMM_SPEED, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<CommSpeed> loadCommSpeed(C::mip_interface& device, uint8_t port)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, port);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COMM_SPEED, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<CommSpeed> defaultCommSpeed(C::mip_interface& device, uint8_t port)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, port);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COMM_SPEED, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const GpsTimeUpdate& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.field_id);
        
        insert(serializer, self.value);
        
    }
}
void extract(Serializer& serializer, GpsTimeUpdate& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.field_id);
        
        extract(serializer, self.value);
        
    }
}

TypedResult<GpsTimeUpdate> writeGpsTimeUpdate(C::mip_interface& device, GpsTimeUpdate::FieldId fieldId, uint32_t value)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, fieldId);
    
    insert(serializer, value);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GPS_TIME_UPDATE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const SoftReset& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, SoftReset& self)
{
    (void)serializer;
    (void)self;
}

TypedResult<SoftReset> softReset(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_RESET, NULL, 0);
}

} // namespace commands_base
} // namespace mip

