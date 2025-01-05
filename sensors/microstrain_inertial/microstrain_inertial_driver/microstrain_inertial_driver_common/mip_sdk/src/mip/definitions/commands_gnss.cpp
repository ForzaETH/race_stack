
#include "commands_gnss.hpp"

#include "../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mip {
class Serializer;

namespace C {
struct mip_interface;
} // namespace C

namespace commands_gnss {

using ::mip::insert;
using ::mip::extract;
using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(Serializer& serializer, const ReceiverInfo& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, ReceiverInfo& self)
{
    (void)serializer;
    (void)self;
}

void insert(Serializer& serializer, const ReceiverInfo::Response& self)
{
    insert(serializer, self.num_receivers);
    
    for(unsigned int i=0; i < self.num_receivers; i++)
        insert(serializer, self.receiver_info[i]);
    
}
void extract(Serializer& serializer, ReceiverInfo::Response& self)
{
    C::extract_count(&serializer, &self.num_receivers, sizeof(self.receiver_info)/sizeof(self.receiver_info[0]));
    for(unsigned int i=0; i < self.num_receivers; i++)
        extract(serializer, self.receiver_info[i]);
    
}

void insert(Serializer& serializer, const ReceiverInfo::Info& self)
{
    insert(serializer, self.receiver_id);
    
    insert(serializer, self.mip_data_descriptor_set);
    
    for(unsigned int i=0; i < 32; i++)
        insert(serializer, self.description[i]);
    
}
void extract(Serializer& serializer, ReceiverInfo::Info& self)
{
    extract(serializer, self.receiver_id);
    
    extract(serializer, self.mip_data_descriptor_set);
    
    for(unsigned int i=0; i < 32; i++)
        extract(serializer, self.description[i]);
    
}

TypedResult<ReceiverInfo> receiverInfo(C::mip_interface& device, uint8_t* numReceiversOut, uint8_t numReceiversOutMax, ReceiverInfo::Info* receiverInfoOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<ReceiverInfo> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_LIST_RECEIVERS, NULL, 0, REPLY_LIST_RECEIVERS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        C::extract_count(&deserializer, numReceiversOut, numReceiversOutMax);
        assert(receiverInfoOut || (numReceiversOut == 0));
        for(unsigned int i=0; i < *numReceiversOut; i++)
            extract(deserializer, receiverInfoOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const SignalConfiguration& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.gps_enable);
        
        insert(serializer, self.glonass_enable);
        
        insert(serializer, self.galileo_enable);
        
        insert(serializer, self.beidou_enable);
        
        for(unsigned int i=0; i < 4; i++)
            insert(serializer, self.reserved[i]);
        
    }
}
void extract(Serializer& serializer, SignalConfiguration& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.gps_enable);
        
        extract(serializer, self.glonass_enable);
        
        extract(serializer, self.galileo_enable);
        
        extract(serializer, self.beidou_enable);
        
        for(unsigned int i=0; i < 4; i++)
            extract(serializer, self.reserved[i]);
        
    }
}

void insert(Serializer& serializer, const SignalConfiguration::Response& self)
{
    insert(serializer, self.gps_enable);
    
    insert(serializer, self.glonass_enable);
    
    insert(serializer, self.galileo_enable);
    
    insert(serializer, self.beidou_enable);
    
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.reserved[i]);
    
}
void extract(Serializer& serializer, SignalConfiguration::Response& self)
{
    extract(serializer, self.gps_enable);
    
    extract(serializer, self.glonass_enable);
    
    extract(serializer, self.galileo_enable);
    
    extract(serializer, self.beidou_enable);
    
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.reserved[i]);
    
}

TypedResult<SignalConfiguration> writeSignalConfiguration(C::mip_interface& device, uint8_t gpsEnable, uint8_t glonassEnable, uint8_t galileoEnable, uint8_t beidouEnable, const uint8_t* reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, gpsEnable);
    
    insert(serializer, glonassEnable);
    
    insert(serializer, galileoEnable);
    
    insert(serializer, beidouEnable);
    
    assert(reserved || (4 == 0));
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, reserved[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SIGNAL_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<SignalConfiguration> readSignalConfiguration(C::mip_interface& device, uint8_t* gpsEnableOut, uint8_t* glonassEnableOut, uint8_t* galileoEnableOut, uint8_t* beidouEnableOut, uint8_t* reservedOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<SignalConfiguration> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SIGNAL_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_SIGNAL_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(gpsEnableOut);
        extract(deserializer, *gpsEnableOut);
        
        assert(glonassEnableOut);
        extract(deserializer, *glonassEnableOut);
        
        assert(galileoEnableOut);
        extract(deserializer, *galileoEnableOut);
        
        assert(beidouEnableOut);
        extract(deserializer, *beidouEnableOut);
        
        assert(reservedOut || (4 == 0));
        for(unsigned int i=0; i < 4; i++)
            extract(deserializer, reservedOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<SignalConfiguration> saveSignalConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SIGNAL_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<SignalConfiguration> loadSignalConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SIGNAL_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<SignalConfiguration> defaultSignalConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SIGNAL_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const RtkDongleConfiguration& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable);
        
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.reserved[i]);
        
    }
}
void extract(Serializer& serializer, RtkDongleConfiguration& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable);
        
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.reserved[i]);
        
    }
}

void insert(Serializer& serializer, const RtkDongleConfiguration::Response& self)
{
    insert(serializer, self.enable);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.reserved[i]);
    
}
void extract(Serializer& serializer, RtkDongleConfiguration::Response& self)
{
    extract(serializer, self.enable);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.reserved[i]);
    
}

TypedResult<RtkDongleConfiguration> writeRtkDongleConfiguration(C::mip_interface& device, uint8_t enable, const uint8_t* reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    
    assert(reserved || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, reserved[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RTK_DONGLE_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<RtkDongleConfiguration> readRtkDongleConfiguration(C::mip_interface& device, uint8_t* enableOut, uint8_t* reservedOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<RtkDongleConfiguration> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_RTK_DONGLE_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_RTK_DONGLE_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        assert(reservedOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, reservedOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<RtkDongleConfiguration> saveRtkDongleConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RTK_DONGLE_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<RtkDongleConfiguration> loadRtkDongleConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RTK_DONGLE_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<RtkDongleConfiguration> defaultRtkDongleConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RTK_DONGLE_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}

} // namespace commands_gnss
} // namespace mip

