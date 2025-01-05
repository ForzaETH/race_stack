
#include "commands_3dm.hpp"

#include "../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mip {
class Serializer;

namespace C {
struct mip_interface;
} // namespace C

namespace commands_3dm {

using ::mip::insert;
using ::mip::extract;
using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

void insert(Serializer& serializer, const NmeaMessage& self)
{
    insert(serializer, self.message_id);
    
    insert(serializer, self.talker_id);
    
    insert(serializer, self.source_desc_set);
    
    insert(serializer, self.decimation);
    
}
void extract(Serializer& serializer, NmeaMessage& self)
{
    extract(serializer, self.message_id);
    
    extract(serializer, self.talker_id);
    
    extract(serializer, self.source_desc_set);
    
    extract(serializer, self.decimation);
    
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(Serializer& serializer, const PollImuMessage& self)
{
    insert(serializer, self.suppress_ack);
    
    insert(serializer, self.num_descriptors);
    
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
    
}
void extract(Serializer& serializer, PollImuMessage& self)
{
    extract(serializer, self.suppress_ack);
    
    C::extract_count(&serializer, &self.num_descriptors, sizeof(self.descriptors)/sizeof(self.descriptors[0]));
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
    
}

TypedResult<PollImuMessage> pollImuMessage(C::mip_interface& device, bool suppressAck, uint8_t numDescriptors, const DescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, suppressAck);
    
    insert(serializer, numDescriptors);
    
    assert(descriptors || (numDescriptors == 0));
    for(unsigned int i=0; i < numDescriptors; i++)
        insert(serializer, descriptors[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POLL_IMU_MESSAGE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const PollGnssMessage& self)
{
    insert(serializer, self.suppress_ack);
    
    insert(serializer, self.num_descriptors);
    
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
    
}
void extract(Serializer& serializer, PollGnssMessage& self)
{
    extract(serializer, self.suppress_ack);
    
    C::extract_count(&serializer, &self.num_descriptors, sizeof(self.descriptors)/sizeof(self.descriptors[0]));
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
    
}

TypedResult<PollGnssMessage> pollGnssMessage(C::mip_interface& device, bool suppressAck, uint8_t numDescriptors, const DescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, suppressAck);
    
    insert(serializer, numDescriptors);
    
    assert(descriptors || (numDescriptors == 0));
    for(unsigned int i=0; i < numDescriptors; i++)
        insert(serializer, descriptors[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POLL_GNSS_MESSAGE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const PollFilterMessage& self)
{
    insert(serializer, self.suppress_ack);
    
    insert(serializer, self.num_descriptors);
    
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
    
}
void extract(Serializer& serializer, PollFilterMessage& self)
{
    extract(serializer, self.suppress_ack);
    
    C::extract_count(&serializer, &self.num_descriptors, sizeof(self.descriptors)/sizeof(self.descriptors[0]));
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
    
}

TypedResult<PollFilterMessage> pollFilterMessage(C::mip_interface& device, bool suppressAck, uint8_t numDescriptors, const DescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, suppressAck);
    
    insert(serializer, numDescriptors);
    
    assert(descriptors || (numDescriptors == 0));
    for(unsigned int i=0; i < numDescriptors; i++)
        insert(serializer, descriptors[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POLL_FILTER_MESSAGE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const ImuMessageFormat& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.num_descriptors);
        
        for(unsigned int i=0; i < self.num_descriptors; i++)
            insert(serializer, self.descriptors[i]);
        
    }
}
void extract(Serializer& serializer, ImuMessageFormat& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        C::extract_count(&serializer, &self.num_descriptors, sizeof(self.descriptors)/sizeof(self.descriptors[0]));
        for(unsigned int i=0; i < self.num_descriptors; i++)
            extract(serializer, self.descriptors[i]);
        
    }
}

void insert(Serializer& serializer, const ImuMessageFormat::Response& self)
{
    insert(serializer, self.num_descriptors);
    
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
    
}
void extract(Serializer& serializer, ImuMessageFormat::Response& self)
{
    C::extract_count(&serializer, &self.num_descriptors, sizeof(self.descriptors)/sizeof(self.descriptors[0]));
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
    
}

TypedResult<ImuMessageFormat> writeImuMessageFormat(C::mip_interface& device, uint8_t numDescriptors, const DescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, numDescriptors);
    
    assert(descriptors || (numDescriptors == 0));
    for(unsigned int i=0; i < numDescriptors; i++)
        insert(serializer, descriptors[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_IMU_MESSAGE_FORMAT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<ImuMessageFormat> readImuMessageFormat(C::mip_interface& device, uint8_t* numDescriptorsOut, uint8_t numDescriptorsOutMax, DescriptorRate* descriptorsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<ImuMessageFormat> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_IMU_MESSAGE_FORMAT, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_IMU_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        C::extract_count(&deserializer, numDescriptorsOut, numDescriptorsOutMax);
        assert(descriptorsOut || (numDescriptorsOut == 0));
        for(unsigned int i=0; i < *numDescriptorsOut; i++)
            extract(deserializer, descriptorsOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<ImuMessageFormat> saveImuMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_IMU_MESSAGE_FORMAT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<ImuMessageFormat> loadImuMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_IMU_MESSAGE_FORMAT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<ImuMessageFormat> defaultImuMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_IMU_MESSAGE_FORMAT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const GpsMessageFormat& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.num_descriptors);
        
        for(unsigned int i=0; i < self.num_descriptors; i++)
            insert(serializer, self.descriptors[i]);
        
    }
}
void extract(Serializer& serializer, GpsMessageFormat& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        C::extract_count(&serializer, &self.num_descriptors, sizeof(self.descriptors)/sizeof(self.descriptors[0]));
        for(unsigned int i=0; i < self.num_descriptors; i++)
            extract(serializer, self.descriptors[i]);
        
    }
}

void insert(Serializer& serializer, const GpsMessageFormat::Response& self)
{
    insert(serializer, self.num_descriptors);
    
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
    
}
void extract(Serializer& serializer, GpsMessageFormat::Response& self)
{
    C::extract_count(&serializer, &self.num_descriptors, sizeof(self.descriptors)/sizeof(self.descriptors[0]));
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
    
}

TypedResult<GpsMessageFormat> writeGpsMessageFormat(C::mip_interface& device, uint8_t numDescriptors, const DescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, numDescriptors);
    
    assert(descriptors || (numDescriptors == 0));
    for(unsigned int i=0; i < numDescriptors; i++)
        insert(serializer, descriptors[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_MESSAGE_FORMAT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GpsMessageFormat> readGpsMessageFormat(C::mip_interface& device, uint8_t* numDescriptorsOut, uint8_t numDescriptorsOutMax, DescriptorRate* descriptorsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GpsMessageFormat> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GNSS_MESSAGE_FORMAT, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_GNSS_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        C::extract_count(&deserializer, numDescriptorsOut, numDescriptorsOutMax);
        assert(descriptorsOut || (numDescriptorsOut == 0));
        for(unsigned int i=0; i < *numDescriptorsOut; i++)
            extract(deserializer, descriptorsOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<GpsMessageFormat> saveGpsMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_MESSAGE_FORMAT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GpsMessageFormat> loadGpsMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_MESSAGE_FORMAT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GpsMessageFormat> defaultGpsMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_MESSAGE_FORMAT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const FilterMessageFormat& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.num_descriptors);
        
        for(unsigned int i=0; i < self.num_descriptors; i++)
            insert(serializer, self.descriptors[i]);
        
    }
}
void extract(Serializer& serializer, FilterMessageFormat& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        C::extract_count(&serializer, &self.num_descriptors, sizeof(self.descriptors)/sizeof(self.descriptors[0]));
        for(unsigned int i=0; i < self.num_descriptors; i++)
            extract(serializer, self.descriptors[i]);
        
    }
}

void insert(Serializer& serializer, const FilterMessageFormat::Response& self)
{
    insert(serializer, self.num_descriptors);
    
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
    
}
void extract(Serializer& serializer, FilterMessageFormat::Response& self)
{
    C::extract_count(&serializer, &self.num_descriptors, sizeof(self.descriptors)/sizeof(self.descriptors[0]));
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
    
}

TypedResult<FilterMessageFormat> writeFilterMessageFormat(C::mip_interface& device, uint8_t numDescriptors, const DescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, numDescriptors);
    
    assert(descriptors || (numDescriptors == 0));
    for(unsigned int i=0; i < numDescriptors; i++)
        insert(serializer, descriptors[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FILTER_MESSAGE_FORMAT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<FilterMessageFormat> readFilterMessageFormat(C::mip_interface& device, uint8_t* numDescriptorsOut, uint8_t numDescriptorsOutMax, DescriptorRate* descriptorsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<FilterMessageFormat> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_FILTER_MESSAGE_FORMAT, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_FILTER_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        C::extract_count(&deserializer, numDescriptorsOut, numDescriptorsOutMax);
        assert(descriptorsOut || (numDescriptorsOut == 0));
        for(unsigned int i=0; i < *numDescriptorsOut; i++)
            extract(deserializer, descriptorsOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<FilterMessageFormat> saveFilterMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FILTER_MESSAGE_FORMAT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<FilterMessageFormat> loadFilterMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FILTER_MESSAGE_FORMAT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<FilterMessageFormat> defaultFilterMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FILTER_MESSAGE_FORMAT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const ImuGetBaseRate& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, ImuGetBaseRate& self)
{
    (void)serializer;
    (void)self;
}

void insert(Serializer& serializer, const ImuGetBaseRate::Response& self)
{
    insert(serializer, self.rate);
    
}
void extract(Serializer& serializer, ImuGetBaseRate::Response& self)
{
    extract(serializer, self.rate);
    
}

TypedResult<ImuGetBaseRate> imuGetBaseRate(C::mip_interface& device, uint16_t* rateOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<ImuGetBaseRate> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_IMU_BASE_RATE, NULL, 0, REPLY_IMU_BASE_RATE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(rateOut);
        extract(deserializer, *rateOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const GpsGetBaseRate& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, GpsGetBaseRate& self)
{
    (void)serializer;
    (void)self;
}

void insert(Serializer& serializer, const GpsGetBaseRate::Response& self)
{
    insert(serializer, self.rate);
    
}
void extract(Serializer& serializer, GpsGetBaseRate::Response& self)
{
    extract(serializer, self.rate);
    
}

TypedResult<GpsGetBaseRate> gpsGetBaseRate(C::mip_interface& device, uint16_t* rateOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<GpsGetBaseRate> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_GNSS_BASE_RATE, NULL, 0, REPLY_GNSS_BASE_RATE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(rateOut);
        extract(deserializer, *rateOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const FilterGetBaseRate& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, FilterGetBaseRate& self)
{
    (void)serializer;
    (void)self;
}

void insert(Serializer& serializer, const FilterGetBaseRate::Response& self)
{
    insert(serializer, self.rate);
    
}
void extract(Serializer& serializer, FilterGetBaseRate::Response& self)
{
    extract(serializer, self.rate);
    
}

TypedResult<FilterGetBaseRate> filterGetBaseRate(C::mip_interface& device, uint16_t* rateOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<FilterGetBaseRate> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_FILTER_BASE_RATE, NULL, 0, REPLY_FILTER_BASE_RATE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(rateOut);
        extract(deserializer, *rateOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const PollData& self)
{
    insert(serializer, self.desc_set);
    
    insert(serializer, self.suppress_ack);
    
    insert(serializer, self.num_descriptors);
    
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
    
}
void extract(Serializer& serializer, PollData& self)
{
    extract(serializer, self.desc_set);
    
    extract(serializer, self.suppress_ack);
    
    C::extract_count(&serializer, &self.num_descriptors, sizeof(self.descriptors)/sizeof(self.descriptors[0]));
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
    
}

TypedResult<PollData> pollData(C::mip_interface& device, uint8_t descSet, bool suppressAck, uint8_t numDescriptors, const uint8_t* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, descSet);
    
    insert(serializer, suppressAck);
    
    insert(serializer, numDescriptors);
    
    assert(descriptors || (numDescriptors == 0));
    for(unsigned int i=0; i < numDescriptors; i++)
        insert(serializer, descriptors[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POLL_DATA, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const GetBaseRate& self)
{
    insert(serializer, self.desc_set);
    
}
void extract(Serializer& serializer, GetBaseRate& self)
{
    extract(serializer, self.desc_set);
    
}

void insert(Serializer& serializer, const GetBaseRate::Response& self)
{
    insert(serializer, self.desc_set);
    
    insert(serializer, self.rate);
    
}
void extract(Serializer& serializer, GetBaseRate::Response& self)
{
    extract(serializer, self.desc_set);
    
    extract(serializer, self.rate);
    
}

TypedResult<GetBaseRate> getBaseRate(C::mip_interface& device, uint8_t descSet, uint16_t* rateOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, descSet);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GetBaseRate> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_BASE_RATE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_BASE_RATE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, descSet);
        
        assert(rateOut);
        extract(deserializer, *rateOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const MessageFormat& self)
{
    insert(serializer, self.function);
    
    insert(serializer, self.desc_set);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.num_descriptors);
        
        for(unsigned int i=0; i < self.num_descriptors; i++)
            insert(serializer, self.descriptors[i]);
        
    }
}
void extract(Serializer& serializer, MessageFormat& self)
{
    extract(serializer, self.function);
    
    extract(serializer, self.desc_set);
    
    if( self.function == FunctionSelector::WRITE )
    {
        C::extract_count(&serializer, &self.num_descriptors, sizeof(self.descriptors)/sizeof(self.descriptors[0]));
        for(unsigned int i=0; i < self.num_descriptors; i++)
            extract(serializer, self.descriptors[i]);
        
    }
}

void insert(Serializer& serializer, const MessageFormat::Response& self)
{
    insert(serializer, self.desc_set);
    
    insert(serializer, self.num_descriptors);
    
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
    
}
void extract(Serializer& serializer, MessageFormat::Response& self)
{
    extract(serializer, self.desc_set);
    
    C::extract_count(&serializer, &self.num_descriptors, sizeof(self.descriptors)/sizeof(self.descriptors[0]));
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
    
}

TypedResult<MessageFormat> writeMessageFormat(C::mip_interface& device, uint8_t descSet, uint8_t numDescriptors, const DescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, descSet);
    
    insert(serializer, numDescriptors);
    
    assert(descriptors || (numDescriptors == 0));
    for(unsigned int i=0; i < numDescriptors; i++)
        insert(serializer, descriptors[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MESSAGE_FORMAT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MessageFormat> readMessageFormat(C::mip_interface& device, uint8_t descSet, uint8_t* numDescriptorsOut, uint8_t numDescriptorsOutMax, DescriptorRate* descriptorsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, descSet);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<MessageFormat> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_MESSAGE_FORMAT, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, descSet);
        
        C::extract_count(&deserializer, numDescriptorsOut, numDescriptorsOutMax);
        assert(descriptorsOut || (numDescriptorsOut == 0));
        for(unsigned int i=0; i < *numDescriptorsOut; i++)
            extract(deserializer, descriptorsOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<MessageFormat> saveMessageFormat(C::mip_interface& device, uint8_t descSet)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, descSet);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MESSAGE_FORMAT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MessageFormat> loadMessageFormat(C::mip_interface& device, uint8_t descSet)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, descSet);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MESSAGE_FORMAT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MessageFormat> defaultMessageFormat(C::mip_interface& device, uint8_t descSet)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, descSet);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MESSAGE_FORMAT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const NmeaPollData& self)
{
    insert(serializer, self.suppress_ack);
    
    insert(serializer, self.count);
    
    for(unsigned int i=0; i < self.count; i++)
        insert(serializer, self.format_entries[i]);
    
}
void extract(Serializer& serializer, NmeaPollData& self)
{
    extract(serializer, self.suppress_ack);
    
    C::extract_count(&serializer, &self.count, sizeof(self.format_entries)/sizeof(self.format_entries[0]));
    for(unsigned int i=0; i < self.count; i++)
        extract(serializer, self.format_entries[i]);
    
}

TypedResult<NmeaPollData> nmeaPollData(C::mip_interface& device, bool suppressAck, uint8_t count, const NmeaMessage* formatEntries)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, suppressAck);
    
    insert(serializer, count);
    
    assert(formatEntries || (count == 0));
    for(unsigned int i=0; i < count; i++)
        insert(serializer, formatEntries[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POLL_NMEA_MESSAGE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const NmeaMessageFormat& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.count);
        
        for(unsigned int i=0; i < self.count; i++)
            insert(serializer, self.format_entries[i]);
        
    }
}
void extract(Serializer& serializer, NmeaMessageFormat& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        C::extract_count(&serializer, &self.count, sizeof(self.format_entries)/sizeof(self.format_entries[0]));
        for(unsigned int i=0; i < self.count; i++)
            extract(serializer, self.format_entries[i]);
        
    }
}

void insert(Serializer& serializer, const NmeaMessageFormat::Response& self)
{
    insert(serializer, self.count);
    
    for(unsigned int i=0; i < self.count; i++)
        insert(serializer, self.format_entries[i]);
    
}
void extract(Serializer& serializer, NmeaMessageFormat::Response& self)
{
    C::extract_count(&serializer, &self.count, sizeof(self.format_entries)/sizeof(self.format_entries[0]));
    for(unsigned int i=0; i < self.count; i++)
        extract(serializer, self.format_entries[i]);
    
}

TypedResult<NmeaMessageFormat> writeNmeaMessageFormat(C::mip_interface& device, uint8_t count, const NmeaMessage* formatEntries)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, count);
    
    assert(formatEntries || (count == 0));
    for(unsigned int i=0; i < count; i++)
        insert(serializer, formatEntries[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_NMEA_MESSAGE_FORMAT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<NmeaMessageFormat> readNmeaMessageFormat(C::mip_interface& device, uint8_t* countOut, uint8_t countOutMax, NmeaMessage* formatEntriesOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<NmeaMessageFormat> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_NMEA_MESSAGE_FORMAT, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_NMEA_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        C::extract_count(&deserializer, countOut, countOutMax);
        assert(formatEntriesOut || (countOut == 0));
        for(unsigned int i=0; i < *countOut; i++)
            extract(deserializer, formatEntriesOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<NmeaMessageFormat> saveNmeaMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_NMEA_MESSAGE_FORMAT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<NmeaMessageFormat> loadNmeaMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_NMEA_MESSAGE_FORMAT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<NmeaMessageFormat> defaultNmeaMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_NMEA_MESSAGE_FORMAT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const DeviceSettings& self)
{
    insert(serializer, self.function);
    
}
void extract(Serializer& serializer, DeviceSettings& self)
{
    extract(serializer, self.function);
    
}

TypedResult<DeviceSettings> saveDeviceSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DEVICE_STARTUP_SETTINGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<DeviceSettings> loadDeviceSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DEVICE_STARTUP_SETTINGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<DeviceSettings> defaultDeviceSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DEVICE_STARTUP_SETTINGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const UartBaudrate& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.baud);
        
    }
}
void extract(Serializer& serializer, UartBaudrate& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.baud);
        
    }
}

void insert(Serializer& serializer, const UartBaudrate::Response& self)
{
    insert(serializer, self.baud);
    
}
void extract(Serializer& serializer, UartBaudrate::Response& self)
{
    extract(serializer, self.baud);
    
}

TypedResult<UartBaudrate> writeUartBaudrate(C::mip_interface& device, uint32_t baud)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, baud);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_UART_BAUDRATE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<UartBaudrate> readUartBaudrate(C::mip_interface& device, uint32_t* baudOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<UartBaudrate> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_UART_BAUDRATE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_UART_BAUDRATE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(baudOut);
        extract(deserializer, *baudOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<UartBaudrate> saveUartBaudrate(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_UART_BAUDRATE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<UartBaudrate> loadUartBaudrate(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_UART_BAUDRATE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<UartBaudrate> defaultUartBaudrate(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_UART_BAUDRATE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const FactoryStreaming& self)
{
    insert(serializer, self.action);
    
    insert(serializer, self.reserved);
    
}
void extract(Serializer& serializer, FactoryStreaming& self)
{
    extract(serializer, self.action);
    
    extract(serializer, self.reserved);
    
}

TypedResult<FactoryStreaming> factoryStreaming(C::mip_interface& device, FactoryStreaming::Action action, uint8_t reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, action);
    
    insert(serializer, reserved);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONFIGURE_FACTORY_STREAMING, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const DatastreamControl& self)
{
    insert(serializer, self.function);
    
    insert(serializer, self.desc_set);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable);
        
    }
}
void extract(Serializer& serializer, DatastreamControl& self)
{
    extract(serializer, self.function);
    
    extract(serializer, self.desc_set);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable);
        
    }
}

void insert(Serializer& serializer, const DatastreamControl::Response& self)
{
    insert(serializer, self.desc_set);
    
    insert(serializer, self.enabled);
    
}
void extract(Serializer& serializer, DatastreamControl::Response& self)
{
    extract(serializer, self.desc_set);
    
    extract(serializer, self.enabled);
    
}

TypedResult<DatastreamControl> writeDatastreamControl(C::mip_interface& device, uint8_t descSet, bool enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, descSet);
    
    insert(serializer, enable);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONTROL_DATA_STREAM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<DatastreamControl> readDatastreamControl(C::mip_interface& device, uint8_t descSet, bool* enabledOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, descSet);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<DatastreamControl> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_CONTROL_DATA_STREAM, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_DATASTREAM_ENABLE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, descSet);
        
        assert(enabledOut);
        extract(deserializer, *enabledOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<DatastreamControl> saveDatastreamControl(C::mip_interface& device, uint8_t descSet)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, descSet);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONTROL_DATA_STREAM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<DatastreamControl> loadDatastreamControl(C::mip_interface& device, uint8_t descSet)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, descSet);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONTROL_DATA_STREAM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<DatastreamControl> defaultDatastreamControl(C::mip_interface& device, uint8_t descSet)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, descSet);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONTROL_DATA_STREAM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const ConstellationSettings& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.max_channels);
        
        insert(serializer, self.config_count);
        
        for(unsigned int i=0; i < self.config_count; i++)
            insert(serializer, self.settings[i]);
        
    }
}
void extract(Serializer& serializer, ConstellationSettings& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.max_channels);
        
        C::extract_count(&serializer, &self.config_count, sizeof(self.settings)/sizeof(self.settings[0]));
        for(unsigned int i=0; i < self.config_count; i++)
            extract(serializer, self.settings[i]);
        
    }
}

void insert(Serializer& serializer, const ConstellationSettings::Response& self)
{
    insert(serializer, self.max_channels_available);
    
    insert(serializer, self.max_channels_use);
    
    insert(serializer, self.config_count);
    
    for(unsigned int i=0; i < self.config_count; i++)
        insert(serializer, self.settings[i]);
    
}
void extract(Serializer& serializer, ConstellationSettings::Response& self)
{
    extract(serializer, self.max_channels_available);
    
    extract(serializer, self.max_channels_use);
    
    C::extract_count(&serializer, &self.config_count, sizeof(self.settings)/sizeof(self.settings[0]));
    for(unsigned int i=0; i < self.config_count; i++)
        extract(serializer, self.settings[i]);
    
}

void insert(Serializer& serializer, const ConstellationSettings::Settings& self)
{
    insert(serializer, self.constellation_id);
    
    insert(serializer, self.enable);
    
    insert(serializer, self.reserved_channels);
    
    insert(serializer, self.max_channels);
    
    insert(serializer, self.option_flags);
    
}
void extract(Serializer& serializer, ConstellationSettings::Settings& self)
{
    extract(serializer, self.constellation_id);
    
    extract(serializer, self.enable);
    
    extract(serializer, self.reserved_channels);
    
    extract(serializer, self.max_channels);
    
    extract(serializer, self.option_flags);
    
}

TypedResult<ConstellationSettings> writeConstellationSettings(C::mip_interface& device, uint16_t maxChannels, uint8_t configCount, const ConstellationSettings::Settings* settings)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, maxChannels);
    
    insert(serializer, configCount);
    
    assert(settings || (configCount == 0));
    for(unsigned int i=0; i < configCount; i++)
        insert(serializer, settings[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_CONSTELLATION_SETTINGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<ConstellationSettings> readConstellationSettings(C::mip_interface& device, uint16_t* maxChannelsAvailableOut, uint16_t* maxChannelsUseOut, uint8_t* configCountOut, uint8_t configCountOutMax, ConstellationSettings::Settings* settingsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<ConstellationSettings> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GNSS_CONSTELLATION_SETTINGS, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_GNSS_CONSTELLATION_SETTINGS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(maxChannelsAvailableOut);
        extract(deserializer, *maxChannelsAvailableOut);
        
        assert(maxChannelsUseOut);
        extract(deserializer, *maxChannelsUseOut);
        
        C::extract_count(&deserializer, configCountOut, configCountOutMax);
        assert(settingsOut || (configCountOut == 0));
        for(unsigned int i=0; i < *configCountOut; i++)
            extract(deserializer, settingsOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<ConstellationSettings> saveConstellationSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_CONSTELLATION_SETTINGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<ConstellationSettings> loadConstellationSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_CONSTELLATION_SETTINGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<ConstellationSettings> defaultConstellationSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_CONSTELLATION_SETTINGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const GnssSbasSettings& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable_sbas);
        
        insert(serializer, self.sbas_options);
        
        insert(serializer, self.num_included_prns);
        
        for(unsigned int i=0; i < self.num_included_prns; i++)
            insert(serializer, self.included_prns[i]);
        
    }
}
void extract(Serializer& serializer, GnssSbasSettings& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable_sbas);
        
        extract(serializer, self.sbas_options);
        
        C::extract_count(&serializer, &self.num_included_prns, sizeof(self.included_prns)/sizeof(self.included_prns[0]));
        for(unsigned int i=0; i < self.num_included_prns; i++)
            extract(serializer, self.included_prns[i]);
        
    }
}

void insert(Serializer& serializer, const GnssSbasSettings::Response& self)
{
    insert(serializer, self.enable_sbas);
    
    insert(serializer, self.sbas_options);
    
    insert(serializer, self.num_included_prns);
    
    for(unsigned int i=0; i < self.num_included_prns; i++)
        insert(serializer, self.included_prns[i]);
    
}
void extract(Serializer& serializer, GnssSbasSettings::Response& self)
{
    extract(serializer, self.enable_sbas);
    
    extract(serializer, self.sbas_options);
    
    C::extract_count(&serializer, &self.num_included_prns, sizeof(self.included_prns)/sizeof(self.included_prns[0]));
    for(unsigned int i=0; i < self.num_included_prns; i++)
        extract(serializer, self.included_prns[i]);
    
}

TypedResult<GnssSbasSettings> writeGnssSbasSettings(C::mip_interface& device, uint8_t enableSbas, GnssSbasSettings::SBASOptions sbasOptions, uint8_t numIncludedPrns, const uint16_t* includedPrns)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enableSbas);
    
    insert(serializer, sbasOptions);
    
    insert(serializer, numIncludedPrns);
    
    assert(includedPrns || (numIncludedPrns == 0));
    for(unsigned int i=0; i < numIncludedPrns; i++)
        insert(serializer, includedPrns[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SBAS_SETTINGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GnssSbasSettings> readGnssSbasSettings(C::mip_interface& device, uint8_t* enableSbasOut, GnssSbasSettings::SBASOptions* sbasOptionsOut, uint8_t* numIncludedPrnsOut, uint8_t numIncludedPrnsOutMax, uint16_t* includedPrnsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GnssSbasSettings> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GNSS_SBAS_SETTINGS, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_GNSS_SBAS_SETTINGS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableSbasOut);
        extract(deserializer, *enableSbasOut);
        
        assert(sbasOptionsOut);
        extract(deserializer, *sbasOptionsOut);
        
        C::extract_count(&deserializer, numIncludedPrnsOut, numIncludedPrnsOutMax);
        assert(includedPrnsOut || (numIncludedPrnsOut == 0));
        for(unsigned int i=0; i < *numIncludedPrnsOut; i++)
            extract(deserializer, includedPrnsOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<GnssSbasSettings> saveGnssSbasSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SBAS_SETTINGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GnssSbasSettings> loadGnssSbasSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SBAS_SETTINGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GnssSbasSettings> defaultGnssSbasSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SBAS_SETTINGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const GnssAssistedFix& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.option);
        
        insert(serializer, self.flags);
        
    }
}
void extract(Serializer& serializer, GnssAssistedFix& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.option);
        
        extract(serializer, self.flags);
        
    }
}

void insert(Serializer& serializer, const GnssAssistedFix::Response& self)
{
    insert(serializer, self.option);
    
    insert(serializer, self.flags);
    
}
void extract(Serializer& serializer, GnssAssistedFix::Response& self)
{
    extract(serializer, self.option);
    
    extract(serializer, self.flags);
    
}

TypedResult<GnssAssistedFix> writeGnssAssistedFix(C::mip_interface& device, GnssAssistedFix::AssistedFixOption option, uint8_t flags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, option);
    
    insert(serializer, flags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_ASSISTED_FIX_SETTINGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GnssAssistedFix> readGnssAssistedFix(C::mip_interface& device, GnssAssistedFix::AssistedFixOption* optionOut, uint8_t* flagsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GnssAssistedFix> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GNSS_ASSISTED_FIX_SETTINGS, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_GNSS_ASSISTED_FIX_SETTINGS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(optionOut);
        extract(deserializer, *optionOut);
        
        assert(flagsOut);
        extract(deserializer, *flagsOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<GnssAssistedFix> saveGnssAssistedFix(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_ASSISTED_FIX_SETTINGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GnssAssistedFix> loadGnssAssistedFix(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_ASSISTED_FIX_SETTINGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GnssAssistedFix> defaultGnssAssistedFix(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_ASSISTED_FIX_SETTINGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const GnssTimeAssistance& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.tow);
        
        insert(serializer, self.week_number);
        
        insert(serializer, self.accuracy);
        
    }
}
void extract(Serializer& serializer, GnssTimeAssistance& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.tow);
        
        extract(serializer, self.week_number);
        
        extract(serializer, self.accuracy);
        
    }
}

void insert(Serializer& serializer, const GnssTimeAssistance::Response& self)
{
    insert(serializer, self.tow);
    
    insert(serializer, self.week_number);
    
    insert(serializer, self.accuracy);
    
}
void extract(Serializer& serializer, GnssTimeAssistance::Response& self)
{
    extract(serializer, self.tow);
    
    extract(serializer, self.week_number);
    
    extract(serializer, self.accuracy);
    
}

TypedResult<GnssTimeAssistance> writeGnssTimeAssistance(C::mip_interface& device, double tow, uint16_t weekNumber, float accuracy)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, tow);
    
    insert(serializer, weekNumber);
    
    insert(serializer, accuracy);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_TIME_ASSISTANCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GnssTimeAssistance> readGnssTimeAssistance(C::mip_interface& device, double* towOut, uint16_t* weekNumberOut, float* accuracyOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GnssTimeAssistance> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GNSS_TIME_ASSISTANCE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_GNSS_TIME_ASSISTANCE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(towOut);
        extract(deserializer, *towOut);
        
        assert(weekNumberOut);
        extract(deserializer, *weekNumberOut);
        
        assert(accuracyOut);
        extract(deserializer, *accuracyOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const ImuLowpassFilter& self)
{
    insert(serializer, self.function);
    
    insert(serializer, self.target_descriptor);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable);
        
        insert(serializer, self.manual);
        
        insert(serializer, self.frequency);
        
        insert(serializer, self.reserved);
        
    }
}
void extract(Serializer& serializer, ImuLowpassFilter& self)
{
    extract(serializer, self.function);
    
    extract(serializer, self.target_descriptor);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable);
        
        extract(serializer, self.manual);
        
        extract(serializer, self.frequency);
        
        extract(serializer, self.reserved);
        
    }
}

void insert(Serializer& serializer, const ImuLowpassFilter::Response& self)
{
    insert(serializer, self.target_descriptor);
    
    insert(serializer, self.enable);
    
    insert(serializer, self.manual);
    
    insert(serializer, self.frequency);
    
    insert(serializer, self.reserved);
    
}
void extract(Serializer& serializer, ImuLowpassFilter::Response& self)
{
    extract(serializer, self.target_descriptor);
    
    extract(serializer, self.enable);
    
    extract(serializer, self.manual);
    
    extract(serializer, self.frequency);
    
    extract(serializer, self.reserved);
    
}

TypedResult<ImuLowpassFilter> writeImuLowpassFilter(C::mip_interface& device, uint8_t targetDescriptor, bool enable, bool manual, uint16_t frequency, uint8_t reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, targetDescriptor);
    
    insert(serializer, enable);
    
    insert(serializer, manual);
    
    insert(serializer, frequency);
    
    insert(serializer, reserved);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_IMU_LOWPASS_FILTER, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<ImuLowpassFilter> readImuLowpassFilter(C::mip_interface& device, uint8_t targetDescriptor, bool* enableOut, bool* manualOut, uint16_t* frequencyOut, uint8_t* reservedOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, targetDescriptor);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<ImuLowpassFilter> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_IMU_LOWPASS_FILTER, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ADVANCED_DATA_FILTER, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, targetDescriptor);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        assert(manualOut);
        extract(deserializer, *manualOut);
        
        assert(frequencyOut);
        extract(deserializer, *frequencyOut);
        
        assert(reservedOut);
        extract(deserializer, *reservedOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<ImuLowpassFilter> saveImuLowpassFilter(C::mip_interface& device, uint8_t targetDescriptor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, targetDescriptor);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_IMU_LOWPASS_FILTER, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<ImuLowpassFilter> loadImuLowpassFilter(C::mip_interface& device, uint8_t targetDescriptor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, targetDescriptor);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_IMU_LOWPASS_FILTER, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<ImuLowpassFilter> defaultImuLowpassFilter(C::mip_interface& device, uint8_t targetDescriptor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, targetDescriptor);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_IMU_LOWPASS_FILTER, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const PpsSource& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.source);
        
    }
}
void extract(Serializer& serializer, PpsSource& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.source);
        
    }
}

void insert(Serializer& serializer, const PpsSource::Response& self)
{
    insert(serializer, self.source);
    
}
void extract(Serializer& serializer, PpsSource::Response& self)
{
    extract(serializer, self.source);
    
}

TypedResult<PpsSource> writePpsSource(C::mip_interface& device, PpsSource::Source source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, source);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PPS_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<PpsSource> readPpsSource(C::mip_interface& device, PpsSource::Source* sourceOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<PpsSource> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_PPS_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_PPS_SOURCE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(sourceOut);
        extract(deserializer, *sourceOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<PpsSource> savePpsSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PPS_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<PpsSource> loadPpsSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PPS_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<PpsSource> defaultPpsSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PPS_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const GpioConfig& self)
{
    insert(serializer, self.function);
    
    insert(serializer, self.pin);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.feature);
        
        insert(serializer, self.behavior);
        
        insert(serializer, self.pin_mode);
        
    }
}
void extract(Serializer& serializer, GpioConfig& self)
{
    extract(serializer, self.function);
    
    extract(serializer, self.pin);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.feature);
        
        extract(serializer, self.behavior);
        
        extract(serializer, self.pin_mode);
        
    }
}

void insert(Serializer& serializer, const GpioConfig::Response& self)
{
    insert(serializer, self.pin);
    
    insert(serializer, self.feature);
    
    insert(serializer, self.behavior);
    
    insert(serializer, self.pin_mode);
    
}
void extract(Serializer& serializer, GpioConfig::Response& self)
{
    extract(serializer, self.pin);
    
    extract(serializer, self.feature);
    
    extract(serializer, self.behavior);
    
    extract(serializer, self.pin_mode);
    
}

TypedResult<GpioConfig> writeGpioConfig(C::mip_interface& device, uint8_t pin, GpioConfig::Feature feature, GpioConfig::Behavior behavior, GpioConfig::PinMode pinMode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, pin);
    
    insert(serializer, feature);
    
    insert(serializer, behavior);
    
    insert(serializer, pinMode);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GPIO_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GpioConfig> readGpioConfig(C::mip_interface& device, uint8_t pin, GpioConfig::Feature* featureOut, GpioConfig::Behavior* behaviorOut, GpioConfig::PinMode* pinModeOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, pin);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GpioConfig> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GPIO_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_GPIO_CONFIG, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, pin);
        
        assert(featureOut);
        extract(deserializer, *featureOut);
        
        assert(behaviorOut);
        extract(deserializer, *behaviorOut);
        
        assert(pinModeOut);
        extract(deserializer, *pinModeOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<GpioConfig> saveGpioConfig(C::mip_interface& device, uint8_t pin)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, pin);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GPIO_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GpioConfig> loadGpioConfig(C::mip_interface& device, uint8_t pin)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, pin);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GPIO_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GpioConfig> defaultGpioConfig(C::mip_interface& device, uint8_t pin)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, pin);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GPIO_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const GpioState& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE || self.function == FunctionSelector::READ )
    {
        insert(serializer, self.pin);
        
    }
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.state);
        
    }
}
void extract(Serializer& serializer, GpioState& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE || self.function == FunctionSelector::READ )
    {
        extract(serializer, self.pin);
        
    }
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.state);
        
    }
}

void insert(Serializer& serializer, const GpioState::Response& self)
{
    insert(serializer, self.pin);
    
    insert(serializer, self.state);
    
}
void extract(Serializer& serializer, GpioState::Response& self)
{
    extract(serializer, self.pin);
    
    extract(serializer, self.state);
    
}

TypedResult<GpioState> writeGpioState(C::mip_interface& device, uint8_t pin, bool state)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, pin);
    
    insert(serializer, state);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GPIO_STATE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GpioState> readGpioState(C::mip_interface& device, uint8_t pin, bool* stateOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, pin);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GpioState> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GPIO_STATE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_GPIO_STATE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, pin);
        
        assert(stateOut);
        extract(deserializer, *stateOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const Odometer& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.mode);
        
        insert(serializer, self.scaling);
        
        insert(serializer, self.uncertainty);
        
    }
}
void extract(Serializer& serializer, Odometer& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.mode);
        
        extract(serializer, self.scaling);
        
        extract(serializer, self.uncertainty);
        
    }
}

void insert(Serializer& serializer, const Odometer::Response& self)
{
    insert(serializer, self.mode);
    
    insert(serializer, self.scaling);
    
    insert(serializer, self.uncertainty);
    
}
void extract(Serializer& serializer, Odometer::Response& self)
{
    extract(serializer, self.mode);
    
    extract(serializer, self.scaling);
    
    extract(serializer, self.uncertainty);
    
}

TypedResult<Odometer> writeOdometer(C::mip_interface& device, Odometer::Mode mode, float scaling, float uncertainty)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, mode);
    
    insert(serializer, scaling);
    
    insert(serializer, uncertainty);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ODOMETER_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<Odometer> readOdometer(C::mip_interface& device, Odometer::Mode* modeOut, float* scalingOut, float* uncertaintyOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<Odometer> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ODOMETER_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ODOMETER_CONFIG, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(modeOut);
        extract(deserializer, *modeOut);
        
        assert(scalingOut);
        extract(deserializer, *scalingOut);
        
        assert(uncertaintyOut);
        extract(deserializer, *uncertaintyOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<Odometer> saveOdometer(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ODOMETER_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<Odometer> loadOdometer(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ODOMETER_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<Odometer> defaultOdometer(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ODOMETER_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const GetEventSupport& self)
{
    insert(serializer, self.query);
    
}
void extract(Serializer& serializer, GetEventSupport& self)
{
    extract(serializer, self.query);
    
}

void insert(Serializer& serializer, const GetEventSupport::Response& self)
{
    insert(serializer, self.query);
    
    insert(serializer, self.max_instances);
    
    insert(serializer, self.num_entries);
    
    for(unsigned int i=0; i < self.num_entries; i++)
        insert(serializer, self.entries[i]);
    
}
void extract(Serializer& serializer, GetEventSupport::Response& self)
{
    extract(serializer, self.query);
    
    extract(serializer, self.max_instances);
    
    C::extract_count(&serializer, &self.num_entries, sizeof(self.entries)/sizeof(self.entries[0]));
    for(unsigned int i=0; i < self.num_entries; i++)
        extract(serializer, self.entries[i]);
    
}

void insert(Serializer& serializer, const GetEventSupport::Info& self)
{
    insert(serializer, self.type);
    
    insert(serializer, self.count);
    
}
void extract(Serializer& serializer, GetEventSupport::Info& self)
{
    extract(serializer, self.type);
    
    extract(serializer, self.count);
    
}

TypedResult<GetEventSupport> getEventSupport(C::mip_interface& device, GetEventSupport::Query query, uint8_t* maxInstancesOut, uint8_t* numEntriesOut, uint8_t numEntriesOutMax, GetEventSupport::Info* entriesOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, query);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GetEventSupport> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_EVENT_SUPPORT, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_EVENT_SUPPORT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, query);
        
        assert(maxInstancesOut);
        extract(deserializer, *maxInstancesOut);
        
        C::extract_count(&deserializer, numEntriesOut, numEntriesOutMax);
        assert(entriesOut || (numEntriesOut == 0));
        for(unsigned int i=0; i < *numEntriesOut; i++)
            extract(deserializer, entriesOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const EventControl& self)
{
    insert(serializer, self.function);
    
    insert(serializer, self.instance);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.mode);
        
    }
}
void extract(Serializer& serializer, EventControl& self)
{
    extract(serializer, self.function);
    
    extract(serializer, self.instance);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.mode);
        
    }
}

void insert(Serializer& serializer, const EventControl::Response& self)
{
    insert(serializer, self.instance);
    
    insert(serializer, self.mode);
    
}
void extract(Serializer& serializer, EventControl::Response& self)
{
    extract(serializer, self.instance);
    
    extract(serializer, self.mode);
    
}

TypedResult<EventControl> writeEventControl(C::mip_interface& device, uint8_t instance, EventControl::Mode mode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, instance);
    
    insert(serializer, mode);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<EventControl> readEventControl(C::mip_interface& device, uint8_t instance, EventControl::Mode* modeOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, instance);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<EventControl> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_EVENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_EVENT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, instance);
        
        assert(modeOut);
        extract(deserializer, *modeOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<EventControl> saveEventControl(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<EventControl> loadEventControl(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<EventControl> defaultEventControl(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const GetEventTriggerStatus& self)
{
    insert(serializer, self.requested_count);
    
    for(unsigned int i=0; i < self.requested_count; i++)
        insert(serializer, self.requested_instances[i]);
    
}
void extract(Serializer& serializer, GetEventTriggerStatus& self)
{
    C::extract_count(&serializer, &self.requested_count, sizeof(self.requested_instances)/sizeof(self.requested_instances[0]));
    for(unsigned int i=0; i < self.requested_count; i++)
        extract(serializer, self.requested_instances[i]);
    
}

void insert(Serializer& serializer, const GetEventTriggerStatus::Response& self)
{
    insert(serializer, self.count);
    
    for(unsigned int i=0; i < self.count; i++)
        insert(serializer, self.triggers[i]);
    
}
void extract(Serializer& serializer, GetEventTriggerStatus::Response& self)
{
    C::extract_count(&serializer, &self.count, sizeof(self.triggers)/sizeof(self.triggers[0]));
    for(unsigned int i=0; i < self.count; i++)
        extract(serializer, self.triggers[i]);
    
}

void insert(Serializer& serializer, const GetEventTriggerStatus::Entry& self)
{
    insert(serializer, self.type);
    
    insert(serializer, self.status);
    
}
void extract(Serializer& serializer, GetEventTriggerStatus::Entry& self)
{
    extract(serializer, self.type);
    
    extract(serializer, self.status);
    
}

TypedResult<GetEventTriggerStatus> getEventTriggerStatus(C::mip_interface& device, uint8_t requestedCount, const uint8_t* requestedInstances, uint8_t* countOut, uint8_t countOutMax, GetEventTriggerStatus::Entry* triggersOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, requestedCount);
    
    assert(requestedInstances || (requestedCount == 0));
    for(unsigned int i=0; i < requestedCount; i++)
        insert(serializer, requestedInstances[i]);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GetEventTriggerStatus> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_EVENT_TRIGGER_STATUS, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_EVENT_TRIGGER_STATUS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        C::extract_count(&deserializer, countOut, countOutMax);
        assert(triggersOut || (countOut == 0));
        for(unsigned int i=0; i < *countOut; i++)
            extract(deserializer, triggersOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const GetEventActionStatus& self)
{
    insert(serializer, self.requested_count);
    
    for(unsigned int i=0; i < self.requested_count; i++)
        insert(serializer, self.requested_instances[i]);
    
}
void extract(Serializer& serializer, GetEventActionStatus& self)
{
    C::extract_count(&serializer, &self.requested_count, sizeof(self.requested_instances)/sizeof(self.requested_instances[0]));
    for(unsigned int i=0; i < self.requested_count; i++)
        extract(serializer, self.requested_instances[i]);
    
}

void insert(Serializer& serializer, const GetEventActionStatus::Response& self)
{
    insert(serializer, self.count);
    
    for(unsigned int i=0; i < self.count; i++)
        insert(serializer, self.actions[i]);
    
}
void extract(Serializer& serializer, GetEventActionStatus::Response& self)
{
    C::extract_count(&serializer, &self.count, sizeof(self.actions)/sizeof(self.actions[0]));
    for(unsigned int i=0; i < self.count; i++)
        extract(serializer, self.actions[i]);
    
}

void insert(Serializer& serializer, const GetEventActionStatus::Entry& self)
{
    insert(serializer, self.action_type);
    
    insert(serializer, self.trigger_id);
    
}
void extract(Serializer& serializer, GetEventActionStatus::Entry& self)
{
    extract(serializer, self.action_type);
    
    extract(serializer, self.trigger_id);
    
}

TypedResult<GetEventActionStatus> getEventActionStatus(C::mip_interface& device, uint8_t requestedCount, const uint8_t* requestedInstances, uint8_t* countOut, uint8_t countOutMax, GetEventActionStatus::Entry* actionsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, requestedCount);
    
    assert(requestedInstances || (requestedCount == 0));
    for(unsigned int i=0; i < requestedCount; i++)
        insert(serializer, requestedInstances[i]);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GetEventActionStatus> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_EVENT_ACTION_STATUS, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_EVENT_ACTION_STATUS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        C::extract_count(&deserializer, countOut, countOutMax);
        assert(actionsOut || (countOut == 0));
        for(unsigned int i=0; i < *countOut; i++)
            extract(deserializer, actionsOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const EventTrigger& self)
{
    insert(serializer, self.function);
    
    insert(serializer, self.instance);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.type);
        
        if( self.type == EventTrigger::Type::GPIO )
        {
            insert(serializer, self.parameters.gpio);
            
        }
        if( self.type == EventTrigger::Type::THRESHOLD )
        {
            insert(serializer, self.parameters.threshold);
            
        }
        if( self.type == EventTrigger::Type::COMBINATION )
        {
            insert(serializer, self.parameters.combination);
            
        }
    }
}
void extract(Serializer& serializer, EventTrigger& self)
{
    extract(serializer, self.function);
    
    extract(serializer, self.instance);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.type);
        
        if( self.type == EventTrigger::Type::GPIO )
        {
            extract(serializer, self.parameters.gpio);
            
        }
        if( self.type == EventTrigger::Type::THRESHOLD )
        {
            extract(serializer, self.parameters.threshold);
            
        }
        if( self.type == EventTrigger::Type::COMBINATION )
        {
            extract(serializer, self.parameters.combination);
            
        }
    }
}

void insert(Serializer& serializer, const EventTrigger::Response& self)
{
    insert(serializer, self.instance);
    
    insert(serializer, self.type);
    
    if( self.type == EventTrigger::Type::GPIO )
    {
        insert(serializer, self.parameters.gpio);
        
    }
    if( self.type == EventTrigger::Type::THRESHOLD )
    {
        insert(serializer, self.parameters.threshold);
        
    }
    if( self.type == EventTrigger::Type::COMBINATION )
    {
        insert(serializer, self.parameters.combination);
        
    }
}
void extract(Serializer& serializer, EventTrigger::Response& self)
{
    extract(serializer, self.instance);
    
    extract(serializer, self.type);
    
    if( self.type == EventTrigger::Type::GPIO )
    {
        extract(serializer, self.parameters.gpio);
        
    }
    if( self.type == EventTrigger::Type::THRESHOLD )
    {
        extract(serializer, self.parameters.threshold);
        
    }
    if( self.type == EventTrigger::Type::COMBINATION )
    {
        extract(serializer, self.parameters.combination);
        
    }
}

void insert(Serializer& serializer, const EventTrigger::GpioParams& self)
{
    insert(serializer, self.pin);
    
    insert(serializer, self.mode);
    
}
void extract(Serializer& serializer, EventTrigger::GpioParams& self)
{
    extract(serializer, self.pin);
    
    extract(serializer, self.mode);
    
}

void insert(Serializer& serializer, const EventTrigger::ThresholdParams& self)
{
    insert(serializer, self.desc_set);
    
    insert(serializer, self.field_desc);
    
    insert(serializer, self.param_id);
    
    insert(serializer, self.type);
    
    if( self.type == EventTrigger::ThresholdParams::Type::WINDOW )
    {
        insert(serializer, self.low_thres);
        
    }
    if( self.type == EventTrigger::ThresholdParams::Type::INTERVAL )
    {
        insert(serializer, self.int_thres);
        
    }
    if( self.type == EventTrigger::ThresholdParams::Type::WINDOW )
    {
        insert(serializer, self.high_thres);
        
    }
    if( self.type == EventTrigger::ThresholdParams::Type::INTERVAL )
    {
        insert(serializer, self.interval);
        
    }
}
void extract(Serializer& serializer, EventTrigger::ThresholdParams& self)
{
    extract(serializer, self.desc_set);
    
    extract(serializer, self.field_desc);
    
    extract(serializer, self.param_id);
    
    extract(serializer, self.type);
    
    if( self.type == EventTrigger::ThresholdParams::Type::WINDOW )
    {
        extract(serializer, self.low_thres);
        
    }
    if( self.type == EventTrigger::ThresholdParams::Type::INTERVAL )
    {
        extract(serializer, self.int_thres);
        
    }
    if( self.type == EventTrigger::ThresholdParams::Type::WINDOW )
    {
        extract(serializer, self.high_thres);
        
    }
    if( self.type == EventTrigger::ThresholdParams::Type::INTERVAL )
    {
        extract(serializer, self.interval);
        
    }
}

void insert(Serializer& serializer, const EventTrigger::CombinationParams& self)
{
    insert(serializer, self.logic_table);
    
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.input_triggers[i]);
    
}
void extract(Serializer& serializer, EventTrigger::CombinationParams& self)
{
    extract(serializer, self.logic_table);
    
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.input_triggers[i]);
    
}

TypedResult<EventTrigger> writeEventTrigger(C::mip_interface& device, uint8_t instance, EventTrigger::Type type, const EventTrigger::Parameters& parameters)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, instance);
    
    insert(serializer, type);
    
    if( type == EventTrigger::Type::GPIO )
    {
        insert(serializer, parameters.gpio);
        
    }
    if( type == EventTrigger::Type::THRESHOLD )
    {
        insert(serializer, parameters.threshold);
        
    }
    if( type == EventTrigger::Type::COMBINATION )
    {
        insert(serializer, parameters.combination);
        
    }
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_TRIGGER_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<EventTrigger> readEventTrigger(C::mip_interface& device, uint8_t instance, EventTrigger::Type* typeOut, EventTrigger::Parameters* parametersOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, instance);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<EventTrigger> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_EVENT_TRIGGER_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_EVENT_TRIGGER_CONFIG, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, instance);
        
        assert(typeOut);
        extract(deserializer, *typeOut);
        
        if( *typeOut == EventTrigger::Type::GPIO )
        {
            extract(deserializer, parametersOut->gpio);
            
        }
        if( *typeOut == EventTrigger::Type::THRESHOLD )
        {
            extract(deserializer, parametersOut->threshold);
            
        }
        if( *typeOut == EventTrigger::Type::COMBINATION )
        {
            extract(deserializer, parametersOut->combination);
            
        }
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<EventTrigger> saveEventTrigger(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_TRIGGER_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<EventTrigger> loadEventTrigger(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_TRIGGER_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<EventTrigger> defaultEventTrigger(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_TRIGGER_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const EventAction& self)
{
    insert(serializer, self.function);
    
    insert(serializer, self.instance);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.trigger);
        
        insert(serializer, self.type);
        
        if( self.type == EventAction::Type::GPIO )
        {
            insert(serializer, self.parameters.gpio);
            
        }
        if( self.type == EventAction::Type::MESSAGE )
        {
            insert(serializer, self.parameters.message);
            
        }
    }
}
void extract(Serializer& serializer, EventAction& self)
{
    extract(serializer, self.function);
    
    extract(serializer, self.instance);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.trigger);
        
        extract(serializer, self.type);
        
        if( self.type == EventAction::Type::GPIO )
        {
            extract(serializer, self.parameters.gpio);
            
        }
        if( self.type == EventAction::Type::MESSAGE )
        {
            extract(serializer, self.parameters.message);
            
        }
    }
}

void insert(Serializer& serializer, const EventAction::Response& self)
{
    insert(serializer, self.instance);
    
    insert(serializer, self.trigger);
    
    insert(serializer, self.type);
    
    if( self.type == EventAction::Type::GPIO )
    {
        insert(serializer, self.parameters.gpio);
        
    }
    if( self.type == EventAction::Type::MESSAGE )
    {
        insert(serializer, self.parameters.message);
        
    }
}
void extract(Serializer& serializer, EventAction::Response& self)
{
    extract(serializer, self.instance);
    
    extract(serializer, self.trigger);
    
    extract(serializer, self.type);
    
    if( self.type == EventAction::Type::GPIO )
    {
        extract(serializer, self.parameters.gpio);
        
    }
    if( self.type == EventAction::Type::MESSAGE )
    {
        extract(serializer, self.parameters.message);
        
    }
}

void insert(Serializer& serializer, const EventAction::GpioParams& self)
{
    insert(serializer, self.pin);
    
    insert(serializer, self.mode);
    
}
void extract(Serializer& serializer, EventAction::GpioParams& self)
{
    extract(serializer, self.pin);
    
    extract(serializer, self.mode);
    
}

void insert(Serializer& serializer, const EventAction::MessageParams& self)
{
    insert(serializer, self.desc_set);
    
    insert(serializer, self.decimation);
    
    insert(serializer, self.num_fields);
    
    for(unsigned int i=0; i < self.num_fields; i++)
        insert(serializer, self.descriptors[i]);
    
}
void extract(Serializer& serializer, EventAction::MessageParams& self)
{
    extract(serializer, self.desc_set);
    
    extract(serializer, self.decimation);
    
    C::extract_count(&serializer, &self.num_fields, sizeof(self.descriptors)/sizeof(self.descriptors[0]));
    for(unsigned int i=0; i < self.num_fields; i++)
        extract(serializer, self.descriptors[i]);
    
}

TypedResult<EventAction> writeEventAction(C::mip_interface& device, uint8_t instance, uint8_t trigger, EventAction::Type type, const EventAction::Parameters& parameters)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, instance);
    
    insert(serializer, trigger);
    
    insert(serializer, type);
    
    if( type == EventAction::Type::GPIO )
    {
        insert(serializer, parameters.gpio);
        
    }
    if( type == EventAction::Type::MESSAGE )
    {
        insert(serializer, parameters.message);
        
    }
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_ACTION_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<EventAction> readEventAction(C::mip_interface& device, uint8_t instance, uint8_t* triggerOut, EventAction::Type* typeOut, EventAction::Parameters* parametersOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, instance);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<EventAction> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_EVENT_ACTION_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_EVENT_ACTION_CONFIG, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, instance);
        
        assert(triggerOut);
        extract(deserializer, *triggerOut);
        
        assert(typeOut);
        extract(deserializer, *typeOut);
        
        if( *typeOut == EventAction::Type::GPIO )
        {
            extract(deserializer, parametersOut->gpio);
            
        }
        if( *typeOut == EventAction::Type::MESSAGE )
        {
            extract(deserializer, parametersOut->message);
            
        }
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<EventAction> saveEventAction(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_ACTION_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<EventAction> loadEventAction(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_ACTION_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<EventAction> defaultEventAction(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_ACTION_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const AccelBias& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.bias[i]);
        
    }
}
void extract(Serializer& serializer, AccelBias& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.bias[i]);
        
    }
}

void insert(Serializer& serializer, const AccelBias::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.bias[i]);
    
}
void extract(Serializer& serializer, AccelBias::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.bias[i]);
    
}

TypedResult<AccelBias> writeAccelBias(C::mip_interface& device, const float* bias)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(bias || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, bias[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AccelBias> readAccelBias(C::mip_interface& device, float* biasOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<AccelBias> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ACCEL_BIAS_VECTOR, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(biasOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, biasOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<AccelBias> saveAccelBias(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AccelBias> loadAccelBias(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AccelBias> defaultAccelBias(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const GyroBias& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.bias[i]);
        
    }
}
void extract(Serializer& serializer, GyroBias& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.bias[i]);
        
    }
}

void insert(Serializer& serializer, const GyroBias::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.bias[i]);
    
}
void extract(Serializer& serializer, GyroBias::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.bias[i]);
    
}

TypedResult<GyroBias> writeGyroBias(C::mip_interface& device, const float* bias)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(bias || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, bias[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GyroBias> readGyroBias(C::mip_interface& device, float* biasOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GyroBias> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_GYRO_BIAS_VECTOR, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(biasOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, biasOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<GyroBias> saveGyroBias(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GyroBias> loadGyroBias(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GyroBias> defaultGyroBias(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const CaptureGyroBias& self)
{
    insert(serializer, self.averaging_time_ms);
    
}
void extract(Serializer& serializer, CaptureGyroBias& self)
{
    extract(serializer, self.averaging_time_ms);
    
}

void insert(Serializer& serializer, const CaptureGyroBias::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.bias[i]);
    
}
void extract(Serializer& serializer, CaptureGyroBias::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.bias[i]);
    
}

TypedResult<CaptureGyroBias> captureGyroBias(C::mip_interface& device, uint16_t averagingTimeMs, float* biasOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, averagingTimeMs);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<CaptureGyroBias> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_CAPTURE_GYRO_BIAS, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_GYRO_BIAS_VECTOR, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(biasOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, biasOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const MagHardIronOffset& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.offset[i]);
        
    }
}
void extract(Serializer& serializer, MagHardIronOffset& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.offset[i]);
        
    }
}

void insert(Serializer& serializer, const MagHardIronOffset::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.offset[i]);
    
}
void extract(Serializer& serializer, MagHardIronOffset::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.offset[i]);
    
}

TypedResult<MagHardIronOffset> writeMagHardIronOffset(C::mip_interface& device, const float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(offset || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, offset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MagHardIronOffset> readMagHardIronOffset(C::mip_interface& device, float* offsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<MagHardIronOffset> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_HARD_IRON_OFFSET_VECTOR, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(offsetOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, offsetOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<MagHardIronOffset> saveMagHardIronOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MagHardIronOffset> loadMagHardIronOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MagHardIronOffset> defaultMagHardIronOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const MagSoftIronMatrix& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 9; i++)
            insert(serializer, self.offset[i]);
        
    }
}
void extract(Serializer& serializer, MagSoftIronMatrix& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 9; i++)
            extract(serializer, self.offset[i]);
        
    }
}

void insert(Serializer& serializer, const MagSoftIronMatrix::Response& self)
{
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, self.offset[i]);
    
}
void extract(Serializer& serializer, MagSoftIronMatrix::Response& self)
{
    for(unsigned int i=0; i < 9; i++)
        extract(serializer, self.offset[i]);
    
}

TypedResult<MagSoftIronMatrix> writeMagSoftIronMatrix(C::mip_interface& device, const float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(offset || (9 == 0));
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, offset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MagSoftIronMatrix> readMagSoftIronMatrix(C::mip_interface& device, float* offsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<MagSoftIronMatrix> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_SOFT_IRON_COMP_MATRIX, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(offsetOut || (9 == 0));
        for(unsigned int i=0; i < 9; i++)
            extract(deserializer, offsetOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<MagSoftIronMatrix> saveMagSoftIronMatrix(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MagSoftIronMatrix> loadMagSoftIronMatrix(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MagSoftIronMatrix> defaultMagSoftIronMatrix(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const ConingScullingEnable& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable);
        
    }
}
void extract(Serializer& serializer, ConingScullingEnable& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable);
        
    }
}

void insert(Serializer& serializer, const ConingScullingEnable::Response& self)
{
    insert(serializer, self.enable);
    
}
void extract(Serializer& serializer, ConingScullingEnable::Response& self)
{
    extract(serializer, self.enable);
    
}

TypedResult<ConingScullingEnable> writeConingScullingEnable(C::mip_interface& device, bool enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONING_AND_SCULLING_ENABLE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<ConingScullingEnable> readConingScullingEnable(C::mip_interface& device, bool* enableOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<ConingScullingEnable> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_CONING_AND_SCULLING_ENABLE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_CONING_AND_SCULLING_ENABLE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<ConingScullingEnable> saveConingScullingEnable(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONING_AND_SCULLING_ENABLE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<ConingScullingEnable> loadConingScullingEnable(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONING_AND_SCULLING_ENABLE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<ConingScullingEnable> defaultConingScullingEnable(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONING_AND_SCULLING_ENABLE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const Sensor2VehicleTransformEuler& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.roll);
        
        insert(serializer, self.pitch);
        
        insert(serializer, self.yaw);
        
    }
}
void extract(Serializer& serializer, Sensor2VehicleTransformEuler& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.roll);
        
        extract(serializer, self.pitch);
        
        extract(serializer, self.yaw);
        
    }
}

void insert(Serializer& serializer, const Sensor2VehicleTransformEuler::Response& self)
{
    insert(serializer, self.roll);
    
    insert(serializer, self.pitch);
    
    insert(serializer, self.yaw);
    
}
void extract(Serializer& serializer, Sensor2VehicleTransformEuler::Response& self)
{
    extract(serializer, self.roll);
    
    extract(serializer, self.pitch);
    
    extract(serializer, self.yaw);
    
}

TypedResult<Sensor2VehicleTransformEuler> writeSensor2VehicleTransformEuler(C::mip_interface& device, float roll, float pitch, float yaw)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, roll);
    
    insert(serializer, pitch);
    
    insert(serializer, yaw);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<Sensor2VehicleTransformEuler> readSensor2VehicleTransformEuler(C::mip_interface& device, float* rollOut, float* pitchOut, float* yawOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<Sensor2VehicleTransformEuler> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(rollOut);
        extract(deserializer, *rollOut);
        
        assert(pitchOut);
        extract(deserializer, *pitchOut);
        
        assert(yawOut);
        extract(deserializer, *yawOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<Sensor2VehicleTransformEuler> saveSensor2VehicleTransformEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<Sensor2VehicleTransformEuler> loadSensor2VehicleTransformEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<Sensor2VehicleTransformEuler> defaultSensor2VehicleTransformEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const Sensor2VehicleTransformQuaternion& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 4; i++)
            insert(serializer, self.q[i]);
        
    }
}
void extract(Serializer& serializer, Sensor2VehicleTransformQuaternion& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 4; i++)
            extract(serializer, self.q[i]);
        
    }
}

void insert(Serializer& serializer, const Sensor2VehicleTransformQuaternion::Response& self)
{
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.q[i]);
    
}
void extract(Serializer& serializer, Sensor2VehicleTransformQuaternion::Response& self)
{
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.q[i]);
    
}

TypedResult<Sensor2VehicleTransformQuaternion> writeSensor2VehicleTransformQuaternion(C::mip_interface& device, const float* q)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(q || (4 == 0));
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, q[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<Sensor2VehicleTransformQuaternion> readSensor2VehicleTransformQuaternion(C::mip_interface& device, float* qOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<Sensor2VehicleTransformQuaternion> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(qOut || (4 == 0));
        for(unsigned int i=0; i < 4; i++)
            extract(deserializer, qOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<Sensor2VehicleTransformQuaternion> saveSensor2VehicleTransformQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<Sensor2VehicleTransformQuaternion> loadSensor2VehicleTransformQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<Sensor2VehicleTransformQuaternion> defaultSensor2VehicleTransformQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const Sensor2VehicleTransformDcm& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 9; i++)
            insert(serializer, self.dcm[i]);
        
    }
}
void extract(Serializer& serializer, Sensor2VehicleTransformDcm& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 9; i++)
            extract(serializer, self.dcm[i]);
        
    }
}

void insert(Serializer& serializer, const Sensor2VehicleTransformDcm::Response& self)
{
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, self.dcm[i]);
    
}
void extract(Serializer& serializer, Sensor2VehicleTransformDcm::Response& self)
{
    for(unsigned int i=0; i < 9; i++)
        extract(serializer, self.dcm[i]);
    
}

TypedResult<Sensor2VehicleTransformDcm> writeSensor2VehicleTransformDcm(C::mip_interface& device, const float* dcm)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(dcm || (9 == 0));
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, dcm[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<Sensor2VehicleTransformDcm> readSensor2VehicleTransformDcm(C::mip_interface& device, float* dcmOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<Sensor2VehicleTransformDcm> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(dcmOut || (9 == 0));
        for(unsigned int i=0; i < 9; i++)
            extract(deserializer, dcmOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<Sensor2VehicleTransformDcm> saveSensor2VehicleTransformDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<Sensor2VehicleTransformDcm> loadSensor2VehicleTransformDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<Sensor2VehicleTransformDcm> defaultSensor2VehicleTransformDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const ComplementaryFilter& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.pitch_roll_enable);
        
        insert(serializer, self.heading_enable);
        
        insert(serializer, self.pitch_roll_time_constant);
        
        insert(serializer, self.heading_time_constant);
        
    }
}
void extract(Serializer& serializer, ComplementaryFilter& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.pitch_roll_enable);
        
        extract(serializer, self.heading_enable);
        
        extract(serializer, self.pitch_roll_time_constant);
        
        extract(serializer, self.heading_time_constant);
        
    }
}

void insert(Serializer& serializer, const ComplementaryFilter::Response& self)
{
    insert(serializer, self.pitch_roll_enable);
    
    insert(serializer, self.heading_enable);
    
    insert(serializer, self.pitch_roll_time_constant);
    
    insert(serializer, self.heading_time_constant);
    
}
void extract(Serializer& serializer, ComplementaryFilter::Response& self)
{
    extract(serializer, self.pitch_roll_enable);
    
    extract(serializer, self.heading_enable);
    
    extract(serializer, self.pitch_roll_time_constant);
    
    extract(serializer, self.heading_time_constant);
    
}

TypedResult<ComplementaryFilter> writeComplementaryFilter(C::mip_interface& device, bool pitchRollEnable, bool headingEnable, float pitchRollTimeConstant, float headingTimeConstant)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, pitchRollEnable);
    
    insert(serializer, headingEnable);
    
    insert(serializer, pitchRollTimeConstant);
    
    insert(serializer, headingTimeConstant);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_LEGACY_COMP_FILTER, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<ComplementaryFilter> readComplementaryFilter(C::mip_interface& device, bool* pitchRollEnableOut, bool* headingEnableOut, float* pitchRollTimeConstantOut, float* headingTimeConstantOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<ComplementaryFilter> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_LEGACY_COMP_FILTER, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_LEGACY_COMP_FILTER, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(pitchRollEnableOut);
        extract(deserializer, *pitchRollEnableOut);
        
        assert(headingEnableOut);
        extract(deserializer, *headingEnableOut);
        
        assert(pitchRollTimeConstantOut);
        extract(deserializer, *pitchRollTimeConstantOut);
        
        assert(headingTimeConstantOut);
        extract(deserializer, *headingTimeConstantOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<ComplementaryFilter> saveComplementaryFilter(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_LEGACY_COMP_FILTER, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<ComplementaryFilter> loadComplementaryFilter(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_LEGACY_COMP_FILTER, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<ComplementaryFilter> defaultComplementaryFilter(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_LEGACY_COMP_FILTER, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const SensorRange& self)
{
    insert(serializer, self.function);
    
    insert(serializer, self.sensor);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.setting);
        
    }
}
void extract(Serializer& serializer, SensorRange& self)
{
    extract(serializer, self.function);
    
    extract(serializer, self.sensor);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.setting);
        
    }
}

void insert(Serializer& serializer, const SensorRange::Response& self)
{
    insert(serializer, self.sensor);
    
    insert(serializer, self.setting);
    
}
void extract(Serializer& serializer, SensorRange::Response& self)
{
    extract(serializer, self.sensor);
    
    extract(serializer, self.setting);
    
}

TypedResult<SensorRange> writeSensorRange(C::mip_interface& device, SensorRangeType sensor, uint8_t setting)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, sensor);
    
    insert(serializer, setting);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR_RANGE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<SensorRange> readSensorRange(C::mip_interface& device, SensorRangeType sensor, uint8_t* settingOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, sensor);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<SensorRange> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR_RANGE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_SENSOR_RANGE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, sensor);
        
        assert(settingOut);
        extract(deserializer, *settingOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<SensorRange> saveSensorRange(C::mip_interface& device, SensorRangeType sensor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, sensor);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR_RANGE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<SensorRange> loadSensorRange(C::mip_interface& device, SensorRangeType sensor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, sensor);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR_RANGE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<SensorRange> defaultSensorRange(C::mip_interface& device, SensorRangeType sensor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, sensor);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR_RANGE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const CalibratedSensorRanges& self)
{
    insert(serializer, self.sensor);
    
}
void extract(Serializer& serializer, CalibratedSensorRanges& self)
{
    extract(serializer, self.sensor);
    
}

void insert(Serializer& serializer, const CalibratedSensorRanges::Response& self)
{
    insert(serializer, self.sensor);
    
    insert(serializer, self.num_ranges);
    
    for(unsigned int i=0; i < self.num_ranges; i++)
        insert(serializer, self.ranges[i]);
    
}
void extract(Serializer& serializer, CalibratedSensorRanges::Response& self)
{
    extract(serializer, self.sensor);
    
    C::extract_count(&serializer, &self.num_ranges, sizeof(self.ranges)/sizeof(self.ranges[0]));
    for(unsigned int i=0; i < self.num_ranges; i++)
        extract(serializer, self.ranges[i]);
    
}

void insert(Serializer& serializer, const CalibratedSensorRanges::Entry& self)
{
    insert(serializer, self.setting);
    
    insert(serializer, self.range);
    
}
void extract(Serializer& serializer, CalibratedSensorRanges::Entry& self)
{
    extract(serializer, self.setting);
    
    extract(serializer, self.range);
    
}

TypedResult<CalibratedSensorRanges> calibratedSensorRanges(C::mip_interface& device, SensorRangeType sensor, uint8_t* numRangesOut, uint8_t numRangesOutMax, CalibratedSensorRanges::Entry* rangesOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, sensor);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<CalibratedSensorRanges> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_CALIBRATED_RANGES, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_CALIBRATED_RANGES, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, sensor);
        
        C::extract_count(&deserializer, numRangesOut, numRangesOutMax);
        assert(rangesOut || (numRangesOut == 0));
        for(unsigned int i=0; i < *numRangesOut; i++)
            extract(deserializer, rangesOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const LowpassFilter& self)
{
    insert(serializer, self.function);
    
    insert(serializer, self.desc_set);
    
    insert(serializer, self.field_desc);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable);
        
        insert(serializer, self.manual);
        
        insert(serializer, self.frequency);
        
    }
}
void extract(Serializer& serializer, LowpassFilter& self)
{
    extract(serializer, self.function);
    
    extract(serializer, self.desc_set);
    
    extract(serializer, self.field_desc);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable);
        
        extract(serializer, self.manual);
        
        extract(serializer, self.frequency);
        
    }
}

void insert(Serializer& serializer, const LowpassFilter::Response& self)
{
    insert(serializer, self.desc_set);
    
    insert(serializer, self.field_desc);
    
    insert(serializer, self.enable);
    
    insert(serializer, self.manual);
    
    insert(serializer, self.frequency);
    
}
void extract(Serializer& serializer, LowpassFilter::Response& self)
{
    extract(serializer, self.desc_set);
    
    extract(serializer, self.field_desc);
    
    extract(serializer, self.enable);
    
    extract(serializer, self.manual);
    
    extract(serializer, self.frequency);
    
}

TypedResult<LowpassFilter> writeLowpassFilter(C::mip_interface& device, uint8_t descSet, uint8_t fieldDesc, bool enable, bool manual, float frequency)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, descSet);
    
    insert(serializer, fieldDesc);
    
    insert(serializer, enable);
    
    insert(serializer, manual);
    
    insert(serializer, frequency);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_LOWPASS_FILTER, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<LowpassFilter> readLowpassFilter(C::mip_interface& device, uint8_t descSet, uint8_t fieldDesc, bool* enableOut, bool* manualOut, float* frequencyOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, descSet);
    
    insert(serializer, fieldDesc);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<LowpassFilter> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_LOWPASS_FILTER, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_LOWPASS_FILTER, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, descSet);
        
        extract(deserializer, fieldDesc);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        assert(manualOut);
        extract(deserializer, *manualOut);
        
        assert(frequencyOut);
        extract(deserializer, *frequencyOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<LowpassFilter> saveLowpassFilter(C::mip_interface& device, uint8_t descSet, uint8_t fieldDesc)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, descSet);
    
    insert(serializer, fieldDesc);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_LOWPASS_FILTER, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<LowpassFilter> loadLowpassFilter(C::mip_interface& device, uint8_t descSet, uint8_t fieldDesc)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, descSet);
    
    insert(serializer, fieldDesc);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_LOWPASS_FILTER, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<LowpassFilter> defaultLowpassFilter(C::mip_interface& device, uint8_t descSet, uint8_t fieldDesc)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, descSet);
    
    insert(serializer, fieldDesc);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_LOWPASS_FILTER, buffer, (uint8_t)mip_serializer_length(&serializer));
}

} // namespace commands_3dm
} // namespace mip

