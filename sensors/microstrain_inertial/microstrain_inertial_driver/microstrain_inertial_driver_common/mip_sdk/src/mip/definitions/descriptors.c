
#include "descriptors.h"

#include "../utils/serialization.h"

#ifdef __cplusplus
namespace mip {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the descriptor set is valid.
///
///@param descriptor_set
///
///@returns true if the descriptor set is valid.
///
bool mip_is_valid_descriptor_set(uint8_t descriptor_set)
{
    return descriptor_set != MIP_INVALID_DESCRIPTOR_SET;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the descriptor set represents some kind of data.
///
///@param descriptor_set
///
///@returns true if the descriptor set represents data.
///
bool mip_is_data_descriptor_set(uint8_t descriptor_set)
{
    return (descriptor_set >= MIP_DATA_DESCRIPTOR_SET_START) && (descriptor_set < MIP_RESERVED_DESCRIPTOR_SET_START);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the descriptor set contains commands.
///
///@param descriptor_set
///
///@returns true if the descriptor set contains commands.
///
bool mip_is_cmd_descriptor_set(uint8_t descriptor_set)
{
    return !mip_is_data_descriptor_set(descriptor_set);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the descriptor is reserved for special purposes.
///
///@param descriptor_set
///
///@returns true if the descriptor set is reserved.
///
bool mip_is_reserved_descriptor_set(uint8_t descriptor_set)
{
    return (descriptor_set >= MIP_RESERVED_DESCRIPTOR_SET_START);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the descriptor set represents some kind of GNSS data.
///
///@param descriptor_set
///
///@returns true if the descriptor set represents GNSS data.
///
bool mip_is_gnss_data_descriptor_set(uint8_t descriptor_set)
{
    return ((descriptor_set == 0x81) || ((descriptor_set >= 0x91) && (descriptor_set <= 0x95)));
}



////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the field descriptor is valid.
///
///@param field_descriptor
///
///@returns true if the field descriptor is valid.
///
bool mip_is_valid_field_descriptor(uint8_t field_descriptor)
{
    return field_descriptor != MIP_INVALID_FIELD_DESCRIPTOR;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the field descriptor is a command.
///
///@param field_descriptor
///
///@returns true if the field descriptor represents a command.
///
bool mip_is_cmd_field_descriptor(uint8_t field_descriptor)
{
    return (field_descriptor < MIP_RESPONSE_DESCRIPTOR_START);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the field descriptor is for an ack/nack reply.
///
///@param field_descriptor
///
///@returns true if the field descriptor represents an ack/nack reply code.
///
bool mip_is_reply_field_descriptor(uint8_t field_descriptor)
{
    return (field_descriptor == MIP_REPLY_DESCRIPTOR);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the field descriptor contains response data from a
///       command.
///
/// The descriptor set is assumed to be a command set.
///
///@param field_descriptor
///
///@returns true if the associated field contains response data.
///
bool mip_is_response_field_descriptor(uint8_t field_descriptor)
{
    return field_descriptor >= MIP_RESPONSE_DESCRIPTOR_START && !mip_is_reserved_cmd_field_descriptor(field_descriptor);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the field descriptor is reserved.
///
/// The descriptor set is assumed to be a command set.
///
///@param field_descriptor
///
///@returns true if the associated field is neither a command nor response.
///
bool mip_is_reserved_cmd_field_descriptor(uint8_t field_descriptor)
{
    return ((field_descriptor|MIP_RESPONSE_DESCRIPTOR_START) >= MIP_RESERVED_DESCRIPTOR_START);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the field descriptor is from the shared data set.
///
/// The descriptor set is assumed to be a data set.
///
///@param field_descriptor
///
///@returns true if the associated field is from the shared data set.
///
bool mip_is_shared_data_field_descriptor(uint8_t field_descriptor)
{
    return field_descriptor >= MIP_SHARED_DATA_FIELD_DESCRIPTOR_START;
}


void insert_mip_function_selector(mip_serializer* serializer, enum mip_function_selector self)
{
    insert_u8(serializer, self);
}

void extract_mip_function_selector(mip_serializer* serializer, enum mip_function_selector* self)
{
    uint8_t tmp;
    extract_u8(serializer, &tmp);
    *self = (enum mip_function_selector)tmp;
}


#ifdef __cplusplus
} // namespace mip
} // extern "C"
#endif // __cplusplus
