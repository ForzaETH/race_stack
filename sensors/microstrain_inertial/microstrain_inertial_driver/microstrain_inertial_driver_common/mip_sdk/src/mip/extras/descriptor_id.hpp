#pragma once

#include "../definitions/descriptors.h"

#include <stdint.h>


namespace mip
{

////////////////////////////////////////////////////////////////////////////////
///@brief A combination of a MIP descriptor pair and u16 ID value.
///
/// Used by CompositeResult as a way to identify both MIP and non-MIP entries.
///
class DescriptorId
{
public:
    DescriptorId() : m_key(0) {}
    DescriptorId(uint8_t desc_set, uint8_t field_desc, uint16_t index=0) : DescriptorId({desc_set, field_desc}, index) {}
    DescriptorId(CompositeDescriptor desc, uint16_t index=0) : m_key(uint32_t(desc.as_u16() << 16) | index) {}
    //DescriptorId(uint16_t index) : m_key(index) {}
    DescriptorId(uint32_t id) : m_key(id) {}

    bool isNull() const { return m_key == 0x00000000; }

    bool isMip() const { return descriptor().as_u16() != 0x0000; }
    bool isNonMip() const { return !isMip(); }

    CompositeDescriptor descriptor() const { return m_key >> 16; }
    uint16_t                 index() const { return m_key & 0xFFFF; }
    uint32_t                 asU32() const { return m_key; }

    bool operator==(const DescriptorId& other) const { return m_key == other.m_key; }
    bool operator!=(const DescriptorId& other) const { return m_key != other.m_key; }
    bool operator<=(const DescriptorId& other) const { return m_key <= other.m_key; }
    bool operator>=(const DescriptorId& other) const { return m_key >= other.m_key; }
    bool operator< (const DescriptorId& other) const { return m_key <  other.m_key; }
    bool operator> (const DescriptorId& other) const { return m_key >  other.m_key; }

private:
    uint32_t m_key;
};

} // namespace mip
