#pragma once

#include <stdint.h>
#include <stddef.h>

#include <string>

#if __cpp_lib_string_view >= 201606L
#include <string_view>
#endif

namespace mip
{

////////////////////////////////////////////////////////////////////////////
///@brief Represents the device firmware version.
///
/// Device firmware is of the form X.Y.ZZ, where:
/// - X is the major version, exactly 1 digit.
/// - Y is the minor version, exactly 1 digit.
/// - Z is the patch version, exactly 2 digits.
///
/// Internally this class stores the version as a 16-bit unsigned integer.
///
class FirmwareVersion
{
public:
    FirmwareVersion() = default;
    FirmwareVersion(uint8_t major, uint8_t minor, uint8_t patch) { fromParts(major, minor, patch); }
    explicit FirmwareVersion(uint16_t version) : m_version(version) {}

    FirmwareVersion(const FirmwareVersion&) = default;
    FirmwareVersion& operator=(const FirmwareVersion&) = default;
    FirmwareVersion& operator=(uint16_t version) { m_version = version; return *this; }

    bool isNull()           const { return m_version == 0; }
    bool isValid()          const { return !isNull() && major() < 10 && minor() < 10 && patch() < 100; }
    bool isDevVersion()     const { return major() == 0; }
    bool isReleaseVersion() const { return major() >  0; }
    bool isSpecialVersion() const { return major() >  1; }

    uint16_t asU16() const { return m_version; }
    uint16_t& asU16() { return m_version; }

    void fromParts(uint8_t major, uint8_t minor, uint8_t patch) { m_version = major * 1000 + minor * 100 + patch; }

    uint8_t major() const { return uint8_t(m_version / 1000); }
    uint8_t minor() const { return (m_version / 100) % 10; }
    uint8_t patch() const { return m_version % 100; }

    bool operator==(FirmwareVersion other) const { return m_version == other.m_version; }
    bool operator!=(FirmwareVersion other) const { return m_version != other.m_version; }
    bool operator<=(FirmwareVersion other) const { return m_version <= other.m_version; }
    bool operator>=(FirmwareVersion other) const { return m_version >= other.m_version; }
    bool operator< (FirmwareVersion other) const { return m_version <  other.m_version; }
    bool operator> (FirmwareVersion other) const { return m_version >  other.m_version; }

    void toString(char* buffer, size_t buffer_size) const;
    bool fromString(const char* str, size_t length=-1);

    std::string toString() const;
#if __cpp_lib_string_view >= 201606L
    bool fromString(std::string_view str) { return fromString(str.data(), str.size()); }
#endif

private:
    uint16_t m_version = 0;
};

} // namespace mip
