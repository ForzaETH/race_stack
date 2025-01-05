
#include "version.hpp"

#include <cstdio>


namespace mip
{

////////////////////////////////////////////////////////////////////////////////
///@brief Convert the version to a string in the standard X.Y.ZZ format.
///
/// Note: The result is unspecified for invalid version numbers.
///
///@param buffer      Character buffer to write into.
///@param buffer_size Length (including space for null terminator) of buffer.
///
void FirmwareVersion::toString(char* buffer, size_t buffer_size) const
{
    std::snprintf(buffer, buffer_size, "%u.%u.%02u", major(), minor(), patch());
}

////////////////////////////////////////////////////////////////////////////////
///@brief Reads a standard-format string (X.Y.ZZ\0 or XYZZ\0).
///
///@param str    Input string. Can be unterminated if length is specified.
///@param length Length of input string. Assumed to be NULL-terminated if -1.
///
///@return True if a valid version was parsed.
///
bool FirmwareVersion::fromString(const char* str, size_t length)
{
    m_version = 0;

    unsigned int digit = 0;
    for(unsigned int i=0; i<length; i++)
    {
        if(!std::isdigit(str[i]))
        {
            if(str[i] == '.')
                continue;

            if(str[i] == '\0')
                break;

            return false;
        }

        m_version *= 10;
        m_version += str[i] - '0';

        if(++digit >= 4)
            break;
    }

    return isValid();
}

////////////////////////////////////////////////////////////////////////////////
///@brief Convert a FirmwareVersion to a string separated by periods.
///
/// This is different from Version::toString in that the patch number uses
/// zero-padding.
///
std::string FirmwareVersion::toString() const
{
    char buffer[3+1+3+1+3+1];

    toString(buffer, sizeof(buffer));

    return buffer;
}

} // namespace mip
