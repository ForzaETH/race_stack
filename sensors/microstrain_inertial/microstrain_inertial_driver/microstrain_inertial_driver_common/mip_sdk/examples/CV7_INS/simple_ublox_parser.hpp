/////////////////////////////////////////////////////////////////////////////
//
// simple_ublox_parser.hpp
//
// Basic UBlox binary message parser
//
// This class intends to be a simple helper utility for an example to demonstrate CV7-INS functionality and is not intended
// to be reused for any other application
//
//!@section LICENSE
//!
//! THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
//! CUSTOMERS WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER
//! FOR THEM TO SAVE TIME. AS A RESULT, HBK MICROSTRAIN SHALL NOT BE HELD
//! LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY
//! CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS
//! OF THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
//
/////////////////////////////////////////////////////////////////////////////

#pragma once

#include <vector>
#include <deque>
#include <cstdint>
#include <cstring>
#include <functional>

const int HEADER_SIZE = 6;
const int CHECKSUM_SIZE = 2;

namespace mip
{
    namespace ublox
    {

        bool verify_checksum(const std::vector<uint8_t>& packet)
        {
            uint8_t ck_a, ck_b;

            ck_a = 0;
            ck_b = 0;

            int num_bytes = packet.size();
            int num_bytes_without_checksum = num_bytes - 2;

            for (int i = 2; i < num_bytes_without_checksum; i++) {
                ck_a += packet[i];
                ck_b += ck_a;
            }

            if (ck_a == packet[num_bytes - 2] && ck_b == packet[num_bytes - 1])
                return true;

            return false;
        }


        class UbloxMessageParser
        {
        public:

            UbloxMessageParser(std::function<void(std::vector<uint8_t>)> packet_callback) : _packet_callback(
                    packet_callback)
            {}

            void parse_bytes(uint8_t *buffer, size_t num_input_bytes)
            {
                // Copy into parser buffer
                for (size_t i = 0; i < num_input_bytes; i++) {
                    _buffer.emplace_back(buffer[i]);
                }

                // Wait for header bytes
                while (_buffer.size() >= 2) {
                    if (header_found())
                        break;

                    _buffer.pop_front();
                }

                // Check if header is valid
                if (!header_found())
                    return;

                // Check if buffer has full message header
                if (_buffer.size() < 6)
                    return;

                // Get message length
                uint8_t payload_length_bytes[2] = {_buffer[4], _buffer[5]};
                uint16_t payload_length;
                memcpy(&payload_length, payload_length_bytes, sizeof(uint16_t));

                unsigned int total_message_length = HEADER_SIZE + payload_length + CHECKSUM_SIZE;

                // Check if buffer contains full packet size
                if (_buffer.size() < total_message_length)
                    return;

                // Extract packet
                std::vector<uint8_t> packet(total_message_length);
                for (unsigned int i = 0; i < total_message_length; i++)
                    packet[i] = _buffer[i];

                // Validate checksum
                if (verify_checksum(packet)) {
                    // Call packet callback
                    _packet_callback(packet);

                    // Clear packet from buffer
                    for (unsigned int i = 0; i < total_message_length; i++)
                        _buffer.pop_front();
                } else
                    _buffer.pop_front();
            }

            bool header_found()
            {
                if (_buffer.size() < 2)
                    return false;

                return (_buffer[0] == 0xB5) && (_buffer[1] == 0x62);
            }

        protected:

            std::function<void(std::vector<uint8_t>)> _packet_callback;

            std::deque<uint8_t> _buffer;
        };
    }
}
