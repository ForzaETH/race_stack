/////////////////////////////////////////////////////////////////////////////
//
// ublox_device.hpp
//
// Basic UBlox serial device interface to parse out the UBlox UBX-NAV-PVT message from a serial port
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

#include <cmath>

#include "simple_ublox_parser.hpp"

#include "mip/platform/serial_connection.hpp"

#define PVT_PAYLOAD_SIZE 92

namespace mip
{
    namespace ublox
    {

#pragma pack(1)
        struct UbloxPVTMessageRaw
        {
            uint32_t iTOW;
            uint16_t utc_year;
            uint8_t utc_month;
            uint8_t utc_day;
            uint8_t utc_hour;
            uint8_t utc_minute;
            uint8_t utc_second;
            uint8_t time_valid_flag;
            uint32_t time_accuracy;
            int32_t nano_second;
            uint8_t fix_type;
            uint8_t fix_valid_flags;
            uint8_t confirmed_time_date_flags;
            uint8_t number_of_satellites;
            int32_t longitude;
            int32_t latitude;
            int32_t height_above_ellipsoid;
            int32_t height_above_sea_level;
            uint32_t horizontal_accuracy;
            uint32_t vertical_accuracy;
            int32_t north_velocity;
            int32_t east_velocity;
            int32_t down_velocity;
            int32_t ground_speed;
            int32_t heading_of_motion_2d;
            uint32_t speed_accuracy;
            uint32_t heading_accuracy;
            uint16_t pDOP;
            uint8_t llh_invalid_flag;
            uint8_t reserved_bytes[5];
            int32_t heading_of_vehicle;
            int16_t magnetic_declination;
            uint16_t magnetic_declination_accuracy;
        };
#pragma pack()


        struct UbloxPVTMessage
        {
            // Time
            uint16_t utc_year = 0;
            uint8_t utc_month = 0;
            uint8_t utc_day = 0;
            float time_of_week = 0;
            bool time_valid = false;

            // LLH position
            double latitude = 0;
            double longitude = 0;
            double height_above_ellipsoid = 0;
            float llh_position_uncertainty[3] = {0, 0, 0};
            bool llh_position_valid = false;

            // NED velocity
            float ned_velocity[3] = {0, 0, 0};
            float ned_velocity_uncertainty[3] = {0, 0, 0};
        };


        UbloxPVTMessage extract_pvt_message(const uint8_t payload[PVT_PAYLOAD_SIZE])
        {
            // Unpack raw UBlox message data
            UbloxPVTMessageRaw ublox_message_raw;
            std::memcpy(&ublox_message_raw, payload, sizeof(ublox_message_raw));

            // Build output message with properly scaled units
            UbloxPVTMessage ublox_message;

            // Time
            ublox_message.utc_year = ublox_message_raw.utc_year;
            ublox_message.utc_month = ublox_message_raw.utc_month;
            ublox_message.utc_day = ublox_message_raw.utc_day;
            ublox_message.time_of_week = ublox_message_raw.iTOW * 1e-3;
            ublox_message.time_valid = ublox_message_raw.time_valid_flag;

            // LLH position
            ublox_message.latitude = ublox_message_raw.latitude * 1e-7;
            ublox_message.longitude = ublox_message_raw.longitude * 1e-7;;
            ublox_message.height_above_ellipsoid = ublox_message_raw.height_above_ellipsoid * 1e-3;
            ublox_message.llh_position_uncertainty[0] = ublox_message_raw.horizontal_accuracy * 1e-3;
            ublox_message.llh_position_uncertainty[1] = ublox_message_raw.horizontal_accuracy * 1e-3;
            ublox_message.llh_position_uncertainty[2] = ublox_message_raw.vertical_accuracy * 1e-3;
            ublox_message.llh_position_valid = !ublox_message_raw.llh_invalid_flag;

            // NED velocity
            ublox_message.ned_velocity[0] = ublox_message_raw.north_velocity * 1e-3;
            ublox_message.ned_velocity[1] = ublox_message_raw.east_velocity * 1e-3;
            ublox_message.ned_velocity[2] = ublox_message_raw.down_velocity * 1e-3;
            for (int i = 0; i < 3; i++)
                ublox_message.ned_velocity_uncertainty[i] = ublox_message_raw.speed_accuracy * 1e-3;

            return ublox_message;
        }


        class UbloxDevice
        {
        public:

            UbloxDevice(std::unique_ptr<mip::Connection> connection) : _connection(std::move(connection)),
                                                                       _message_parser(
                                                                               [this](const std::vector<uint8_t>& packet) {
                                                                                   handle_packet(packet);
                                                                               })
            {}

            void handle_packet(const std::vector<uint8_t>& packet)
            {
                bool is_pvt_message = (packet[2] == 0x01) && (packet[3] == 0x07);
                if (!is_pvt_message)
                    return;

                // Should never happen
                size_t expected_packet_size = HEADER_SIZE + PVT_PAYLOAD_SIZE + CHECKSUM_SIZE;
                if (packet.size() != expected_packet_size)
                    return;

                // Extract message payload
                uint8_t payload_bytes[PVT_PAYLOAD_SIZE];
                for (int i = 0; i < PVT_PAYLOAD_SIZE; i++)
                    payload_bytes[i] = packet[i + HEADER_SIZE];

                // Parse message payload
                _current_message = extract_pvt_message(payload_bytes);

                // Mark flag indicating a new message has been received
                _new_pvt_message_received = true;
            }

            std::pair<bool, UbloxPVTMessage> update()
            {
                // Reset new message indicator flag
                _new_pvt_message_received = false;

                // Get incoming bytes from serial port
                uint8_t input_bytes[1024];
                size_t num_input_bytes;
                mip::Timestamp timestamp_out;
                _connection->recvFromDevice(input_bytes, 1024, 1, &num_input_bytes, &timestamp_out);

                // Spin message parser
                _message_parser.parse_bytes(input_bytes, num_input_bytes);

                return {_new_pvt_message_received, _current_message};
            }

        protected:

            std::unique_ptr<mip::Connection> _connection;
            UbloxMessageParser _message_parser;

            bool _new_pvt_message_received = false;
            UbloxPVTMessage _current_message;
        };
    }
}