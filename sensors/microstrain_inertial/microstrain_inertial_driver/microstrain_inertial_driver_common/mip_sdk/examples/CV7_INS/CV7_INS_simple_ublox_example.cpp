/////////////////////////////////////////////////////////////////////////////
//
// CV7_INS_simple_ublox_example.cpp
//
// C++ Example usage program for the CV7-INS with a UBlox receiver
//
// This example shows the basic interface between the CV7-INS and a UBlox receiver preconfigured to stream a the UBX-NAV-PVT message.
// It is intended to demonstrate the relevant MIP API calls to input GNSS data to the CV7-INS
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


////////////////////////////////////////////////////////////////////////////////
// Include Files
////////////////////////////////////////////////////////////////////////////////

#include "mip/mip_all.hpp"
#include "../example_utils.hpp"
#include "ublox_device.hpp"
#include <cmath>
#include <cstring>
#include <string>
#include <ctime>
#include <array>

using namespace mip;

////////////////////////////////////////////////////////////////////////////////
// Global Variables
////////////////////////////////////////////////////////////////////////////////

data_shared::GpsTimestamp    filter_gps_time;
data_filter::Status          filter_status;
data_filter::EulerAngles     filter_euler_angles;
data_filter::PositionLlh     filter_llh_position;
data_filter::VelocityNed     filter_ned_velocity;
data_system::TimeSyncStatus  system_time_sync_status;

uint8_t gnss_antenna_sensor_id = 1;

bool filter_state_full_nav = false;

struct InputArguments
{
    std::string mip_device_port_name;
    std::string mip_device_baudrate;
    std::string mip_binary_filepath;

    std::string ublox_device_port_name;
    std::string ublox_device_baudrate;

    bool enable_pps_sync = false;

    int pps_input_pin_id = 1;

    commands_filter::InitializationConfiguration::AlignmentSelector filter_heading_alignment_method = commands_filter::InitializationConfiguration::AlignmentSelector::KINEMATIC;

    float gnss_antenna_lever_arm[3] = {0,0,0};
};

////////////////////////////////////////////////////////////////////////////////
// Function Prototypes
////////////////////////////////////////////////////////////////////////////////

int usage(const char* argv0);

void print_device_information(const commands_base::BaseDeviceInfo& device_info);

void exit_gracefully(const char *message);
bool should_exit();

InputArguments parse_input_arguments(int argc, const char* argv[]);

uint64_t convert_gps_tow_to_nanoseconds(int week_number, float time_of_week);

int get_gps_week(int year, int month, int day);

int main(int argc, const char* argv[])
{
    InputArguments input_arguments = parse_input_arguments(argc, argv);

    std::unique_ptr<ExampleUtils> utils = openFromArgs(input_arguments.mip_device_port_name, input_arguments.mip_device_baudrate, input_arguments.mip_binary_filepath);
    std::unique_ptr<mip::DeviceInterface>& device = utils->device;
    
    //
    // Open uBlox serial port
    //

    printf("Connecting to UBlox F9P on %s at %s...\n", input_arguments.ublox_device_port_name.c_str(), input_arguments.ublox_device_baudrate.c_str());
    std::unique_ptr<ExampleUtils> utils_ublox = openFromArgs(input_arguments.ublox_device_port_name, input_arguments.ublox_device_baudrate, {});
    ublox::UbloxDevice ublox_device(std::move(utils_ublox->connection));

    //
    //Attempt to idle the device before pinging
    //
    commands_base::setIdle(*device);

    //
    //Ping the device (note: this is good to do to make sure the device is present)
    //

    if(commands_base::ping(*device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not ping the device!");

    //
    //Read device information
    //

    commands_base::BaseDeviceInfo device_info;
    if(commands_base::getDeviceInfo(*device, &device_info) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Failed to get device info");
    print_device_information(device_info);

    //
    //Idle the device (note: this is good to do during setup)
    //

    if(commands_base::setIdle(*device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set the device to idle!");


    //
    //Load the device default settings (so the device is in a known state)
    //

    if(commands_3dm::defaultDeviceSettings(*device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not load default device settings!");


    //
    //External GNSS antenna reference frame
    //
    commands_aiding::FrameConfig::Rotation external_gnss_antenna_to_vehicle_frame_rotation;
    external_gnss_antenna_to_vehicle_frame_rotation.euler = mip::Vector3f(0.0f, 0.0f, 0.0f);  // GNSS position/velocity measurements are agnostic to rotation, rotation set to zero // GNSS position/velocity measurements are agnostic to rotation, rotation set to zero
    if(commands_aiding::writeFrameConfig(*device, gnss_antenna_sensor_id, mip::commands_aiding::FrameConfig::Format::EULER, true,
                                          input_arguments.gnss_antenna_lever_arm, external_gnss_antenna_to_vehicle_frame_rotation) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Unable to configure external GNSS antenna frame ID");


    //
    // Set heading initialization source
    //

    float default_init[3] = {0,0,0};
    if(commands_filter::writeInitializationConfiguration(*device, false, commands_filter::InitializationConfiguration::InitialConditionSource::AUTO_POS_VEL_ATT, input_arguments.filter_heading_alignment_method,
                                                         0, 0, 0, default_init, default_init, commands_filter::FilterReferenceFrame::ECEF) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not load default device settings!");


    if (input_arguments.enable_pps_sync)
    {

        //
        //Setup SYSTEM data format to monitor PPS status
        //

        uint16_t system_data_base_rate;
        if(commands_3dm::getBaseRate(*device, data_system::DESCRIPTOR_SET, &system_data_base_rate) != CmdResult::ACK_OK)
            exit_gracefully("ERROR: Could not get system data base rate format!");

        const uint16_t system_data_sample_rate = 10; // Hz
        const uint16_t system_data_decimation = system_data_base_rate / system_data_sample_rate;

        std::array<DescriptorRate, 2> system_data_descriptors = {{
                                                                    { data_shared::DATA_GPS_TIME,         system_data_decimation },
                                                                    { data_system::DATA_TIME_SYNC_STATUS, system_data_decimation },
                                                            }};

        if(commands_3dm::writeMessageFormat(*device, data_system::DESCRIPTOR_SET, system_data_descriptors.size(), system_data_descriptors.data()) != CmdResult::ACK_OK)
            exit_gracefully("ERROR: Could not set system data message format!");


        //
        // Setup GPIO for PPS input functionality
        //

        if (commands_3dm::writeGpioConfig(*device, input_arguments.pps_input_pin_id, mip::commands_3dm::GpioConfig::Feature::PPS, mip::commands_3dm::GpioConfig::Behavior::PPS_INPUT, mip::commands_3dm::GpioConfig::PinMode::NONE) != CmdResult::ACK_OK)
            exit_gracefully("ERROR: Could not set GPIO to PPS input!");


        //
        // Setup PPS source as GPIO
        //

        if (mip::commands_3dm::writePpsSource(*device, mip::commands_3dm::PpsSource::Source::GPIO) != CmdResult::ACK_OK)
            exit_gracefully("ERROR: Failed to set PPS source to GPIO!");

    }

    //
    //Configure factory streaming data.  This enables all critical data channels required for post-processing analysis
    //

    if(commands_3dm::factoryStreaming(*device, commands_3dm::FactoryStreaming::Action::MERGE, 0) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not enable factory streaming support!");

    //
    //Reset the filter (note: this is good to do after filter setup is complete)
    //

    if(commands_filter::reset(*device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not reset the filter!");


    //
    // Register data callbacks
    //

    //Filter Data
    DispatchHandler filter_data_handlers[5];

    device->registerExtractor(filter_data_handlers[0], &filter_gps_time, data_filter::DESCRIPTOR_SET);
    device->registerExtractor(filter_data_handlers[1], &filter_status);
    device->registerExtractor(filter_data_handlers[2], &filter_euler_angles);
    device->registerExtractor(filter_data_handlers[3], &filter_llh_position);
    device->registerExtractor(filter_data_handlers[4], &filter_ned_velocity);

    //System Data
    DispatchHandler system_data_handlers[1];

    device->registerExtractor(system_data_handlers[0], &system_time_sync_status, data_system::DESCRIPTOR_SET);

    //
    //Resume the device
    //

    if(commands_base::resume(*device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not resume the device!");
    
    //Main Loop: Update the interface and process data
    //

    bool running = true;
    bool pps_sync_valid = false;
    mip::Timestamp prev_print_timestamp = getCurrentTimestamp();
    mip::Timestamp prev_measurement_update_timestamp = getCurrentTimestamp();

    printf("Sensor is configured... waiting for filter to initialize...\n");

    while(running) {
        mip::Timestamp current_timestamp = getCurrentTimestamp();

        // Spin MIP device
        device->update();

        // Get ublox data
        std::pair<bool, ublox::UbloxPVTMessage> ubox_parser_result = ublox_device.update();
        bool pvt_message_valid = ubox_parser_result.first;
        ublox::UbloxPVTMessage pvt_message = ubox_parser_result.second;
        
        // Wait for valid PPS lock
        if (input_arguments.enable_pps_sync && !pps_sync_valid)
        {
            pps_sync_valid = system_time_sync_status.time_sync;

            mip::Timestamp elapsed_time_from_last_message_print = current_timestamp - prev_print_timestamp;
            if (elapsed_time_from_last_message_print > 1000)
            {
                printf("Waiting for valid PPS lock...\n");
                prev_print_timestamp = current_timestamp;
            }
            continue;
        }
        
        //Check for full nav filter state transition
        if((!filter_state_full_nav) && (filter_status.filter_state == data_filter::FilterMode::FULL_NAV))
        {
            printf("NOTE: Filter has entered full navigation mode.\n");
            filter_state_full_nav = true;
        }

        //Print status at 1Hz
        mip::Timestamp elapsed_time_from_last_message_print = current_timestamp - prev_print_timestamp;
        bool print_new_update_message = elapsed_time_from_last_message_print >= 1000;
        if (print_new_update_message)
        {
            if(filter_status.filter_state == data_filter::FilterMode::FULL_NAV)
            {
                printf("\n\n****Filter navigation state****\n");
                printf("TIMESTAMP: %f\n", filter_gps_time.tow);
                printf("ATTITUDE_EULER = [%f %f %f]\n", filter_euler_angles.roll, filter_euler_angles.pitch, filter_euler_angles.yaw);
                printf("LLH_POSITION = [%f %f %f]\n", filter_llh_position.latitude, filter_llh_position.longitude, filter_llh_position.ellipsoid_height);
                printf("NED_VELOCITY = [%f %f %f]\n", filter_ned_velocity.north, filter_ned_velocity.east, filter_ned_velocity.down);
            }
            else
                printf("Waiting for navigation filter to initialize...\n");

            prev_print_timestamp = current_timestamp;
        }

        // Check if measurement update is valid to send
        mip::Timestamp elapsed_time_from_last_measurement_update = current_timestamp - prev_measurement_update_timestamp;
        bool ublox_data_valid = pvt_message_valid && pvt_message.time_valid && pvt_message.llh_position_valid;
        bool send_measurement_update = ublox_data_valid && elapsed_time_from_last_measurement_update > 250; // Cap maximum GNSS measurement input rate to 4hz

        if (send_measurement_update)
        {
            printf("\n\n****Sending Measurement Update****\n");
            printf("LLH_POSITION_GNSS_MEAS = [%f %f %f]\n", pvt_message.latitude, pvt_message.longitude, pvt_message.height_above_ellipsoid);
            printf("NED_VELOCITY_GNSS_MEAS = [%f %f %f]\n", pvt_message.ned_velocity[0], pvt_message.ned_velocity[1], pvt_message.ned_velocity[2]);

            commands_aiding::Time external_measurement_time;

            if (input_arguments.enable_pps_sync)
            {
                // Send week number update to device
                uint32_t week_number = get_gps_week(pvt_message.utc_year, pvt_message.utc_month, pvt_message.utc_day);
                if (!commands_base::writeGpsTimeUpdate(*device, commands_base::GpsTimeUpdate::FieldId::WEEK_NUMBER, week_number))
                    printf("WARNING: Failed to send week number time update to CV7-INS\n");

                // Send time of week update to device
                uint32_t time_of_week_int = floor(pvt_message.time_of_week);
                if (!commands_base::writeGpsTimeUpdate(*device, commands_base::GpsTimeUpdate::FieldId::TIME_OF_WEEK, time_of_week_int))
                    printf("WARNING: Failed to send time of week update to CV7-INS\n");

                // Mark timestamp for aiding message input
                external_measurement_time.timebase = commands_aiding::Time::Timebase::EXTERNAL_TIME;
                external_measurement_time.nanoseconds = convert_gps_tow_to_nanoseconds(week_number, pvt_message.time_of_week);
            }
            else
            {
                // If no PPS sync is supplied, use device time of arrival for data timestamping method
                external_measurement_time.timebase = commands_aiding::Time::Timebase::TIME_OF_ARRIVAL;
            }

            // External position command
            if (commands_aiding::llhPos(*device, external_measurement_time, gnss_antenna_sensor_id, pvt_message.latitude, pvt_message.longitude, pvt_message.height_above_ellipsoid, pvt_message.llh_position_uncertainty, 1) != CmdResult::ACK_OK)
                printf("WARNING: Failed to send external position to CV7-INS\n");

            // External global velocity command
            if (commands_aiding::nedVel(*device, external_measurement_time, gnss_antenna_sensor_id,pvt_message.ned_velocity, pvt_message.ned_velocity_uncertainty, 1) != CmdResult::ACK_OK)
                printf("WARNING: Failed to send external NED velocity to CV7-INS\n");

            prev_measurement_update_timestamp = current_timestamp;
        }

        running = !should_exit();
    }

    exit_gracefully("Example Completed Successfully.");

}

////////////////////////////////////////////////////////////////////////////////
// Utility functions
////////////////////////////////////////////////////////////////////////////////

uint64_t convert_gps_tow_to_nanoseconds(int week_number, float time_of_week)
{
    return floor(float(week_number) * 604800 * 1e9 + time_of_week * 1e9);
}

time_t time_from_ymd(int year, int month, int day)
{
    struct tm tm = {0};
    tm.tm_year = year - 1900;
    tm.tm_mon = month - 1;
    tm.tm_mday = day;
    return mktime(&tm);
}

#define SECS_PER_WEEK (60L*60*24*7)

int get_gps_week(int year, int month, int day)
{
    // See update below
    double diff = difftime(time_from_ymd(year, month, day), time_from_ymd(1980, 1, 1));  // See update
    return (int) (diff / SECS_PER_WEEK);
}

////////////////////////////////////////////////////////////////////////////////
// Print device information
////////////////////////////////////////////////////////////////////////////////

void print_device_information(const commands_base::BaseDeviceInfo& device_info)
{
    printf("Connected to:\n");

    auto print_info = [](const char* name, const char info[16])
    {
        char msg[17] = {0};
        std::strncpy(msg, info, 16);
        printf("  %s%s\n", name, msg);
    };

    print_info("Model name:       ", device_info.model_name);
    print_info("Model number:     ", device_info.model_number);
    print_info("Serial Number:    ", device_info.serial_number);
    print_info("Device Options:   ", device_info.device_options);
    print_info("Lot Number:       ", device_info.lot_number);

    printf(  "  Firmware version:           %d.%d.%d\n\n",
             (device_info.firmware_version / 1000),
             (device_info.firmware_version / 100) % 10,
             (device_info.firmware_version / 1)   % 100
    );
}

////////////////////////////////////////////////////////////////////////////////
// Parse input arguments
////////////////////////////////////////////////////////////////////////////////

InputArguments parse_input_arguments(int argc, const char* argv[])
{
    // TODO: Set max arg check for this.
    if (argc < 8)
    {
        usage(argv[0]);
        exit_gracefully(nullptr);
    }

    InputArguments input_arguments;

    // MIP device port parameters
    input_arguments.mip_device_port_name = argv[1];
    input_arguments.mip_device_baudrate = argv[2];

    // UBlox device port parameters
    input_arguments.ublox_device_port_name = argv[3];
    input_arguments.ublox_device_baudrate = argv[4];

    // GNSS antenna lever arm
    input_arguments.gnss_antenna_lever_arm[0] = std::stof(argv[5]);
    input_arguments.gnss_antenna_lever_arm[1] = std::stof(argv[6]);
    input_arguments.gnss_antenna_lever_arm[2] = std::stof(argv[7]);

    // Heading alignment method
    if (argc >= 9)
    {
        int heading_alignment_int = std::stoi(argv[8]);

        if (heading_alignment_int == 0)
            input_arguments.filter_heading_alignment_method = commands_filter::InitializationConfiguration::AlignmentSelector::KINEMATIC;
        else if (heading_alignment_int == 1)
            input_arguments.filter_heading_alignment_method = commands_filter::InitializationConfiguration::AlignmentSelector::MAGNETOMETER;
        else
            exit_gracefully("Heading alignment selector out of range");
    }

    // Output binary data filepath
    if (argc >= 10)
        input_arguments.mip_binary_filepath = argv[9];

    // PPS sync enable
    if (argc >= 11)
        input_arguments.enable_pps_sync = std::stoi(argv[10]);

    // PPS input pin ID
    if (argc >= 12)
        input_arguments.pps_input_pin_id = std::stoi(argv[11]);

    return input_arguments;
}


////////////////////////////////////////////////////////////////////////////////
// Print Usage Function
////////////////////////////////////////////////////////////////////////////////

int usage(const char* argv0)
{
    printf("Usage: %s <mip_port> <mip_baudrate> <ublox_port> <ublox_baudrate> <antenna_x> <antenna_y> <antenna_z> [OPTIONAL, (0=Kinematic, 1=Magnetometer)] <heading_alignment_method> [OPTIONAL] <binary_filepath> [OPTIONAL, (bool, (0|1)] <use_pps> [OPTIONAL, (int, 1-4)] <pps_pin_id> \n", argv0);
    return 1;
}


////////////////////////////////////////////////////////////////////////////////
// Exit Function
////////////////////////////////////////////////////////////////////////////////

void exit_gracefully(const char *message)
{
    if(message)
        printf("%s\n", message);

#ifdef _WIN32
    std::cout << "Press ENTER to exit..." << std::endl;
    int dummy = getchar();
#endif

    exit(0);
}


////////////////////////////////////////////////////////////////////////////////
// Check for Exit Condition
////////////////////////////////////////////////////////////////////////////////

bool should_exit()
{
    return false;
}

