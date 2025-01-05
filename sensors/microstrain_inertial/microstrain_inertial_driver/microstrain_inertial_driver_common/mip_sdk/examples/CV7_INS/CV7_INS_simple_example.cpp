/////////////////////////////////////////////////////////////////////////////
//
// CV7_INS_simple_example.cpp
//
// C++ Example usage program for the CV7-INS
//
// This example shows the basic setup for a CV-INS sensor using external aiding measurements.
// It is not an exhaustive example of all CV7 settings.
// If your specific setup needs are not met by this example, please consult
// the MSCL-embedded API documentation for the proper commands.
//
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
#include <array>
#include "../example_utils.hpp"

using namespace mip;

////////////////////////////////////////////////////////////////////////////////
// Global Variables
////////////////////////////////////////////////////////////////////////////////

data_shared::GpsTimestamp filter_gps_time;
data_filter::Status       filter_status;
data_filter::EulerAngles  filter_euler_angles;
data_filter::PositionLlh  filter_llh_position;
data_filter::VelocityNed  filter_ned_velocity;

uint8_t external_heading_sensor_id = 1;
uint8_t gnss_antenna_sensor_id = 2;
uint8_t vehicle_frame_velocity_sensor_id = 3;

bool filter_state_full_nav = false;

////////////////////////////////////////////////////////////////////////////////
// Function Prototypes
////////////////////////////////////////////////////////////////////////////////

int usage(const char* argv0);

void print_device_information(const commands_base::BaseDeviceInfo& device_info);

void exit_gracefully(const char *message);
bool should_exit();

////////////////////////////////////////////////////////////////////////////////
// Main Function
////////////////////////////////////////////////////////////////////////////////


int main(int argc, const char* argv[])
{

    std::unique_ptr<ExampleUtils> utils;
    try {
        utils = handleCommonArgs(argc, argv);
    } catch(const std::underflow_error& ex) {
        return printCommonUsage(argv);
    } catch(const std::exception& ex) {
        fprintf(stderr, "Error: %s\n", ex.what());
        return 1;
    }

    std::unique_ptr<mip::DeviceInterface>& device = utils->device;

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
    //Setup external aiding sensor reference frames
    //


    //
    //External heading sensor reference frame.
    //
    commands_aiding::FrameConfig::Rotation external_heading_sensor_to_vehicle_frame_rotation;
    external_heading_sensor_to_vehicle_frame_rotation.euler = mip::Vector3f(0.0f, 0.0f, 0.0f);  // External heading sensor is aligned with vehicle frame
    float external_heading_sensor_to_vehicle_frame_translation[3] = {0.0, 0.0, 0.0};  // Heading measurements are agnostic to translation, translation set to zero
    if(commands_aiding::writeFrameConfig(*device, external_heading_sensor_id, mip::commands_aiding::FrameConfig::Format::EULER, true,
                                            external_heading_sensor_to_vehicle_frame_translation, external_heading_sensor_to_vehicle_frame_rotation) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Unable to configure external heading sensor frame ID");


    //
    //External GNSS antenna reference frame
    //
    commands_aiding::FrameConfig::Rotation external_gnss_antenna_to_vehicle_frame_rotation;
    external_gnss_antenna_to_vehicle_frame_rotation.euler = mip::Vector3f(0.0f, 0.0f, 0.0f);  // GNSS position/velocity measurements are agnostic to rotation, rotation set to zero
    float external_gnss_antenna_to_vehicle_frame_translation[3] = {0.0, 1.0, 0.0};  // Antenna is translated 1 meter in vehicle frame Y direction
    if(commands_aiding::writeFrameConfig(*device, gnss_antenna_sensor_id, mip::commands_aiding::FrameConfig::Format::EULER, true,
                                            external_gnss_antenna_to_vehicle_frame_translation, external_gnss_antenna_to_vehicle_frame_rotation) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Unable to configure external GNSS antenna frame ID");


    //
    //External bodyframe velocity reference frame
    //
    commands_aiding::FrameConfig::Rotation external_velocity_sensor_to_vehicle_frame_rotation;
    external_gnss_antenna_to_vehicle_frame_rotation.euler= mip::Vector3f(0.0f, 0.0f, 1.57f);  // Rotated 90 deg around yaw axis
    float external_velocity_sensor_to_vehicle_frame_translation[3] = {1.0, 0.0, 0.0};  // Sensor is translated 1 meter in X direction
    if(commands_aiding::writeFrameConfig(*device, vehicle_frame_velocity_sensor_id, mip::commands_aiding::FrameConfig::Format::EULER, true,
                                            external_velocity_sensor_to_vehicle_frame_translation, external_velocity_sensor_to_vehicle_frame_rotation) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Unable to configure external vehicle frame velocity sensor ID");


    //
    //Setup FILTER data format
    //

    uint16_t filter_base_rate;

    if(commands_3dm::getBaseRate(*device, data_filter::DESCRIPTOR_SET, &filter_base_rate) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not get filter base rate format!");

    const uint16_t filter_sample_rate = 100; // Hz
    const uint16_t filter_decimation = filter_base_rate / filter_sample_rate;

    std::array<DescriptorRate, 5> filter_descriptors = {{
                                                                { data_shared::DATA_GPS_TIME,         filter_decimation },
                                                                { data_filter::DATA_FILTER_STATUS,    filter_decimation },
                                                                { data_filter::DATA_ATT_EULER_ANGLES, filter_decimation },
                                                                { data_filter::DATA_POS_LLH,          filter_decimation },
                                                                { data_filter::DATA_VEL_NED,          filter_decimation },
                                                        }};

    if(commands_3dm::writeMessageFormat(*device, data_filter::DESCRIPTOR_SET, filter_descriptors.size(), filter_descriptors.data()) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set filter message format!");


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

    //
    //Resume the device
    //

    if(commands_base::resume(*device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not resume the device!");


    //
    //Main Loop: Update the interface and process data
    //

    bool running = true;
    mip::Timestamp prev_print_timestamp = getCurrentTimestamp();
    mip::Timestamp prev_measurement_update_timestamp = getCurrentTimestamp();

    printf("Sensor is configured... waiting for filter to initialize...\n");

    while(running)
    {
        device->update();

        //Check for full nav filter state transition
        if((!filter_state_full_nav) && (filter_status.filter_state == data_filter::FilterMode::FULL_NAV))
        {
            printf("NOTE: Filter has entered full navigation mode.\n");
            filter_state_full_nav = true;
        }

        // Check that enough time has elapsed to send a new measurement update
        mip::Timestamp current_timestamp = getCurrentTimestamp();
        mip::Timestamp elapsed_time_from_last_measurement_update = current_timestamp - prev_measurement_update_timestamp;
        mip::Timestamp elapsed_time_from_last_message_print = current_timestamp - prev_print_timestamp;

        if (elapsed_time_from_last_measurement_update > 500)
        {
            // Use measurement time of arrival for timestamping method
            commands_aiding::Time external_measurement_time;
            external_measurement_time.timebase = commands_aiding::Time::Timebase::TIME_OF_ARRIVAL;

            // External heading command
            float external_heading = 0.0;
            float external_heading_uncertainty = .001;
            if(commands_aiding::trueHeading(*device, external_measurement_time, external_heading_sensor_id, external_heading, external_heading_uncertainty, 1) != CmdResult::ACK_OK)
                printf("WARNING: Failed to send external heading to CV7-INS\n");

            // External position command
            double latitude = 44.43729093897896; // Lat/Lon for MicroStrain headquarters
            double longitude = -73.10628129871753;
            double height = 0.0;
            float llh_uncertainty[3] = {1.0, 1.0, 1.0};
            if(commands_aiding::llhPos(*device, external_measurement_time, gnss_antenna_sensor_id, latitude, longitude, height, llh_uncertainty, 1) != CmdResult::ACK_OK)
                printf("WARNING: Failed to send external position to CV7-INS\n");

            // External global velocity command
            float ned_velocity[3] = {0.0, 0.0, 0.0};
            float ned_velocity_uncertainty[3] = {0.1, 0.1, 0.1};
            if(commands_aiding::nedVel(*device, external_measurement_time,  gnss_antenna_sensor_id, ned_velocity, ned_velocity_uncertainty, 1) != CmdResult::ACK_OK)
                printf("WARNING: Failed to send external NED velocity to CV7-INS\n");

            // External vehicle frame velocity command
            float vehicle_frame_velocity[3] = {0.0, 0.0, 0.0};
            float vehicle_frame_velocity_uncertainty[3] = {0.1, 0.1, 0.1};
            if(commands_aiding::vehicleFixedFrameVelocity(*device, external_measurement_time, vehicle_frame_velocity_sensor_id, vehicle_frame_velocity, vehicle_frame_velocity_uncertainty, 1) != CmdResult::ACK_OK)
                printf("WARNING: Failed to send external vehicle frame velocity to CV7-INS\n");

            prev_measurement_update_timestamp = current_timestamp;
        }

        //Once in full nav, print out data at 1 Hz
        if((filter_status.filter_state == data_filter::FilterMode::FULL_NAV) && (elapsed_time_from_last_message_print >= 1000))
        {
            printf("\n\n****Filter navigation state****\n");
            printf("TIMESTAMP: %f\n", filter_gps_time.tow);
            printf("ATTITUDE_EULER = [%f %f %f]\n", filter_euler_angles.roll, filter_euler_angles.pitch, filter_euler_angles.yaw);
            printf("LLH_POSITION = [%f %f %f]\n", filter_llh_position.latitude, filter_llh_position.longitude, filter_llh_position.ellipsoid_height);
            printf("NED_VELOCITY = [%f %f %f]\n", filter_ned_velocity.north, filter_ned_velocity.east, filter_ned_velocity.down);

            prev_print_timestamp = current_timestamp;
        }

        running = !should_exit();
    }

    exit_gracefully("Example Completed Successfully.");
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
// Print Usage Function
////////////////////////////////////////////////////////////////////////////////

int usage(const char* argv0)
{
    printf("Usage: %s <port> <baudrate>\n", argv0);
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

