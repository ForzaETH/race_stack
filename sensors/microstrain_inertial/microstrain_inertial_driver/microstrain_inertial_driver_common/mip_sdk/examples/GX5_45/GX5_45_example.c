
/////////////////////////////////////////////////////////////////////////////
//
// GX5_45_Example.c
//
// C Example set-up program for the GX5-45
//
// This example shows a typical setup for the GX5-45 sensor in a wheeled-vehicle application using using C.
// It is not an exhaustive example of all GX5-45 settings.
// If your specific setup needs are not met by this example, please consult
// the MSCL-embedded API documentation for the proper commands.
//
//
//!@section LICENSE
//!
//! THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
//! CUSTOMERS WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER
//! FOR THEM TO SAVE TIME. AS A RESULT, PARKER MICROSTRAIN SHALL NOT BE HELD
//! LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY
//! CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS
//! OF THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
//
/////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Include Files
////////////////////////////////////////////////////////////////////////////////

#include <mip/mip_all.h>
#include <mip/utils/serial_port.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>


////////////////////////////////////////////////////////////////////////////////
// Global Variables
////////////////////////////////////////////////////////////////////////////////

serial_port device_port;
clock_t start_time;

int port = -1;
uint8_t parse_buffer[1024];
mip_interface device;

//Sensor-to-vehicle frame rotation (Euler Angles)
float sensor_to_vehicle_rotation_euler[3] = {0.0, 0.0, 0.0};

//GNSS antenna offset
float gnss_antenna_offset_meters[3] = {-0.25, 0.0, 0.0};

//Device data stores
mip_sensor_gps_timestamp_data sensor_gps_time;
mip_sensor_scaled_accel_data  sensor_accel;
mip_sensor_scaled_gyro_data   sensor_gyro;
mip_sensor_scaled_mag_data    sensor_mag;

mip_gnss_fix_info_data        gnss_fix_info;

bool gnss_fix_info_valid = false;

mip_filter_timestamp_data     filter_gps_time;
mip_filter_status_data        filter_status;
mip_filter_position_llh_data  filter_position_llh;
mip_filter_velocity_ned_data  filter_velocity_ned;
mip_filter_euler_angles_data  filter_euler_angles;

bool filter_state_running = false;


////////////////////////////////////////////////////////////////////////////////
// Function Prototypes
////////////////////////////////////////////////////////////////////////////////


//Required MIP interface user-defined functions
mip_timestamp get_current_timestamp();

bool mip_interface_user_recv_from_device(mip_interface* device, uint8_t* buffer, size_t max_length, mip_timeout wait_time, size_t* out_length, mip_timestamp* timestamp_out);
bool mip_interface_user_send_to_device(mip_interface* device, const uint8_t* data, size_t length);

int usage(const char* argv0);

void exit_gracefully(const char *message);
bool should_exit();


////////////////////////////////////////////////////////////////////////////////
// Main Function
////////////////////////////////////////////////////////////////////////////////


int main(int argc, const char* argv[])
{

    //
    //Process arguments
    //

    if(argc != 3)
        return usage(argv[0]);

    const char* port_name = argv[1];
    uint32_t baudrate     = atoi(argv[2]);

    if(baudrate == 0)
        return usage(argv[0]);

    //
    //Get the program start time
    //

    start_time = clock();

    printf("Connecting to and configuring sensor.\n");

    //
    //Open the device port
    //

    if(!serial_port_open(&device_port, port_name, baudrate))
        exit_gracefully("ERROR: Could not open device port!");


    //
    //Initialize the MIP interface
    //

    mip_interface_init(
        &device, parse_buffer, sizeof(parse_buffer), mip_timeout_from_baudrate(baudrate), 1000,
        &mip_interface_user_send_to_device, &mip_interface_user_recv_from_device, &mip_interface_default_update, NULL
    );



    //
    //Ping the device (note: this is good to do to make sure the device is present)
    //

    if(mip_base_ping(&device) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not ping the device!");


    //
    //Idle the device (note: this is good to do during setup)
    //

    if(mip_base_set_idle(&device) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set the device to idle!");


    //
    //Load the device default settings (so the device is in a known state)
    //

    if(mip_3dm_default_device_settings(&device) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not load default device settings!");


    //
    //Setup Sensor data format to 100 Hz
    //

    uint16_t sensor_base_rate;

    //Note: Querying the device base rate is only one way to calculate the descriptor decimation.
    //We could have also set it directly with information from the datasheet (shown in GNSS setup).

    if(mip_3dm_imu_get_base_rate(&device, &sensor_base_rate) != MIP_ACK_OK)
         exit_gracefully("ERROR: Could not get sensor base rate format!");

    const uint16_t sensor_sample_rate = 100; // Hz
    const uint16_t sensor_decimation = sensor_base_rate / sensor_sample_rate;

    const mip_descriptor_rate sensor_descriptors[4] = {
        { MIP_DATA_DESC_SENSOR_TIME_STAMP_GPS, sensor_decimation },
        { MIP_DATA_DESC_SENSOR_ACCEL_SCALED,   sensor_decimation },
        { MIP_DATA_DESC_SENSOR_GYRO_SCALED,    sensor_decimation },
        { MIP_DATA_DESC_SENSOR_MAG_SCALED,     sensor_decimation },
    };

    if(mip_3dm_write_imu_message_format(&device, 4, sensor_descriptors) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set sensor message format!");


    //
    //Setup GNSS data format to 4 Hz (decimation of 1)
    //

    const mip_descriptor_rate gnss_descriptors[1] = {
        { MIP_DATA_DESC_GNSS_FIX_INFO, 1 }
     };

    //GNSS
    if(mip_3dm_write_gps_message_format(&device, 1, gnss_descriptors) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set GNSS1 message format!");


    //
    //Setup FILTER data format
    //

    uint16_t filter_base_rate;

    if(mip_3dm_filter_get_base_rate(&device, &filter_base_rate) != MIP_ACK_OK)
         exit_gracefully("ERROR: Could not get filter base rate format!");

    const uint16_t filter_sample_rate = 100; // Hz
    const uint16_t filter_decimation = filter_base_rate / filter_sample_rate;

    const mip_descriptor_rate filter_descriptors[5] = {
        { MIP_DATA_DESC_FILTER_FILTER_TIMESTAMP, filter_decimation },
        { MIP_DATA_DESC_FILTER_FILTER_STATUS,    filter_decimation },
        { MIP_DATA_DESC_FILTER_POS_LLH,          filter_decimation },
        { MIP_DATA_DESC_FILTER_VEL_NED,          filter_decimation },
        { MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES, filter_decimation },
    };

    if(mip_3dm_write_filter_message_format(&device, 5, filter_descriptors) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set filter message format!");


    //
    //Setup the sensor to vehicle rotation
    //

    if(mip_filter_write_sensor_to_vehicle_rotation_euler(&device, sensor_to_vehicle_rotation_euler[0], sensor_to_vehicle_rotation_euler[1], sensor_to_vehicle_rotation_euler[2]) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set sensor-2-vehicle rotation!");


    //
    //Setup the GNSS antenna offset
    //

    if(mip_filter_write_antenna_offset(&device, gnss_antenna_offset_meters) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set GNSS1 antenna offset!");


    //
    //Setup heading update control
    //

    if(mip_filter_write_heading_source(&device, MIP_FILTER_HEADING_SOURCE_COMMAND_SOURCE_GNSS_VEL_AND_MAG) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set filter heading update control!");


    //
    //Enable filter auto-initialization
    //

    if(mip_filter_write_auto_init_control(&device, 1) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set filter autoinit control!");


    //
    //Reset the filter (note: this is good to do after filter setup is complete)
    //

    if(mip_filter_reset(&device) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not reset the filter!");


    //
    // Register data callbacks
    //

    //Sensor Data
    mip_dispatch_handler sensor_data_handlers[4];

    mip_interface_register_extractor(&device, &sensor_data_handlers[0], MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_TIME_STAMP_GPS, extract_mip_sensor_gps_timestamp_data_from_field,    &sensor_gps_time);
    mip_interface_register_extractor(&device, &sensor_data_handlers[1], MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_ACCEL_SCALED,   extract_mip_sensor_scaled_accel_data_from_field, &sensor_accel);
    mip_interface_register_extractor(&device, &sensor_data_handlers[2], MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_GYRO_SCALED,    extract_mip_sensor_scaled_gyro_data_from_field,  &sensor_gyro);
    mip_interface_register_extractor(&device, &sensor_data_handlers[3], MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_MAG_SCALED,     extract_mip_sensor_scaled_mag_data_from_field,   &sensor_mag);

    //GNSS Data
    mip_dispatch_handler gnss_data_handlers[1];

    mip_interface_register_extractor(&device, &gnss_data_handlers[0], MIP_GNSS1_DATA_DESC_SET, MIP_DATA_DESC_GNSS_FIX_INFO, extract_mip_gnss_fix_info_data_from_field, &gnss_fix_info);

    //Filter Data
    mip_dispatch_handler filter_data_handlers[5];

    mip_interface_register_extractor(&device, &filter_data_handlers[0], MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_FILTER_TIMESTAMP, extract_mip_filter_timestamp_data_from_field,     &filter_gps_time);
    mip_interface_register_extractor(&device, &filter_data_handlers[1], MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_FILTER_STATUS,    extract_mip_filter_status_data_from_field,        &filter_status);
    mip_interface_register_extractor(&device, &filter_data_handlers[2], MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_POS_LLH,          extract_mip_filter_position_llh_data_from_field,  &filter_position_llh);
    mip_interface_register_extractor(&device, &filter_data_handlers[3], MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_VEL_NED,          extract_mip_filter_velocity_ned_data_from_field,  &filter_velocity_ned);
    mip_interface_register_extractor(&device, &filter_data_handlers[4], MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES, extract_mip_filter_euler_angles_data_from_field,  &filter_euler_angles);

    //
    //Resume the device
    //

    if(mip_base_resume(&device) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not resume the device!");


    //
    //Main Loop: Update the interface and process data
    //

    bool          running              = true;
    mip_timestamp prev_print_timestamp = 0;

    printf("Sensor is configured... waiting for filter to enter running mode.\n");

    while(running)
    {
        mip_interface_update(&device, false);


        //Check GNSS fixes and alert the user when they become valid
        if((gnss_fix_info_valid == false) && (gnss_fix_info.fix_type == MIP_GNSS_FIX_INFO_DATA_FIX_TYPE_FIX_3D) &&
            (gnss_fix_info.valid_flags & MIP_GNSS_FIX_INFO_DATA_VALID_FLAGS_FIX_TYPE))
        {
            printf("NOTE: GNSS fix info valid.\n");
            gnss_fix_info_valid = true;
        }

        //Check Filter State
        if((!filter_state_running) && ((filter_status.filter_state == MIP_FILTER_MODE_GX5_RUN_SOLUTION_ERROR) || (filter_status.filter_state == MIP_FILTER_MODE_GX5_RUN_SOLUTION_VALID)))
        {
            printf("NOTE: Filter has entered running mode.\n");
            filter_state_running = true;
        }

        //Once in running mode, print out data at 1 Hz
        if(filter_state_running)
        {
           mip_timestamp curr_time = get_current_timestamp();

           if(curr_time - prev_print_timestamp >= 1000)
           {
                printf("TOW = %f: POS_LLH = [%f, %f, %f], VEL_NED = [%f, %f, %f], ATT_EULER = [%f %f %f]\n",
                       filter_gps_time.tow, filter_position_llh.latitude, filter_position_llh.longitude, filter_position_llh.ellipsoid_height,
                       filter_velocity_ned.north, filter_velocity_ned.east, filter_velocity_ned.down,
                       filter_euler_angles.roll, filter_euler_angles.pitch, filter_euler_angles.yaw);

                prev_print_timestamp = curr_time;
           }
        }

        running = !should_exit();
    }

    exit_gracefully("Example Completed Successfully.");
}


////////////////////////////////////////////////////////////////////////////////
// MIP Interface Time Access Function
////////////////////////////////////////////////////////////////////////////////

mip_timestamp get_current_timestamp()
{
    clock_t curr_time;
    curr_time = clock();

    return (mip_timestamp)((double)(curr_time - start_time) / (double)CLOCKS_PER_SEC * 1000.0);
}


////////////////////////////////////////////////////////////////////////////////
// MIP Interface User Recv Data Function
////////////////////////////////////////////////////////////////////////////////

bool mip_interface_user_recv_from_device(mip_interface* device, uint8_t* buffer, size_t max_length, mip_timeout wait_time, size_t* out_length, mip_timestamp* timestamp_out)
{
    *timestamp_out = get_current_timestamp();
    return serial_port_read(&device_port, buffer, max_length, wait_time, out_length);
}


////////////////////////////////////////////////////////////////////////////////
// MIP Interface User Send Data Function
////////////////////////////////////////////////////////////////////////////////

bool mip_interface_user_send_to_device(mip_interface* device, const uint8_t* data, size_t length)
{
    size_t bytes_written;

    return serial_port_write(&device_port, data, length, &bytes_written);
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

    //Close com port
    if(serial_port_is_open(&device_port))
        serial_port_close(&device_port);

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

