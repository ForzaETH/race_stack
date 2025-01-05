
/////////////////////////////////////////////////////////////////////////////
//
// CV7_Example.c
//
// C Example set-up program for the CV7
//
// This example shows a typical setup for the CV7 sensor using C.
// It is not an exhaustive example of all CV7 settings.
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

//Sensor-to-vehicle frame transformation (Euler Angles)
float sensor_to_vehicle_transformation_euler[3] = {0.0, 0.0, 0.0};

//Device data stores
mip_shared_gps_timestamp_data sensor_gps_time;
mip_sensor_scaled_accel_data  sensor_accel;
mip_sensor_scaled_gyro_data   sensor_gyro;
mip_sensor_scaled_mag_data    sensor_mag;

mip_shared_gps_timestamp_data filter_gps_time;
mip_filter_status_data        filter_status;
mip_filter_euler_angles_data  filter_euler_angles;

bool filter_state_ahrs = false;

const uint8_t FILTER_ROLL_EVENT_ACTION_ID  = 1;
const uint8_t FILTER_PITCH_EVENT_ACTION_ID = 2;


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

void handle_filter_event_source(void* user, const mip_field* field, mip_timestamp timestamp);


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

    if(mip_3dm_get_base_rate(&device, MIP_SENSOR_DATA_DESC_SET, &sensor_base_rate) != MIP_ACK_OK)
         exit_gracefully("ERROR: Could not get sensor base rate format!");

    const uint16_t sensor_sample_rate = 100; // Hz
    const uint16_t sensor_decimation = sensor_base_rate / sensor_sample_rate;

    const mip_descriptor_rate sensor_descriptors[4] = {
        { MIP_DATA_DESC_SHARED_GPS_TIME,     sensor_decimation },
        { MIP_DATA_DESC_SENSOR_ACCEL_SCALED, sensor_decimation },
        { MIP_DATA_DESC_SENSOR_GYRO_SCALED,  sensor_decimation },
        { MIP_DATA_DESC_SENSOR_MAG_SCALED,   sensor_decimation },
    };

    if(mip_3dm_write_message_format(&device, MIP_SENSOR_DATA_DESC_SET, 4, sensor_descriptors) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set sensor message format!");


    //
    //Setup FILTER data format
    //

    uint16_t filter_base_rate;

    if(mip_3dm_get_base_rate(&device, MIP_FILTER_DATA_DESC_SET, &filter_base_rate) != MIP_ACK_OK)
         exit_gracefully("ERROR: Could not get filter base rate format!");

    const uint16_t filter_sample_rate = 100; // Hz
    const uint16_t filter_decimation = filter_base_rate / filter_sample_rate;

    const mip_descriptor_rate filter_descriptors[3] = {
        { MIP_DATA_DESC_SHARED_GPS_TIME,         filter_decimation },
        { MIP_DATA_DESC_FILTER_FILTER_STATUS,    filter_decimation },
        { MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES, filter_decimation },
    };

    if(mip_3dm_write_message_format(&device, MIP_FILTER_DATA_DESC_SET, 3, filter_descriptors) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set filter message format!");


    //
    // Setup event triggers/actions on > 45 degrees filter pitch and roll Euler angles
    // (Note 1: we are reusing the event and action structs, since the settings for pitch/roll are so similar)
    // (Note 2: we are using the same value for event and action ids.  This is not necessary, but done here for convenience)
    //

    //EVENTS

    //Roll
    union mip_3dm_event_trigger_command_parameters event_params;
    event_params.threshold.type       = MIP_3DM_EVENT_TRIGGER_COMMAND_THRESHOLD_PARAMS_TYPE_WINDOW;
    event_params.threshold.desc_set   = MIP_FILTER_DATA_DESC_SET;
    event_params.threshold.field_desc = MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES;
    event_params.threshold.param_id   = 1;
    event_params.threshold.high_thres = -0.7853981;
    event_params.threshold.low_thres  = 0.7853981;

    if(mip_3dm_write_event_trigger(&device, FILTER_ROLL_EVENT_ACTION_ID, MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_THRESHOLD, &event_params) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set pitch event parameters!");

    //Pitch
    event_params.threshold.param_id = 2;

    if(mip_3dm_write_event_trigger(&device, FILTER_PITCH_EVENT_ACTION_ID, MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_THRESHOLD, &event_params) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set roll event parameters!");

    //ACTIONS

    //Roll
    union mip_3dm_event_action_command_parameters event_action;
    event_action.message.desc_set       = MIP_FILTER_DATA_DESC_SET;
    event_action.message.num_fields     = 1;
    event_action.message.descriptors[0] = MIP_DATA_DESC_SHARED_EVENT_SOURCE;
    event_action.message.decimation     = 0;

    if(mip_3dm_write_event_action(&device, FILTER_ROLL_EVENT_ACTION_ID, FILTER_ROLL_EVENT_ACTION_ID, MIP_3DM_EVENT_ACTION_COMMAND_TYPE_MESSAGE, &event_action) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set roll action parameters!");

    //Pitch
    if(mip_3dm_write_event_action(&device, FILTER_PITCH_EVENT_ACTION_ID, FILTER_PITCH_EVENT_ACTION_ID, MIP_3DM_EVENT_ACTION_COMMAND_TYPE_MESSAGE, &event_action) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set pitch action parameters!");

    //ENABLE EVENTS

    //Roll
    if(mip_3dm_write_event_control(&device, FILTER_ROLL_EVENT_ACTION_ID, MIP_3DM_EVENT_CONTROL_COMMAND_MODE_ENABLED) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not enable roll event!");

    //Pitch
    if(mip_3dm_write_event_control(&device, FILTER_PITCH_EVENT_ACTION_ID, MIP_3DM_EVENT_CONTROL_COMMAND_MODE_ENABLED) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not enable pitch event!");


    //
    //Setup the sensor to vehicle transformation
    //

    if(mip_3dm_write_sensor_2_vehicle_transform_euler(&device, sensor_to_vehicle_transformation_euler[0], sensor_to_vehicle_transformation_euler[1], sensor_to_vehicle_transformation_euler[2]) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set sensor-to-vehicle transformation!");


    //
    //Setup the filter aiding measurements (GNSS position/velocity and dual antenna [aka gnss heading])
    //

    if(mip_filter_write_aiding_measurement_enable(&device, MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_MAGNETOMETER, true) != MIP_ACK_OK)
        exit_gracefully("ERROR: Could not set filter aiding measurement enable!");


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

    mip_interface_register_extractor(&device, &sensor_data_handlers[0], MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SHARED_GPS_TIME,     extract_mip_shared_gps_timestamp_data_from_field, &sensor_gps_time);
    mip_interface_register_extractor(&device, &sensor_data_handlers[1], MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_ACCEL_SCALED, extract_mip_sensor_scaled_accel_data_from_field,  &sensor_accel);
    mip_interface_register_extractor(&device, &sensor_data_handlers[2], MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_GYRO_SCALED,  extract_mip_sensor_scaled_gyro_data_from_field,   &sensor_gyro);
    mip_interface_register_extractor(&device, &sensor_data_handlers[3], MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_MAG_SCALED,   extract_mip_sensor_scaled_mag_data_from_field,    &sensor_mag);

    //Filter Data
    mip_dispatch_handler filter_data_handlers[4];

    mip_interface_register_extractor(&device, &filter_data_handlers[0], MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_SHARED_GPS_TIME,         extract_mip_shared_gps_timestamp_data_from_field, &filter_gps_time);
    mip_interface_register_extractor(&device, &filter_data_handlers[1], MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_FILTER_STATUS,    extract_mip_filter_status_data_from_field,        &filter_status);
    mip_interface_register_extractor(&device, &filter_data_handlers[2], MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES, extract_mip_filter_euler_angles_data_from_field,  &filter_euler_angles);

    mip_interface_register_field_callback(&device, &filter_data_handlers[3], MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_SHARED_EVENT_SOURCE, handle_filter_event_source,  NULL);

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

    printf("Sensor is configured... waiting for filter to enter AHRS mode.\n");

    while(running)
    {
        mip_interface_update(&device, false);

        //Check Filter State
        if((!filter_state_ahrs) && (filter_status.filter_state == MIP_FILTER_MODE_AHRS))
        {
            printf("NOTE: Filter has entered AHRS mode.\n");
            filter_state_ahrs = true;
        }

        //Once in full nav, print out data at 10 Hz
        if(filter_state_ahrs)
        {
           mip_timestamp curr_time = get_current_timestamp();

           if(curr_time - prev_print_timestamp >= 100)
           {
                printf("TOW = %f: ATT_EULER = [%f %f %f]\n",
                       filter_gps_time.tow, filter_euler_angles.roll, filter_euler_angles.pitch, filter_euler_angles.yaw);

                prev_print_timestamp = curr_time;
           }
        }

        running = !should_exit();
    }


    exit_gracefully("Example Completed Successfully.");
}


////////////////////////////////////////////////////////////////////////////////
// Filter Event Source Field Handler
////////////////////////////////////////////////////////////////////////////////

void handle_filter_event_source(void* user, const mip_field* field, mip_timestamp timestamp)
{
    mip_shared_event_source_data data;

    if(extract_mip_shared_event_source_data_from_field(field, &data))
    {
      if(data.trigger_id == FILTER_ROLL_EVENT_ACTION_ID)
        printf("WARNING: Roll event triggered!\n");
      else if(data.trigger_id == FILTER_PITCH_EVENT_ACTION_ID)
        printf("WARNING: Pitch event triggered!\n");
    }
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

