
#include "mip_cmdqueue.h"

#include "mip_field.h"
#include "mip_packet.h"

#include <string.h>
#include <assert.h>


#define MIP_REPLY_DESC_GLOBAL_ACK_NACK 0xF1

#define MIP_INDEX_REPLY_DESCRIPTOR 0
#define MIP_INDEX_REPLY_ACK_CODE   1


////////////////////////////////////////////////////////////////////////////////
///@brief Initialize a pending command with no reponse data or additional time.
///
///@param cmd
///@param descriptor_set
///       Command descriptor set.
///@param field_descriptor
///       Command field descriptor.
///
void mip_pending_cmd_init(mip_pending_cmd* cmd, uint8_t descriptor_set, uint8_t field_descriptor)
{
    mip_pending_cmd_init_full(cmd, descriptor_set, field_descriptor, 0x00, NULL, 0, 0);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Initialize a pending mip commmand with extra timeout time.
///
///@param cmd
///@param descriptor_set
///       Command descriptor set.
///@param field_descriptor
///       Command field descriptor.
///@param additional_time
///       Additional time on top of the base reply timeout for this specific command.
///
void mip_pending_cmd_init_with_timeout(mip_pending_cmd* cmd, uint8_t descriptor_set, uint8_t field_descriptor, mip_timeout additional_time)
{
    mip_pending_cmd_init_full(cmd, descriptor_set, field_descriptor, 0x00, NULL, 0, additional_time);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Initialize a pending mip commmand with expected response data.
///
///@param cmd
///@param descriptor_set
///       Command descriptor set.
///@param field_descriptor
///       Command field descriptor.
///@param response_descriptor
///       Optional response data descriptor. Use 0x00 if no data is expected.
///@param response_buffer
///       Optional buffer to hold response data, if any. If NULL, response_buffer_size must be 0.
///@param response_buffer_size
///       Size of the response buffer. The response will be limited to this size.
///
void mip_pending_cmd_init_with_response(mip_pending_cmd* cmd, uint8_t descriptor_set, uint8_t field_descriptor, uint8_t response_descriptor, uint8_t* response_buffer, uint8_t response_buffer_size)
{
    mip_pending_cmd_init_full(cmd, descriptor_set, field_descriptor, response_descriptor, response_buffer, response_buffer_size, 0);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Initialize a pending mip commmand with all parameters.
///
///@param cmd
///@param descriptor_set
///       Command descriptor set.
///@param field_descriptor
///       Command field descriptor.
///@param response_descriptor
///       Optional response data descriptor. Use 0x00 if no data is expected.
///@param response_buffer
///       Optional buffer to hold response data, if any. If NULL, response_buffer_size must be 0.
///@param response_buffer_size
///       Size of the response buffer. The response will be limited to this size.
///@param additional_time
///       Additional time on top of the base reply timeout for this specific command.
///
void mip_pending_cmd_init_full(mip_pending_cmd* cmd, uint8_t descriptor_set, uint8_t field_descriptor, uint8_t response_descriptor, uint8_t* response_buffer, uint8_t response_buffer_size, mip_timeout additional_time)
{
    cmd->_next                 = NULL;
    cmd->_response_buffer      = NULL;
    cmd->_extra_timeout        = additional_time;
    cmd->_descriptor_set       = descriptor_set;
    cmd->_field_descriptor     = field_descriptor;
    cmd->_response_descriptor  = response_descriptor;
    cmd->_response_buffer      = response_buffer;
    cmd->_response_buffer_size = response_buffer_size;
    // cmd->_ack_code            = 0xFF; // invalid
    cmd->_status               = MIP_STATUS_NONE;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Returns the status of the pending command.
///
///@see mip_cmd_status
///
enum mip_cmd_result mip_pending_cmd_status(const mip_pending_cmd* cmd)
{
    return cmd->_status;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the response descriptor.
///
uint8_t mip_pending_cmd_response_descriptor(const mip_pending_cmd* cmd)
{
    return cmd->_response_descriptor;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the response payload pointer.
///
/// This function may only be called after the command finishes with an ACK.
///
const uint8_t* mip_pending_cmd_response(const mip_pending_cmd* cmd)
{
    assert(mip_cmd_result_is_finished(cmd->_status));

    return cmd->_response_buffer;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the length of the response data.
///
/// This function may only be called after the command finishes.
/// If the command completed with a NACK, or if it timed out, the response
/// length will be zero.
///
uint8_t mip_pending_cmd_response_length(const mip_pending_cmd* cmd)
{
    assert(mip_cmd_result_is_finished(cmd->_status));

    return cmd->_response_length;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines how much time is remaining before the command times out.
///
/// For most cases you should instead call mip_pending_cmd_check_timeout() to
/// know if the command has timed out or not.
///
///@param cmd The command to check. Must be in MIP_STATUS_WAITING state.
///@param now The current timestamp.
///
///@warning The command must be in the MIP_STATUS_WAITING state, otherwise the
///         result is unspecified.
///
///@returns The time remaining before the command times out. The value will be
///         negative if the timeout time has passed.
///
int mip_pending_cmd_remaining_time(const mip_pending_cmd* cmd, mip_timestamp now)
{
    assert(cmd->_status == MIP_STATUS_WAITING);

    // result <= 0 if timed out.
    // Note: this still works with unsigned overflow.
    return (int)(now - cmd->_timeout_time);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Checks if the command should time out.
///
///@param cmd
///@param now Current time
///
///@returns true if the command should time out. Only possible for MIP_STATUS_WAITING.
///@returns false if the command should not time out.
///
bool mip_pending_cmd_check_timeout(const mip_pending_cmd* cmd, mip_timestamp now)
{
    if( cmd->_status == MIP_STATUS_WAITING )
    {
        if( mip_pending_cmd_remaining_time(cmd, now) > 0 )
        {
            return true;
        }
    }

    return false;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Initializes a command queue.
///
///@param queue
///@param base_reply_timeout
///       The minimum timeout given to all MIP commands. Additional time may be
///       given to specific commands which take longer. This is intended to be
///       used to accommodate long communication latencies, such as when using
///       a TCP connection.
///
void mip_cmd_queue_init(mip_cmd_queue* queue, mip_timeout base_reply_timeout)
{
    queue->_first_pending_cmd = NULL;
    queue->_base_timeout = base_reply_timeout;

    MIP_DIAG_ZERO(queue->_diag_cmds_queued);
    MIP_DIAG_ZERO(queue->_diag_cmds_acked);
    MIP_DIAG_ZERO(queue->_diag_cmds_nacked);
    MIP_DIAG_ZERO(queue->_diag_cmds_timedout);
    MIP_DIAG_ZERO(queue->_diag_cmds_failed);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Queue a command to wait for replies.
///
///@param queue
///@param cmd Listens for replies to this command.
///
///@warning The command must not be deallocated or go out of scope while the
///         mip_cmd_status_is_finished returns false.
///
void mip_cmd_queue_enqueue(mip_cmd_queue* queue, mip_pending_cmd* cmd)
{
    // For now only one command can be queued at a time.
    if( queue->_first_pending_cmd )
    {
        cmd->_status = MIP_STATUS_CANCELLED;
        return;
    }

    MIP_DIAG_INC(queue->_diag_cmds_queued, 1);

    cmd->_status = MIP_STATUS_PENDING;
    queue->_first_pending_cmd = cmd;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Removes a pending command from the queue.
///
///@internal
///
///@param queue
///@param cmd
///
void mip_cmd_queue_dequeue(mip_cmd_queue* queue, mip_pending_cmd* cmd)
{
    if( queue->_first_pending_cmd == cmd )
    {
        queue->_first_pending_cmd = NULL;
        cmd->_status = MIP_STATUS_CANCELLED;
    }
}

////////////////////////////////////////////////////////////////////////////////
///@brief Iterate over a packet, checking for replies to the pending command.
///
///@internal
///
///@param pending
///       Pending command which is awaiting replies
///@param packet
///@param base_timeout
///@param timestamp
///
///@returns The new status of the pending command (the command status field is
///         not updated). The caller should set pending->_status to this value
///         after doing any additional processing requiring the pending struct.
///
static enum mip_cmd_result process_fields_for_pending_cmd(mip_pending_cmd* pending, const mip_packet* packet, mip_timeout base_timeout, mip_timestamp timestamp)
{
    assert( pending->_status != MIP_STATUS_NONE );         // pending->_status must be set to MIP_STATUS_PENDING in mip_cmd_queue_enqueue to get here.
    assert( !mip_cmd_result_is_finished(pending->_status) );  // Command shouldn't be finished yet - make sure the queue is processed properly.

    if( pending->_status == MIP_STATUS_PENDING )
    {
        // Update the timeout to the timestamp of the timeout time.
        pending->_timeout_time = timestamp + base_timeout + pending->_extra_timeout;
        pending->_status = MIP_STATUS_WAITING;
    }

    // ------+------+------+------+------+------+------+------+------+------------------------
    //  ...  | 0x02 | 0xF1 | cmd1 | nack | 0x02 | 0xF1 | cmd2 |  ack |  response field ...
    // ------+------+------+------+------+------+------+------+------+------------------------

    if( mip_packet_descriptor_set(packet) == pending->_descriptor_set )
    {
        mip_field field = {0};
        while( mip_field_next_in_packet(&field, packet) )
        {
            // Not an ack/nack reply field, skip it.
            if( mip_field_field_descriptor(&field) != MIP_REPLY_DESC_GLOBAL_ACK_NACK )
                continue;

            // Sanity check payload length before accessing it.
            if( mip_field_payload_length(&field) != 2 )
                continue;

            const uint8_t* const payload = mip_field_payload(&field);

            const uint8_t cmd_descriptor = payload[MIP_INDEX_REPLY_DESCRIPTOR];
            const uint8_t ack_code       = payload[MIP_INDEX_REPLY_ACK_CODE];

            // Is this the right command reply?
            if( pending->_field_descriptor != cmd_descriptor )
                continue;

            // Descriptor matches!

            uint8_t response_length = 0;
            mip_field response_field;

            // If the command was ACK'd, check if response data is expected.
            if( pending->_response_descriptor != 0x00 && ack_code == MIP_ACK_OK )
            {
                // Look ahead one field for response data.
                response_field = mip_field_next_after(&field);
                if( mip_field_is_valid(&response_field) )
                {
                    const uint8_t response_descriptor = mip_field_field_descriptor(&response_field);

                    // This is a wildcard to accept any response data descriptor.
                    // Needed when the response descriptor is not known or is wrong.
                    if( pending->_response_descriptor == MIP_REPLY_DESC_GLOBAL_ACK_NACK )
                        pending->_response_descriptor = response_descriptor;

                    // Make sure the response descriptor matches what is expected.
                    if( response_descriptor == pending->_response_descriptor )
                    {
                        // Update the response_size field to reflect the actual size.
                        response_length = mip_field_payload_length(&response_field);

                        // Skip this field when iterating for next ack/nack reply.
                        field = response_field;
                    }
                }
            }

            // Limit response data size to lesser of buffer size or actual response length.
            pending->_response_length = (response_length < pending->_response_buffer_size) ? response_length : pending->_response_buffer_size;

            // Copy response data to the pending buffer (skip if response_field is invalid).
            if( pending->_response_length > 0 )
                memcpy(pending->_response_buffer, mip_field_payload(&response_field), pending->_response_length);

            // pending->_ack_code   = ack_code;
            pending->_reply_time = timestamp;  // Completion time

            return (enum mip_cmd_result)ack_code;
        }
    }

    // No matching reply descriptors in this packet.

    // Check for timeout
    if( mip_pending_cmd_check_timeout(pending, timestamp) )
    {
        pending->_response_length = 0;
        // pending->_ack_code        = MIP_NACK_COMMAND_TIMEOUT;

        // Must be last!
        return MIP_STATUS_TIMEDOUT;
    }

    return pending->_status;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Process an incoming packet and check for replies to pending commands.
///
/// Call this from the Mip_parser callback, passing the arguments directly.
///
///@param queue
///@param packet The received MIP packet. Assumed to be valid.
///@param timestamp The time the packet was received
///
void mip_cmd_queue_process_packet(mip_cmd_queue* queue, const mip_packet* packet, mip_timestamp timestamp)
{
    // Check if the packet is a command descriptor set.
    const uint8_t descriptor_set = mip_packet_descriptor_set(packet);
    if( descriptor_set >= 0x80 && descriptor_set < 0xF0 )
        return;

    if( queue->_first_pending_cmd )
    {
        mip_pending_cmd* pending = queue->_first_pending_cmd;

        const enum mip_cmd_result status = process_fields_for_pending_cmd(pending, packet, queue->_base_timeout, timestamp);

        if( mip_cmd_result_is_finished(status) )
        {
            queue->_first_pending_cmd = queue->_first_pending_cmd->_next;

            // This must be done last b/c it may trigger the thread which queued the command.
            // The command could go out of scope or its attributes inspected.
            pending->_status = status;

#ifdef MIP_ENABLE_DIAGNOSTICS
            if( mip_cmd_result_is_ack(status) )
                MIP_DIAG_INC(queue->_diag_cmds_acked, 1);
            else if( mip_cmd_result_is_reply(status) )
                MIP_DIAG_INC(queue->_diag_cmds_nacked, 1);
            else if( status == MIP_STATUS_TIMEDOUT )
                MIP_DIAG_INC(queue->_diag_cmds_timedout, 1);
            else
                MIP_DIAG_INC(queue->_diag_cmds_failed, 1);
#endif // MIP_ENABLE_DIAGNOSTICS
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
///@brief Clears the command queue.
///
/// This must be called from the same thread context as the update function.
///
///@param queue
///
void mip_cmd_queue_clear(mip_cmd_queue* queue)
{
    while( queue->_first_pending_cmd )
    {
        mip_pending_cmd* pending = queue->_first_pending_cmd;
        queue->_first_pending_cmd = pending->_next;

        // This may deallocate the pending command in another thread (make sure to fetch the next cmd first).
        pending->_status = MIP_STATUS_CANCELLED;
    }
}

////////////////////////////////////////////////////////////////////////////////
///@brief Call periodically to make sure commands time out if no packets are
///       received.
///
/// Call this during the device update if no calls to mip_cmd_queue_process_packet
/// are made (e.g. because no packets were received). It is safe to call this
/// in either case.
///
///@param queue
///@param now
///
void mip_cmd_queue_update(mip_cmd_queue* queue, mip_timestamp now)
{
    if( queue->_first_pending_cmd )
    {
        mip_pending_cmd* pending = queue->_first_pending_cmd;

        if( pending->_status == MIP_STATUS_PENDING )
        {
            // Update the timeout to the timestamp of the timeout time.
            pending->_timeout_time = now + queue->_base_timeout + pending->_extra_timeout;
            pending->_status = MIP_STATUS_WAITING;
        }
        else if( mip_pending_cmd_check_timeout(pending, now) )
        {
            queue->_first_pending_cmd = queue->_first_pending_cmd->_next;

            // Clear response length and mark when it timed out.
            pending->_response_length = 0;
            pending->_reply_time = now;

            // This must be last!
            pending->_status = MIP_STATUS_TIMEDOUT;

            MIP_DIAG_INC(queue->_diag_cmds_timedout, 1);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
///@brief Sets the base reply timeout for all commands.
///
/// The base reply timeout is the minimum time to wait for a reply.
/// Takes effect for any commands queued after this function call.
///
///@param queue
///@param timeout
///
void mip_cmd_queue_set_base_reply_timeout(mip_cmd_queue* queue, mip_timeout timeout)
{
    queue->_base_timeout = timeout;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the base reply timeout for all commands.
///
///@returns The minimum time to wait for a reply to any command.
///
mip_timeout mip_cmd_queue_base_reply_timeout(const mip_cmd_queue* queue)
{
    return queue->_base_timeout;
}


#ifdef MIP_ENABLE_DIAGNOSTICS

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the number of commands ever put into the queue.
///
/// In most cases this is the number of commands sent to the device.
///
uint16_t mip_cmd_queue_diagnostic_cmds_queued(const mip_cmd_queue* queue)
{
    return queue->_diag_cmds_queued;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the number of commands that have failed for any reason.
///
uint16_t mip_cmd_queue_diagnostic_cmds_failed(const mip_cmd_queue* queue)
{
    return (uint16_t)queue->_diag_cmds_nacked + queue->_diag_cmds_failed + queue->_diag_cmds_timedout;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the number of successful commands.
///
/// Same as mip_cmd_queue_diagnostic_cmd_acks().
///
uint16_t mip_cmd_queue_diagnostic_cmds_successful(const mip_cmd_queue* queue)
{
    return queue->_diag_cmds_acked;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the number of successful commands.
///
/// Same as mip_cmd_queue_diagnostic_cmds_successful().
///
uint16_t mip_cmd_queue_diagnostic_cmd_acks(const mip_cmd_queue* queue)
{
    return queue->_diag_cmds_acked;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the number of commands nack'd by the device.
///
uint16_t mip_cmd_queue_diagnostic_cmd_nacks(const mip_cmd_queue* queue)
{
    return queue->_diag_cmds_nacked;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the number of commands that did not receive a reply within the
///       time limit.
///
uint16_t mip_cmd_queue_diagnostic_cmd_timeouts(const mip_cmd_queue* queue)
{
    return queue->_diag_cmds_timedout;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the number of command errors not caused by the device.
///
uint16_t mip_cmd_queue_diagnostic_cmd_errors(const mip_cmd_queue* queue)
{
    return queue->_diag_cmds_failed;
}

#endif // MIP_ENABLE_DIAGNOSTICS
