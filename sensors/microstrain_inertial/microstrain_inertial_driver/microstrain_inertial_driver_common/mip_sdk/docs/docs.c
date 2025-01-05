////////////////////////////////////////////////////////////////////////////////
///@mainpage MIP SDK
///
/// Welcome to the official MIP Software Development Kit. This software package
/// provides everything you need to communicate with any MIP-compatible
/// MicroStrain inertial sensor.
/// See @ref mip_interface for details on how to get started.
///
///@par Main Features
///
///@li MIP packet creation
///@li Send commands using a single function
///@li Packet parsing and field iteration
///@li Data field deserialization
///@li Simple interface requires only two functions to be defined
///@li Can be used to parse offline binary files
///@li Dual C and C++ API for maximum usability, safety, flexibility, and convenience.
///@li Suitable for bare-metal microcontrollers (Minimal code size and memory footprint, No dynamic memory allocation, No dependence on any RTOS or threading)
///
///@section quickref_cpp Quick Reference [C++]
///
/// All C++ functions and classes reside within the mip namespace.
/// The C functions can be accessed via the mip::C namespace.
///
///@li @ref mip::DeviceInterface Top-level MIP interface class.
///@li @ref mip::PacketRef       An interface to a MIP packet for either transmission or reception.
///@li @ref mip::PacketBuf       Similar to PacketRef but includes the data buffer.
///@li @ref mip::Field           An interface to a MIP field within a packet.
///@li @ref mip::Parser          MIP parser class for converting received bytes into packets.
///@li @ref mip::CmdResult       Stores the status or result of a MIP command.
///
///@section quickref_c Quick Reference [C]
///
/// C does not support the equivalent of C++ namespaces, so all definitions are
/// global. Most names start with `mip_` to avoid conflicts.
/// In these documentation pages, objects are referred to by their fully-
/// qualified C++ names for clarity.
///
///@li @ref mip_interface_c
///@li @ref mip_packet_c
///@li @ref mip_field_c
///@li @ref mip_parser_c
///@li @ref mip::C::mip_cmd_result
///
///
////////////////////////////////////////////////////////////////////////////////
///@page mip_interface Mip Interface
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///@section mip_interface_interface Application Interface
///
/// The MIP interface is a high-level abstraction of a physical device which
/// communicates using the MIP protocol. It provides both data callbacks and
/// command functions for controlling and configuring the device:
///
///@image html mip_interface.svg
///
/// When an application calls one of the command functions, the MIP interface
/// creates a packet, sends it to the device, and waits for a reply. When the
/// reply is received, the command function returns the reply code to the
/// application. If no reply is received, or if an error occurs, the function
/// will return a status code.
///
/// Sending and receiving to or from the device occurs via two functions:
///@li mip::DeviceInterface::sendToDevice() or
///    mip_interface_send_to_device() for transmission and
///@li mip::DeviceInterface::recvFromDevice() or
///    mip_interface_recv_from_device() for reception.
///
/// Each of these has a corresponding callback to the application. The
/// application is expected to implement the required behavior for each as
/// described below. Additionally, there is an @ref update "update function",
/// which handles periodic tasks such as command timeouts and streaming data
/// collection. An application may optionally override the update callback.
///
///@li @ref mip::C::mip_send_callback "mip_send_callback"
///@li @ref mip::C::mip_recv_callback "mip_recv_callback"
///@li @ref mip::C::mip_update_callback "mip_update_callback"
///
/// An application obtains sensor data via the
/// @ref mip_dispatch "dispatch subsystem". There are 3 ways to do so:
///@li Packet callbacks, which call a function when a packet matching the
///    MIP descriptor set is received,
///@li Field callbacks, which call a function when a MIP field matching the
///    descriptor set and field descriptor is called, and
///@li Data pointers, which point to a data structure in memory and update it
///    when the associated data is received.
///
///
////////////////////////////////////////////////////////////////////////////////
///@section mip_commands Sending Commands
///
/// Typically an application will configure the device, initialize some
/// settings, and start streaming. To do so, it must send commands. In many
/// cases, applications will call a single function for each needed command
/// (e.g. @ref MipCommands_c / @ref MipCommands_cpp).
/// These functions take the command parameters as arguments, send the packet,
/// wait for the reply, and return a result code. Additionally some commands can
/// report back responses from the device.
///
/// The command functions are blocking, that is, they halt execution until the
/// device responds or the command times out.
///
/// Note that since MIP data is received serially and is not buffered, data may
/// be received and processed while waiting for command responses. It is
/// recommended (but not required) to set the device to idle during configuration.
///
////////////////////////////////////////////////////////////////////////////////
///@section mip_dispatch The Dispatch System
///
/// Because of the limited resources on embedded platforms, the MIP SDK will not
/// buffer received data, and instead requires the application to process data
/// as it arrives. The MIP interface will dispatch callbacks to the application
/// when the requested data is received.
///
/// The MIP interface can dispatch data in 3 ways:
///@li Packet callbacks, which call a function with a mip packet,
///@li Field callbacks, which call a function with a single mip field, and
///@li Data pointers, which are updated with data from a single mip field.
///
/// With the first two options, the callback function will receive a handle to
/// the MIP interface, the associated MIP packet or field, and the reception
/// timestamp.
///
/// An application must register callbacks with the system during
/// initialization. Each method requires a pointer and the MIP descriptor
/// associated with the data of interest. There is no limit on the number
/// of registered dispatchers, though performance may be affected by using
/// too many. Multiple dispatchers may be registered to the same data.
///
///@par Packet callbacks
///
///@code{.cpp}
/// void packet_callback(void* userdata, const mip::PacketRef& packet, Timestamp parseTime)
///@endcode
///
/// Packet callbacks are invoked when a packet is received which matches the
/// registered descriptor set. The descriptor set may also be a wildcard,
/// allowing the application to process any type of packet.
///
/// An application can register a packet callback to occur either before or
/// after the field callbacks for the data in the same packet. For example,
/// to print a summary of the packet before displaying information about each
/// field, an application would set the callback to occur first. Usually
/// applications will set a packet callback to occur last, so that they can
/// determine if all of the fields have been processed.
///
///@par Field callbacks
///
///@code{.cpp}
/// void field_callback(void* userdata, const mip::Field& field, Timestamp parseTime)
///@endcode
///
/// Similar to packet callbacks, field callbacks are invoked when a MIP
/// field is received which matches the specified descriptor set and
/// field descriptor. Either descriptor may be a wildcard.
///
///@par Data pointers
///
/// Data pointer dispatchers can alleviate a lot of boilerplate code having to
/// do with deserializing a MIP field and storing it somewhere. An application
/// can register a pointer one of the MIP data structures, along with the
/// associated descriptors, and have it automatically updated. The descriptors
/// cannot be wildcards because the type of the data structure is fixed.
///
///@par Data callbacks
///
///@code{.cpp}
/// void data_callback(void* userdata, const mip::data_sensor::ScaledAccel& packet, Timestamp parseTime)
///@endcode
///
/// Thanks to the power of templates, one additional dispatch mechanism is
/// available for C++ applications. A data callback is similar to a field
/// callback except that instead of getting the raw MIP field data, the function
/// is passed the fully-deserialized data structure.
///
///
/// Typically an application will register a series of data or field callbacks
/// and write the data to some kind of data structure. Because the order of
/// these callbacks depends on the device configuration, it can be difficult
/// to know which fields belong together in one sample. The solution is to use
/// a packet callback after all of the fields are received. In the case of
/// wraparound "overflow" MIP packets (see the MIP documentation), packets
/// containing a shared timestamp or event source field at the beginning can
/// be used to group data together.
///
///
////////////////////////////////////////////////////////////////////////////////
///@section update The Update Function
///
/// The application should call mip_interface_update() periodically to process
/// data sent by the device. This update function will call
/// mip_interface_user_recv_from_device() to parse packets. When a data packet is
/// received, the list of packet and data callbacks is checked, and any
/// matching callbacks are invoked. The update function should be called at
/// a high enough rate to avoid overflowing the connection buffers. The
/// precision of the reception timestamp is dependent on the update rate.
///
/// The command functions in @ref MipCommands_c / @ref MipCommands_cpp (e.g. mip::C::mip_write_message_format() / mip::writeMessageFormat())
/// will block execution until the command completes. Either the device will
/// respond with an ack/nack code, or the command will time out. During this
/// time, the system must be able to receive data from the device in order for
/// command replies to be detected. This occurs via the mip_interface_update()
/// function as well.
///
///@par Single-threaded applications
///
/// For single-threaded applications, data can be read from the port directly
/// from within the command function. While the command is waiting (status code
/// MIP_STATUS_WAITING / CmdResult::STATUS_WAITING), repeated calls to the
/// update function will be made. By default, the update function calls
/// mip_interface_user_recv_from_device(). Because the function is called from
/// within a loop, it should sleep for a short time to wait for data if none
/// has been received yet. Doing so prevents excessive CPU usage and lowers
/// power consumption.
///
/// The following diagram shows the typical control flow for a single-threaded
/// application. First, the device is configured by setting the message format.
/// Execution flows down into the command processing functions until
/// mip_interface_wait_for_reply() is called. This will repeatedly call
/// mip_interface_update() to pump packets from the device through the system,
/// until either an ack/nack is received or the command times out.
/// Once the device acknowledges the command, control is returned to the
/// application which then registers some data or packet callbacks. It finally
/// goes into a loop in collect_data(). Inside this loop, the update function
/// is called to process data packets.
///
/// Notice that the same update function is called from both the command
/// function and the data collection loop. If any data packets are received
/// while waiting for a command reply, associated callbacks may be executed.
/// This is why this example application registers its callbacks after the
/// format is configured properly.
///
///@image html device_update.svg
///
///@par Multi-threaded applications
///
/// For some applications, it may be desirable to run all of the data collection
/// from a separate thread. In this case, the command functions must not
/// call the update function as that would cause a race condition between the
/// command thread and the data thread. Instead, the command thread should
/// simply sleep or yield and let the data thread process the ack/nack packet.
///
/// To allow this behavior, the update function takes a boolean parameter which
/// is true when waiting on a command and false when processing data. The
/// default update function, mip_interface_default_update(), ignores this flag,
/// but applications may override it via mip_interface_set_update_function(). In
/// this case, a wrapper function can be created which implements the above
/// behavior:
///@code{.c}
/// bool user_update_function(struct mip_device* device, bool blocking)
/// {
///     // If called from the data thread, do the normal processing.
///     if( !blocking )
///         return mip_interface_default_update(device, blocking);
///
///     // Otherwise, sleep and let the data thread process the reply.
///     std::this_thread::sleep_for(std::chrono::milliseconds(10));
///     return true;
/// }
///
/// mip_interface_set_update_function(device, &user_update_function);
///@endcode
///
///@image html device_update_threaded.svg
///
/// See the threading demo for an example application.
///
///@par Other thread-safety concerns
///
///@li Data transmission to the device (but not sending commands) is thread-safe
///    within the MIP SDK. If multiple threads will send to the device, the
///    application should ensure that the device interface is properly protected.
///
///@li It is up to the application to ensure that sending and receiving from
///    separate threads is safe. This is true for the built-in serial and TCP
///    connections on most operating systems.
///
///@par Using a custom update function for other purposes
///
/// An alternate update function may be used for single-threaded
/// applications, too:
///@li To update a progress bar while waiting for commands to complete
///@li To process data from other devices
///@li To avoid blocking inside mip_interface_user_recv_from_device() when
///    called from a data processing loop.
///@li To push data through the system in a different way (e.g. without using
///    mip_interface_user_recv_from_device())
///
/// Data may be pushed into the system by calling any of these functions:
///@li mip_interface_default_update() - this is the default behavior.
///@li mip_interface_receive_bytes() - process bytes, given a buffer.
///@li mip_interface_receive_packet() - process pre-parsed packets.
///@li mip_interface_process_unparsed_packets() - continue parsing buffered data.
///
///
////////////////////////////////////////////////////////////////////////////////
///@page parsing_packets Mip Parser
////////////////////////////////////////////////////////////////////////////////
///
/// The MIP Parser takes in bytes from the device connection or recorded binary
/// file and extracts the packet data. Data is input to the ring buffer and
/// packets are parsed out one at a time and sent to a callback function.
///
/// The parser uses a ring buffer to store data temporarily between reception
/// and parsing. This helps even out processor workload on embedded systems
/// when data arrives in large bursts.
///
///@image html mip_parser.svg
///
////////////////////////////////////////////////////////////////////////////////
///@section parsing_data Parsing Data
///
/// Data is supplied by calling mip_parser_parse() / mip::Parser::parse() with
/// a buffer and length. Along with the data, the user must provide a timestamp.
/// The timestamp serves two purposes: to provide a time of reception indicator
/// and to allow the parser to time out waiting for more data.
///
/// The parse function takes an additional parameter, `max_packets`, which
/// limits the number of packets parsed. This can be used to prevent a large
/// quantity of packets from consuming too much CPU time and denying service
/// to other subsystems. If the limit is reached, parsing stops and the
/// unparsed portion of the data remains in the ring buffer. The constant value
///@ref MIPPARSER_UNLIMITED_PACKETS disables this limit.
///
/// To continue parsing, call the parse function again. You may choose to
/// not supply any new data by passing NULL and a length of 0. The timestamp
/// should be unchanged from the previous call for highest accuracy, but it's
/// permissible to use the current time as well. If new data is supplied, the
/// new data is appended to the ring buffer and parsing resumes. The timestamp
/// should be the time of the new data. Previously received but unparsed packets
/// will be assigned the new timestamp.
///
/// The application must parse enough packets to keep up with the incoming
/// data stream. Failure to do so will result in the ring buffer becoming
/// full. If this happens, the parse function will return a negative number,
/// indicating the number of bytes that couldn't be copied. This will never
/// happen if max_packets is `MIPPARSER_UNLIMITED_PACKETS` because all
/// of the data will be processed as soon as it is received.
///
////////////////////////////////////////////////////////////////////////////////
///@section ring_buffer The Ring Buffer
///
/// The ring buffer's backing buffer is a byte array that is allocated by
/// the application during initialization. It must be large enough to store
/// the biggest burst of data seen at any one time. For example, applications
/// expecting to deal with lots of GNSS-related data will need a bigger buffer
/// because there may be a large number of satellite messages. These messages
/// are sent relatively infrequently but contain a lot of data. If max_packets
/// is `MIPPARSER_UNLIMITED_PACKETS`, then it needs only 512 bytes (enough for
/// one packet, rounded up to a power of 2).
///
/// In addition to passing data to the parse function, data can be written
/// directly to the ring buffer by obtaining a writable pointer and length
/// from mip_parser_get_write_ptr(). This may be more efficient by skipping a
/// copy operation. Call mip_parser_process_written() to tell the parser how
/// many bytes were written to the pointer. Note that the length returned by
/// `mip_parser_get_write_ptr` can frequently be less than the total
/// available space. An application should call it in a loop as long as there
/// is more data to process and the returned size is greater than 0.
///
////////////////////////////////////////////////////////////////////////////////
///@section packet_timeouts Packet Timeouts
///
/// In some cases it's possible for a packet to be corrupted during
/// transmission or reception (e.g. EMI while in transit on the wire, serial
/// baud rate too low, etc). If the payload length byte is corrupted, it may
/// falsely indicate that the packet is longer than what was sent. Without a
/// timeout, the parser would wait until this extra data (potentially up to 255
/// bytes) was received before checking and realizing that the checksum failed.
/// Any following packets would be delayed, possibly causing additional commands
/// to time out and make the device appear temporarily unresponsive. Setting a
/// reasonable timeout ensures that the bad packet is rejected more quickly.
/// The timeout should be set so that a MIP packet of the largest possible
/// size (261 bytes) can be transfered well within the transmission time plus
/// any additional processing delays in the application or operating system.
/// As an example, for a 115200 baud serial link a timeout of 30 ms would be
/// about right. You can use the mip_timeout_from_baudrate() function to
/// compute an appropriate timeout.
///
///@see timestamp_type
///@see mip::Timestamp
///
////////////////////////////////////////////////////////////////////////////////
///@section parsing_process The Packet Parsing Process
///
/// Packets are parsed from the internal ring buffer one at a time in the parse
/// function.
///
/// If a packet was previously started but not completed previously (due to
/// requiring more data) then the timeout is checked. If too much time has
/// passed, the packet is discarded and the parsing state reset. This check is
/// only performed once per parse call because that is the only point where the
/// timestamp changes.
///
///@image html parse_function.svg
///
/// The current status is held by the `expected_length` variable, which tracks
/// how many bytes are expected to be in the current packet. The parse function
/// enters a loop, checking if there is enough data to complete the next parsing
/// step.
///
///@image html parse_one_packet.svg
///
/// `expected_length` starts out as 1 when the parser is searching for the start
/// of a packet. Once a potential start byte (`SYNC1`) is found, the packet's
/// start time is initialized to the current timestamp and `expected_length` is
/// bumped up to the size of a mip packet header (4 bytes).
///
/// When the expected length is 4 bytes, the header's SYNC2 byte is checked for
/// validity and the payload length field is read. `expected_length` is set to
/// the full packet size (computed as the packet header and checksum size plus
/// the payload size).
///
/// Finally, when `expected_length` is neither of the above two conditions, it
/// means that the entire packet has been received. Note that other values less
/// than 6 (the size of an empty packet) are not possible. At this point, the
/// data is copied out from the ring buffer to a linear buffer for processing.
/// The checksum is verified, and if it passes, the entire packet is dropped
/// from the ring buffer and the callback function is invoked.
///
/// If any of the checks in the above steps fails, such as a wrong SYNC2 byte,
/// a single byte is dropped from the ring buffer and the loop is continued.
/// Only a single byte can be dropped, because rogue SYNC1 bytes or truncated
/// packets may hide real mip packets in what would have been their payload.
///