MIP SDK
=======

Welcome to the official MIP Software Development Kit.


Features
--------

* Send commands using a single function
* Suitable for bare-metal microcontrollers
  * Minimal code size and memory footprint
  * No dynamic memory allocation
  * No dependence on any RTOS or threading
* Simple to interface with existing projects
  * FindMip.cmake is included for CMake-based projects
* Can be used to parse offline binary files
* C API for those who can't use C++
* C++ API for safety, flexibility, and convenience.

* Advanced Features
  * MIP packet creation
  * MIP packet parsing and field iteration
  * Data field deserialization

Examples
--------

<!-- NOTE: I commented out the descriptions below as they seemed like they would
           be better suited to being in the example code files themselves. Feel
           free to uncomment these again if you want, or remove them completely.
-->
* **Get device information** [[C++](./examples/device_info.cpp)]. <!-- - Queries the device strings and prints them to stdout. -->
* **Watch IMU** [[C](./examples/watch_imu.c) | [C++](./examples/watch_imu.cpp)]. <!-- - Configures the IMU for streaming and prints the data to stdout. -->
* **Threading** [[C++](./examples/threading.cpp)].
* **Ping** [[C++](./examples/ping.cpp)].
* **Product-specific examples:**
  * **GQ7 setup** [[C](./examples/GQ7/GQ7_example.c), [C++](./examples/GQ7/GQ7_example.cpp)]. <!-- - Configures a GQ7 device for typical usage in a wheeled-vehicle application. -->
  * **CV7 setup** [[C](./examples/CV7/CV7_example.c), [C++](./examples/CV7/CV7_example.cpp)]. <!-- - Configures a CV7 device for typical usage and includes an example of using the event system. -->
  * **GX5-45 setup** [[C](./examples/GX5_45/GX5_45_example.c), [C++](./examples/GX5_45/GX5_45_example.cpp)]. <!-- - Configures a GX5-45 device for typical usage in a wheeled-vehicle application. -->
  * **CV7_INS setup** [[C++](./examples/CV7_INS/CV7_INS_simple_example.cpp)]. <!-- > - Configures a CV7_INS device for typical usage. -->
  * **CV7_INS with UBlox setup** [[C++](./examples/CV7_INS/CV7_INS_simple_ublox_example.cpp)]. <!-- > - Configures a CV7_INS device for typical usage. -->

You'll need to enable at least one of the [communications interfaces](#communications-interfaces) in the CMake configuration to use the examples.

The examples take two parameters for the device connection:
* For a serial connection: Port and baudrate. Port must start with `/dev/` on Linux or `COM` on Windows.
* For a TCP connection: Hostname and port. Hostname can be either a hostname like `localhost` or an IPv4 address.


Documentation
-------------

Documentation for all released versions can be found [here](https://lord-microstrain.github.io/mip_sdk_documentation).


Communications Interfaces
-------------------------

Two connection types are provided with the MIP SDK to make it easy to run the examples on both Windows and Linux systems.

### Serial Port

A basic serial port interface is provided in C and C++ for Linux and Windows. These can be modified for other platforms by the user.
The serial port connection will be used in most cases, when the MIP device is connected
via a serial or USB cable (the USB connection acts like a virtual serial port).

[Enable it](#build-configuration) in the CMake configuration with `-DMIP_USE_SERIAL=1`.

### TCP Client

The TCP client connection allows you to connect to a MIP device remotely. The MIP device must be connected
via the normal serial or USB cable to a computer system running a TCP server which forwards data between
the serial port and TCP clients.

[Enable it](#build-configuration) in the CMake configuration with `-DMIP_USE_TCP=1`.


How to Build
------------

### Prerequisites

* CMake version 3.10 or later
* A working C compiler
  * C99 or later required
* A working C++ compiler
  * For C++ API only. Define `MIP_DISABLE_CPP=ON` if you don't want to use any C++.
  * C++11 or later required for the mip library
  * C++14 or later for the examples (currently CMakeLists.txt assumes C++14 is required regardless)
* Doxygen, if building documentation

### Build configuration

The following options may be specified when configuring the build with CMake (e.g. `cmake .. -DOPTION=VALUE`):
* MIP_USE_SERIAL - Builds the included serial port library (default enabled).
* MIP_USE_TCP - Builds the included socket library (default enabled).
* MIP_USE_EXTRAS - Builds some higher level utility classes and functions that may use dynamic memory.
* MIP_ENABLE_LOGGING - Builds logging functionality into the library. The user is responsible for configuring a logging callback (default enabled)
* MIP_LOGGING_MAX_LEVEL - Max log level the SDK is allowed to log. If this is defined, any log level logged at a higher level than this will result in a noop regardless of runtime configuration. Useful if you want some logs, but do not want the overhead of the higher level functions compiled into the code
* MIP_ENABLE_DIAGNOSTICS - Adds some counters to various entities which can serve as a debugging aid.
* BUILD_EXAMPLES - If enabled (`-DBUILD_EXAMPLES=ON`), the example projects will be built (default disabled).
* BUILD_TESTING - If enabled (`-DBUILD_TESTING=ON`), the test programs in the /test directory will be compiled and linked. Run the tests with `ctest`.
* BUILD_DOCUMENTATION - If enabled, the documentation will be built with doxygen. You must have doxygen installed.
* BUILD_DOCUMENTATION_FULL - Builds internal documentation (default disabled).
* BUILD_DOCUMENTATION_QUIET - Suppress standard doxygen output (default enabled).
* MIP_DISABLE_CPP - Ignores .hpp/.cpp files during the build and does not add them to the project.
* BUILD_PACKAGE - Adds a `package` target to the project that will build a `.deb`, `.rpm`, or `.7z` file containing the library
* MIP_TIMESTAMP_TYPE - Overrides the default timestamp type. See the timestamps section below.

### Compilation with CMake

1. Create the build directory (e.g. `mkdir build`).
2. In the build directory, run `cmake .. <options>`
   * Replace `<options>` with your configuration options, such as `-DMIP_USE_SERIAL=1`.
   * You can use `cmake-gui ..` instead if you'd prefer to use the GUI tool (and have it installed).
   * An alternative generator may be used, such as ninja, code blocks, etc. by specifying `-G <generatopr>`
3. Invoke `cmake --build .` in the build directory
4. (Optional, if BUILD_PACKAGE was enabled) Run `cmake --build . --target package` to build the packages.


Implementation Notes
--------------------

### User-Implemented Functions

There are two C functions which must be implemented to use this library.

The first, `mip_interface_user_recv_from_device()`, must fetch raw data bytes from the connected MIP device. Typically this means reading from
a serial port or TCP socket.

The second, `mip_interface_send_to_device()`, must pass the provided data bytes directly to the connected MIP device.

See [`mip_interface`](https://lord-microstrain.github.io/mip_sdk_documentation/latest/mip_interface.html) for details on how to implement these functions.

#### C++
For C++ applications, these functions are implemented by the `DeviceInterface` class, which takes a `Connection` object responsible
for reading and writing to the device. Create a class derived from `Connection` and implement the pure virtual `recvFromDevice` and
`sendToDevice` methods.

If you do not wish to use the `DeviceInterface` class, do not compile the corresponding source file and create the
C functions yourself. Declare them functions as `extern "C"` to avoid linking problems between the C and C++ code.

### Command Results (mip_cmd_result / MipCmdResult)

Command results are divided into two categories:
* Reply codes are returned by the device, e.g.:
  * ACK / OK
  * Invalid parameter
  * Unknown command
* Status codes are set by this library, e.g.:
  * General ERROR
  * TIMEDOUT
  * Other statuses are used while the command is still in process

### Timestamps and Timeouts

#### Timestamp type
Timestamps (`timestamp_type` / `Timestamp`) represent the local time when data was received or a packet was parsed. These timestamps
are used to implement command timeouts and provide the user with an approximate timestamp of received data. It is not intended to be
a precise timestamp or used for synchronization, and it generally cannot be used instead of the timestamps from the connected MIP device.
In particular, if you limit the maximum number of packets processed per `update` call, the timestamp of some packets may be delayed.

Because different applications may keep track of time differently (especially on embedded platforms), it is up to the user to provide
the current time whenever data is received from the device. On a PC, this might come from the poxis `time()` function or from the
`std::chrono` library. On ARM systems, it is often derived from the Systick timer. It should be a monotonically increasing value;
jumps backwards in time will cause problems.

By default, timestamps are `typedef`'d to `uint64_t`. Typically timestamps are in milliseconds. Embedded systems may wish to use
`uint32_t` or even `uint16_t` instead. The value is allowed to wrap around as long as the time between wraparounds is longer than
twice the longest timeout needed. If higher precision is needed or wraparound can't be tolerated by your application, define it to
`uint64_t`. It must be a standard unsigned integer type.

#### Command Timeouts

Timeouts for commands are broken down into two parts.
* A "base reply timeout" applies to all commands. This is useful to compensate for communication latency, such as over a TCP socket.
* "Additional time" which applies per command, because some commands may take longer to complete.

Currently, only the C++ api offers a way to set the additional time parameter, and only when using the `runCommand` function taking
the command structure and the `additionalTime` parameter.

The `timeout_type` / `Timeout` typedef is an alias to the timestamp type.

### C and C++ APIs

The C++ API is implemented on top of the C API to provide additional features:
* Object-oriented interfaces
* Improved type safety and sanity checking
* Better clarity / reduced verbosity (e.g. with `using namespace mip`)

The C++ API uses `TitleCase` for types and `camelCase` for functions and variables, while the C api uses `snake_case` naming for
everything. This makes it easy to tell which is being used when looking at the examples.

The C API can be accessed directly from C++ via the `mip::C` namesace.
