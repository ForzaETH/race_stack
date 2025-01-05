#pragma once

#include <stdint.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_c
///@{
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_logging  Mip Logging [C]
///
///@brief High-level C functions for logging information from within the MIP SDK
///
/// This module contains functions that allow the MIP SDK to log information
/// and allows users to override the logging functions
///
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief Logging level enum
///
typedef uint8_t mip_log_level;
#define MIP_LOG_LEVEL_OFF   0  ///< Signifies that the log is turned off
#define MIP_LOG_LEVEL_FATAL 1  ///< Fatal logs are logged when an unrecoverable error occurs
#define MIP_LOG_LEVEL_ERROR 2  ///< Error logs are logged when an error occurs
#define MIP_LOG_LEVEL_WARN  3  ///< Warning logs are logged when something concerning happens that may or not be a mistake
#define MIP_LOG_LEVEL_INFO  4  ///< Info logs are logged when some general info needs to be conveyed to the user
#define MIP_LOG_LEVEL_DEBUG 5  ///< Debug logs are logged for debug purposes.
#define MIP_LOG_LEVEL_TRACE 6  ///< Trace logs are logged in similar cases to debug logs but can be logged in tight loops

////////////////////////////////////////////////////////////////////////////////
///@brief Callback function typedef for custom logging behavior.
///
///@param level   The log level that this log should be logged at
///@param fmt     printf style format string
///@param ...     Variadic args used to populate the fmt string
///
typedef void (*mip_log_callback)(void* user, mip_log_level level, const char* fmt, va_list args);

void mip_logging_init(mip_log_callback callback, mip_log_level level, void* user);

mip_log_callback mip_logging_callback();
mip_log_level mip_logging_level();
void* mip_logging_user_data();

void mip_logging_log(mip_log_level level, const char* fmt, ...);

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to initialize the MIP logger.
///       This function does not need to be called unless the user wants logging
///
///@param callback The callback to execute when there is data to log
///@param level    The level that the MIP SDK should log at
///@param user     User data that will be passed to the callback every time it is excuted
///
#ifdef MIP_ENABLE_LOGGING
#define MIP_LOG_INIT(callback, level, user) mip_logging_init(callback, level, user)
#else
#define MIP_LOG_INIT(callback, level, user) (void)0
#endif

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK. Prefer specific
///       log level functions like MIP_LOG_INFO, etc. when possible.
///@copydetails mip::C::mip_log_callback
///
#ifdef MIP_ENABLE_LOGGING
#define MIP_LOG_LOG(level, ...) mip_logging_log(level, __VA_ARGS__)
#else
#define MIP_LOG_LOG(level, ...) (void)0
#endif

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK at fatal level
///
///@param context Context of what called this function. Could be a MIP device, serial connection, etc.
///@param fmt     printf style format string
///@param ...     Variadic args used to populate the fmt string
///
#if !defined(MIP_LOGGING_MAX_LEVEL) || MIP_LOGGING_MAX_LEVEL >= MIP_LOG_LEVEL_FATAL
#define MIP_LOG_FATAL(...) MIP_LOG_LOG(MIP_LOG_LEVEL_FATAL, __VA_ARGS__)
#else
#define MIP_LOG_FATAL(...) (void)0
#endif

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK at error level
///@copydetails mip::C::MIP_LOG_FATAL
#if !defined(MIP_LOGGING_MAX_LEVEL) || MIP_LOGGING_MAX_LEVEL >= MIP_LOG_LEVEL_ERROR
#define MIP_LOG_ERROR(...) MIP_LOG_LOG(MIP_LOG_LEVEL_ERROR, __VA_ARGS__)
#else
#define MIP_LOG_ERROR(...) (void)0
#endif

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK at warn level
///@copydetails mip::C::MIP_LOG_FATAL
#if !defined(MIP_LOGGING_MAX_LEVEL) || MIP_LOGGING_MAX_LEVEL >= MIP_LOG_LEVEL_WARN
#define MIP_LOG_WARN(...) MIP_LOG_LOG(MIP_LOG_LEVEL_WARN, __VA_ARGS__)
#else
#define MIP_LOG_WARN(...) (void)0
#endif

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK at info level
///@copydetails mip::C::MIP_LOG_FATAL
#if !defined(MIP_LOGGING_MAX_LEVEL) || MIP_LOGGING_MAX_LEVEL >= MIP_LOG_LEVEL_INFO
#define MIP_LOG_INFO(...) MIP_LOG_LOG(MIP_LOG_LEVEL_INFO, __VA_ARGS__)
#else
#define MIP_LOG_INFO(...) (void)0
#endif

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK at debug level
///@copydetails mip::C::MIP_LOG_FATAL
#if !defined(MIP_LOGGING_MAX_LEVEL) || MIP_LOGGING_MAX_LEVEL >= MIP_LOG_LEVEL_DEBUG
#define MIP_LOG_DEBUG(...) MIP_LOG_LOG(MIP_LOG_LEVEL_DEBUG, __VA_ARGS__)
#else
#define MIP_LOG_DEBUG(...) (void)0
#endif

////////////////////////////////////////////////////////////////////////////////
///@brief Helper macro used to log data inside the MIP SDK at trace level
///@copydetails mip::C::MIP_LOG_FATAL
#if !defined(MIP_LOGGING_MAX_LEVEL) || MIP_LOGGING_MAX_LEVEL >= MIP_LOG_LEVEL_TRACE
#define MIP_LOG_TRACE(...) MIP_LOG_LOG(MIP_LOG_LEVEL_TRACE, __VA_ARGS__)
#else
#define MIP_LOG_TRACE(...) (void)0
#endif

///@}
///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"
#endif