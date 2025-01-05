#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {
#endif



// Used like a signed version of size_t
typedef int mip_remaining_count;


///@brief Type used for packet timestamps and timeouts.
///
/// Timestamps must be monotonic except for overflow at the maximum value back to 0.
/// The units can be anything, but typically are milliseconds. Choose a long enough
/// unit so that consecutive calls to the parser will not exceed half of the maximum
/// value for this type. For milliseconds, the time to overflow is approximately 50
/// days, so the parser should be invoked at least every 25 days. Failure to observe
/// this requirement may result in false timeouts or delays in getting parsed packets.
///
#ifdef MIP_TIMESTAMP_TYPE
    typedef MIP_TIMESTAMP_TYPE mip_timestamp;
    static_assert( sizeof(mip_timestamp) >= 8 || mip_timestamp(-1) > 0, "MIP_TIMESTAMP_TYPE must be unsigned unless 64 bits.");
#else
    typedef uint64_t mip_timestamp;
#endif

typedef mip_timestamp mip_timeout;


#ifdef MIP_ENABLE_DIAGNOSTICS

// Saturating addition
#define MIP_DIAG_INC(counter, amount) do { if (counter + amount < counter) counter = -1; else counter += amount; } while(false)

#define MIP_DIAG_ZERO(counter) counter = 0

#else // MIP_ENABLE_DIAGNOSTICS

// Do nothing if diagnostic counters diabled. Cast amount to void to avoid "unused local variable" warnings.
#define MIP_DIAG_INC(counter, amount) (void)amount

#define MIP_DIAG_ZERO(counter) (void)0

#endif // MIP_ENABLE_DIAGNOSTICS


#ifdef __cplusplus

} // extern "C"
} // namespace C

using RemainingCount = C::mip_remaining_count;
using Timestamp      = C::mip_timestamp;
using Timeout        = C::mip_timeout;

} // namespace mip

#endif
