
#include "device_models.h"

#include <ctype.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {
#endif// __cplusplus

mip_model_number get_model_from_string(const char* model_or_serial)
{
    unsigned int start_index = 0;

    // The model number is just the portion of the serial or model string before the dot or dash
    // serial and model number fields are 16 chars
    unsigned int i = 0;
    for (; model_or_serial[i] != '\0'; i++)
    {
        // Unsigned is important. Passing a negative value to isdigit or
        // isspace is undefined behavior and can trigger assertions.
        const unsigned char c = model_or_serial[i];

        if (!isdigit(c))
        {
            if (c == '.' || c == '-')
                break;

            if (isspace(c) && (start_index == i))
                start_index++;
            else
                return MODEL_UNKNOWN;
        }
    }

    // Must get at least 4 digits.
    if (i < start_index + 4)
        return MODEL_UNKNOWN;

    return atoi(model_or_serial + start_index);
}

const char* get_model_name_from_number(mip_model_number model)
{
    switch (model)
    {
    case MODEL_3DM_DH3:       return "3DM-DH3";
    case MODEL_3DM_GX3_15:    return "3DM-GX3-15";
    case MODEL_3DM_GX3_25:    return "3DM-GX3-25";
    case MODEL_3DM_GX3_35:    return "3DM-GX3-35";
    case MODEL_3DM_GX3_45:    return "3DM-GX3-45";
    case MODEL_3DM_RQ1_45_LT: return "3DM-RQ1-45_LT";
    case MODEL_3DM_GX4_15:    return "3DM-GX4-15";
    case MODEL_3DM_GX4_25:    return "3DM-GX4-25";
    case MODEL_3DM_GX4_45:    return "3DM-GX4-45";
    case MODEL_3DM_RQ1_45_ST: return "3DM-RQ1-45_ST";
    case MODEL_3DM_GX5_10:    return "3DM-GX5-10";
    case MODEL_3DM_GX5_15:    return "3DM-GX5-15";
    case MODEL_3DM_GX5_25:    return "3DM-GX5-25";
    case MODEL_3DM_GX5_35:    return "3DM-GX5-35";
    case MODEL_3DM_GX5_45:    return "3DM-GX5-45";
    case MODEL_3DM_CV5_10:    return "3DM-CV5-10";
    case MODEL_3DM_CV5_15:    return "3DM-CV5-15";
    case MODEL_3DM_CV5_25:    return "3DM-CV5-25";
    case MODEL_3DM_CV5_45:    return "3DM-CV5-45";
    case MODEL_3DM_GQ4_45:    return "3DM-GQ4-45";
    case MODEL_3DM_CX5_45:    return "3DM-CX5-45";
    case MODEL_3DM_CX5_35:    return "3DM-CX5-35";
    case MODEL_3DM_CX5_25:    return "3DM-CX5-25";
    case MODEL_3DM_CX5_15:    return "3DM-CX5-15";
    case MODEL_3DM_CX5_10:    return "3DM-CX5-10";
    case MODEL_3DM_CL5_15:    return "3DM-CL5-15";
    case MODEL_3DM_CL5_25:    return "3DM-CL5-25";
    case MODEL_3DM_GQ7:       return "3DM-GQ7";
    case MODEL_3DM_RTK:       return "3DM-RTK";
    case MODEL_3DM_CV7_AHRS:  return "3DM-CV7-AHRS";
    case MODEL_3DM_CV7_AR:    return "3DM-CV7-AR";
    case MODEL_3DM_GV7_AHRS:  return "3DM-GV7-AHRS";
    case MODEL_3DM_GV7_AR:    return "3DM-GV7-AR";
    case MODEL_3DM_GV7_INS:   return "3DM-GV7-INS";
    case MODEL_3DM_CV7_INS:   return "3DM-CV7-INS";

    default:
    case MODEL_UNKNOWN: return "";
    }
}

#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif // __cplusplus
