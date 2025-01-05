
#include "common.h"

#include "../utils/serialization.h"


void insert_mip_descriptor_rate(mip_serializer* serializer, const mip_descriptor_rate* self)
{
    insert_u8(serializer, self->descriptor);
    insert_u16(serializer, self->decimation);
}

void extract_mip_descriptor_rate(mip_serializer* serializer, mip_descriptor_rate* self)
{
    extract_u8(serializer, &self->descriptor);
    extract_u16(serializer, &self->decimation);
}

#define IMPLEMENT_MIP_VECTOR_FUNCTIONS(n,type,name) \
void insert_##name(mip_serializer* serializer, const name self) \
{ \
    for(unsigned int i=0; i<n; i++) \
        insert_##type(serializer, self[i]); \
} \
void extract_##name(mip_serializer* serializer, name self) \
{ \
    for(unsigned int i=0; i<n; i++) \
        extract_##type(serializer, &self[i]); \
}

IMPLEMENT_MIP_VECTOR_FUNCTIONS(3, float,  mip_vector3f)
IMPLEMENT_MIP_VECTOR_FUNCTIONS(4, float,  mip_vector4f)
IMPLEMENT_MIP_VECTOR_FUNCTIONS(9, float,  mip_matrix3f)
IMPLEMENT_MIP_VECTOR_FUNCTIONS(3, double, mip_vector3d)
IMPLEMENT_MIP_VECTOR_FUNCTIONS(4, double, mip_vector4d)
IMPLEMENT_MIP_VECTOR_FUNCTIONS(9, double, mip_matrix3d)
IMPLEMENT_MIP_VECTOR_FUNCTIONS(4, float,  mip_quatf)

#undef IMPLEMENT_MIP_VECTOR_FUNCTIONS
