/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.1 */

#ifndef PB_COMMON_COMMON_COMMON_PB_H_INCLUDED
#define PB_COMMON_COMMON_COMMON_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Enum definitions */
typedef enum _common_Modulation {
    common_Modulation_LORA = 0,
    common_Modulation_FSK = 1
} common_Modulation;

typedef enum _common_Region {
    common_Region_EU868 = 0,
    common_Region_US915 = 2,
    common_Region_CN779 = 3,
    common_Region_EU433 = 4,
    common_Region_AU915 = 5,
    common_Region_CN470 = 6,
    common_Region_AS923 = 7,
    common_Region_KR920 = 8,
    common_Region_IN865 = 9,
    common_Region_RU864 = 10
} common_Region;

typedef enum _common_LocationSource {
    common_LocationSource_UNKNOWN = 0,
    common_LocationSource_GPS = 1,
    common_LocationSource_CONFIG = 2,
    common_LocationSource_GEO_RESOLVER = 3
} common_LocationSource;

/* Struct definitions */
typedef struct _common_KeyEnvelope {
    pb_callback_t kek_label;
    pb_callback_t aes_key;
} common_KeyEnvelope;

typedef struct _common_Location {
    double latitude;
    double longitude;
    double altitude;
    common_LocationSource source;
    uint32_t accuracy;
} common_Location;


/* Helper constants for enums */
#define _common_Modulation_MIN common_Modulation_LORA
#define _common_Modulation_MAX common_Modulation_FSK
#define _common_Modulation_ARRAYSIZE ((common_Modulation)(common_Modulation_FSK+1))

#define _common_Region_MIN common_Region_EU868
#define _common_Region_MAX common_Region_RU864
#define _common_Region_ARRAYSIZE ((common_Region)(common_Region_RU864+1))

#define _common_LocationSource_MIN common_LocationSource_UNKNOWN
#define _common_LocationSource_MAX common_LocationSource_GEO_RESOLVER
#define _common_LocationSource_ARRAYSIZE ((common_LocationSource)(common_LocationSource_GEO_RESOLVER+1))


/* Initializer values for message structs */
#define common_KeyEnvelope_init_default          {{{NULL}, NULL}, {{NULL}, NULL}}
#define common_Location_init_default             {0, 0, 0, _common_LocationSource_MIN, 0}
#define common_KeyEnvelope_init_zero             {{{NULL}, NULL}, {{NULL}, NULL}}
#define common_Location_init_zero                {0, 0, 0, _common_LocationSource_MIN, 0}

/* Field tags (for use in manual encoding/decoding) */
#define common_KeyEnvelope_kek_label_tag         1
#define common_KeyEnvelope_aes_key_tag           2
#define common_Location_latitude_tag             1
#define common_Location_longitude_tag            2
#define common_Location_altitude_tag             3
#define common_Location_source_tag               4
#define common_Location_accuracy_tag             5

/* Struct field encoding specification for nanopb */
#define common_KeyEnvelope_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, STRING,   kek_label,         1) \
X(a, CALLBACK, SINGULAR, BYTES,    aes_key,           2)
#define common_KeyEnvelope_CALLBACK pb_default_field_callback
#define common_KeyEnvelope_DEFAULT NULL

#define common_Location_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, DOUBLE,   latitude,          1) \
X(a, STATIC,   SINGULAR, DOUBLE,   longitude,         2) \
X(a, STATIC,   SINGULAR, DOUBLE,   altitude,          3) \
X(a, STATIC,   SINGULAR, UENUM,    source,            4) \
X(a, STATIC,   SINGULAR, UINT32,   accuracy,          5)
#define common_Location_CALLBACK NULL
#define common_Location_DEFAULT NULL

extern const pb_msgdesc_t common_KeyEnvelope_msg;
extern const pb_msgdesc_t common_Location_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define common_KeyEnvelope_fields &common_KeyEnvelope_msg
#define common_Location_fields &common_Location_msg

/* Maximum encoded size of messages (where known) */
/* common_KeyEnvelope_size depends on runtime parameters */
#define common_Location_size                     35

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
