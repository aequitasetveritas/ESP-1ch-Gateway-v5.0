/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.1 */

#ifndef PB_GOOGLE_PROTOBUF_GOOGLE_PROTOBUF_TYPE_PB_H_INCLUDED
#define PB_GOOGLE_PROTOBUF_GOOGLE_PROTOBUF_TYPE_PB_H_INCLUDED
#include <pb.h>
#include "any.pb.h"
#include "source_context.pb.h"

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Enum definitions */
typedef enum _google_protobuf_Syntax {
    google_protobuf_Syntax_SYNTAX_PROTO2 = 0,
    google_protobuf_Syntax_SYNTAX_PROTO3 = 1
} google_protobuf_Syntax;

typedef enum _google_protobuf_Field_Kind {
    google_protobuf_Field_Kind_TYPE_UNKNOWN = 0,
    google_protobuf_Field_Kind_TYPE_DOUBLE = 1,
    google_protobuf_Field_Kind_TYPE_FLOAT = 2,
    google_protobuf_Field_Kind_TYPE_INT64 = 3,
    google_protobuf_Field_Kind_TYPE_UINT64 = 4,
    google_protobuf_Field_Kind_TYPE_INT32 = 5,
    google_protobuf_Field_Kind_TYPE_FIXED64 = 6,
    google_protobuf_Field_Kind_TYPE_FIXED32 = 7,
    google_protobuf_Field_Kind_TYPE_BOOL = 8,
    google_protobuf_Field_Kind_TYPE_STRING = 9,
    google_protobuf_Field_Kind_TYPE_GROUP = 10,
    google_protobuf_Field_Kind_TYPE_MESSAGE = 11,
    google_protobuf_Field_Kind_TYPE_BYTES = 12,
    google_protobuf_Field_Kind_TYPE_UINT32 = 13,
    google_protobuf_Field_Kind_TYPE_ENUM = 14,
    google_protobuf_Field_Kind_TYPE_SFIXED32 = 15,
    google_protobuf_Field_Kind_TYPE_SFIXED64 = 16,
    google_protobuf_Field_Kind_TYPE_SINT32 = 17,
    google_protobuf_Field_Kind_TYPE_SINT64 = 18
} google_protobuf_Field_Kind;

typedef enum _google_protobuf_Field_Cardinality {
    google_protobuf_Field_Cardinality_CARDINALITY_UNKNOWN = 0,
    google_protobuf_Field_Cardinality_CARDINALITY_OPTIONAL = 1,
    google_protobuf_Field_Cardinality_CARDINALITY_REQUIRED = 2,
    google_protobuf_Field_Cardinality_CARDINALITY_REPEATED = 3
} google_protobuf_Field_Cardinality;

/* Struct definitions */
typedef struct _google_protobuf_Enum {
    pb_callback_t name;
    pb_callback_t enumvalue;
    pb_callback_t options;
    bool has_source_context;
    google_protobuf_SourceContext source_context;
    google_protobuf_Syntax syntax;
} google_protobuf_Enum;

typedef struct _google_protobuf_EnumValue {
    pb_callback_t name;
    int32_t number;
    pb_callback_t options;
} google_protobuf_EnumValue;

typedef struct _google_protobuf_Field {
    google_protobuf_Field_Kind kind;
    google_protobuf_Field_Cardinality cardinality;
    int32_t number;
    pb_callback_t name;
    pb_callback_t type_url;
    int32_t oneof_index;
    bool packed;
    pb_callback_t options;
    pb_callback_t json_name;
    pb_callback_t default_value;
} google_protobuf_Field;

typedef struct _google_protobuf_Option {
    pb_callback_t name;
    bool has_value;
    google_protobuf_Any value;
} google_protobuf_Option;

typedef struct _google_protobuf_Type {
    pb_callback_t name;
    pb_callback_t fields;
    pb_callback_t oneofs;
    pb_callback_t options;
    bool has_source_context;
    google_protobuf_SourceContext source_context;
    google_protobuf_Syntax syntax;
} google_protobuf_Type;


/* Helper constants for enums */
#define _google_protobuf_Syntax_MIN google_protobuf_Syntax_SYNTAX_PROTO2
#define _google_protobuf_Syntax_MAX google_protobuf_Syntax_SYNTAX_PROTO3
#define _google_protobuf_Syntax_ARRAYSIZE ((google_protobuf_Syntax)(google_protobuf_Syntax_SYNTAX_PROTO3+1))

#define _google_protobuf_Field_Kind_MIN google_protobuf_Field_Kind_TYPE_UNKNOWN
#define _google_protobuf_Field_Kind_MAX google_protobuf_Field_Kind_TYPE_SINT64
#define _google_protobuf_Field_Kind_ARRAYSIZE ((google_protobuf_Field_Kind)(google_protobuf_Field_Kind_TYPE_SINT64+1))

#define _google_protobuf_Field_Cardinality_MIN google_protobuf_Field_Cardinality_CARDINALITY_UNKNOWN
#define _google_protobuf_Field_Cardinality_MAX google_protobuf_Field_Cardinality_CARDINALITY_REPEATED
#define _google_protobuf_Field_Cardinality_ARRAYSIZE ((google_protobuf_Field_Cardinality)(google_protobuf_Field_Cardinality_CARDINALITY_REPEATED+1))


/* Initializer values for message structs */
#define google_protobuf_Type_init_default        {{{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}, false, google_protobuf_SourceContext_init_default, _google_protobuf_Syntax_MIN}
#define google_protobuf_Field_init_default       {_google_protobuf_Field_Kind_MIN, _google_protobuf_Field_Cardinality_MIN, 0, {{NULL}, NULL}, {{NULL}, NULL}, 0, 0, {{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}}
#define google_protobuf_Enum_init_default        {{{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}, false, google_protobuf_SourceContext_init_default, _google_protobuf_Syntax_MIN}
#define google_protobuf_EnumValue_init_default   {{{NULL}, NULL}, 0, {{NULL}, NULL}}
#define google_protobuf_Option_init_default      {{{NULL}, NULL}, false, google_protobuf_Any_init_default}
#define google_protobuf_Type_init_zero           {{{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}, false, google_protobuf_SourceContext_init_zero, _google_protobuf_Syntax_MIN}
#define google_protobuf_Field_init_zero          {_google_protobuf_Field_Kind_MIN, _google_protobuf_Field_Cardinality_MIN, 0, {{NULL}, NULL}, {{NULL}, NULL}, 0, 0, {{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}}
#define google_protobuf_Enum_init_zero           {{{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}, false, google_protobuf_SourceContext_init_zero, _google_protobuf_Syntax_MIN}
#define google_protobuf_EnumValue_init_zero      {{{NULL}, NULL}, 0, {{NULL}, NULL}}
#define google_protobuf_Option_init_zero         {{{NULL}, NULL}, false, google_protobuf_Any_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define google_protobuf_Enum_name_tag            1
#define google_protobuf_Enum_enumvalue_tag       2
#define google_protobuf_Enum_options_tag         3
#define google_protobuf_Enum_source_context_tag  4
#define google_protobuf_Enum_syntax_tag          5
#define google_protobuf_EnumValue_name_tag       1
#define google_protobuf_EnumValue_number_tag     2
#define google_protobuf_EnumValue_options_tag    3
#define google_protobuf_Field_kind_tag           1
#define google_protobuf_Field_cardinality_tag    2
#define google_protobuf_Field_number_tag         3
#define google_protobuf_Field_name_tag           4
#define google_protobuf_Field_type_url_tag       6
#define google_protobuf_Field_oneof_index_tag    7
#define google_protobuf_Field_packed_tag         8
#define google_protobuf_Field_options_tag        9
#define google_protobuf_Field_json_name_tag      10
#define google_protobuf_Field_default_value_tag  11
#define google_protobuf_Option_name_tag          1
#define google_protobuf_Option_value_tag         2
#define google_protobuf_Type_name_tag            1
#define google_protobuf_Type_fields_tag          2
#define google_protobuf_Type_oneofs_tag          3
#define google_protobuf_Type_options_tag         4
#define google_protobuf_Type_source_context_tag  5
#define google_protobuf_Type_syntax_tag          6

/* Struct field encoding specification for nanopb */
#define google_protobuf_Type_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, STRING,   name,              1) \
X(a, CALLBACK, REPEATED, MESSAGE,  fields,            2) \
X(a, CALLBACK, REPEATED, STRING,   oneofs,            3) \
X(a, CALLBACK, REPEATED, MESSAGE,  options,           4) \
X(a, STATIC,   OPTIONAL, MESSAGE,  source_context,    5) \
X(a, STATIC,   SINGULAR, UENUM,    syntax,            6)
#define google_protobuf_Type_CALLBACK pb_default_field_callback
#define google_protobuf_Type_DEFAULT NULL
#define google_protobuf_Type_fields_MSGTYPE google_protobuf_Field
#define google_protobuf_Type_options_MSGTYPE google_protobuf_Option
#define google_protobuf_Type_source_context_MSGTYPE google_protobuf_SourceContext

#define google_protobuf_Field_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UENUM,    kind,              1) \
X(a, STATIC,   SINGULAR, UENUM,    cardinality,       2) \
X(a, STATIC,   SINGULAR, INT32,    number,            3) \
X(a, CALLBACK, SINGULAR, STRING,   name,              4) \
X(a, CALLBACK, SINGULAR, STRING,   type_url,          6) \
X(a, STATIC,   SINGULAR, INT32,    oneof_index,       7) \
X(a, STATIC,   SINGULAR, BOOL,     packed,            8) \
X(a, CALLBACK, REPEATED, MESSAGE,  options,           9) \
X(a, CALLBACK, SINGULAR, STRING,   json_name,        10) \
X(a, CALLBACK, SINGULAR, STRING,   default_value,    11)
#define google_protobuf_Field_CALLBACK pb_default_field_callback
#define google_protobuf_Field_DEFAULT NULL
#define google_protobuf_Field_options_MSGTYPE google_protobuf_Option

#define google_protobuf_Enum_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, STRING,   name,              1) \
X(a, CALLBACK, REPEATED, MESSAGE,  enumvalue,         2) \
X(a, CALLBACK, REPEATED, MESSAGE,  options,           3) \
X(a, STATIC,   OPTIONAL, MESSAGE,  source_context,    4) \
X(a, STATIC,   SINGULAR, UENUM,    syntax,            5)
#define google_protobuf_Enum_CALLBACK pb_default_field_callback
#define google_protobuf_Enum_DEFAULT NULL
#define google_protobuf_Enum_enumvalue_MSGTYPE google_protobuf_EnumValue
#define google_protobuf_Enum_options_MSGTYPE google_protobuf_Option
#define google_protobuf_Enum_source_context_MSGTYPE google_protobuf_SourceContext

#define google_protobuf_EnumValue_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, STRING,   name,              1) \
X(a, STATIC,   SINGULAR, INT32,    number,            2) \
X(a, CALLBACK, REPEATED, MESSAGE,  options,           3)
#define google_protobuf_EnumValue_CALLBACK pb_default_field_callback
#define google_protobuf_EnumValue_DEFAULT NULL
#define google_protobuf_EnumValue_options_MSGTYPE google_protobuf_Option

#define google_protobuf_Option_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, STRING,   name,              1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  value,             2)
#define google_protobuf_Option_CALLBACK pb_default_field_callback
#define google_protobuf_Option_DEFAULT NULL
#define google_protobuf_Option_value_MSGTYPE google_protobuf_Any

extern const pb_msgdesc_t google_protobuf_Type_msg;
extern const pb_msgdesc_t google_protobuf_Field_msg;
extern const pb_msgdesc_t google_protobuf_Enum_msg;
extern const pb_msgdesc_t google_protobuf_EnumValue_msg;
extern const pb_msgdesc_t google_protobuf_Option_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define google_protobuf_Type_fields &google_protobuf_Type_msg
#define google_protobuf_Field_fields &google_protobuf_Field_msg
#define google_protobuf_Enum_fields &google_protobuf_Enum_msg
#define google_protobuf_EnumValue_fields &google_protobuf_EnumValue_msg
#define google_protobuf_Option_fields &google_protobuf_Option_msg

/* Maximum encoded size of messages (where known) */
/* google_protobuf_Type_size depends on runtime parameters */
/* google_protobuf_Field_size depends on runtime parameters */
/* google_protobuf_Enum_size depends on runtime parameters */
/* google_protobuf_EnumValue_size depends on runtime parameters */
/* google_protobuf_Option_size depends on runtime parameters */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
