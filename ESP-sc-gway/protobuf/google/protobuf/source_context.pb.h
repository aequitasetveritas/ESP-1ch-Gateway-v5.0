/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.1 */

#ifndef PB_GOOGLE_PROTOBUF_GOOGLE_PROTOBUF_SOURCE_CONTEXT_PB_H_INCLUDED
#define PB_GOOGLE_PROTOBUF_GOOGLE_PROTOBUF_SOURCE_CONTEXT_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _google_protobuf_SourceContext {
    pb_callback_t file_name;
} google_protobuf_SourceContext;


/* Initializer values for message structs */
#define google_protobuf_SourceContext_init_default {{{NULL}, NULL}}
#define google_protobuf_SourceContext_init_zero  {{{NULL}, NULL}}

/* Field tags (for use in manual encoding/decoding) */
#define google_protobuf_SourceContext_file_name_tag 1

/* Struct field encoding specification for nanopb */
#define google_protobuf_SourceContext_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, STRING,   file_name,         1)
#define google_protobuf_SourceContext_CALLBACK pb_default_field_callback
#define google_protobuf_SourceContext_DEFAULT NULL

extern const pb_msgdesc_t google_protobuf_SourceContext_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define google_protobuf_SourceContext_fields &google_protobuf_SourceContext_msg

/* Maximum encoded size of messages (where known) */
/* google_protobuf_SourceContext_size depends on runtime parameters */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
