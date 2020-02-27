/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.1 */

#ifndef PB_GW_GW_PB_H_INCLUDED
#define PB_GW_GW_PB_H_INCLUDED
#include <pb.h>
#include "common/common.pb.h"
#include "google/protobuf/timestamp.pb.h"
#include "google/protobuf/duration.pb.h"

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Enum definitions */
typedef enum _gw_DownlinkTiming {
    gw_DownlinkTiming_IMMEDIATELY = 0,
    gw_DownlinkTiming_DELAY = 1,
    gw_DownlinkTiming_GPS_EPOCH = 2
} gw_DownlinkTiming;

typedef enum _gw_FineTimestampType {
    gw_FineTimestampType_NONE = 0,
    gw_FineTimestampType_ENCRYPTED = 1,
    gw_FineTimestampType_PLAIN = 2
} gw_FineTimestampType;

typedef enum _gw_CRCStatus {
    gw_CRCStatus_NO_CRC = 0,
    gw_CRCStatus_BAD_CRC = 1,
    gw_CRCStatus_CRC_OK = 2
} gw_CRCStatus;

/* Struct definitions */
typedef struct _gw_GatewayCommandExecRequest {
    pb_callback_t gateway_id;
    pb_callback_t command;
    pb_callback_t ExecId;
    pb_callback_t stdin_1;
    pb_callback_t environment;
} gw_GatewayCommandExecRequest;

typedef struct _gw_GatewayCommandExecRequest_EnvironmentEntry {
    pb_callback_t key;
    pb_callback_t value;
} gw_GatewayCommandExecRequest_EnvironmentEntry;

typedef struct _gw_GatewayCommandExecResponse {
    pb_callback_t gateway_id;
    pb_callback_t exec_id;
    pb_callback_t stdout_1;
    pb_callback_t stderr_1;
    pb_callback_t error;
} gw_GatewayCommandExecResponse;

typedef struct _gw_GatewayConfiguration {
    pb_callback_t gateway_id;
    pb_callback_t version;
    pb_callback_t channels;
} gw_GatewayConfiguration;

typedef struct _gw_GatewayStats_MetaDataEntry {
    pb_callback_t key;
    pb_callback_t value;
} gw_GatewayStats_MetaDataEntry;

typedef struct _gw_ImmediatelyTimingInfo {
    char dummy_field;
} gw_ImmediatelyTimingInfo;

typedef struct _gw_RawPacketForwarderCommand {
    pb_callback_t gateway_id;
    pb_callback_t raw_id;
    pb_callback_t payload;
} gw_RawPacketForwarderCommand;

typedef struct _gw_RawPacketForwarderEvent {
    pb_callback_t gateway_id;
    pb_callback_t raw_id;
    pb_callback_t payload;
} gw_RawPacketForwarderEvent;

typedef struct _gw_DelayTimingInfo {
    bool has_delay;
    google_protobuf_Duration delay;
} gw_DelayTimingInfo;

typedef struct _gw_DownlinkTXAck {
    pb_callback_t gateway_id;
    uint32_t token;
    pb_callback_t error;
    pb_callback_t downlink_id;
} gw_DownlinkTXAck;

typedef struct _gw_EncryptedFineTimestamp {
    uint32_t aes_key_index;
    pb_callback_t encrypted_ns;
    pb_callback_t fpga_id;
} gw_EncryptedFineTimestamp;

typedef struct _gw_FSKModulationConfig {
    uint32_t bandwidth;
    uint32_t bitrate;
} gw_FSKModulationConfig;

typedef struct _gw_FSKModulationInfo {
    uint32_t frequency_deviation;
    uint32_t datarate;
} gw_FSKModulationInfo;

typedef struct _gw_GPSEpochTimingInfo {
    bool has_time_since_gps_epoch;
    google_protobuf_Duration time_since_gps_epoch;
} gw_GPSEpochTimingInfo;

typedef struct _gw_GatewayStats {
    pb_callback_t gateway_id;
    bool has_time;
    google_protobuf_Timestamp time;
    bool has_location;
    common_Location location;
    pb_callback_t config_version;
    uint32_t rx_packets_received;
    uint32_t rx_packets_received_ok;
    uint32_t tx_packets_received;
    uint32_t tx_packets_emitted;
    pb_callback_t ip;
    pb_callback_t meta_data;
    pb_callback_t stats_id;
} gw_GatewayStats;

typedef struct _gw_LoRaModulationConfig {
    uint32_t bandwidth;
    pb_callback_t spreading_factors;
} gw_LoRaModulationConfig;

typedef struct _gw_LoRaModulationInfo {
    uint32_t bandwidth;
    uint32_t spreading_factor;
    pb_callback_t code_rate;
    bool polarization_inversion;
} gw_LoRaModulationInfo;

typedef struct _gw_PlainFineTimestamp {
    bool has_time;
    google_protobuf_Timestamp time;
} gw_PlainFineTimestamp;

typedef struct _gw_ChannelConfiguration {
    uint32_t frequency;
    common_Modulation modulation;
    pb_size_t which_modulation_config;
    union {
        gw_LoRaModulationConfig lora_modulation_config;
        gw_FSKModulationConfig fsk_modulation_config;
    } modulation_config;
    uint32_t board;
    uint32_t demodulator;
} gw_ChannelConfiguration;

typedef struct _gw_DownlinkTXInfo {
    pb_callback_t gateway_id;
    uint32_t frequency;
    int32_t power;
    common_Modulation modulation;
    pb_size_t which_modulation_info;
    union {
        gw_LoRaModulationInfo lora_modulation_info;
        gw_FSKModulationInfo fsk_modulation_info;
    } modulation_info;
    uint32_t board;
    uint32_t antenna;
    gw_DownlinkTiming timing;
    pb_size_t which_timing_info;
    union {
        gw_ImmediatelyTimingInfo immediately_timing_info;
        gw_DelayTimingInfo delay_timing_info;
        gw_GPSEpochTimingInfo gps_epoch_timing_info;
    } timing_info;
    pb_callback_t context;
} gw_DownlinkTXInfo;

typedef struct _gw_UplinkRXInfo {
    pb_callback_t gateway_id;
    bool has_time;
    google_protobuf_Timestamp time;
    bool has_time_since_gps_epoch;
    google_protobuf_Duration time_since_gps_epoch;
    int32_t rssi;
    double lora_snr;
    uint32_t channel;
    uint32_t rf_chain;
    uint32_t board;
    uint32_t antenna;
    bool has_location;
    common_Location location;
    gw_FineTimestampType fine_timestamp_type;
    pb_size_t which_fine_timestamp;
    union {
        gw_EncryptedFineTimestamp encrypted_fine_timestamp;
        gw_PlainFineTimestamp plain_fine_timestamp;
    } fine_timestamp;
    pb_callback_t context;
    pb_callback_t uplink_id;
    gw_CRCStatus crc_status;
} gw_UplinkRXInfo;

typedef struct _gw_UplinkTXInfo {
    uint32_t frequency;
    common_Modulation modulation;
    pb_size_t which_modulation_info;
    union {
        gw_LoRaModulationInfo lora_modulation_info;
        gw_FSKModulationInfo fsk_modulation_info;
    } modulation_info;
} gw_UplinkTXInfo;

typedef struct _gw_DownlinkFrame {
    pb_callback_t phy_payload;
    bool has_tx_info;
    gw_DownlinkTXInfo tx_info;
    uint32_t token;
    pb_callback_t downlink_id;
} gw_DownlinkFrame;

typedef struct _gw_UplinkFrame {
    pb_callback_t phy_payload;
    bool has_tx_info;
    gw_UplinkTXInfo tx_info;
    bool has_rx_info;
    gw_UplinkRXInfo rx_info;
} gw_UplinkFrame;

typedef struct _gw_UplinkFrameSet {
    pb_callback_t phy_payload;
    bool has_tx_info;
    gw_UplinkTXInfo tx_info;
    pb_callback_t rx_info;
} gw_UplinkFrameSet;


/* Helper constants for enums */
#define _gw_DownlinkTiming_MIN gw_DownlinkTiming_IMMEDIATELY
#define _gw_DownlinkTiming_MAX gw_DownlinkTiming_GPS_EPOCH
#define _gw_DownlinkTiming_ARRAYSIZE ((gw_DownlinkTiming)(gw_DownlinkTiming_GPS_EPOCH+1))

#define _gw_FineTimestampType_MIN gw_FineTimestampType_NONE
#define _gw_FineTimestampType_MAX gw_FineTimestampType_PLAIN
#define _gw_FineTimestampType_ARRAYSIZE ((gw_FineTimestampType)(gw_FineTimestampType_PLAIN+1))

#define _gw_CRCStatus_MIN gw_CRCStatus_NO_CRC
#define _gw_CRCStatus_MAX gw_CRCStatus_CRC_OK
#define _gw_CRCStatus_ARRAYSIZE ((gw_CRCStatus)(gw_CRCStatus_CRC_OK+1))


/* Initializer values for message structs */
#define gw_UplinkTXInfo_init_default             {0, _common_Modulation_MIN, 0, {gw_LoRaModulationInfo_init_default}}
#define gw_LoRaModulationInfo_init_default       {0, 0, {{NULL}, NULL}, 0}
#define gw_FSKModulationInfo_init_default        {0, 0}
#define gw_EncryptedFineTimestamp_init_default   {0, {{NULL}, NULL}, {{NULL}, NULL}}
#define gw_PlainFineTimestamp_init_default       {false, google_protobuf_Timestamp_init_default}
#define gw_GatewayStats_init_default             {{{NULL}, NULL}, false, google_protobuf_Timestamp_init_default, false, common_Location_init_default, {{NULL}, NULL}, 0, 0, 0, 0, {{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}}
#define gw_GatewayStats_MetaDataEntry_init_default {{{NULL}, NULL}, {{NULL}, NULL}}
#define gw_UplinkRXInfo_init_default             {{{NULL}, NULL}, false, google_protobuf_Timestamp_init_default, false, google_protobuf_Duration_init_default, 0, 0, 0, 0, 0, 0, false, common_Location_init_default, _gw_FineTimestampType_MIN, 0, {gw_EncryptedFineTimestamp_init_default}, {{NULL}, NULL}, {{NULL}, NULL}, _gw_CRCStatus_MIN}
#define gw_DownlinkTXInfo_init_default           {{{NULL}, NULL}, 0, 0, _common_Modulation_MIN, 0, {gw_LoRaModulationInfo_init_default}, 0, 0, _gw_DownlinkTiming_MIN, 0, {gw_ImmediatelyTimingInfo_init_default}, {{NULL}, NULL}}
#define gw_ImmediatelyTimingInfo_init_default    {0}
#define gw_DelayTimingInfo_init_default          {false, google_protobuf_Duration_init_default}
#define gw_GPSEpochTimingInfo_init_default       {false, google_protobuf_Duration_init_default}
#define gw_UplinkFrame_init_default              {{{NULL}, NULL}, false, gw_UplinkTXInfo_init_default, false, gw_UplinkRXInfo_init_default}
#define gw_UplinkFrameSet_init_default           {{{NULL}, NULL}, false, gw_UplinkTXInfo_init_default, {{NULL}, NULL}}
#define gw_DownlinkFrame_init_default            {{{NULL}, NULL}, false, gw_DownlinkTXInfo_init_default, 0, {{NULL}, NULL}}
#define gw_DownlinkTXAck_init_default            {{{NULL}, NULL}, 0, {{NULL}, NULL}, {{NULL}, NULL}}
#define gw_GatewayConfiguration_init_default     {{{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}}
#define gw_ChannelConfiguration_init_default     {0, _common_Modulation_MIN, 0, {gw_LoRaModulationConfig_init_default}, 0, 0}
#define gw_LoRaModulationConfig_init_default     {0, {{NULL}, NULL}}
#define gw_FSKModulationConfig_init_default      {0, 0}
#define gw_GatewayCommandExecRequest_init_default {{{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}}
#define gw_GatewayCommandExecRequest_EnvironmentEntry_init_default {{{NULL}, NULL}, {{NULL}, NULL}}
#define gw_GatewayCommandExecResponse_init_default {{{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}}
#define gw_RawPacketForwarderEvent_init_default  {{{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}}
#define gw_RawPacketForwarderCommand_init_default {{{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}}
#define gw_UplinkTXInfo_init_zero                {0, _common_Modulation_MIN, 0, {gw_LoRaModulationInfo_init_zero}}
#define gw_LoRaModulationInfo_init_zero          {0, 0, {{NULL}, NULL}, 0}
#define gw_FSKModulationInfo_init_zero           {0, 0}
#define gw_EncryptedFineTimestamp_init_zero      {0, {{NULL}, NULL}, {{NULL}, NULL}}
#define gw_PlainFineTimestamp_init_zero          {false, google_protobuf_Timestamp_init_zero}
#define gw_GatewayStats_init_zero                {{{NULL}, NULL}, false, google_protobuf_Timestamp_init_zero, false, common_Location_init_zero, {{NULL}, NULL}, 0, 0, 0, 0, {{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}}
#define gw_GatewayStats_MetaDataEntry_init_zero  {{{NULL}, NULL}, {{NULL}, NULL}}
#define gw_UplinkRXInfo_init_zero                {{{NULL}, NULL}, false, google_protobuf_Timestamp_init_zero, false, google_protobuf_Duration_init_zero, 0, 0, 0, 0, 0, 0, false, common_Location_init_zero, _gw_FineTimestampType_MIN, 0, {gw_EncryptedFineTimestamp_init_zero}, {{NULL}, NULL}, {{NULL}, NULL}, _gw_CRCStatus_MIN}
#define gw_DownlinkTXInfo_init_zero              {{{NULL}, NULL}, 0, 0, _common_Modulation_MIN, 0, {gw_LoRaModulationInfo_init_zero}, 0, 0, _gw_DownlinkTiming_MIN, 0, {gw_ImmediatelyTimingInfo_init_zero}, {{NULL}, NULL}}
#define gw_ImmediatelyTimingInfo_init_zero       {0}
#define gw_DelayTimingInfo_init_zero             {false, google_protobuf_Duration_init_zero}
#define gw_GPSEpochTimingInfo_init_zero          {false, google_protobuf_Duration_init_zero}
#define gw_UplinkFrame_init_zero                 {{{NULL}, NULL}, false, gw_UplinkTXInfo_init_zero, false, gw_UplinkRXInfo_init_zero}
#define gw_UplinkFrameSet_init_zero              {{{NULL}, NULL}, false, gw_UplinkTXInfo_init_zero, {{NULL}, NULL}}
#define gw_DownlinkFrame_init_zero               {{{NULL}, NULL}, false, gw_DownlinkTXInfo_init_zero, 0, {{NULL}, NULL}}
#define gw_DownlinkTXAck_init_zero               {{{NULL}, NULL}, 0, {{NULL}, NULL}, {{NULL}, NULL}}
#define gw_GatewayConfiguration_init_zero        {{{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}}
#define gw_ChannelConfiguration_init_zero        {0, _common_Modulation_MIN, 0, {gw_LoRaModulationConfig_init_zero}, 0, 0}
#define gw_LoRaModulationConfig_init_zero        {0, {{NULL}, NULL}}
#define gw_FSKModulationConfig_init_zero         {0, 0}
#define gw_GatewayCommandExecRequest_init_zero   {{{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}}
#define gw_GatewayCommandExecRequest_EnvironmentEntry_init_zero {{{NULL}, NULL}, {{NULL}, NULL}}
#define gw_GatewayCommandExecResponse_init_zero  {{{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}}
#define gw_RawPacketForwarderEvent_init_zero     {{{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}}
#define gw_RawPacketForwarderCommand_init_zero   {{{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}}

/* Field tags (for use in manual encoding/decoding) */
#define gw_GatewayCommandExecRequest_gateway_id_tag 1
#define gw_GatewayCommandExecRequest_command_tag 2
#define gw_GatewayCommandExecRequest_ExecId_tag  3
#define gw_GatewayCommandExecRequest_stdin_tag   4
#define gw_GatewayCommandExecRequest_environment_tag 5
#define gw_GatewayCommandExecRequest_EnvironmentEntry_key_tag 1
#define gw_GatewayCommandExecRequest_EnvironmentEntry_value_tag 2
#define gw_GatewayCommandExecResponse_gateway_id_tag 1
#define gw_GatewayCommandExecResponse_exec_id_tag 2
#define gw_GatewayCommandExecResponse_stdout_tag 3
#define gw_GatewayCommandExecResponse_stderr_tag 4
#define gw_GatewayCommandExecResponse_error_tag  5
#define gw_GatewayConfiguration_gateway_id_tag   1
#define gw_GatewayConfiguration_version_tag      2
#define gw_GatewayConfiguration_channels_tag     3
#define gw_GatewayStats_MetaDataEntry_key_tag    1
#define gw_GatewayStats_MetaDataEntry_value_tag  2
#define gw_RawPacketForwarderCommand_gateway_id_tag 1
#define gw_RawPacketForwarderCommand_raw_id_tag  2
#define gw_RawPacketForwarderCommand_payload_tag 3
#define gw_RawPacketForwarderEvent_gateway_id_tag 1
#define gw_RawPacketForwarderEvent_raw_id_tag    2
#define gw_RawPacketForwarderEvent_payload_tag   3
#define gw_DelayTimingInfo_delay_tag             1
#define gw_DownlinkTXAck_gateway_id_tag          1
#define gw_DownlinkTXAck_token_tag               2
#define gw_DownlinkTXAck_error_tag               3
#define gw_DownlinkTXAck_downlink_id_tag         4
#define gw_EncryptedFineTimestamp_aes_key_index_tag 1
#define gw_EncryptedFineTimestamp_encrypted_ns_tag 2
#define gw_EncryptedFineTimestamp_fpga_id_tag    3
#define gw_FSKModulationConfig_bandwidth_tag     1
#define gw_FSKModulationConfig_bitrate_tag       2
#define gw_FSKModulationInfo_frequency_deviation_tag 1
#define gw_FSKModulationInfo_datarate_tag        2
#define gw_GPSEpochTimingInfo_time_since_gps_epoch_tag 1
#define gw_GatewayStats_gateway_id_tag           1
#define gw_GatewayStats_ip_tag                   9
#define gw_GatewayStats_time_tag                 2
#define gw_GatewayStats_location_tag             3
#define gw_GatewayStats_config_version_tag       4
#define gw_GatewayStats_rx_packets_received_tag  5
#define gw_GatewayStats_rx_packets_received_ok_tag 6
#define gw_GatewayStats_tx_packets_received_tag  7
#define gw_GatewayStats_tx_packets_emitted_tag   8
#define gw_GatewayStats_meta_data_tag            10
#define gw_GatewayStats_stats_id_tag             11
#define gw_LoRaModulationConfig_bandwidth_tag    1
#define gw_LoRaModulationConfig_spreading_factors_tag 2
#define gw_LoRaModulationInfo_bandwidth_tag      1
#define gw_LoRaModulationInfo_spreading_factor_tag 2
#define gw_LoRaModulationInfo_code_rate_tag      3
#define gw_LoRaModulationInfo_polarization_inversion_tag 4
#define gw_PlainFineTimestamp_time_tag           1
#define gw_ChannelConfiguration_lora_modulation_config_tag 3
#define gw_ChannelConfiguration_fsk_modulation_config_tag 4
#define gw_ChannelConfiguration_frequency_tag    1
#define gw_ChannelConfiguration_modulation_tag   2
#define gw_ChannelConfiguration_board_tag        5
#define gw_ChannelConfiguration_demodulator_tag  6
#define gw_DownlinkTXInfo_lora_modulation_info_tag 8
#define gw_DownlinkTXInfo_fsk_modulation_info_tag 9
#define gw_DownlinkTXInfo_immediately_timing_info_tag 13
#define gw_DownlinkTXInfo_delay_timing_info_tag  14
#define gw_DownlinkTXInfo_gps_epoch_timing_info_tag 15
#define gw_DownlinkTXInfo_gateway_id_tag         1
#define gw_DownlinkTXInfo_frequency_tag          5
#define gw_DownlinkTXInfo_power_tag              6
#define gw_DownlinkTXInfo_modulation_tag         7
#define gw_DownlinkTXInfo_board_tag              10
#define gw_DownlinkTXInfo_antenna_tag            11
#define gw_DownlinkTXInfo_timing_tag             12
#define gw_DownlinkTXInfo_context_tag            16
#define gw_UplinkRXInfo_encrypted_fine_timestamp_tag 13
#define gw_UplinkRXInfo_plain_fine_timestamp_tag 14
#define gw_UplinkRXInfo_gateway_id_tag           1
#define gw_UplinkRXInfo_time_tag                 2
#define gw_UplinkRXInfo_time_since_gps_epoch_tag 3
#define gw_UplinkRXInfo_rssi_tag                 5
#define gw_UplinkRXInfo_lora_snr_tag             6
#define gw_UplinkRXInfo_channel_tag              7
#define gw_UplinkRXInfo_rf_chain_tag             8
#define gw_UplinkRXInfo_board_tag                9
#define gw_UplinkRXInfo_antenna_tag              10
#define gw_UplinkRXInfo_location_tag             11
#define gw_UplinkRXInfo_fine_timestamp_type_tag  12
#define gw_UplinkRXInfo_context_tag              15
#define gw_UplinkRXInfo_uplink_id_tag            16
#define gw_UplinkRXInfo_crc_status_tag           17
#define gw_UplinkTXInfo_lora_modulation_info_tag 3
#define gw_UplinkTXInfo_fsk_modulation_info_tag  4
#define gw_UplinkTXInfo_frequency_tag            1
#define gw_UplinkTXInfo_modulation_tag           2
#define gw_DownlinkFrame_phy_payload_tag         1
#define gw_DownlinkFrame_tx_info_tag             2
#define gw_DownlinkFrame_token_tag               3
#define gw_DownlinkFrame_downlink_id_tag         4
#define gw_UplinkFrame_phy_payload_tag           1
#define gw_UplinkFrame_tx_info_tag               2
#define gw_UplinkFrame_rx_info_tag               3
#define gw_UplinkFrameSet_phy_payload_tag        1
#define gw_UplinkFrameSet_tx_info_tag            2
#define gw_UplinkFrameSet_rx_info_tag            3

/* Struct field encoding specification for nanopb */
#define gw_UplinkTXInfo_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   frequency,         1) \
X(a, STATIC,   SINGULAR, UENUM,    modulation,        2) \
X(a, STATIC,   ONEOF,    MESSAGE,  (modulation_info,lora_modulation_info,modulation_info.lora_modulation_info),   3) \
X(a, STATIC,   ONEOF,    MESSAGE,  (modulation_info,fsk_modulation_info,modulation_info.fsk_modulation_info),   4)
#define gw_UplinkTXInfo_CALLBACK NULL
#define gw_UplinkTXInfo_DEFAULT NULL
#define gw_UplinkTXInfo_modulation_info_lora_modulation_info_MSGTYPE gw_LoRaModulationInfo
#define gw_UplinkTXInfo_modulation_info_fsk_modulation_info_MSGTYPE gw_FSKModulationInfo

#define gw_LoRaModulationInfo_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   bandwidth,         1) \
X(a, STATIC,   SINGULAR, UINT32,   spreading_factor,   2) \
X(a, CALLBACK, SINGULAR, STRING,   code_rate,         3) \
X(a, STATIC,   SINGULAR, BOOL,     polarization_inversion,   4)
#define gw_LoRaModulationInfo_CALLBACK pb_default_field_callback
#define gw_LoRaModulationInfo_DEFAULT NULL

#define gw_FSKModulationInfo_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   frequency_deviation,   1) \
X(a, STATIC,   SINGULAR, UINT32,   datarate,          2)
#define gw_FSKModulationInfo_CALLBACK NULL
#define gw_FSKModulationInfo_DEFAULT NULL

#define gw_EncryptedFineTimestamp_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   aes_key_index,     1) \
X(a, CALLBACK, SINGULAR, BYTES,    encrypted_ns,      2) \
X(a, CALLBACK, SINGULAR, BYTES,    fpga_id,           3)
#define gw_EncryptedFineTimestamp_CALLBACK pb_default_field_callback
#define gw_EncryptedFineTimestamp_DEFAULT NULL

#define gw_PlainFineTimestamp_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  time,              1)
#define gw_PlainFineTimestamp_CALLBACK NULL
#define gw_PlainFineTimestamp_DEFAULT NULL
#define gw_PlainFineTimestamp_time_MSGTYPE google_protobuf_Timestamp

#define gw_GatewayStats_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, BYTES,    gateway_id,        1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  time,              2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  location,          3) \
X(a, CALLBACK, SINGULAR, STRING,   config_version,    4) \
X(a, STATIC,   SINGULAR, UINT32,   rx_packets_received,   5) \
X(a, STATIC,   SINGULAR, UINT32,   rx_packets_received_ok,   6) \
X(a, STATIC,   SINGULAR, UINT32,   tx_packets_received,   7) \
X(a, STATIC,   SINGULAR, UINT32,   tx_packets_emitted,   8) \
X(a, CALLBACK, SINGULAR, STRING,   ip,                9) \
X(a, CALLBACK, REPEATED, MESSAGE,  meta_data,        10) \
X(a, CALLBACK, SINGULAR, BYTES,    stats_id,         11)
#define gw_GatewayStats_CALLBACK pb_default_field_callback
#define gw_GatewayStats_DEFAULT NULL
#define gw_GatewayStats_time_MSGTYPE google_protobuf_Timestamp
#define gw_GatewayStats_location_MSGTYPE common_Location
#define gw_GatewayStats_meta_data_MSGTYPE gw_GatewayStats_MetaDataEntry

#define gw_GatewayStats_MetaDataEntry_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, STRING,   key,               1) \
X(a, CALLBACK, SINGULAR, STRING,   value,             2)
#define gw_GatewayStats_MetaDataEntry_CALLBACK pb_default_field_callback
#define gw_GatewayStats_MetaDataEntry_DEFAULT NULL

#define gw_UplinkRXInfo_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, BYTES,    gateway_id,        1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  time,              2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  time_since_gps_epoch,   3) \
X(a, STATIC,   SINGULAR, INT32,    rssi,              5) \
X(a, STATIC,   SINGULAR, DOUBLE,   lora_snr,          6) \
X(a, STATIC,   SINGULAR, UINT32,   channel,           7) \
X(a, STATIC,   SINGULAR, UINT32,   rf_chain,          8) \
X(a, STATIC,   SINGULAR, UINT32,   board,             9) \
X(a, STATIC,   SINGULAR, UINT32,   antenna,          10) \
X(a, STATIC,   OPTIONAL, MESSAGE,  location,         11) \
X(a, STATIC,   SINGULAR, UENUM,    fine_timestamp_type,  12) \
X(a, STATIC,   ONEOF,    MESSAGE,  (fine_timestamp,encrypted_fine_timestamp,fine_timestamp.encrypted_fine_timestamp),  13) \
X(a, STATIC,   ONEOF,    MESSAGE,  (fine_timestamp,plain_fine_timestamp,fine_timestamp.plain_fine_timestamp),  14) \
X(a, CALLBACK, SINGULAR, BYTES,    context,          15) \
X(a, CALLBACK, SINGULAR, BYTES,    uplink_id,        16) \
X(a, STATIC,   SINGULAR, UENUM,    crc_status,       17)
#define gw_UplinkRXInfo_CALLBACK pb_default_field_callback
#define gw_UplinkRXInfo_DEFAULT NULL
#define gw_UplinkRXInfo_time_MSGTYPE google_protobuf_Timestamp
#define gw_UplinkRXInfo_time_since_gps_epoch_MSGTYPE google_protobuf_Duration
#define gw_UplinkRXInfo_location_MSGTYPE common_Location
#define gw_UplinkRXInfo_fine_timestamp_encrypted_fine_timestamp_MSGTYPE gw_EncryptedFineTimestamp
#define gw_UplinkRXInfo_fine_timestamp_plain_fine_timestamp_MSGTYPE gw_PlainFineTimestamp

#define gw_DownlinkTXInfo_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, BYTES,    gateway_id,        1) \
X(a, STATIC,   SINGULAR, UINT32,   frequency,         5) \
X(a, STATIC,   SINGULAR, INT32,    power,             6) \
X(a, STATIC,   SINGULAR, UENUM,    modulation,        7) \
X(a, STATIC,   ONEOF,    MESSAGE,  (modulation_info,lora_modulation_info,modulation_info.lora_modulation_info),   8) \
X(a, STATIC,   ONEOF,    MESSAGE,  (modulation_info,fsk_modulation_info,modulation_info.fsk_modulation_info),   9) \
X(a, STATIC,   SINGULAR, UINT32,   board,            10) \
X(a, STATIC,   SINGULAR, UINT32,   antenna,          11) \
X(a, STATIC,   SINGULAR, UENUM,    timing,           12) \
X(a, STATIC,   ONEOF,    MESSAGE,  (timing_info,immediately_timing_info,timing_info.immediately_timing_info),  13) \
X(a, STATIC,   ONEOF,    MESSAGE,  (timing_info,delay_timing_info,timing_info.delay_timing_info),  14) \
X(a, STATIC,   ONEOF,    MESSAGE,  (timing_info,gps_epoch_timing_info,timing_info.gps_epoch_timing_info),  15) \
X(a, CALLBACK, SINGULAR, BYTES,    context,          16)
#define gw_DownlinkTXInfo_CALLBACK pb_default_field_callback
#define gw_DownlinkTXInfo_DEFAULT NULL
#define gw_DownlinkTXInfo_modulation_info_lora_modulation_info_MSGTYPE gw_LoRaModulationInfo
#define gw_DownlinkTXInfo_modulation_info_fsk_modulation_info_MSGTYPE gw_FSKModulationInfo
#define gw_DownlinkTXInfo_timing_info_immediately_timing_info_MSGTYPE gw_ImmediatelyTimingInfo
#define gw_DownlinkTXInfo_timing_info_delay_timing_info_MSGTYPE gw_DelayTimingInfo
#define gw_DownlinkTXInfo_timing_info_gps_epoch_timing_info_MSGTYPE gw_GPSEpochTimingInfo

#define gw_ImmediatelyTimingInfo_FIELDLIST(X, a) \

#define gw_ImmediatelyTimingInfo_CALLBACK NULL
#define gw_ImmediatelyTimingInfo_DEFAULT NULL

#define gw_DelayTimingInfo_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  delay,             1)
#define gw_DelayTimingInfo_CALLBACK NULL
#define gw_DelayTimingInfo_DEFAULT NULL
#define gw_DelayTimingInfo_delay_MSGTYPE google_protobuf_Duration

#define gw_GPSEpochTimingInfo_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  time_since_gps_epoch,   1)
#define gw_GPSEpochTimingInfo_CALLBACK NULL
#define gw_GPSEpochTimingInfo_DEFAULT NULL
#define gw_GPSEpochTimingInfo_time_since_gps_epoch_MSGTYPE google_protobuf_Duration

#define gw_UplinkFrame_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, BYTES,    phy_payload,       1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  tx_info,           2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  rx_info,           3)
#define gw_UplinkFrame_CALLBACK pb_default_field_callback
#define gw_UplinkFrame_DEFAULT NULL
#define gw_UplinkFrame_tx_info_MSGTYPE gw_UplinkTXInfo
#define gw_UplinkFrame_rx_info_MSGTYPE gw_UplinkRXInfo

#define gw_UplinkFrameSet_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, BYTES,    phy_payload,       1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  tx_info,           2) \
X(a, CALLBACK, REPEATED, MESSAGE,  rx_info,           3)
#define gw_UplinkFrameSet_CALLBACK pb_default_field_callback
#define gw_UplinkFrameSet_DEFAULT NULL
#define gw_UplinkFrameSet_tx_info_MSGTYPE gw_UplinkTXInfo
#define gw_UplinkFrameSet_rx_info_MSGTYPE gw_UplinkRXInfo

#define gw_DownlinkFrame_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, BYTES,    phy_payload,       1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  tx_info,           2) \
X(a, STATIC,   SINGULAR, UINT32,   token,             3) \
X(a, CALLBACK, SINGULAR, BYTES,    downlink_id,       4)
#define gw_DownlinkFrame_CALLBACK pb_default_field_callback
#define gw_DownlinkFrame_DEFAULT NULL
#define gw_DownlinkFrame_tx_info_MSGTYPE gw_DownlinkTXInfo

#define gw_DownlinkTXAck_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, BYTES,    gateway_id,        1) \
X(a, STATIC,   SINGULAR, UINT32,   token,             2) \
X(a, CALLBACK, SINGULAR, STRING,   error,             3) \
X(a, CALLBACK, SINGULAR, BYTES,    downlink_id,       4)
#define gw_DownlinkTXAck_CALLBACK pb_default_field_callback
#define gw_DownlinkTXAck_DEFAULT NULL

#define gw_GatewayConfiguration_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, BYTES,    gateway_id,        1) \
X(a, CALLBACK, SINGULAR, STRING,   version,           2) \
X(a, CALLBACK, REPEATED, MESSAGE,  channels,          3)
#define gw_GatewayConfiguration_CALLBACK pb_default_field_callback
#define gw_GatewayConfiguration_DEFAULT NULL
#define gw_GatewayConfiguration_channels_MSGTYPE gw_ChannelConfiguration

#define gw_ChannelConfiguration_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   frequency,         1) \
X(a, STATIC,   SINGULAR, UENUM,    modulation,        2) \
X(a, STATIC,   ONEOF,    MESSAGE,  (modulation_config,lora_modulation_config,modulation_config.lora_modulation_config),   3) \
X(a, STATIC,   ONEOF,    MESSAGE,  (modulation_config,fsk_modulation_config,modulation_config.fsk_modulation_config),   4) \
X(a, STATIC,   SINGULAR, UINT32,   board,             5) \
X(a, STATIC,   SINGULAR, UINT32,   demodulator,       6)
#define gw_ChannelConfiguration_CALLBACK NULL
#define gw_ChannelConfiguration_DEFAULT NULL
#define gw_ChannelConfiguration_modulation_config_lora_modulation_config_MSGTYPE gw_LoRaModulationConfig
#define gw_ChannelConfiguration_modulation_config_fsk_modulation_config_MSGTYPE gw_FSKModulationConfig

#define gw_LoRaModulationConfig_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   bandwidth,         1) \
X(a, CALLBACK, REPEATED, UINT32,   spreading_factors,   2)
#define gw_LoRaModulationConfig_CALLBACK pb_default_field_callback
#define gw_LoRaModulationConfig_DEFAULT NULL

#define gw_FSKModulationConfig_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   bandwidth,         1) \
X(a, STATIC,   SINGULAR, UINT32,   bitrate,           2)
#define gw_FSKModulationConfig_CALLBACK NULL
#define gw_FSKModulationConfig_DEFAULT NULL

#define gw_GatewayCommandExecRequest_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, BYTES,    gateway_id,        1) \
X(a, CALLBACK, SINGULAR, STRING,   command,           2) \
X(a, CALLBACK, SINGULAR, BYTES,    ExecId,            3) \
X(a, CALLBACK, SINGULAR, BYTES,    stdin,             4) \
X(a, CALLBACK, REPEATED, MESSAGE,  environment,       5)
#define gw_GatewayCommandExecRequest_CALLBACK pb_default_field_callback
#define gw_GatewayCommandExecRequest_DEFAULT NULL
#define gw_GatewayCommandExecRequest_environment_MSGTYPE gw_GatewayCommandExecRequest_EnvironmentEntry

#define gw_GatewayCommandExecRequest_EnvironmentEntry_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, STRING,   key,               1) \
X(a, CALLBACK, SINGULAR, STRING,   value,             2)
#define gw_GatewayCommandExecRequest_EnvironmentEntry_CALLBACK pb_default_field_callback
#define gw_GatewayCommandExecRequest_EnvironmentEntry_DEFAULT NULL

#define gw_GatewayCommandExecResponse_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, BYTES,    gateway_id,        1) \
X(a, CALLBACK, SINGULAR, BYTES,    exec_id,           2) \
X(a, CALLBACK, SINGULAR, BYTES,    stdout,            3) \
X(a, CALLBACK, SINGULAR, BYTES,    stderr,            4) \
X(a, CALLBACK, SINGULAR, STRING,   error,             5)
#define gw_GatewayCommandExecResponse_CALLBACK pb_default_field_callback
#define gw_GatewayCommandExecResponse_DEFAULT NULL

#define gw_RawPacketForwarderEvent_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, BYTES,    gateway_id,        1) \
X(a, CALLBACK, SINGULAR, BYTES,    raw_id,            2) \
X(a, CALLBACK, SINGULAR, BYTES,    payload,           3)
#define gw_RawPacketForwarderEvent_CALLBACK pb_default_field_callback
#define gw_RawPacketForwarderEvent_DEFAULT NULL

#define gw_RawPacketForwarderCommand_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, BYTES,    gateway_id,        1) \
X(a, CALLBACK, SINGULAR, BYTES,    raw_id,            2) \
X(a, CALLBACK, SINGULAR, BYTES,    payload,           3)
#define gw_RawPacketForwarderCommand_CALLBACK pb_default_field_callback
#define gw_RawPacketForwarderCommand_DEFAULT NULL

extern const pb_msgdesc_t gw_UplinkTXInfo_msg;
extern const pb_msgdesc_t gw_LoRaModulationInfo_msg;
extern const pb_msgdesc_t gw_FSKModulationInfo_msg;
extern const pb_msgdesc_t gw_EncryptedFineTimestamp_msg;
extern const pb_msgdesc_t gw_PlainFineTimestamp_msg;
extern const pb_msgdesc_t gw_GatewayStats_msg;
extern const pb_msgdesc_t gw_GatewayStats_MetaDataEntry_msg;
extern const pb_msgdesc_t gw_UplinkRXInfo_msg;
extern const pb_msgdesc_t gw_DownlinkTXInfo_msg;
extern const pb_msgdesc_t gw_ImmediatelyTimingInfo_msg;
extern const pb_msgdesc_t gw_DelayTimingInfo_msg;
extern const pb_msgdesc_t gw_GPSEpochTimingInfo_msg;
extern const pb_msgdesc_t gw_UplinkFrame_msg;
extern const pb_msgdesc_t gw_UplinkFrameSet_msg;
extern const pb_msgdesc_t gw_DownlinkFrame_msg;
extern const pb_msgdesc_t gw_DownlinkTXAck_msg;
extern const pb_msgdesc_t gw_GatewayConfiguration_msg;
extern const pb_msgdesc_t gw_ChannelConfiguration_msg;
extern const pb_msgdesc_t gw_LoRaModulationConfig_msg;
extern const pb_msgdesc_t gw_FSKModulationConfig_msg;
extern const pb_msgdesc_t gw_GatewayCommandExecRequest_msg;
extern const pb_msgdesc_t gw_GatewayCommandExecRequest_EnvironmentEntry_msg;
extern const pb_msgdesc_t gw_GatewayCommandExecResponse_msg;
extern const pb_msgdesc_t gw_RawPacketForwarderEvent_msg;
extern const pb_msgdesc_t gw_RawPacketForwarderCommand_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define gw_UplinkTXInfo_fields &gw_UplinkTXInfo_msg
#define gw_LoRaModulationInfo_fields &gw_LoRaModulationInfo_msg
#define gw_FSKModulationInfo_fields &gw_FSKModulationInfo_msg
#define gw_EncryptedFineTimestamp_fields &gw_EncryptedFineTimestamp_msg
#define gw_PlainFineTimestamp_fields &gw_PlainFineTimestamp_msg
#define gw_GatewayStats_fields &gw_GatewayStats_msg
#define gw_GatewayStats_MetaDataEntry_fields &gw_GatewayStats_MetaDataEntry_msg
#define gw_UplinkRXInfo_fields &gw_UplinkRXInfo_msg
#define gw_DownlinkTXInfo_fields &gw_DownlinkTXInfo_msg
#define gw_ImmediatelyTimingInfo_fields &gw_ImmediatelyTimingInfo_msg
#define gw_DelayTimingInfo_fields &gw_DelayTimingInfo_msg
#define gw_GPSEpochTimingInfo_fields &gw_GPSEpochTimingInfo_msg
#define gw_UplinkFrame_fields &gw_UplinkFrame_msg
#define gw_UplinkFrameSet_fields &gw_UplinkFrameSet_msg
#define gw_DownlinkFrame_fields &gw_DownlinkFrame_msg
#define gw_DownlinkTXAck_fields &gw_DownlinkTXAck_msg
#define gw_GatewayConfiguration_fields &gw_GatewayConfiguration_msg
#define gw_ChannelConfiguration_fields &gw_ChannelConfiguration_msg
#define gw_LoRaModulationConfig_fields &gw_LoRaModulationConfig_msg
#define gw_FSKModulationConfig_fields &gw_FSKModulationConfig_msg
#define gw_GatewayCommandExecRequest_fields &gw_GatewayCommandExecRequest_msg
#define gw_GatewayCommandExecRequest_EnvironmentEntry_fields &gw_GatewayCommandExecRequest_EnvironmentEntry_msg
#define gw_GatewayCommandExecResponse_fields &gw_GatewayCommandExecResponse_msg
#define gw_RawPacketForwarderEvent_fields &gw_RawPacketForwarderEvent_msg
#define gw_RawPacketForwarderCommand_fields &gw_RawPacketForwarderCommand_msg

/* Maximum encoded size of messages (where known) */
/* gw_UplinkTXInfo_size depends on runtime parameters */
/* gw_LoRaModulationInfo_size depends on runtime parameters */
#define gw_FSKModulationInfo_size                12
/* gw_EncryptedFineTimestamp_size depends on runtime parameters */
#define gw_PlainFineTimestamp_size               24
/* gw_GatewayStats_size depends on runtime parameters */
/* gw_GatewayStats_MetaDataEntry_size depends on runtime parameters */
/* gw_UplinkRXInfo_size depends on runtime parameters */
/* gw_DownlinkTXInfo_size depends on runtime parameters */
#define gw_ImmediatelyTimingInfo_size            0
#define gw_DelayTimingInfo_size                  24
#define gw_GPSEpochTimingInfo_size               24
/* gw_UplinkFrame_size depends on runtime parameters */
/* gw_UplinkFrameSet_size depends on runtime parameters */
/* gw_DownlinkFrame_size depends on runtime parameters */
/* gw_DownlinkTXAck_size depends on runtime parameters */
/* gw_GatewayConfiguration_size depends on runtime parameters */
/* gw_ChannelConfiguration_size depends on runtime parameters */
/* gw_LoRaModulationConfig_size depends on runtime parameters */
#define gw_FSKModulationConfig_size              12
/* gw_GatewayCommandExecRequest_size depends on runtime parameters */
/* gw_GatewayCommandExecRequest_EnvironmentEntry_size depends on runtime parameters */
/* gw_GatewayCommandExecResponse_size depends on runtime parameters */
/* gw_RawPacketForwarderEvent_size depends on runtime parameters */
/* gw_RawPacketForwarderCommand_size depends on runtime parameters */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif