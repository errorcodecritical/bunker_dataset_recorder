# Copyright (c) Sensrad 2024-2025
# Best known radar state, reflecting most recent commands sent.

# Standard header
std_msgs/Header header

# In tx
bool tx_enabled

Thresholds thresholds

int32[]  mode_ids
string[] mode_names
uint32 selected_mode_id
string selected_mode_name

bool is_connected

bool ptp_is_active

# Last timestamp sent to radar
uint64 timestamp_ms
