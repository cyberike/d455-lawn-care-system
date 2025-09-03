# Laser Weed Incisor Status Message

Header header

# System state
string current_state
bool laser_enabled 
bool emergency_stop_active
bool safety_confirmed

# Laser specifications
float32 max_power_watts
float32 current_power_watts
float32 wavelength_nm
string safety_class

# Targeting statistics
int32 total_weeds_targeted
int32 targeting_queue_length
float32 targeting_accuracy_mm
float32 last_fire_time_ms

# Safety status
bool safety_interlocks_ok
float32 system_temperature_celsius
string[] active_warnings
string[] safety_violations

# Operational parameters
float32 laser_power_percentage
int32 exposure_time_ms
float32 cooldown_remaining_ms