# Overall lawn status message
# Combines information from grass detection, obstacles, and navigation

# Header
std_msgs/Header header

# Grass status
GrassDetection grass_status
float32 total_grass_area       # Total grass area in square meters
float32 mowable_area          # Area that can be safely mowed
float32 completion_percentage  # Mowing completion percentage

# Obstacle status  
ObstacleArray obstacle_status
int32 blocked_areas           # Number of areas blocked by obstacles

# Navigation status
bool navigation_active        # Whether navigation is active
geometry_msgs/Pose current_pose      # Current robot pose
geometry_msgs/Pose target_pose       # Target pose
nav_msgs/Path current_path           # Current planned path
float32 path_completion      # Path completion percentage

# System status
bool camera_healthy          # D455 camera status
bool grass_detection_active # Grass detection system status
bool obstacle_detection_active # Obstacle detection status
string system_state         # Overall system state
string[] warnings           # System warnings
string[] errors             # System errors

# Weather and environmental
float32 light_level         # Current light level (0-1)
bool weather_suitable       # Whether weather is suitable for mowing
string weather_condition    # Current weather description