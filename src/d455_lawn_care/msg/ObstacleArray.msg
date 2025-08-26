# Array of detected obstacles
# Contains all obstacles currently tracked by the system

# Header for timestamp and frame information  
std_msgs/Header header

# Array of detected obstacles
Obstacle[] obstacles

# Summary statistics
int32 total_obstacles          # Total number of obstacles
int32 static_obstacles         # Number of static obstacles  
int32 dynamic_obstacles        # Number of moving obstacles
int32 temporary_obstacles      # Number of temporary obstacles

# Detection area information
geometry_msgs/Polygon detection_area  # Area that was scanned
float32 detection_range        # Maximum detection range used
float32 coverage_percentage    # Percentage of lawn area scanned