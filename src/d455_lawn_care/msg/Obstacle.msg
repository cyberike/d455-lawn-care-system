# Individual Obstacle Message
# Represents a single detected obstacle in the lawn

# Obstacle identification
int32 id                        # Unique obstacle ID
string type                     # Obstacle type (tree, rock, furniture, etc.)

# 3D position and size
geometry_msgs/Point position    # Center position in 3D space
geometry_msgs/Vector3 size      # Width, depth, height of obstacle
float32 height                  # Height above ground in meters

# Obstacle shape approximation
geometry_msgs/Polygon footprint # 2D footprint polygon
sensor_msgs/PointCloud2 point_cloud  # Detailed point cloud of obstacle

# Classification confidence
float32 confidence             # Detection confidence (0-1)
float32 classification_confidence  # Type classification confidence (0-1)

# Temporal information
builtin_interfaces/Time first_seen    # When obstacle was first detected
builtin_interfaces/Time last_seen     # Most recent detection
int32 detection_count          # Number of times detected

# Motion properties
geometry_msgs/Vector3 velocity  # Estimated velocity if moving
bool is_static                 # Whether obstacle is considered static
bool is_temporary             # Whether obstacle might be temporary

# Safety information
float32 danger_level          # Danger level for mowing (0-1)
float32 min_safe_distance     # Minimum safe approach distance