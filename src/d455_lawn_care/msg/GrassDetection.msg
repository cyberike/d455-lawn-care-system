# Grass Detection Message
# Contains information about detected grass areas and characteristics

# Header for timestamp and frame information
std_msgs/Header header

# Grass detection results
float32 grass_coverage_percentage    # Percentage of image covered by grass (0-100)
float32 average_grass_height        # Average detected grass height in meters
float32 min_grass_height           # Minimum grass height detected
float32 max_grass_height           # Maximum grass height detected

# Grass health indicators  
float32 grass_density              # Grass density metric (0-1)
float32 grass_greenness           # Green color intensity (0-1)
bool grass_needs_cutting          # Whether grass height exceeds cutting threshold

# Detection confidence
float32 detection_confidence      # Confidence in detection (0-1)
int32 grass_pixel_count          # Number of pixels classified as grass
int32 total_pixel_count          # Total pixels in analyzed region

# Grass area polygon (in image coordinates)
geometry_msgs/Polygon[] grass_regions

# Additional metadata
string grass_type                # Detected grass type (if classifier available)
float32 moisture_estimate        # Estimated moisture level (0-1)
bool shadow_detected            # Whether significant shadows are present