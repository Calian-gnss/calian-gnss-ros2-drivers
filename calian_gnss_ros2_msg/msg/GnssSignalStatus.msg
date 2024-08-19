# header.stamp specifies the ROS time for this measurement (the
#        corresponding satellite time may be reported using the
#        sensor_msgs/TimeReference message).
#
# header.frame_id is the frame of reference reported by the satellite
#        receiver, usually the location of the antenna.  This is a
#        Euclidean frame relative to the vehicle, not a reference
#        ellipsoid.
std_msgs/Header header

# satellite fix status information
sensor_msgs/NavSatStatus status

# Latitude [degrees]. Positive is north of equator; negative is south.
float64 latitude

# Longitude [degrees]. Positive is east of prime meridian; negative is west.
float64 longitude

# Altitude [m]. Positive is above the WGS 84 ellipsoid
# (quiet NaN if no altitude is available).
float64 altitude

# Position covariance [m^2] defined relative to a tangential plane
# through the reported position. The components are East, North, and
# Up (ENU), in row-major order.
#
# Beware: this coordinate system exhibits singularities at the poles.

float64[9] position_covariance

# If the covariance of the fix is known, fill it in completely. If the
# GPS receiver provides the variance of each measurement, put them
# along the diagonal. If only Dilution of Precision is available,
# estimate an approximate covariance from that.

uint8 COVARIANCE_TYPE_UNKNOWN = 0
uint8 COVARIANCE_TYPE_APPROXIMATED = 1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
uint8 COVARIANCE_TYPE_KNOWN = 3

uint8 position_covariance_type

# Two dimentional accuracy of the GPS receiver in meters.
float64 accuracy_2d
# Three dimentional accuracy of the GPS receiver in meters.
float64 accuracy_3d
# heading of the GPS receiver acting as rover relative to base in degrees.
float64 heading
# length of the GPS receiver acting as rover relative to base in meters.
float64 length
# quality string of the antenna.
string quality
# True if augmentations were used.(RTCM corrections if moving_baseline_configuration / SPARTN messages if in disabled configuration)
bool augmentations_used
# True if fix is valid. False otherwise
bool valid_fix

uint16 no_of_satellites

NavSatInfo[] satellite_information