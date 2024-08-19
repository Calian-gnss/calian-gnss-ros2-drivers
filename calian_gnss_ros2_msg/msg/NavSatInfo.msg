uint8 GPS = 0
uint8 SBAS = 1
uint8 GALILEO = 2
uint8 BEIDOU = 3
uint8 IMES = 4
uint8 QZSS = 5
uint8 GLONASS = 6

# The range should be between 1 and 7. Please provide the GNSS IDs of the satellites, which are defined as constants above
uint8 gnss_id
uint8 count