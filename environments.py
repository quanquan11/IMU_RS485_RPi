# System settings
OPERATION_FREQ = 0.1
SEGMENT_LENGTH_CM = [3, 6]
INVERT_DIRECTION = False
STATION_ID = "903056a7-e59a-42b6-b644-baa6a45de45a"
IOT_STATION_KEY = "LSA_RPi"

# Kalman filter settings
ERR_COVAR_MATRIX = 1.0           # Initial uncertainty
MEASURE_NOISE_MATRIX = 0.01      # Measurement noise (adjust as needed)
PROC_NOISE_COVAR_MATRIX = 0.001  # Process noise (adjust for smoothness)

# Weather API settings
OPENMETEO_URL = "https://api.open-meteo.com/v1/forecast"
OPENMETEO_LATITUDE = 3.0466
OPENMETEO_LONGITUDE = 101.72955
OPENMETEO_LIMIT = 900

# RS485 settings
RS485_DEV = "/dev/ttyUSB0"
RS485_BAUD = 9600
RS485_TIMEOUT = 1
RS485_DATA_LENGTH = 37  # Protocol overhead (excluding SOH): receiverID + senderID + length + STX + payload + checksum + ETX + EOT

# Server settings
SERVER_URL = "Put your server URL here."
SERVER_PORT = 7072