# System settings
OPERATION_FREQ = 0.1  # Delay between polling cycles in seconds (affects overall polling rate)
SEGMENT_LENGTH_CM = [100, 100, 100, 100, 100, 100, 100, 100, 100, 100]  # Lengths of each segment in cm
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

# RS485 settings for IMU slaves
RS485_DEV = "/dev/ttyUSB0"
RS485_BAUD = 9600
RS485_TIMEOUT = 1
# Packet structure: SOH(1) + receiverID(1) + senderID(1) + length(1) + STX(1) + payload(N) + checksum(1) + ETX(1) + EOT(1)
# For IMU packets: payload = 29 bytes → total packet = 37 bytes
# For POLL commands: payload = 4 bytes → total packet = 12 bytes
RS485_DATA_LENGTH = 37  # Maximum expected packet size (for IMU data)

# Master polling settings
MASTER_ID = 0xFF  # Master node ID (255 = broadcast address)
SLAVE_IDS = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]  # List of slave node IDs to poll
# POLL_TIMEOUT = 0.5  # Timeout for slave response in seconds (100ms)

# RS485 settings for Weather sensor (separate port)
RS485_WEATHER_DEV = "/dev/ttyUSB1"  # Separate RS485 port for weather sensor
RS485_WEATHER_BAUD = 9600
RS485_WEATHER_TIMEOUT = 1
RS485_WEATHER_SENSOR_ADDR = 1  # Modbus address for temp/humidity sensor

# Rain gauge settings (I2C)
RAIN_SENSOR_I2C_BUS = 1        # I2C bus number
RAIN_SENSOR_I2C_ADDR = 0x1D    # I2C address for rain sensor

# Server settings
SERVER_URL = "Put your server URL here."
SERVER_PORT = 7072