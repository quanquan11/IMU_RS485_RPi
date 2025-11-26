import minimalmodbus
from logger import logger
from environments import RS485_WEATHER_DEV, RS485_WEATHER_BAUD, RS485_WEATHER_TIMEOUT, RS485_WEATHER_SENSOR_ADDR, RAIN_SENSOR_I2C_BUS
from DFRobot_RainfallSensor import DFRobot_RainfallSensor_I2C

class WeatherSensor:
    """Class to manage weather sensor readings (temperature, humidity, rainfall)."""

    def __init__(self):
        self.instrument = None
        self.rain_sensor = None
        self.initialized = False

    def initialize(self):
        """Initialize the weather sensors."""
        modbus_success = False

        # Initialize RS485 Modbus instrument for temp/humidity
        try:
            if self.instrument is None:
                self.instrument = minimalmodbus.Instrument(RS485_WEATHER_DEV, RS485_WEATHER_SENSOR_ADDR)
                self.instrument.serial.baudrate = RS485_WEATHER_BAUD
                self.instrument.serial.timeout = RS485_WEATHER_TIMEOUT
                logger.info(f"Weather sensor (Modbus) initialized on {RS485_WEATHER_DEV} at address {RS485_WEATHER_SENSOR_ADDR}")
                modbus_success = True
        except Exception as err:
            logger.error(f"Failed to initialize Modbus weather sensor: {err}")
            modbus_success = False

        # Initialize I2C rainfall sensor (optional - don't fail if this doesn't work)
        try:
            if self.rain_sensor is None:
                self.rain_sensor = DFRobot_RainfallSensor_I2C(bus=RAIN_SENSOR_I2C_BUS)
                if not self.rain_sensor.begin():
                    logger.warning("Rain sensor initialization failed! Will continue without rain data.")
                    self.rain_sensor = None
                else:
                    logger.info("Rain sensor initialized successfully")
        except Exception as err:
            logger.warning(f"Rain sensor error (will continue without rain data): {err}")
            self.rain_sensor = None

        # Consider initialized if at least Modbus sensor works
        self.initialized = modbus_success
        return modbus_success

    def read_data(self):
        """
        Read temperature, humidity, and rainfall data.
        Returns: dict with keys 'temperature', 'humidity', 'rain' or None if read fails
        """
        if not self.initialized:
            logger.warning("Weather sensors not initialized. Attempting to initialize...")
            if not self.initialize():
                return None

        try:
            # Read temperature (register 1) and humidity (register 2)
            temp = self.instrument.read_register(1, 1, functioncode=4)
            humid = self.instrument.read_register(2, 1, functioncode=4)

            # Read rainfall data
            rain_qty = 0.0
            if self.rain_sensor is not None:
                try:
                    rain_qty = self.rain_sensor.get_rainfall()
                except Exception as rain_err:
                    logger.warning(f"Failed to read rain sensor: {rain_err}")
                    rain_qty = 0.0

            logger.debug(f"Weather data: Temp={temp}°C, Humidity={humid}%, Rain={rain_qty:.2f}mm")

            return {
                "temperature": float(temp),
                "humidity": float(humid),
                "rain": float(rain_qty)
            }

        except Exception as err:
            logger.error(f"Failed to read weather sensors: {err}")
            return None

    def close(self):
        """Clean up resources."""
        if self.instrument is not None and hasattr(self.instrument, 'serial'):
            try:
                self.instrument.serial.close()
                logger.info("Weather sensor serial port closed")
            except:
                pass
        self.initialized = False
