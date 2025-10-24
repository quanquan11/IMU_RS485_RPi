import time, serial, struct
import numpy as np

from logger import logger
from restapi import stream_to_server, update_weather_data
from helpers import create_kalman, apply_kalman_filter, quaternion_to_roll, calculate_displacement

from environments import RS485_DATA_LENGTH, STATION_ID, OPERATION_FREQ, RS485_DEV, RS485_BAUD, RS485_TIMEOUT

ser = None 

def read_rs485():
    """Read BNO055 IMU data sent through RS485."""
    global ser

    try:
        if ser is None:
            ser = serial.Serial(port=RS485_DEV,
                                   baudrate=RS485_BAUD,
                                   timeout=RS485_TIMEOUT)
            logger.info(f"Serial port opened: {RS485_DEV} at {RS485_BAUD} baud")

        # Synchronize to SOH (0x01) byte with STX/ETX validation
        while ser.in_waiting > 0:
            byte = ser.read(1)

            if byte[0] == 0x01:  # Found potential SOH
                # Read the rest of the packet (35 more bytes)
                remaining = ser.read(RS485_DATA_LENGTH - 1)

                if len(remaining) == RS485_DATA_LENGTH - 1:
                    data = byte + remaining

                    # Validate STX (byte 4) and ETX (byte 35) to confirm real packet boundary
                    if data[4] == 0x02 and data[35] == 0x03:
                        break  # Valid packet structure found
                    else:
                        # False SOH - continue searching
                        logger.debug(f"False SOH detected (STX={data[4]:02x}, ETX={data[35]:02x}). Continuing sync...")
                        continue
        else:
            return None, None, None

        if len(data) == RS485_DATA_LENGTH:
                # RS485 Protocol Structure (36 bytes total):
                # Header (5 bytes):
                #   Byte 0: SOH (0x01)
                #   Byte 1: receiverID
                #   Byte 2: senderID (NODE_ID)
                #   Byte 3: length (29)
                #   Byte 4: STX (0x02)
                # Payload (29 bytes):
                #   Byte 5: NODE_ID (redundant)
                #   Bytes 6-21: Quaternion w,x,y,z (4 floats = 16 bytes)
                #   Bytes 22-33: Accel ax,ay,az (3 floats = 12 bytes)
                # Footer (2 bytes):
                #   Byte 34: Checksum (XOR of payload bytes 5-33)
                #   Byte 35: ETX (0x03)
                #   Byte 36: EOT (not used)

                # Debug: Print raw bytes
                logger.debug(f"Raw data ({len(data)} bytes): {data.hex()}")
                logger.debug(f"Header bytes: SOH={data[0]:02x} RxID={data[1]:02x} TxID={data[2]:02x} Len={data[3]:02x} STX={data[4]:02x}")
                logger.debug(f"Footer bytes: CHKSUM={data[34]:02x} ETX={data[35]:02x}")

                # Extract node_id from protocol header (senderID)
                node_id = data[2]

                # Verify checksum
                payload = data[5:34]  # Payload (29 bytes)
                checksum = data[34]
                calculated_checksum = 0
                for byte in payload:
                    calculated_checksum ^= byte

                if calculated_checksum != checksum:
                    logger.warning(f"Checksum mismatch! Expected {checksum}, got {calculated_checksum}. Discarding packet.")
                    return None, None, None

                # Extract quaternion and acceleration (skip redundant NODE_ID at byte 5)
                float_data = data[6:34]  # 28 bytes = 7 floats (quat + accel)
                w, x, y, z, ax, ay, az = struct.unpack("fffffff", float_data)

                logger.debug(f"Node {node_id}: Quat({w:.3f},{x:.3f},{y:.3f},{z:.3f}) Accel({ax:.3f},{ay:.3f},{az:.3f})")

                return node_id, np.array([w, x, y, z]), np.array([ax, ay, az])

        return None, None, None

    except serial.SerialException as err:
        logger.error("Error related to serial ports: {0}".format(err))
        exit(1)

def main():
    """Main code is written here."""

    kalman = create_kalman()

    while True:
        sensor_id, quat, accel = read_rs485()

        if sensor_id is not None:
            logger.info("Received Data: From node ID {0}, QuatW: {1}, QuatX: {2}, QuatY: {3}, QuatZ: {4}.".format(sensor_id,
                                                                                                                  quat[0],
                                                                                                                  quat[1],
                                                                                                                  quat[2],
                                                                                                                  quat[3]))

            try:
                quat_filtered = apply_kalman_filter(kalman, quat)
                roll = quaternion_to_roll(quat_filtered)
                disp = calculate_displacement(roll, sensor_id, True)
                # weather_data = update_weather_data()

                data = {
                    "profile_id": "",
                    "station_id": STATION_ID,
                    "sensor_id": sensor_id,
                    "displacement": disp,
                    "accel_x": accel[0],
                    "accel_y": accel[1],
                    "accel_z": accel[2],
                    "temperature": 32.12,
                    "humidity": 98.7,
                    "wind_speed": 2.5,
                    "wind_direction": 180.0
                }

                # stream_to_server(data = data)
            except ValueError as e:
                logger.error(f"Error processing data from node {sensor_id}: {e}. Skipping packet.")
        else:
            logger.info("No data received.")

        time.sleep(OPERATION_FREQ)

if __name__ == "__main__":
    main()
