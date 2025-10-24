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

        # Synchronize to SOH (0x01) byte and read variable-length packets
        data = None
        while ser.in_waiting > 0:
            byte = ser.read(1)

            if len(byte) == 0:
                break

            if byte[0] == 0x01:  # Found potential SOH
                # Read header: receiverID, senderID, length (3 bytes)
                header = ser.read(3)

                if len(header) != 3:
                    logger.debug("Incomplete header after SOH. Continuing sync...")
                    continue

                receiver_id = header[0]
                sender_id = header[1]
                payload_length = header[2]

                # Calculate remaining bytes: STX(1) + payload(N) + checksum(1) + ETX(1) + EOT(1)
                remaining_bytes = 1 + payload_length + 1 + 1 + 1  # = payload_length + 4

                # Read the rest of the packet
                remaining = ser.read(remaining_bytes)

                if len(remaining) != remaining_bytes:
                    logger.debug(f"Incomplete packet. Expected {remaining_bytes} bytes, got {len(remaining)}. Continuing sync...")
                    continue

                # Reconstruct full packet
                data = byte + header + remaining

                # Calculate indices for validation
                stx_index = 4
                etx_index = 4 + 1 + payload_length + 1  # After STX + payload + checksum
                eot_index = etx_index + 1

                # Validate STX, ETX, EOT
                if data[stx_index] == 0x02 and data[etx_index] == 0x03 and data[eot_index] == 0x04:
                    logger.debug(f"Valid packet found: RxID={receiver_id:02x}, TxID={sender_id}, Length={payload_length}")
                    break  # Valid packet structure found
                else:
                    # False SOH - continue searching
                    logger.debug(f"False SOH detected (STX={data[stx_index]:02x}, ETX={data[etx_index]:02x}, EOT={data[eot_index]:02x}). Continuing sync...")
                    data = None
                    continue

        if data is None:
            return None, None, None

        # RS485 Protocol Structure (variable length):
        # Header (4 bytes):
        #   Byte 0: SOH (0x01)
        #   Byte 1: receiverID
        #   Byte 2: senderID (NODE_ID)
        #   Byte 3: payload_length
        # Body:
        #   Byte 4: STX (0x02)
        #   Bytes 5 to (5+payload_length-1): Payload
        #   Byte (5+payload_length): Checksum (XOR of payload)
        #   Byte (5+payload_length+1): ETX (0x03)
        #   Byte (5+payload_length+2): EOT (0x04)

        # Extract packet info
        receiver_id = data[1]
        sender_id = data[2]
        payload_length = data[3]

        # Only process IMU data packets (29 bytes payload)
        # Skip POLL commands (4 bytes) and other packet types
        if payload_length != 29:
            logger.debug(f"Skipping non-IMU packet from node {sender_id} (length={payload_length})")
            return None, None, None

        # Debug: Print raw bytes (only for IMU packets)
        logger.debug(f"Raw data ({len(data)} bytes): {data.hex()}")
        logger.debug(f"Header bytes: SOH={data[0]:02x} RxID={receiver_id:02x} TxID={sender_id} Len={payload_length} STX={data[4]:02x}")

        # Extract payload
        payload_start = 5
        payload_end = payload_start + payload_length
        payload = data[payload_start:payload_end]

        # Verify checksum
        checksum = data[payload_end]
        calculated_checksum = 0
        for byte in payload:
            calculated_checksum ^= byte

        if calculated_checksum != checksum:
            logger.warning(f"Checksum mismatch! Expected {checksum:02x}, got {calculated_checksum:02x}. Discarding packet.")
            return None, None, None

        logger.debug(f"Footer bytes: CHKSUM={checksum:02x} ETX={data[payload_end+1]:02x} EOT={data[payload_end+2]:02x}")

        # Extract node_id from sender
        node_id = sender_id

        # IMU Payload structure (29 bytes):
        #   Byte 0: NODE_ID (redundant)
        #   Bytes 1-16: Quaternion w,x,y,z (4 floats = 16 bytes)
        #   Bytes 17-28: Accel ax,ay,az (3 floats = 12 bytes)

        # Extract quaternion and acceleration (skip redundant NODE_ID at byte 0)
        float_data = payload[1:29]  # 28 bytes = 7 floats (quat + accel)
        w, x, y, z, ax, ay, az = struct.unpack("fffffff", float_data)

        logger.debug(f"Node {node_id}: Quat({w:.3f},{x:.3f},{y:.3f},{z:.3f}) Accel({ax:.3f},{ay:.3f},{az:.3f})")

        return node_id, np.array([w, x, y, z]), np.array([ax, ay, az])

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
