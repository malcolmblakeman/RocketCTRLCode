import os
import time
import struct
from datetime import datetime

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import csv


# =========================================================
# USER SETTINGS
# =========================================================
BAUD_RATE = 115200
TIMEOUT = 1

FLASH_DUMP_FILE = "flash_dump.bin"
CSV_OUTPUT_FILE = "telemetry_output.csv"

RECONNECT_DELAY = 2

PACKET_SIZE = 16
START_MARKER = b'\xAA\x55'
END_MESSAGE = b"FLASH_DUMP_COMPLETE"

# Packet format:
# int16 accel_x
# int16 accel_y
# int16 accel_z
# int16 gyro_pitch
# int16 gyro_roll
# int16 gyro_yaw
# uint32 timestamp
#
# total = 16 bytes
#
# little endian
STRUCT_FORMAT = "<hhhhhhI"


# =========================================================
# PORT DETECTION
# =========================================================
def is_bluetooth_port(port):
    text = f"{port.device} {port.description} {port.manufacturer} {port.hwid}".lower()

    bluetooth_keywords = [
        "bluetooth",
        "bth",
        "rfcomm"
    ]

    return any(word in text for word in bluetooth_keywords)


def is_preferred_usb_serial_port(port):
    text = f"{port.device} {port.description} {port.manufacturer} {port.hwid}".lower()

    preferred_keywords = [
        "usb serial device",
        "usb serial",
        "usb",
        "ch340",
        "cp210",
        "ftdi",
        "arduino",
        "cdc",
        "uart",
        "silicon labs",
        "wch",
    ]

    return any(word in text for word in preferred_keywords)


def find_usb_serial_port():
    ports = list(serial.tools.list_ports.comports())

    if not ports:
        print("No COM ports found.")
        return None

    print("\nDetected COM ports:")

    for port in ports:
        print(f"Device       : {port.device}")
        print(f"Description  : {port.description}")
        print(f"Manufacturer : {port.manufacturer}")
        print(f"HWID         : {port.hwid}")
        print()

    for port in ports:

        if is_bluetooth_port(port):
            print(f"Skipping Bluetooth port: {port.device}")
            continue

        if is_preferred_usb_serial_port(port):
            print(f"Selected USB serial port: {port.device}")
            return port.device

    print("No suitable USB serial port found.")
    return None


# =========================================================
# SERIAL CONNECTION
# =========================================================
def connect_serial():

    while True:

        port_name = find_usb_serial_port()

        if port_name is None:
            print(f"Retrying in {RECONNECT_DELAY} seconds...")
            time.sleep(RECONNECT_DELAY)
            continue

        try:
            print(f"Connecting to {port_name} at {BAUD_RATE} baud...")
            ser = serial.Serial(port_name, BAUD_RATE, timeout=TIMEOUT)

            time.sleep(2)

            print(f"Connected to {port_name}")

            return ser

        except serial.SerialException as e:
            print(f"Connection failed: {e}")
            time.sleep(RECONNECT_DELAY)


# =========================================================
# RECEIVE FLASH DUMP
# =========================================================
def receive_flash_dump(ser):

    print("\nWaiting for flash dump...")

    total_bytes = 0
    start_time = time.time()

    with open(FLASH_DUMP_FILE, "wb") as f:

        while True:

            try:
                data = ser.read(1024)

                if not data:
                    continue

                if END_MESSAGE in data:

                    idx = data.find(END_MESSAGE)

                    f.write(data[:idx])

                    print("\nReceived FLASH_DUMP_COMPLETE")

                    break

                f.write(data)

                total_bytes += len(data)

                elapsed = time.time() - start_time

                rate = total_bytes / elapsed if elapsed > 0 else 0

                print(
                    f"\rReceived: {total_bytes} bytes | "
                    f"{rate:.1f} bytes/sec",
                    end=""
                )

            except serial.SerialException as e:
                print(f"\nSerial error: {e}")
                break

    print(f"\nSaved raw dump to: {FLASH_DUMP_FILE}")


# =========================================================
# PARSE BINARY FILE
# =========================================================
def parse_binary_file():

    packets = []

    with open(FLASH_DUMP_FILE, "rb") as f:

        while True:

            packet = f.read(PACKET_SIZE)

            if len(packet) != PACKET_SIZE:
                break

            try:

                unpacked = struct.unpack(STRUCT_FORMAT, packet)

                packets.append(unpacked)

            except struct.error:
                continue

    return packets


# =========================================================
# SAVE CSV
# =========================================================
def save_csv(packets):

    with open(CSV_OUTPUT_FILE, "w", newline="") as f:

        writer = csv.writer(f)

        writer.writerow([
            "timestamp_ms",
            "accel_x",
            "accel_y",
            "accel_z",
            "gyro_pitch",
            "gyro_roll",
            "gyro_yaw"
        ])

        for p in packets:

            accel_x = p[0]
            accel_y = p[1]
            accel_z = p[2]

            gyro_pitch = p[3]
            gyro_roll = p[4]
            gyro_yaw = p[5]

            timestamp = p[6]

            writer.writerow([
                timestamp,
                accel_x,
                accel_y,
                accel_z,
                gyro_pitch,
                gyro_roll,
                gyro_yaw
            ])

    print(f"CSV saved to: {CSV_OUTPUT_FILE}")


# =========================================================
# PLOT TELEMETRY
# =========================================================
def plot_packets(packets):

    if len(packets) == 0:
        print("No packets to plot.")
        return

    timestamps = []
    accel_x = []
    accel_y = []
    accel_z = []

    gyro_pitch = []
    gyro_roll = []
    gyro_yaw = []

    for p in packets:

        timestamps.append(p[6] / 1000.0)

        accel_x.append(p[0])
        accel_y.append(p[1])
        accel_z.append(p[2])

        gyro_pitch.append(p[3])
        gyro_roll.append(p[4])
        gyro_yaw.append(p[5])

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

    # ============================================
    # Gyro
    # ============================================
    ax1.plot(timestamps, gyro_pitch, label="Pitch")
    ax1.plot(timestamps, gyro_roll, label="Roll")
    ax1.plot(timestamps, gyro_yaw, label="Yaw")

    ax1.set_title("Gyroscope")

    ax1.set_ylabel("deg/sec")

    ax1.grid(True)

    ax1.legend()

    # ============================================
    # Acceleration
    # ============================================
    ax2.plot(timestamps, accel_x, label="Accel X")
    ax2.plot(timestamps, accel_y, label="Accel Y")
    ax2.plot(timestamps, accel_z, label="Accel Z")

    ax2.set_title("Acceleration")

    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Acceleration")

    ax2.grid(True)

    ax2.legend()

    plt.tight_layout()

    plt.show()


# =========================================================
# MAIN
# =========================================================
def main():

    print("========================================")
    print("FLASH MEMORY TELEMETRY DOWNLOADER")
    print("========================================")

    ser = connect_serial()

    # ============================================
    # Tell rocket to begin dump
    # ============================================
    print("\nSending DUMP_FLASH command...")

    ser.write(b"DUMP_FLASH\n")

    # ============================================
    # Receive raw binary dump
    # ============================================
    receive_flash_dump(ser)

    ser.close()

    # ============================================
    # Parse packets
    # ============================================
    print("\nParsing packets...")

    packets = parse_binary_file()

    print(f"Parsed {len(packets)} packets")

    # ============================================
    # Save CSV
    # ============================================
    save_csv(packets)

    # ============================================
    # Plot
    # ============================================
    plot_packets(packets)

    print("\nDone.")


# =========================================================
# ENTRY
# =========================================================
if __name__ == "__main__":
    main()
