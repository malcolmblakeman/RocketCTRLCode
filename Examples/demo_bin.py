import struct
import csv
import matplotlib.pyplot as plt

# =========================================================
# CONFIG
# =========================================================
FLASH_DUMP_FILE = "/content/rocket_flash_dump.bin"
CSV_OUTPUT_FILE = "output.csv"

PACKET_SIZE = 32

# Packet layout:
#
# 0-1   ID (uint16)
# 2     STATE
# 3     CONT
# 4-25  11 int16 values
# 26-29 TIMESTAMP uint32
# 30    FLAGS
# 31    CRC8

STRUCT_FORMAT = "<HBB11hIBB"

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

            # Check packet ID
            # LSB first:
            # 0xDC 0xAC -> 0xACDC
            packet_id = struct.unpack("<H", packet[0:2])[0]

            if packet_id != 0xACDC:
                continue

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
            "id",
            "state",
            "cont",

            "velocity",
            "altitude",

            "lg_x",
            "lg_y",
            "lg_z",

            "hg_x",
            "hg_y",
            "hg_z",

            "gyro_pitch",
            "gyro_roll",
            "gyro_yaw",

            "timestamp",

            "flags",
            "crc8"
        ])

        for p in packets:

            packet_id = p[0]

            state = p[1]
            cont = p[2]

            velocity = p[3]
            altitude = p[4]

            lg_x = p[5]
            lg_y = p[6]
            lg_z = p[7]

            hg_x = p[8]
            hg_y = p[9]
            hg_z = p[10]

            gyro_pitch = p[11]
            gyro_roll = p[12]
            gyro_yaw = p[13]

            timestamp = p[14]

            flags = p[15]
            crc8 = p[16]

            writer.writerow([
                hex(packet_id),
                state,
                cont,

                velocity,
                altitude,

                lg_x,
                lg_y,
                lg_z,

                hg_x,
                hg_y,
                hg_z,

                gyro_pitch,
                gyro_roll,
                gyro_yaw,

                timestamp,

                flags,
                crc8
            ])

    print(f"CSV saved to: {CSV_OUTPUT_FILE}")


# =========================================================
# PLOT
# =========================================================
def plot_packets(csv_file):

    timestamps = []

    velocity = []
    altitude = []

    lg_x = []
    lg_y = []
    lg_z = []

    hg_x = []
    hg_y = []
    hg_z = []

    gyro_pitch = []
    gyro_roll = []
    gyro_yaw = []

    # ============================================
    # Read CSV
    # ============================================
    with open(csv_file, "r") as f:

        reader = csv.DictReader(f)

        for row in reader:

            timestamps.append(float(row["timestamp"]) / 1000.0)

            velocity.append(float(row["velocity"]))
            altitude.append(float(row["altitude"]))

            lg_x.append(float(row["lg_x"]))
            lg_y.append(float(row["lg_y"]))
            lg_z.append(float(row["lg_z"]))

            hg_x.append(float(row["hg_x"]))
            hg_y.append(float(row["hg_y"]))
            hg_z.append(float(row["hg_z"]))

            gyro_pitch.append(float(row["gyro_pitch"]))
            gyro_roll.append(float(row["gyro_roll"]))
            gyro_yaw.append(float(row["gyro_yaw"]))

    # ============================================
    # Create plots
    # ============================================
    fig, axes = plt.subplots(4, 1, figsize=(12, 12))

    # ============================================
    # Velocity / Altitude
    # ============================================
    axes[0].plot(timestamps, velocity, label="Velocity")
    axes[0].plot(timestamps, altitude, label="Altitude")

    axes[0].set_title("Velocity / Altitude")
    axes[0].grid(True)
    axes[0].legend()

    # ============================================
    # Low-G Accelerometer
    # ============================================
    axes[1].plot(timestamps, lg_x, label="LG X")
    axes[1].plot(timestamps, lg_y, label="LG Y")
    axes[1].plot(timestamps, lg_z, label="LG Z")

    axes[1].set_title("Low-G Accelerometer")
    axes[1].grid(True)
    axes[1].legend()

    # ============================================
    # High-G Accelerometer
    # ============================================
    axes[2].plot(timestamps, hg_x, label="HG X")
    axes[2].plot(timestamps, hg_y, label="HG Y")
    axes[2].plot(timestamps, hg_z, label="HG Z")

    axes[2].set_title("High-G Accelerometer")
    axes[2].grid(True)
    axes[2].legend()

    # ============================================
    # Gyroscope
    # ============================================
    axes[3].plot(timestamps, gyro_pitch, label="Pitch")
    axes[3].plot(timestamps, gyro_roll, label="Roll")
    axes[3].plot(timestamps, gyro_yaw, label="Yaw")

    axes[3].set_title("Gyroscope")
    axes[3].set_xlabel("Time (s)")
    axes[3].grid(True)
    axes[3].legend()

    plt.tight_layout()
    plt.show()


# =========================================================
# MAIN
# =========================================================
packets = parse_binary_file()

print(f"Parsed {len(packets)} packets")

save_csv(packets)

plot_packets(CSV_OUTPUT_FILE)
