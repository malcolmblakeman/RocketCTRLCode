import os
import re
import time
from datetime import datetime

import serial
import serial.tools.list_ports
from openpyxl import Workbook, load_workbook

import matplotlib.pyplot as plt


# =========================
# User settings
# =========================
BAUD_RATE = 9600
TIMEOUT = 1
EXCEL_FILE = r"C:\Users\joshu\Documents\vsCode\rocket_ground_board_communication\usb_serial_log.xlsx"
SHEET_NAME = "Data"
RECONNECT_DELAY = 2
LOG_DURATION_SECONDS = 300


def is_bluetooth_port(port):
    text = f"{port.device} {port.description} {port.manufacturer} {port.hwid}".lower()
    bluetooth_keywords = ["bluetooth", "bth", "rfcomm"]
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
        print(f"  Device       : {port.device}")
        print(f"  Description  : {port.description}")
        print(f"  Manufacturer : {port.manufacturer}")
        print(f"  HWID         : {port.hwid}")
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


def create_or_load_workbook(filename, sheet_name):
    if os.path.exists(filename):
        wb = load_workbook(filename)
        if sheet_name in wb.sheetnames:
            ws = wb[sheet_name]
        else:
            ws = wb.create_sheet(sheet_name)
            ws.append(["Timestamp", "Raw Data"])
    else:
        wb = Workbook()
        ws = wb.active
        ws.title = sheet_name
        ws.append(["Timestamp", "Raw Data"])

    return wb, ws


def clean_excel_string(value):
    if value is None:
        return ""

    value = str(value)
    value = re.sub(r"[\x00-\x08\x0B-\x0C\x0E-\x1F]", "", value)
    return value.strip()


def write_row(ws, line):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    clean_line = clean_excel_string(line)

    if not clean_line:
        return

    if "," in clean_line:
        parts = [clean_excel_string(part) for part in clean_line.split(",")]
        ws.append([timestamp] + parts)
    else:
        ws.append([timestamp, clean_line])


def connect_serial():
    port_name = find_usb_serial_port()

    if port_name is None:
        return None

    try:
        print(f"Connecting to {port_name} at {BAUD_RATE} baud...")
        ser = serial.Serial(port_name, BAUD_RATE, timeout=TIMEOUT)
        time.sleep(2)
        print(f"Connected to {port_name}")
        return ser
    except serial.SerialException as e:
        print(f"Could not open {port_name}: {e}")
        return None


def ascii_positions_to_decimal(values, array_pos_1, array_pos_2):
    value_1 = values[array_pos_1]
    value_2 = values[array_pos_2]

    bin_1 = format(value_1, '08b')
    bin_2 = format(value_2, '08b')

    combined_binary = bin_2 + bin_1
    unsigned_value = int(combined_binary, 2)

    if combined_binary[0] == '1':
        signed_value = unsigned_value - (1 << 16)
    else:
        signed_value = unsigned_value

    return signed_value


def initialize_plot():
    """
    Create a live-updating window with 2 stacked plots:
    1. Gyro (Pitch, Roll, Yaw)
    2. Acceleration (X, Y, Z)
    """
    plt.ion()

    fig, (ax_gyro, ax_accel) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    line_pitch, = ax_gyro.plot([], [], label="Pitch")
    line_roll, = ax_gyro.plot([], [], label="Roll")
    line_yaw, = ax_gyro.plot([], [], label="Yaw")

    line_accel_x, = ax_accel.plot([], [], label="Accel X")
    line_accel_y, = ax_accel.plot([], [], label="Accel Y")
    line_accel_z, = ax_accel.plot([], [], label="Accel Z")

    ax_gyro.set_title("Gyro")
    ax_gyro.set_ylabel("degrees/sec")
    ax_gyro.grid(True)
    ax_gyro.legend()
    ax_gyro.tick_params(axis='y', labelleft=False)

    ax_accel.set_title("Acceleration")
    ax_accel.set_xlabel("Time (s)")
    ax_accel.set_ylabel("meters/sec")
    ax_accel.grid(True)
    ax_accel.legend()
    ax_accel.tick_params(axis='y', labelleft=False)

    plt.tight_layout()

    return (
        fig,
        ax_gyro,
        ax_accel,
        line_pitch,
        line_roll,
        line_yaw,
        line_accel_x,
        line_accel_y,
        line_accel_z
    )


def update_plot(
    ax_gyro,
    ax_accel,
    line_pitch,
    line_roll,
    line_yaw,
    line_accel_x,
    line_accel_y,
    line_accel_z,
    time_data,
    pitch_data,
    roll_data,
    yaw_data,
    accel_x_data,
    accel_y_data,
    accel_z_data
):
    line_pitch.set_xdata(time_data)
    line_pitch.set_ydata(pitch_data)

    line_roll.set_xdata(time_data)
    line_roll.set_ydata(roll_data)

    line_yaw.set_xdata(time_data)
    line_yaw.set_ydata(yaw_data)

    line_accel_x.set_xdata(time_data)
    line_accel_x.set_ydata(accel_x_data)

    line_accel_y.set_xdata(time_data)
    line_accel_y.set_ydata(accel_y_data)

    line_accel_z.set_xdata(time_data)
    line_accel_z.set_ydata(accel_z_data)

    ax_gyro.relim()
    ax_gyro.autoscale_view()

    ax_accel.relim()
    ax_accel.autoscale_view()

    plt.draw()
    plt.pause(0.01)


def main():
    print("Starting USB serial logger...")
    print(f"Excel file will be saved to:\n{os.path.abspath(EXCEL_FILE)}\n")

    wb, ws = create_or_load_workbook(EXCEL_FILE, SHEET_NAME)
    ser = None

    data_array = []

    plot_times = []
    plot_pitch = []
    plot_roll = []
    plot_yaw = []
    plot_accel_x = []
    plot_accel_y = []
    plot_accel_z = []

    fig, ax_gyro, ax_accel, line_pitch, line_roll, line_yaw, line_accel_x, line_accel_y, line_accel_z = initialize_plot()

    try:
        start_time = time.time()

        while time.time() - start_time < LOG_DURATION_SECONDS:
            if ser is None or not ser.is_open:
                ser = connect_serial()
                if ser is None:
                    print(f"Retrying in {RECONNECT_DELAY} seconds...\n")
                    time.sleep(RECONNECT_DELAY)
                    continue

            try:
                cur_time = time.time()
                line = ser.readline()

                if not line:
                    continue

                decoded_line = line.decode("utf-8", errors="replace")
                print(f"Received raw: {repr(decoded_line)}")

                cleaned_line = clean_excel_string(decoded_line)

                if cleaned_line:
                    values = [int(x.strip()) for x in cleaned_line.split(",")]
                    data_array.append(values)

                    gyro_pitch = ascii_positions_to_decimal(values, 18, 19)
                    gyro_roll = ascii_positions_to_decimal(values, 20, 21)
                    gyro_yaw = ascii_positions_to_decimal(values, 22, 23)
                    acceleration_x = ascii_positions_to_decimal(values, 6, 7)
                    acceleration_y = ascii_positions_to_decimal(values, 8, 9)
                    acceleration_z = ascii_positions_to_decimal(values, 10, 11)

                    print(f"Pitch: {gyro_pitch}")
                    print(f"Roll: {gyro_roll}")
                    print(f"Yaw: {gyro_yaw}")
                    print(f"Acceleration X: {acceleration_x}")
                    print(f"Acceleration Y: {acceleration_y}")
                    print(f"Acceleration Z: {acceleration_z}")

                    try:
                        elapsed_time = time.time() - start_time
                        

                        plot_times.append(elapsed_time)
                        plot_pitch.append(int(gyro_pitch))
                        plot_roll.append(int(gyro_roll))
                        plot_yaw.append(int(gyro_yaw))
                        plot_accel_x.append(int(acceleration_x))
                        plot_accel_y.append(int(acceleration_y))
                        plot_accel_z.append(int(acceleration_z))

                        update_plot(
                            ax_gyro,
                            ax_accel,
                            line_pitch,
                            line_roll,
                            line_yaw,
                            line_accel_x,
                            line_accel_y,
                            line_accel_z,
                            plot_times,
                            plot_pitch,
                            plot_roll,
                            plot_yaw,
                            plot_accel_x,
                            plot_accel_y,
                            plot_accel_z
                        )

                    except ValueError as e:
                        print(f"Could not convert one of the values to integer: {e}")

                    write_row(ws, cleaned_line)
                    wb.save(EXCEL_FILE)
                    print(time.time() - cur_time)

            except serial.SerialException as e:
                print(f"Serial error: {e}")
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                print(f"Disconnected. Retrying in {RECONNECT_DELAY} seconds...\n")
                time.sleep(RECONNECT_DELAY)

            except Exception as e:
                print(f"Unexpected error while reading/writing: {e}")

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        try:
            if ser is not None and ser.is_open:
                ser.close()
        except Exception:
            pass

        wb.save(EXCEL_FILE)
        print(f"Final save complete: {os.path.abspath(EXCEL_FILE)}")

        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main()
