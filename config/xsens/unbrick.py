import time
import serial

# Unbrick Script for XSens MTi-630(R) used to restore device if it is stuck in a non-responsive state. 
# This script sends specific XBUS commands to the device to force it into configuration mode, set the output configuration, 
# and then return it to measurement mode.

# --- PARAMETERS FROM YOUR CONFIG ---
PORT = '/dev/xsens'
BAUDRATE = 460800
TIMEOUT = 0.550  # 550ms converted to seconds

# --- RAW XBUS COMMANDS ---
GOTO_CONFIG = bytes.fromhex("FA FF 30 00 D1")
GOTO_MEASUREMENT = bytes.fromhex("FA FF 10 00 F1")

# Command 5.3: Set Output Configuration for MTi-630(R) @ 400Hz
SET_OUTPUT_400HZ = bytes.fromhex(
    "FA FF C0 24 10 20 FF FF 10 60 FF FF 10 10 FF FF 20 10 01 90 "
    "40 20 01 90 80 20 01 90 C0 20 00 64 08 10 01 90 E0 20 FF FF 95"
)


def send_xbus_command(ser, command_bytes, description):
    print(f"Sending: {description}...")
    ser.write(command_bytes)
    time.sleep(0.1)  # Give the hardware a brief moment to process

    # Read the acknowledgment response from the device
    if ser.in_waiting:
        response = ser.read(ser.in_waiting)
        print(f"Response: {response.hex().upper()}")
        return response
    else:
        print("No response received.")
        return None


def configure_device():
    try:
        # Open serial port with your exact timeout parameter
        ser = serial.Serial(PORT, baudrate=BAUDRATE, timeout=TIMEOUT)
        print(f"Opened port {PORT} successfully.")

        # 1. Force the device into Configuration Mode
        send_xbus_command(ser, GOTO_CONFIG, "GoToConfig")

        # 2. Inject your 400Hz output matrix configuration
        send_xbus_command(ser, SET_OUTPUT_400HZ, "Set MTI-630(R) 400Hz Output")

        # 3. Put it back to Measurement Mode to save and stream
        send_xbus_command(ser, GOTO_MEASUREMENT, "GoToMeasurement")

        print("\nConfiguration complete! Reading streaming data stream:")
        # Stream raw data to verify it works
        for _ in range(20):
            if ser.in_waiting:
                data = ser.readline()
                print(f"Live Packet: {data.hex()[:40]}...")
            time.sleep(0.01)

        ser.close()

    except serial.SerialException as e:
        print(f"Serial Error: {e}. Check if another process is using the port.")
    except Exception as e:
        print(f"Unexpected error: {e}")


if __name__ == "__main__":
    configure_device()
