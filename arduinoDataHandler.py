import numpy as np
import serial

def split_data(data):
    values = data.split(',')
    float_values = [float(v) for v in values]
    return {
        "temp": float(float_values[0]),
        "resultantG": float(float_values[1]),
        "xa": float(float_values[2]),
        "ya": float(float_values[3]),
        "za": float(float_values[4]),
        "xd": float(float_values[5]),
        "yd": float(float_values[6]),
        "zd": float(float_values[7]),
        "xlv": float(float_values[8]),
        "ylv": float(float_values[9]),
        "zlv": float(float_values[10]),
        "xla": float(float_values[11]),
        "yla": float(float_values[12]),
        "zla": float(float_values[13]),
    }

bData = None
def get_arduino_data():
    global bData
    if bData is None:
        return {
            "temp": 0,
            "resultantG": 0,
            "xaa": 0,
            "yaa": 0,
            "zaa": 0,
            "xd": 0,
            "yd": 0,
            "zd": 0,
            "xlv": 0,
            "ylv": 0,
            "zlv": 0,
            "xla": 0,
            "yla": 0,
            "zla": 0,
        }
    return bData

def get_arduino_angular_data():
    global bData
    if bData is None:
        return {
            "xd": 0,
            "yd": 0,
            "zd": 0,
        }
    return {
        "xd": bData["xd"],
        "yd": bData["yd"],
        "zd": bData["zd"],
    }

def get_arduino_acceleation_data():
    global bData
    if bData is None:
        return {
            "xa": 0,
            "ya": 0,
            "za": 0,
        }
    return {
        "xa": bData["xa"],
        "ya": bData["ya"],
        "za": bData["za"],
    }

def get_arduino_linear_adata():
    global bData
    if bData is None:
        return {
            "xla": 0,
            "yla": 0,
            "zla": 0,
        }
    return {
        "xla": bData["xla"],
        "yla": bData["yla"],
        "zla": bData["zla"],
    }

def get_arduino_linear_vdata():
    global bData
    if bData is None:
        return {
            "xlv": 0,
            "ylv": 0,
            "zlv": 0,
        }
    return {
        "xlv": bData["xlv"],
        "ylv": bData["ylv"],
        "zlv": bData["zlv"],
    }
def arduinoData():
    try:
        # Configure the serial connection (adjust port and baudrate as needed)
        ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1)  # Change '/dev/ttyUSB0' to your actual port

        print("Listening for serial data... Press Ctrl+C to stop.")
        global bData
        while True:
            if ser.in_waiting > 0:  # Check if data is available
                try:
                    data = ser.readline().decode('utf-8', errors='ignore').strip()  # Read and decode the data
                    if data:
                        print(f"Received: {data}")
                        bData = split_data(data)
                except Exception as e:
                    print(f"Error reading data: {e}")

    except serial.SerialException as e:
        print(f"Serial connection error: {e}")
    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial connection closed.")