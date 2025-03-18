import time
from datetime import datetime
import board
import adafruit_dht
import serial
import pynmea2
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Set up DHT22 sensor - connected to GPIO 18
dht_device = adafruit_dht.DHT22(board.D18)  # GPIO 18 for DHT-22

# Set up GPS - connected to GPIO 14 (TX) and 15 (RX)
# On newer Raspberry Pi models, the primary UART is often at /dev/serial0
gps_port = "/dev/serial0"  # Try this instead of /dev/ttyAMA0
# Alternative options if serial0 doesn't work:
# gps_port = "/dev/ttyS0"
# gps_port = "/dev/ttyUSB0"  # If using a USB-to-Serial adapter
gps_baudrate = 9600
try:
    ser = serial.Serial(gps_port, gps_baudrate, timeout=1)
    print(f"Successfully connected to GPS on {gps_port}")
except serial.SerialException as e:
    print(f"Error opening serial port {gps_port}: {e}")
    print("Available ports may include:")
    import glob
    available_ports = glob.glob("/dev/tty*")
    for port in available_ports:
        if "ttyS" in port or "ttyAMA" in port or "ttyUSB" in port or "serial" in port:
            print(f"  - {port}")
    raise

# Data storage
timestamps = []
temperature_data = []
humidity_data = []
latitude_data = []
longitude_data = []

# Plot setup
fig, (ax1, ax2) = plt.subplots(2, 1)
fig.suptitle("Real-Time Sensor Data")

def update_plot(frame):
    global timestamps, temperature_data, humidity_data, latitude_data, longitude_data

    # Read DHT22 sensor
    try:
        temp = dht_device.temperature
        humidity = dht_device.humidity
    except RuntimeError as e:
        print(f"DHT error: {e}")
        temp, humidity = None, None

    # Read GPS data
    lat, lon = None, None
    try:
        line = ser.readline().decode('utf-8', errors='ignore')
        if line.startswith('$GPGGA'):
            msg = pynmea2.parse(line)
            lat, lon = msg.latitude, msg.longitude
    except Exception as e:
        print(f"GPS error: {e}")

    # Store values
    timestamps.append(time.time())
    temperature_data.append(temp)
    humidity_data.append(humidity)
    latitude_data.append(lat)
    longitude_data.append(lon)

    # Keep last 100 values for visualization
    timestamps, temperature_data, humidity_data = timestamps[-100:], temperature_data[-100:], humidity_data[-100:]
    latitude_data, longitude_data = latitude_data[-100:], longitude_data[-100:]

    # Update Temperature & Humidity Plot
    ax1.clear()
    ax1.plot(timestamps, temperature_data, label="Temperature (Â°C)", color='r')
    ax1.plot(timestamps, humidity_data, label="Humidity (%)", color='b')
    ax1.set_xlabel("Time")
    ax1.set_ylabel("Values")
    ax1.legend()
    ax1.grid()

    # Update GPS Plot
    ax2.clear()
    ax2.scatter(longitude_data, latitude_data, label="GPS Location", color='g')
    ax2.set_xlabel("Longitude")
    ax2.set_ylabel("Latitude")
    ax2.legend()
    ax2.grid()

# Start live update
ani = animation.FuncAnimation(fig, update_plot, interval=1000)
plt.show()
