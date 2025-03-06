import serial
import matplotlib.pyplot as plt
import time

def main():
    # Update these to match your system:
    SERIAL_PORT = "/dev/tty.usbserial-0001"  # adjust as needed
    BAUD_RATE = 115200
    
    # Initialize serial connection
    try:
        ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
    except Exception as e:
        print("Failed to open serial port:", e)
        return
    
    # Give the port a moment to stabilize
    time.sleep(2)
    
    # Lists to store gyro data
    gx_data = []
    gy_data = []
    gz_data = []
    
    # Set up live plotting
    plt.ion()  # turn on interactive mode
    fig, ax = plt.subplots(figsize=(8, 6))
    line_x, = ax.plot([], [], label="Gyro X", color="red")
    line_y, = ax.plot([], [], label="Gyro Y", color="green")
    line_z, = ax.plot([], [], label="Gyro Z", color="blue")
    ax.set_title("MPU6050 Gyro Data")
    ax.set_xlabel("Sample")
    ax.set_ylabel("Gyro Value (raw)")
    ax.legend()
    ax.grid(True)
    
    # Display the plot window immediately
    plt.show(block=False)
    
    print("Starting live data collection. Press Ctrl+C to stop.")
    
    try:
        while True:
            raw_line = ser.readline()
            # Decode the incoming bytes; replace errors so the loop keeps running
            line = raw_line.decode("utf-8", errors="replace").strip()
            
            if line.startswith("Gyro Data:"):
                try:
                    # Expected format: "Gyro Data: X = 123, Y = 456, Z = 789"
                    parts = line.split(",")
                    gx = float(parts[0].split("=")[1].strip())
                    gy = float(parts[1].split("=")[1].strip())
                    gz = float(parts[2].split("=")[1].strip())
                    
                    # Append new data to lists
                    gx_data.append(gx)
                    gy_data.append(gy)
                    gz_data.append(gz)
                    
                    # Update x-axis values (sample indices)
                    x_axis = range(len(gx_data))
                    
                    # Update the plot lines
                    line_x.set_data(x_axis, gx_data)
                    line_y.set_data(x_axis, gy_data)
                    line_z.set_data(x_axis, gz_data)
                    
                    # Recalculate axes limits
                    ax.relim()
                    ax.autoscale_view(True, True, True)
                    
                    # Update the figure
                    plt.draw()
                    plt.pause(0.05)
                    
                    print(f"Parsed Gyro => X: {gx}, Y: {gy}, Z: {gz}")
                except Exception as parse_error:
                    print("Parsing error:", parse_error)
    except KeyboardInterrupt:
        print("Data collection stopped by user.")
    
    ser.close()
    plt.ioff()
    plt.show()  # Show final plot

if __name__ == "__main__":
    main()