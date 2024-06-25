import serial
import matplotlib.pyplot as plt
import time

ser = serial.Serial('/dev/ttyUSB0', 9600)

time.sleep(2)

time_data = []
output_data = []

plt.ion()  # Turn on interactive mode
fig, ax = plt.subplots()
line, = ax.plot(time_data, output_data, label='PID Output')
plt.xlabel('Time (ms)')
plt.ylabel('Output')
plt.title('PID Controller Output Over Time')
plt.legend()

try:
    while True:
        if ser.in_waiting > 0:
            serial_line = ser.readline().decode('utf-8', errors='ignore').rstrip()
            if serial_line:
                data = serial_line.split(',')
                if len(data) == 2:
                    try:
                        time_ms = int(data[0])
                        response = float(data[1])
                        time_data.append(time_ms)
                        output_data.append(response)
                        print(f'Time: {time_ms} ms, PID Output: {response}')

                        # Update plot data
                        line.set_xdata(time_data)
                        line.set_ydata(output_data)
                        ax.relim()
                        ax.autoscale_view()
                        plt.draw()
                        plt.pause(0.001)

                    except ValueError as e:
                        print(f"Error converting data: {data}, {e}")
except KeyboardInterrupt:
    print("Interrupted by user")
finally:
    ser.close()
    plt.ioff()  # Turn off interactive mode
    plt.show()
