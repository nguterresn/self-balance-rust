import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# Configure serial port (adjust as needed)
SERIAL_PORT = '/dev/tty.usbmodem11101'  # Windows: COM3, Linux: /dev/ttyUSB0, Mac: /dev/tty.usbserial-*
BAUD_RATE = 115200

# Data storage
max_points = 200  # Reduced for better performance
duty_data = deque(maxlen=max_points)
pitch_data = deque(maxlen=max_points)
p_data = deque(maxlen=max_points)
i_data = deque(maxlen=max_points)
d_data = deque(maxlen=max_points)
time_data = deque(maxlen=max_points)

# Initialize serial connection
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.001)  # Very short timeout
    ser.reset_input_buffer()  # Clear any existing data
    print(f"Connected to {SERIAL_PORT}")
except:
    print("Could not connect to serial port")
    ser = None

# Set up the plot with performance optimizations
plt.ion()  # Turn on interactive mode
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))  # Smaller figure
fig.suptitle('Real-time PID Control Data')

# Plot lines with reduced line width for performance
line_duty, = ax1.plot([], [], 'r-', label='Duty', linewidth=1)
line_pitch, = ax1.plot([], [], 'b-', label='Pitch', linewidth=1)
ax1.set_ylabel('Duty / Pitch')
ax1.set_ylim(-100, 100)  # Fixed range for duty/pitch
ax1.legend()
ax1.grid(True, alpha=0.3)  # Lighter grid

line_p, = ax2.plot([], [], 'g-', label='P', linewidth=1)
line_i, = ax2.plot([], [], 'm-', label='I', linewidth=1)
line_d, = ax2.plot([], [], 'c-', label='D', linewidth=1)
ax2.set_xlabel('Time')
ax2.set_ylabel('PID Components')
ax2.legend()
ax2.grid(True, alpha=0.3)  # Lighter grid

# Set matplotlib backend for better performance
plt.rcParams['animation.html'] = 'jshtml'

time_counter = 0

def parse_data_line(line):
    """Parse CSV format: duty,pitch,p,i,d"""
    try:
        values = [float(x.strip()) for x in line.split(',')]
        if len(values) == 6:
            return values
    except ValueError:
        # Skip lines that can't be parsed as numbers
        pass
    except Exception as e:
        print(f"Parse error: {e}")

    return None

def animate(frame):
    global time_counter

    if ser and ser.in_waiting > 0:
        # Read multiple lines if available (batch processing)
        lines_processed = 0
        while ser.in_waiting > 0 and lines_processed < 10:  # Limit batch size
            try:
                line = ser.readline().decode('utf-8').strip()
                if not line:  # Skip empty lines
                    continue

                values = parse_data_line(line)
                if values:
                    duty, pitch, error, p, i, d = values

                    # Store data
                    time_data.append(time_counter)
                    duty_data.append(duty)
                    pitch_data.append(pitch)
                    p_data.append(p)
                    i_data.append(i)
                    d_data.append(d)

                    time_counter += 1
                    lines_processed += 1

            except Exception as e:
                break  # Skip problematic lines and continue

        # Update plots only if we have new data
        if lines_processed > 0 and len(time_data) > 1:
            # Convert to lists once for better performance
            time_list = list(time_data)

            # Upper plot
            line_duty.set_data(time_list, list(duty_data))
            line_pitch.set_data(time_list, list(pitch_data))

            # Lower plot
            line_p.set_data(time_list, list(p_data))
            line_i.set_data(time_list, list(i_data))
            line_d.set_data(time_list, list(d_data))

            # Only rescale occasionally for performance
            if time_counter % 10 == 0:  # Every 10 points
                ax1.set_xlim(max(0, time_counter - 100), time_counter + 10)  # Rolling window
                ax2.autoscale_view()
                ax2.relim()

    return line_duty, line_pitch, line_p, line_i, line_d

# Start animation with fastest possible interval
ani = animation.FuncAnimation(fig, animate, interval=5, blit=True, cache_frame_data=False)

plt.tight_layout()
plt.show(block=True)

# Cleanup
if ser:
    ser.close()
