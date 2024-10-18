import numpy as np
import matplotlib.pyplot as plt

# PID Controller class definition
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def update(self, measured_value):
        error = self.setpoint - measured_value
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

# Simulation parameters
time_steps = 100
target_distance = 30  # Desired distance from the vehicle ahead
vehicle_speed = 0  # Initial speed
lead_vehicle_speed = 20  # Constant speed of the lead vehicle
pid = PIDController(Kp=0.6, Ki=0.1, Kd=0.05, setpoint=target_distance)

# Data logging
distance_data = []
speed_data = []
lead_vehicle_data = []

# Simulation loop
for step in range(time_steps):
    lead_vehicle_position = lead_vehicle_speed * step * 0.1
    vehicle_position = vehicle_speed * step * 0.1
    measured_distance = lead_vehicle_position - vehicle_position

    # Update vehicle speed based on the control signal
    control_signal = pid.update(measured_distance)
    vehicle_speed += control_signal  # Adjust speed
    distance_data.append(measured_distance)
    speed_data.append(vehicle_speed)
    lead_vehicle_data.append(lead_vehicle_speed)

# Plotting results
plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(distance_data, label='Distance to Lead Vehicle')
plt.axhline(y=target_distance, color='r', linestyle='--', label='Target Distance')
plt.xlabel('Time Step')
plt.ylabel('Distance (m)')
plt.title('Distance to Lead Vehicle')
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(speed_data, label='Vehicle Speed')
plt.plot(lead_vehicle_data, label='Lead Vehicle Speed', linestyle='--')
plt.xlabel('Time Step')
plt.ylabel('Speed (m/s)')
plt.title('Vehicle Speed vs Lead Vehicle Speed')
plt.legend()

plt.tight_layout()
plt.show()
