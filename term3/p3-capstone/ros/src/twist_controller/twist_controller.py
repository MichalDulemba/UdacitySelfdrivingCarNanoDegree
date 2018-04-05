from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
import math
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement

        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.brake_deadband = kwargs['brake_deadband']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.wheel_radius = kwargs['wheel_radius']
        self.wheel_base = kwargs['wheel_base']
        self.steer_ratio = kwargs['steer_ratio']
        self.max_lat_accel = kwargs['max_lat_accel']
        self.max_steer_angle = kwargs['max_steer_angle']
        min_speed = 0.1


        self.linear_pid = PID(kp=0.3, ki=0.1, kd=0.05, mn=self.decel_limit, mx=0.5 * self.accel_limit)
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, min_speed, self.max_lat_accel, self.max_steer_angle)
        self.steering_pid = PID(kp=0.15, ki=0.001, kd=0.1, mn=-self.max_steer_angle, mx=self.max_steer_angle)

        self.velocity_lpf = LowPassFilter(0.5, 0.02)
        
    def reset(self):
        self.linear_pid.reset()
        self.steering_pid.reset()

    def control(self, proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity, duration_in_seconds):
        linear_velocity_error = proposed_linear_velocity - current_linear_velocity

        velocity_correction = self.linear_pid.step(linear_velocity_error, duration_in_seconds)

        current_vel_lpf = self.velocity_lpf.filt(proposed_linear_velocity)
        
        brake = 0
        throttle = velocity_correction
        
        if (math.fabs(current_linear_velocity) < 0.001 and current_vel_lpf < 0.1):
            throttle = 0
            brake = 400
        elif(throttle < 0):
            deceleration = abs(throttle)
            brake = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius * deceleration if deceleration > self.brake_deadband else 0.
            throttle = 0

        predictive_steering = self.yaw_controller.get_steering(proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity)
        
        steering = predictive_steering

        return throttle, brake, steering
