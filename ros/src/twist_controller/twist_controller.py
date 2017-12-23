import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, cp):
        # TODO: Implement
        self.carprops = cp
        self.yaw_controller = YawController(
            wheel_base=cp['wheel_base'],
            steer_ratio=cp['steer_ratio'],
            min_speed=cp['min_speed'],
            max_lat_accel=cp['max_lat_accel'],
            max_steer_angle=cp['max_steer_angle'])
        self.pid = PID(
            kp=5,
            ki=.5,
            kd=.5,
            mn=cp['decel_limit'],
            mx=cp['accel_limit'])
        self.s_lpf = LowPassFilter(tau=3, ts=1)
        self.t_lpf = LowPassFilter(tau=3, ts=1)

    def control(self, twist_cmd, curr_vel, d_time):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        lin_vel = abs(twist_cmd.twist.linear.x)
        ang_vel = twist_cmd.twist.angular.z
        vel_e = lin_vel - curr_vel.twist.linear.x
        next_steer = self.yaw_controller.get_steering(
            lin_vel, ang_vel, curr_vel.twist.linear.x)
        next_steer = self.s_lpf.filt(next_steer)
        acceleration = self.pid.step(vel_e, d_time)
        acceleration = self.t_lpf.filt(acceleration)
        if acceleration > .0:
            throttle = acceleration
            brake = .0
        else:
            throttle = .0
            deceleration = -acceleration
            if deceleration < self.carprops['brake_deadband']:
                deceleration = .0
            brake = deceleration 
                * (self.carprops['vehicle_mass'] 
                + self.carprops['fuel_capacity'] 
                * GAS_DENSITY) 
                * self.carprops['wheel_radius']
        return throttle, brake, next_steer

    def reset(self):
        self.pid.reset()
