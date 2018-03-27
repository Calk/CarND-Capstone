from yaw_controller import YawController
from pid import PID
#from rospy import get_param
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, car_properties):
        self.yaw_controller = YawController(car_properties)
        
        self.acceleration_controller = PID(0.1,3.e-3,4.e-3, mn=-1., mx=1.)

        self.car_properties = car_properties
        self.max_velocity = (rospy.get_param('/waypoint_loader/velocity') * 1000.) / (60.*60.) # velocity in mps

    def control(self, linear_v, angular_v, current_v, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.acceleration_controller.reset()
            return 0., 0., 0.
            
        if linear_v < ONE_MPH and current_v < ONE_MPH:
            return 0., 2, 0
            
        accelerate = self.acceleration_controller.step(linear_v - current_v , 1./50.)
        
        
        brake = 0.
        throttle = 0.
        if accelerate > 0:
            throttle = accelerate
        else:
            brake = -accelerate*200
        
        steer = self.yaw_controller.get_steering(linear_v, angular_v, current_v)
        
        if current_v>=self.max_velocity*.98:
            throttle = 0
            brake += 2
            
        brake = brake*self.car_properties.vehicle_mass*self.car_properties.accel_limit*self.car_properties.wheel_radius
        return throttle, brake, steer
