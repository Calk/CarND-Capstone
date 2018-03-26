from yaw_controller import YawController
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, car_properties):
        self.yaw_controller = YawController(car_properties)
        
        self.acceleration_controller = PID(0.1,3.e-3,4.e-3, mn=-1., mx=1.)

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
        
        return throttle, brake, steer
