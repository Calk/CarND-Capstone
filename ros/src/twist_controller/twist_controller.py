from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, car_properties):
        # TODO: Implement
        self.yaw_controller = YawController(car_properties)
        pass

    def control(self, linear_v, angular_v, current_v, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        steer = self.yaw_controller.get_steering(linear_v, angular_v, current_v)
        
        return 0.3, 0., steer
