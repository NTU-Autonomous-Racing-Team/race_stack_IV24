import numpy as np

steering_angle_in = np.atan2(wheel_base/ (turning_radius - track_width/2)
steering_angle_out = np.atan2(wheel_base/ (turning_radius + track_width/2)

steering_angle_avg = (steering_angle_in + steering_angle_out)/2

