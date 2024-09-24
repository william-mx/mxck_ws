#!/usr/bin/env python3
import numpy as np

class OdomSolver:

    def __init__(self):
        self.path = np.empty((0, 3), dtype=np.float32)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None

    def plot_path(self, ax, label = None):
        x, y = self.path[:,0], self.path[:,1]
        ax.plot(x, y, label = label)
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.axis('equal')

    def get_delta_t(self, msg):

        # Get the current time from the message header
        current_time = msg.header.stamp

        # If last_time is None, set it to current time and return
        if self.last_time is None:
            self.last_time = current_time
            dt = 0
        else:
            # Calculate time difference (dt)
            dt = (current_time - self.last_time).to_sec()
            self.last_time = current_time
        
        return dt

    def _apply_motion(self, speed, theta, dt):

        self.x += speed * np.sin(theta) * dt
        self.y += speed * np.cos(theta) * dt

        self.path = np.vstack((self.path, [self.x, self.y, theta]))

        return self.path
        
class AckermannOdomSolver(OdomSolver):
  
    def __init__(self, wheelbase=0.36):
        super().__init__()

        # Parameters
        self.wheelbase = wheelbase
        self.angle_tol = 0.08

        self.isclose = lambda v: np.isclose(abs(v), 0.0, atol = self.angle_tol)

    def compute_angle(self, speed, steering_angle, dt):

        if not self.isclose(steering_angle):
          self.theta += (speed * np.tan(steering_angle) * dt / self.wheelbase)

        return self.theta

    def update_pose(self, speed, steering_angle, dt):


        theta = self.compute_angle(speed, steering_angle, dt)

        path = self._apply_motion(speed, self.theta, dt)

        return path


class InertialOdomSolver(OdomSolver):

  def __init__(self, speed_gain = 0.5, theta_gain = 1.0):
    super().__init__()

    self.velocity = 0
    self.speed_gain = speed_gain
    self.theta_gain = theta_gain

  def compute_velocity(self, acceleration, dt):

      self.velocity += acceleration * dt
      return self.velocity

  def compute_angle(self, angular_velocity, dt):
      self.theta += angular_velocity * dt
      return self.theta

  def update_pose(self, acceleration, angular_velocity, dt):

      speed = self.speed_gain * self.compute_velocity(acceleration, dt)
      theta = self.theta_gain * self.compute_angle(angular_velocity, dt)

      path = self._apply_motion(speed, theta, dt)

      return path


