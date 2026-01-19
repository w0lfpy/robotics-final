"""
Runner Controller (Gris) - State Machine & Odometry
"""

from controller import Robot
import math
import random

TIME_STEP = 32
MAX_SPEED = 6.0
BASE_SPEED = 5.0
MAX_ACCEL = 0.7  # Max wheel speed change per step for smoother motion

R = 0.025
D = 0.09

STATE_WANDER = "WANDER"
STATE_AVOID = "AVOID"
STATE_RECOVERY = "RECOVERY"

class RunnerRobot:
    def __init__(self):
        self.robot = Robot()
        self.name = self.robot.getName()
        
        self.left_motor = self.robot.getDevice("motor_1")
        self.right_motor = self.robot.getDevice("motor_2")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        try:
            self.ps_enc_l = self.robot.getDevice("ps_1")
            self.ps_enc_r = self.robot.getDevice("ps_2")
            self.ps_enc_l.enable(TIME_STEP)
            self.ps_enc_r.enable(TIME_STEP)
            self.odometry_available = True
        except:
            self.odometry_available = False

        self.ds_fl = self.robot.getDevice("ds_front_left") or self.robot.getDevice("ds_left")
        self.ds_fr = self.robot.getDevice("ds_front_right") or self.robot.getDevice("ds_right")
        if self.ds_fl: self.ds_fl.enable(TIME_STEP)
        if self.ds_fr: self.ds_fr.enable(TIME_STEP)

        self.gps = self.robot.getDevice("gps")
        self.compass = self.robot.getDevice("compass")
        self.gps.enable(TIME_STEP)
        self.compass.enable(TIME_STEP)

        self.receiver = self.robot.getDevice("receiver")
        self.receiver.enable(TIME_STEP)

        self.current_state = STATE_WANDER
        self.captured = False
        self.stuck_counter = 0
        self.recovery_timer = 0
        self.recovery_dir = 1
        self.avoid_timer = 0
        self.avoid_dir = 1
        self.left_speed = 0.0
        self.right_speed = 0.0

        self.x = 0.0
        self.y = 0.0
        self.phi = 0.0
        self.prev_enc_l = 0.0
        self.prev_enc_r = 0.0
        
        for _ in range(10):
            self.robot.step(TIME_STEP)
            
        if self.gps and self.compass:
            pos = self.gps.getValues()
            north = self.compass.getValues()
            self.x = pos[0]
            self.y = pos[1]
            self.phi = math.atan2(north[0], north[1])
            if self.odometry_available:
                self.prev_enc_l = self.ps_enc_l.getValue()
                self.prev_enc_r = self.ps_enc_r.getValue()

    def ramp_speed(self, current, target, max_delta):
        delta = target - current
        if delta > max_delta:
            delta = max_delta
        elif delta < -max_delta:
            delta = -max_delta
        return current + delta

    def apply_wheel_speeds(self, left_target, right_target):
        m = max(abs(left_target), abs(right_target))
        if m > MAX_SPEED:
            left_target = left_target / m * MAX_SPEED
            right_target = right_target / m * MAX_SPEED

        self.left_speed = self.ramp_speed(self.left_speed, left_target, MAX_ACCEL)
        self.right_speed = self.ramp_speed(self.right_speed, right_target, MAX_ACCEL)

        self.left_motor.setVelocity(-self.left_speed)
        self.right_motor.setVelocity(-self.right_speed)

    def update_odometry(self, dt):
        if not self.odometry_available:
            pos = self.gps.getValues()
            north = self.compass.getValues()
            self.x = pos[0]
            self.y = pos[1]
            self.phi = math.atan2(north[0], north[1])
            return

        curr_l = self.ps_enc_l.getValue()
        curr_r = self.ps_enc_r.getValue()

        d_l = curr_l - self.prev_enc_l
        d_r = curr_r - self.prev_enc_r

        dist_l = d_l * R
        dist_r = d_r * R

        d_center = (dist_l + dist_r) / 2.0
        d_theta = (dist_r - dist_l) / D

        self.x += d_center * math.cos(self.phi)
        self.y += d_center * math.sin(self.phi)
        self.phi += d_theta
        self.phi = math.atan2(math.sin(self.phi), math.cos(self.phi))

        self.prev_enc_l = curr_l
        self.prev_enc_r = curr_r

    def update_sensors(self):
        self.fl_val = self.ds_fl.getValue() if self.ds_fl else 0
        self.fr_val = self.ds_fr.getValue() if self.ds_fr else 0
        
        while self.receiver.getQueueLength() > 0:
            msg = self.receiver.getString()
            if msg:
                if msg.startswith("CAPTURED:") and msg.split(":")[1] == self.name:
                    self.captured = True
            self.receiver.nextPacket()

    def check_stuck(self):
        if self.fl_val > 900 or self.fr_val > 900:
            self.stuck_counter += 1
        else:
            self.stuck_counter = max(0, self.stuck_counter - 1)
        
        return self.stuck_counter > 40

    def select_state(self):
        if self.captured:
            return "CAPTURED"
            
        if self.current_state == STATE_RECOVERY:
            if self.recovery_timer > 0: return STATE_RECOVERY
            else:
                self.stuck_counter = 0
                return STATE_WANDER

        if self.current_state == STATE_AVOID:
            if self.avoid_timer > 0: return STATE_AVOID

        if self.check_stuck():
            self.recovery_timer = 30
            self.recovery_dir = random.choice([-1, 1])
            return STATE_RECOVERY

        if self.fl_val > 200 or self.fr_val > 200:
            self.avoid_timer = random.randint(10, 18)
            self.avoid_dir = random.choice([-1, 1])
            return STATE_AVOID

        return STATE_WANDER

    def run_state(self):
        if self.captured:
            self.left_motor.setVelocity(0)
            self.right_motor.setVelocity(0)
            return

        left_speed = 0
        right_speed = 0

        if self.current_state == STATE_RECOVERY:
            self.recovery_timer -= 1
            if self.recovery_timer > 15:
                left_speed = -0.5 * MAX_SPEED
                right_speed = -0.5 * MAX_SPEED
            else:
                curve = 0.6 * MAX_SPEED * self.recovery_dir
                left_speed = -0.4 * MAX_SPEED - curve
                right_speed = -0.4 * MAX_SPEED + curve

        elif self.current_state == STATE_AVOID:
            self.avoid_timer -= 1
            
            turn = 0.9 * MAX_SPEED * self.avoid_dir
            left_speed = -turn
            right_speed = turn
                
            if self.fl_val > 600 or self.fr_val > 600:
                left_speed = -0.5 * MAX_SPEED
                right_speed = -0.5 * MAX_SPEED

        elif self.current_state == STATE_WANDER:
            left_speed = BASE_SPEED
            right_speed = BASE_SPEED

        self.apply_wheel_speeds(left_speed, right_speed)

    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            dt = TIME_STEP / 1000.0
            
            self.update_sensors()
            self.update_odometry(dt)
            
            self.current_state = self.select_state()
            self.run_state()

if __name__ == "__main__":
    bot = RunnerRobot()
    bot.run()
