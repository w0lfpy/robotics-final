"""
Chaser Controller (Dorado) - State Machine & Odometry
"""

from controller import Robot
import math
import random

TIME_STEP = 32
MAX_SPEED = 18.0
BASE_SPEED = 14.0
MAX_ACCEL = 0.8  # Max wheel speed change per step for smoother motion
TURN_GAIN_CHASE = 3.0
MIN_CHASE_SPEED = 7.0
AVOID_BASE = 0.5

R = 0.025
D = 0.09

STATE_SEARCH = "SEARCH"
STATE_CHASE = "CHASE"
STATE_AVOID = "AVOID"
STATE_RECOVERY = "RECOVERY"
STATE_CLEAR = "CLEAR"

class ChaserRobot:
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

        self.current_state = STATE_SEARCH
        self.runners = {}
        self.stuck_counter = 0
        self.recovery_timer = 0
        self.recovery_dir = 1
        self.avoid_timer = 0    # Timer for avoidance maneuver
        self.avoid_dir = 1
        self.clear_timer = 0
        self.clear_dir = 1
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.goal_pos = None
        self.goal_name = None

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
        
        self.runners = {}
        while self.receiver.getQueueLength() > 0:
            msg = self.receiver.getString()
            if msg and not msg.startswith("CAPTURED"):
                for part in msg.split(";"):
                    coords = part.split(",")
                    if len(coords) == 4:
                        try:
                            name = coords[0]
                            self.runners[name] = (float(coords[1]), float(coords[2]), float(coords[3]))
                        except: pass
            self.receiver.nextPacket()

    def select_goal(self):
        if not self.runners:
            return None

        if self.goal_name in self.runners:
            return self.goal_name, self.runners[self.goal_name]

        current_pos = (self.x, self.y)
        best_name = min(
            self.runners.keys(),
            key=lambda name: (self.runners[name][0] - current_pos[0])**2 +
                             (self.runners[name][1] - current_pos[1])**2
        )
        return best_name, self.runners[best_name]

    def check_stuck(self):
        if self.fl_val > 900 or self.fr_val > 900:
            self.stuck_counter += 1
        else:
            self.stuck_counter = max(0, self.stuck_counter - 1)
        
        if self.stuck_counter > 40: 
            return True
        return False

    def select_state(self):
        # Maintain states with timers
        if self.current_state == STATE_RECOVERY:
            if self.recovery_timer > 0: return STATE_RECOVERY
            else:
                self.stuck_counter = 0
                return STATE_SEARCH
        
        if self.current_state == STATE_AVOID:
            if self.avoid_timer > 0: return STATE_AVOID
            # else fall through to re-evaluate
        if self.current_state == STATE_CLEAR:
            if self.clear_timer > 0: return STATE_CLEAR

        if self.check_stuck():
            self.recovery_timer = 30
            self.recovery_dir = random.choice([-1, 1])
            return STATE_RECOVERY

        if self.fl_val > 300 or self.fr_val > 300:
            # Initialize Avoidance
            self.avoid_timer = 12
            self.avoid_dir = -1 if self.fl_val > self.fr_val else 1
            return STATE_AVOID
        if self.clear_timer > 0:
            return STATE_CLEAR

        if self.runners:
            return STATE_CHASE

        return STATE_SEARCH

    def run_state(self):
        left_speed = 0
        right_speed = 0

        if self.current_state == STATE_RECOVERY:
            self.recovery_timer -= 1
            if self.recovery_timer > 15:
                left_speed = -0.6 * MAX_SPEED
                right_speed = -0.6 * MAX_SPEED
            else:
                curve = 0.6 * MAX_SPEED * self.recovery_dir
                left_speed = -0.4 * MAX_SPEED - curve
                right_speed = -0.4 * MAX_SPEED + curve

        elif self.current_state == STATE_AVOID:
            self.avoid_timer -= 1
            if self.avoid_timer > 6:
                # Back up to break contact with obstacle
                left_speed = -0.6 * MAX_SPEED
                right_speed = -0.6 * MAX_SPEED
            else:
                # Turn away to clear the obstacle
                turn = 0.8 * MAX_SPEED * self.avoid_dir
                left_speed = -turn
                right_speed = turn
                self.clear_timer = 10
                self.clear_dir = self.avoid_dir
                
            # If obstacle still VERY Close, maybe keep backing up?
            if self.fl_val > 800 or self.fr_val > 800:
                 left_speed = -0.5 * MAX_SPEED
                 right_speed = -0.5 * MAX_SPEED
        elif self.current_state == STATE_CLEAR:
            self.clear_timer -= 1
            curve = 0.4 * MAX_SPEED * self.clear_dir
            left_speed = 0.6 * MAX_SPEED - curve
            right_speed = 0.6 * MAX_SPEED + curve

        elif self.current_state == STATE_CHASE:
            current_pos = (self.x, self.y)
            goal = self.select_goal()
            if goal is None:
                left_speed = BASE_SPEED
                right_speed = BASE_SPEED
                self.apply_wheel_speeds(left_speed, right_speed)
                return

            self.goal_name, self.goal_pos = goal
            
            dx = self.goal_pos[0] - current_pos[0]
            dy = self.goal_pos[1] - current_pos[1]
            target_phi = math.atan2(dy, dx)
            
            error = target_phi - self.phi
            while error > math.pi: error -= 2*math.pi
            while error < -math.pi: error += 2*math.pi
            
            dist = math.hypot(dx, dy)
            turn = TURN_GAIN_CHASE * error
            turn = max(-0.8 * MAX_SPEED, min(0.8 * MAX_SPEED, turn))
            heading_scale = 0.5 + 0.5 * max(0.0, 1.0 - abs(error) / math.pi)
            forward = BASE_SPEED + min(2.0, dist * 2.0)
            forward = max(MIN_CHASE_SPEED, min(MAX_SPEED, forward * heading_scale))
            left_speed = forward - turn
            right_speed = forward + turn

        elif self.current_state == STATE_SEARCH:
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
    bot = ChaserRobot()
    bot.run()
