"""
Chaser Controller (Dorado) - State Machine & Odometry
"""

from controller import Robot
import math
import random

TIME_STEP = 32
MAX_SPEED = 9.0
BASE_SPEED = 7.0

R = 0.025
D = 0.09

STATE_SEARCH = "SEARCH"
STATE_CHASE = "CHASE"
STATE_AVOID = "AVOID"
STATE_RECOVERY = "RECOVERY"

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
        self.runners = []
        self.stuck_counter = 0
        self.recovery_timer = 0
        self.recovery_dir = 1
        self.avoid_timer = 0    # Timer for avoidance maneuver
        self.avoid_turn = 0     # Turn magnitude for avoidance
        self.wander_timer = 0
        self.wander_turn = 0

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
        
        self.runners = []
        while self.receiver.getQueueLength() > 0:
            msg = self.receiver.getString()
            if msg and not msg.startswith("CAPTURED"):
                 for part in msg.split(";"):
                    coords = part.split(",")
                    if len(coords) == 3:
                        try:
                            self.runners.append((float(coords[0]), float(coords[1]), float(coords[2])))
                        except: pass
            self.receiver.nextPacket()

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

        if self.check_stuck():
            self.recovery_timer = 30
            self.recovery_dir = random.choice([-1, 1])
            return STATE_RECOVERY

        if self.fl_val > 300 or self.fr_val > 300:
            # Initialize Avoidance
            self.avoid_timer = random.randint(10, 25) # Commit to avoid for X steps
            # Random direction but biased away from obstacle
            if self.fl_val > self.fr_val:
                self.avoid_turn = -1 # Right
            else:
                self.avoid_turn = 1 # Left
            
            # Add randomness to direction (sometimes wrong way to just break patterns?)
            # No, keep it rational but maybe vary magnitude?
            return STATE_AVOID

        if self.runners:
            return STATE_CHASE

        return STATE_SEARCH

    def run_state(self):
        left_speed = 0
        right_speed = 0

        if self.current_state == STATE_RECOVERY:
            self.recovery_timer -= 1
            if self.recovery_timer > 15: 
                left_speed = MAX_SPEED
                right_speed = MAX_SPEED
            else:
                left_speed = -MAX_SPEED * self.recovery_dir
                right_speed = MAX_SPEED * self.recovery_dir

        elif self.current_state == STATE_AVOID:
            self.avoid_timer -= 1
            # Perform a sharp turn
            # To turn effectively with differential drive: one forward, one back (spin)
            # or one stop, one fast.
            
            # Spin in place or tight curve
            turn_speed = MAX_SPEED
            
            if self.avoid_turn > 0: # Turn Left
                left_speed = MAX_SPEED
                right_speed = -MAX_SPEED 
            else: # Turn Right
                left_speed = -MAX_SPEED 
                right_speed = MAX_SPEED
                
            # If obstacle still VERY Close, maybe keep backing up?
            if self.fl_val > 800 or self.fr_val > 800:
                 left_speed = MAX_SPEED
                 right_speed = MAX_SPEED

        elif self.current_state == STATE_CHASE:
            current_pos = (self.x, self.y)
            closest = min(self.runners, key=lambda r: (r[0]-current_pos[0])**2 + (r[1]-current_pos[1])**2)
            
            dx = closest[0] - current_pos[0]
            dy = closest[1] - current_pos[1]
            target_phi = math.atan2(dy, dx)
            
            error = target_phi - self.phi
            while error > math.pi: error -= 2*math.pi
            while error < -math.pi: error += 2*math.pi
            
            turn = 3.5 * error
            left_speed = BASE_SPEED - turn
            right_speed = BASE_SPEED + turn

        elif self.current_state == STATE_SEARCH:
            if self.wander_timer > 0:
                self.wander_timer -= 1
            else:
                self.wander_timer = random.randint(30, 100)
                if random.random() < 0.3:
                    self.wander_turn = random.uniform(-2.0, 2.0)
                else:
                    self.wander_turn = 0
            
            if self.wander_turn != 0:
                left_speed = BASE_SPEED - self.wander_turn
                right_speed = BASE_SPEED + self.wander_turn
            else:
                left_speed = BASE_SPEED
                right_speed = BASE_SPEED

        m = max(abs(left_speed), abs(right_speed))
        if m > MAX_SPEED:
            left_speed = left_speed / m * MAX_SPEED
            right_speed = right_speed / m * MAX_SPEED

        self.left_motor.setVelocity(-left_speed)
        self.right_motor.setVelocity(-right_speed)

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
