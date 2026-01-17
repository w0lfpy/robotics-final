"""
Runner Controller (Gris) - State Machine & Odometry
"""

from controller import Robot
import math
import random

TIME_STEP = 32
MAX_SPEED = 6.0
BASE_SPEED = 5.0

R = 0.025
D = 0.09

STATE_WANDER = "WANDER"
STATE_FLEE = "FLEE"
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
        self.chaser_pos = None
        self.captured = False
        self.stuck_counter = 0
        self.recovery_timer = 0
        self.recovery_dir = 1
        self.avoid_timer = 0
        self.avoid_turn = 0
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
        
        self.chaser_pos = None
        while self.receiver.getQueueLength() > 0:
            msg = self.receiver.getString()
            if msg:
                if msg.startswith("CAPTURED:") and msg.split(":")[1] == self.name:
                    self.captured = True
                elif not msg.startswith("CAPTURED"):
                    parts = msg.split(",")
                    if len(parts) == 3:
                        try:
                             self.chaser_pos = (float(parts[0]), float(parts[1]), float(parts[2]))
                        except: pass
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
            self.avoid_timer = random.randint(10, 20)
            if self.fl_val > self.fr_val:
                self.avoid_turn = -1 
            else:
                self.avoid_turn = 1
            return STATE_AVOID

        if self.chaser_pos:
            return STATE_FLEE

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
                left_speed = MAX_SPEED
                right_speed = MAX_SPEED
            else:
                left_speed = -MAX_SPEED * self.recovery_dir
                right_speed = MAX_SPEED * self.recovery_dir

        elif self.current_state == STATE_AVOID:
            self.avoid_timer -= 1
            
            if self.avoid_turn > 0: # Left
                left_speed = MAX_SPEED
                right_speed = -MAX_SPEED
            else: # Right
                left_speed = -MAX_SPEED
                right_speed = MAX_SPEED
                
            if self.fl_val > 600 or self.fr_val > 600:
                left_speed = MAX_SPEED
                right_speed = MAX_SPEED

        elif self.current_state == STATE_FLEE:
            dx = self.x - self.chaser_pos[0]
            dy = self.y - self.chaser_pos[1]
            target_angle = math.atan2(dy, dx)
            
            error = target_angle - self.phi
            while error > math.pi: error -= 2*math.pi
            while error < -math.pi: error += 2*math.pi
            
            turn = 3.0 * error
            left_speed = BASE_SPEED - turn
            right_speed = BASE_SPEED + turn

        elif self.current_state == STATE_WANDER:
            if self.wander_timer > 0:
                self.wander_timer -= 1
            else:
                self.wander_timer = random.randint(30, 100)
                if random.random() < 0.3:
                    self.wander_turn = random.uniform(-1.5, 1.5)
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
    bot = RunnerRobot()
    bot.run()
