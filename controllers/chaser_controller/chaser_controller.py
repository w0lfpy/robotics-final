from controller import Robot
import math

TIME_STEP = 32

# ===== PARÁMETROS =====
MAX_SPEED = 8.0        # velocidad EXACTA
BASE_SPEED = 5.0
TURN_GAIN = 4.0

AVOID_THRESHOLD = 120.0
KP_WALL = 0.02

# ===== UTILIDADES =====
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def norm_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

def normalize_speeds(lv, rv, max_speed):
    m = max(abs(lv), abs(rv))
    if m < 1e-6:
        return 0.0, 0.0
    scale = max_speed / m
    return lv * scale, rv * scale

def recv_latest(receiver):
    latest = None
    while receiver.getQueueLength() > 0:
        if hasattr(receiver, "getString"):
            latest = receiver.getString()
        else:
            latest = receiver.getData().decode("utf-8", errors="ignore")
        receiver.nextPacket()
    return latest

# ===== INIT ROBOT =====
robot = Robot()

left_motor = robot.getDevice("motor_1")
right_motor = robot.getDevice("motor_2")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

gps = robot.getDevice("gps")
compass = robot.getDevice("compass")
gps.enable(TIME_STEP)
compass.enable(TIME_STEP)

ds_left = robot.getDevice("ds_left")
ds_right = robot.getDevice("ds_right")
ds_left.enable(TIME_STEP)
ds_right.enable(TIME_STEP)

receiver = robot.getDevice("receiver")
receiver.enable(TIME_STEP)

# ===== ESTADO =====
STATE_CHASE = 0
STATE_WALL = 1
state = STATE_CHASE

target_pos = None

# ===== LOOP =====
while robot.step(TIME_STEP) != -1:

    # --- recibir target ---
    msg = recv_latest(receiver)
    if msg:
        p = msg.split(",")
        if len(p) == 3:
            try:
                target_pos = (float(p[0]), float(p[1]), float(p[2]))
            except ValueError:
                pass

    vL = ds_left.getValue()
    vR = ds_right.getValue()

    # ===== DETECTAR OBSTÁCULO =====
    if state == STATE_CHASE and (vL > AVOID_THRESHOLD or vR > AVOID_THRESHOLD):
        state = STATE_WALL

    # ===== WALL FOLLOWING =====
    if state == STATE_WALL:
        error = vL - vR
        turn = KP_WALL * error

        lv = BASE_SPEED - turn
        rv = BASE_SPEED + turn
        lv, rv = normalize_speeds(lv, rv, MAX_SPEED)

        left_motor.setVelocity(-lv)
        right_motor.setVelocity(-rv)

        # salir cuando el target vuelve a estar accesible
        if vL < 60 and vR < 60 and target_pos is not None:
            pos = gps.getValues()
            north = compass.getValues()
            heading = math.atan2(north[0], north[2])
            dx = target_pos[0] - pos[0]
            dz = target_pos[2] - pos[2]
            desired = math.atan2(dx, dz)
            if abs(norm_angle(desired - heading)) < 0.4:
                state = STATE_CHASE
        continue

    # ===== PERSEGUIR =====
    if target_pos is None:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        continue

    pos = gps.getValues()
    north = compass.getValues()

    heading = math.atan2(north[0], north[2])
    dx = target_pos[0] - pos[0]
    dz = target_pos[2] - pos[2]
    desired = math.atan2(dx, dz)

    turn = TURN_GAIN * norm_angle(desired - heading)

    lv = BASE_SPEED + turn
    rv = BASE_SPEED - turn
    lv, rv = normalize_speeds(lv, rv, MAX_SPEED)

    left_motor.setVelocity(-lv)
    right_motor.setVelocity(-rv)
