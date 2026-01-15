from controller import Robot
import math

TIME_STEP = 32

# ===== PARÁMETROS =====
MAX_SPEED = 8.0
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
STATE_ESCAPE = 0
STATE_WALL = 1
state = STATE_ESCAPE

chasers = []

# ===== LOOP =====
while robot.step(TIME_STEP) != -1:

    vL = ds_left.getValue()
    vR = ds_right.getValue()

    # ===== DETECTAR OBSTÁCULO =====
    if state == STATE_ESCAPE and (vL > AVOID_THRESHOLD or vR > AVOID_THRESHOLD):
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

        if vL < 60 and vR < 60:
            state = STATE_ESCAPE
        continue

    # ===== RECIBIR CHASERS =====
    msg = recv_latest(receiver)
    if msg:
        chasers = []
        for part in msg.split(";"):
            p = part.split(",")
            if len(p) == 3:
                try:
                    chasers.append((float(p[0]), float(p[1]), float(p[2])))
                except ValueError:
                    pass

    if not chasers:
        left_motor.setVelocity(-BASE_SPEED)
        right_motor.setVelocity(-BASE_SPEED)
        continue

    pos = gps.getValues()
    north = compass.getValues()

    cx, cy, cz = min(
        chasers,
        key=lambda c: (c[0] - pos[0])**2 + (c[2] - pos[2])**2
    )

    dx = pos[0] - cx
    dz = pos[2] - cz

    heading = math.atan2(north[0], north[2])
    desired = math.atan2(dx, dz)

    turn = TURN_GAIN * norm_angle(desired - heading)

    lv = BASE_SPEED + turn
    rv = BASE_SPEED - turn
    lv, rv = normalize_speeds(lv, rv, MAX_SPEED)

    left_motor.setVelocity(-lv)
    right_motor.setVelocity(-rv)
