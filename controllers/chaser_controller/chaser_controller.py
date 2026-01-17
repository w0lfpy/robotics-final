"""
Chaser Controller (ROJO) - Persigue AZULES
===========================================
Con detección de bloqueo por posición
"""

from controller import Robot
import math
import random

TIME_STEP = 32
MAX_SPEED = 5.5
BASE_SPEED = 4.5


def norm_angle(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a


def recv_latest(receiver):
    latest = None
    while receiver.getQueueLength() > 0:
        try:
            latest = receiver.getString()
        except:
            pass
        receiver.nextPacket()
    return latest


# ===== INIT =====
robot = Robot()
robot_name = robot.getName()
print(f"[ROJO] {robot_name}")

left_motor = robot.getDevice("motor_1")
right_motor = robot.getDevice("motor_2")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

gps = robot.getDevice("gps")
compass = robot.getDevice("compass")
gps.enable(TIME_STEP)
compass.enable(TIME_STEP)

ds_fl = robot.getDevice("ds_front_left") or robot.getDevice("ds_left")
ds_fr = robot.getDevice("ds_front_right") or robot.getDevice("ds_right")
if ds_fl: ds_fl.enable(TIME_STEP)
if ds_fr: ds_fr.enable(TIME_STEP)

receiver = robot.getDevice("receiver")
receiver.enable(TIME_STEP)

runners = []

# Detección de bloqueo
last_positions = []
STUCK_CHECK_SIZE = 20
escape_counter = 0
escape_direction = 1

# Wander state
wander_counter = 0
wander_turn = 0

for _ in range(15):
    robot.step(TIME_STEP)


# ===== LOOP =====
while robot.step(TIME_STEP) != -1:
    
    pos = gps.getValues()
    north = compass.getValues()
    
    fl = ds_fl.getValue() if ds_fl else 0
    fr = ds_fr.getValue() if ds_fr else 0
    front_max = max(fl, fr)
    
    # Guardar posición
    last_positions.append((pos[0], pos[1]))
    if len(last_positions) > STUCK_CHECK_SIZE:
        last_positions.pop(0)
    
    # Detectar bloqueo
    is_stuck = False
    if len(last_positions) >= STUCK_CHECK_SIZE:
        old = last_positions[0]
        dist = math.sqrt((pos[0]-old[0])**2 + (pos[1]-old[1])**2)
        is_stuck = dist < 0.02
    
    # Mensajes
    msg = recv_latest(receiver)
    if msg and not msg.startswith("CAPTURED"):
        runners = []
        for part in msg.split(";"):
            coords = part.split(",")
            if len(coords) == 3:
                try:
                    runners.append((float(coords[0]), float(coords[1]), float(coords[2])))
                except:
                    pass
    
    # ===== OBSTÁCULO FRONTAL -> GIRAR =====
    # Critical: Pivot in place
    if front_max > 500:
        if fl > fr:
            left_motor.setVelocity(-MAX_SPEED)
            right_motor.setVelocity(MAX_SPEED)
        else:
            left_motor.setVelocity(MAX_SPEED)
            right_motor.setVelocity(-MAX_SPEED)
        continue
    
    # Warning: Gentle turn
    if front_max > 80:
        if fl > fr:
            left_motor.setVelocity(-BASE_SPEED)
            right_motor.setVelocity(-BASE_SPEED * 0.2)
        else:
            left_motor.setVelocity(-BASE_SPEED * 0.2)
            right_motor.setVelocity(-BASE_SPEED)
        continue

    # ===== MODO ESCAPE (cuando está atascado) =====
    if escape_counter > 0:
        escape_counter -= 1
        # Fase 1: Retroceder más tiempo
        if escape_counter > 20:
            left_motor.setVelocity(MAX_SPEED)
            right_motor.setVelocity(MAX_SPEED)
        # Fase 2: Girar
        else:
            left_motor.setVelocity(-MAX_SPEED * escape_direction)
            right_motor.setVelocity(MAX_SPEED * escape_direction)
        continue
    
    # Iniciar escape si está atascado
    if is_stuck:
        escape_counter = 40
        escape_direction = random.choice([-1, 1])
        last_positions.clear()
        continue
    
    # ===== PERSEGUIR =====
    if runners:
        heading = math.atan2(north[0], north[1])
        closest = min(runners, key=lambda r: (r[0]-pos[0])**2 + (r[1]-pos[1])**2)
        
        dx = closest[0] - pos[0]
        dy = closest[1] - pos[1]
        target = math.atan2(dx, dy)
        
        error = norm_angle(target - heading)
        turn = 3.5 * error
        
        lv = BASE_SPEED + turn
        rv = BASE_SPEED - turn
        
        m = max(abs(lv), abs(rv))
        if m > MAX_SPEED:
            lv = lv / m * MAX_SPEED
            rv = rv / m * MAX_SPEED
        
        left_motor.setVelocity(-lv)
        right_motor.setVelocity(-rv)
        
    # ===== BUSCAR / DEAMBULAR (Wander) =====
    else:
        if wander_counter > 0:
            wander_counter -= 1
            # Aplicar giro guardado si estamos en modo giro, o seguir recto
            if wander_turn != 0:
                left_motor.setVelocity(-BASE_SPEED - wander_turn)
                right_motor.setVelocity(-BASE_SPEED + wander_turn)
            else:
                left_motor.setVelocity(-BASE_SPEED)
                right_motor.setVelocity(-BASE_SPEED)
        else:
            # Seleccionar nueva acción aleatoria
            action = random.random()
            if action < 0.7:
                # 70% probabilidad: ir recto un rato
                wander_counter = random.randint(20, 100)
                wander_turn = 0
            else:
                # 30% probabilidad: girar un poco
                wander_counter = random.randint(10, 30)
                # Giro aleatorio: positivo o negativo
                wander_turn = random.uniform(-2.0, 2.0)

