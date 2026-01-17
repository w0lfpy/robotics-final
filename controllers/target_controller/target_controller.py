"""
Runner Controller (AZUL) - Huye del ROJO
=========================================
Con detección de bloqueo por posición
"""

from controller import Robot
import math
import random

TIME_STEP = 32
MAX_SPEED = 5.0
BASE_SPEED = 4.0


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
print(f"[AZUL] {robot_name}")

left_motor = robot.getDevice("motor_1")
right_motor = robot.getDevice("motor_2")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

gps = robot.getDevice("gps")
compass = robot.getDevice("compass")
gps.enable(TIME_STEP)
compass.enable(TIME_STEP)

# Sensores - intentar nuevos nombres, si no existen usar antiguos
ds_fl = robot.getDevice("ds_front_left") or robot.getDevice("ds_left")
ds_fr = robot.getDevice("ds_front_right") or robot.getDevice("ds_right")
if ds_fl: ds_fl.enable(TIME_STEP)
if ds_fr: ds_fr.enable(TIME_STEP)

receiver = robot.getDevice("receiver")
receiver.enable(TIME_STEP)

chaser_pos = None
captured = False

# Para detección de bloqueo
last_positions = []
STUCK_CHECK_SIZE = 20
escape_counter = 0
escape_direction = 1

for _ in range(15):
    robot.step(TIME_STEP)


# ===== LOOP =====
while robot.step(TIME_STEP) != -1:
    
    if captured:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        continue
    
    pos = gps.getValues()
    north = compass.getValues()
    
    fl = ds_fl.getValue() if ds_fl else 0
    fr = ds_fr.getValue() if ds_fr else 0
    front_max = max(fl, fr)
    
    # Guardar posición para detección de bloqueo
    last_positions.append((pos[0], pos[1]))
    if len(last_positions) > STUCK_CHECK_SIZE:
        last_positions.pop(0)
    
    # Detectar si está atascado (no se movió significativamente)
    is_stuck = False
    if len(last_positions) >= STUCK_CHECK_SIZE:
        old = last_positions[0]
        dist = math.sqrt((pos[0]-old[0])**2 + (pos[1]-old[1])**2)
        is_stuck = dist < 0.02  # Menos de 2cm en 20 pasos
    
    # Mensajes
    msg = recv_latest(receiver)
    if msg:
        if msg.startswith("CAPTURED:") and msg.split(":")[1] == robot_name:
            captured = True
            continue
        elif not msg.startswith("CAPTURED"):
            coords = msg.split(",")
            if len(coords) == 3:
                try:
                    chaser_pos = (float(coords[0]), float(coords[1]), float(coords[2]))
                except:
                    pass
    
    # ===== OBSTÁCULO FRONTAL -> GIRAR =====
    
    # Panic: Pegado a la pared (>950) -> Retroceder
    if front_max > 950:
         left_motor.setVelocity(MAX_SPEED) # Positive = Back
         right_motor.setVelocity(MAX_SPEED)
         continue

    # Critical: Pivot in place (> 500)
    if front_max > 500:
        # Romper simetría: Si son parecidos, girar siempre a la izquierda
        if abs(fl - fr) < 50:
             # Giro forzado izquierda (motores: izq=+MAX, der=-MAX)
             left_motor.setVelocity(MAX_SPEED) 
             right_motor.setVelocity(-MAX_SPEED)
        elif fl > fr:
            # Obstáculo izq -> girar derecha
            left_motor.setVelocity(-MAX_SPEED)
            right_motor.setVelocity(MAX_SPEED)
        else:
            # Obstáculo der -> girar izquierda
            left_motor.setVelocity(MAX_SPEED)
            right_motor.setVelocity(-MAX_SPEED)
        continue
    
    # Warning: Gentle turn
    elif front_max > 80:
        if fl > fr:
            left_motor.setVelocity(-BASE_SPEED)
            right_motor.setVelocity(0) # Stop inner wheel for tighter turn
        else:
            left_motor.setVelocity(0)
            right_motor.setVelocity(-BASE_SPEED)
        continue

    # ===== MODO ESCAPE (cuando está atascado) =====
    if escape_counter > 0:
        escape_counter -= 1
        # Fase 1: Retroceder más tiempo para despegarse
        if escape_counter > 20: 
            left_motor.setVelocity(MAX_SPEED)
            right_motor.setVelocity(MAX_SPEED)
        # Fase 2: Girar aleatoriamente
        else:
            left_motor.setVelocity(-MAX_SPEED * escape_direction)
            right_motor.setVelocity(MAX_SPEED * escape_direction)
        continue
    
    # Iniciar escape si está atascado
    # Importante: No activar escape solo por sensor alto (eso lo maneja obstacle avoidance arriba)
    # Solo si realmente no nos movemos (is_stuck)
    if is_stuck:
        escape_counter = 40 # Aumentado tiempo de escape
        escape_direction = random.choice([-1, 1])
        last_positions.clear()
        continue
    
    # ===== HUIR DEL ROJO =====
    if chaser_pos is not None:
        heading = math.atan2(north[0], north[1])
        dx = pos[0] - chaser_pos[0]
        dy = pos[1] - chaser_pos[1]
        escape_angle = math.atan2(dx, dy)
        
        error = norm_angle(escape_angle - heading)
        turn = 3.0 * error
        
        lv = BASE_SPEED + turn
        rv = BASE_SPEED - turn
        
        m = max(abs(lv), abs(rv))
        if m > MAX_SPEED:
            lv = lv / m * MAX_SPEED
            rv = rv / m * MAX_SPEED
        
        left_motor.setVelocity(-lv)
        right_motor.setVelocity(-rv)
    else:
        left_motor.setVelocity(-BASE_SPEED)
        right_motor.setVelocity(-BASE_SPEED)
