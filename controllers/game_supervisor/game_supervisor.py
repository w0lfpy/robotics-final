"""
Game Supervisor Controller - Pilla-Pilla
=========================================
"""

from controller import Supervisor
import math

TIME_STEP = 32
GAME_DURATION = 120.0  # 2 minutos iniciales
TIME_BONUS = 30.0      # +30 segundos por captura
CAPTURE_DISTANCE = 0.12


def dist_2d(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.sqrt(dx*dx + dy*dy)


# ===== INIT =====
supervisor = Supervisor()

RUNNER_NAMES = ["RUNNER_1", "RUNNER_2", "RUNNER_3"]
HUNTER_NAME = "HUNTER"

hunter_node = supervisor.getFromDef(HUNTER_NAME)
runner_nodes = [supervisor.getFromDef(name) for name in RUNNER_NAMES]

# Validación de nodos
if hunter_node is None:
    print(f"[ERROR] Falta {HUNTER_NAME}")
for i, node in enumerate(runner_nodes):
    if node is None:
        print(f"[ERROR] Falta {RUNNER_NAMES[i]}")

emitter = supervisor.getDevice("super_emitter")
display = supervisor.getDevice("display")

captured = [False, False, False]
game_over = False
score = 0
start_time = None
step_count = 0
bonus_time = 0.0

print("=" * 40)
print("PILLA-PILLA - HUNTER (Rojo) vs RUNNERS (Azul)")
print(f"Tiempo: {int(GAME_DURATION)}s (+{int(TIME_BONUS)}s por captura)")
print("=" * 40)


def update_display(remaining, score, game_over, winner=None):
    if not display:
        return
    w, h = display.getWidth(), display.getHeight()
    display.setColor(0x1a1a2e)
    display.fillRectangle(0, 0, w, h)
    
    if game_over:
        display.setColor(0xff4444 if winner == "HUNTER" else 0x4444ff)
        display.fillRectangle(0, 0, w, h)
        display.setColor(0xffffff)
        display.setFont("Arial", 14, True)
        if winner == "HUNTER":
            display.drawText("HUNTER GANA!", 20, 40)
        else:
            display.drawText("RUNNERS GANAN!", 15, 40)
    else:
        display.setColor(0xffffff)
        display.setFont("Arial", 12, True)
        display.drawText(f"Tiempo: {int(remaining)}s", 50, 20)
        
        display.setColor(0xff6b6b)
        display.drawText(f"Presas: {3-score}", 60, 50)
        
        # Barra de tiempo
        total_time = GAME_DURATION + bonus_time
        bar = max(1, int((remaining / total_time) * (w - 20)))
        display.setColor(0x00ff00 if remaining > 30 else (0xffff00 if remaining > 15 else 0xff0000))
        display.fillRectangle(10, 80, bar, 10)


# ===== LOOP =====
while supervisor.step(TIME_STEP) != -1:
    step_count += 1
    if step_count < 10:
        continue
    
    if start_time is None:
        start_time = supervisor.getTime()
    
    elapsed = supervisor.getTime() - start_time
    total_time = GAME_DURATION + bonus_time
    remaining = max(0, total_time - elapsed)
    
    if game_over:
        continue
    
    # 1. Obtener posiciones
    if hunter_node:
        hunter_pos = hunter_node.getField("translation").getSFVec3f()
    else:
        hunter_pos = [0, 0, 0]

    runner_positions = []
    for i, runner in enumerate(runner_nodes):
        if runner and not captured[i]:
            runner_positions.append(runner.getField("translation").getSFVec3f())
        else:
            runner_positions.append(None)
    
    # 2. ENVIAR A RUNNERS (Channel 1)
    # Formato: "HUNTER_X,HUNTER_Y,HUNTER_Z"
    emitter.setChannel(1)
    msg_hunter = f"{hunter_pos[0]},{hunter_pos[1]},{hunter_pos[2]}"
    emitter.send(msg_hunter.encode("utf-8"))
    
    # Avisar capturas
    for idx, is_cap in enumerate(captured):
        if is_cap:
            emitter.send(f"CAPTURED:{RUNNER_NAMES[idx]}".encode("utf-8"))
    
    # 3. ENVIAR A HUNTER (Channel 2)
    # Formato: "X1,Y1,Z1;X2,Y2,Z2..."
    emitter.setChannel(2)
    active_str = []
    for pos in runner_positions:
        if pos is not None:
            active_str.append(f"{pos[0]},{pos[1]},{pos[2]}")
    if active_str:
        emitter.send(";".join(active_str).encode("utf-8"))
    
    # 4. Verificar Capturas
    for idx, (runner, rpos) in enumerate(zip(runner_nodes, runner_positions)):
        if captured[idx] or rpos is None or runner is None:
            continue
        
        d = dist_2d(hunter_pos, rpos)
        
        if d < CAPTURE_DISTANCE:
            captured[idx] = True
            score += 1
            bonus_time += TIME_BONUS
            
            print(f"¡CAPTURADO! {RUNNER_NAMES[idx]} | Eliminado")
            
            # 1. Eliminar robot físico
            runner.remove()
            
            update_display(remaining + TIME_BONUS, score, False)
    
    # 5. Condiciones Victoria
    if score >= 3:
        game_over = True
        print("¡¡¡ HUNTER (ROJO) GANA !!!")
        update_display(remaining, score, True, "HUNTER")
        supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
    
    elif remaining <= 0:
        game_over = True
        print("¡¡¡ RUNNERS (AZULES) GANAN !!!")
        update_display(remaining, score, True, "RUNNERS")
        supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
    
    if int(elapsed) != int(elapsed - TIME_STEP/1000):
        update_display(remaining, score, False)
