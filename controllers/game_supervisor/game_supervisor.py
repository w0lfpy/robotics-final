from controller import Supervisor
import math

TIME_STEP = 32

# Captura: MUY pequeña (casi contacto)
CAPTURE_DISTANCE = 0.1  # 10 cm

# Delay para que caigan al suelo y se estabilicen
START_DELAY_STEPS = 5

def dist_2d(a, b, i, j):
    """Distancia 2D usando componentes i y j del vector [x,y,z]."""
    di = a[i] - b[i]
    dj = a[j] - b[j]
    return math.sqrt(di * di + dj * dj)

def pick_ground_plane(points):
    """
    Elige automáticamente el eje 'vertical' como el que menor rango tiene
    entre todos los robots (max-min). La distancia se calcula en los otros dos ejes.
    Devuelve (i, j, up_axis) donde i,j son los ejes del suelo y up_axis el vertical.
    """
    if not points:
        return (0, 2, 1)  # fallback: suelo XZ, vertical Y

    mins = [float("inf")] * 3
    maxs = [float("-inf")] * 3

    for p in points:
        for k in range(3):
            mins[k] = min(mins[k], p[k])
            maxs[k] = max(maxs[k], p[k])

    ranges = [maxs[k] - mins[k] for k in range(3)]
    up_axis = ranges.index(min(ranges))  # el que menos varía -> vertical

    axes = [0, 1, 2]
    axes.remove(up_axis)
    i, j = axes[0], axes[1]
    return (i, j, up_axis)

# ===== INIT =====
supervisor = Supervisor()

target = supervisor.getFromDef("TARGET")
chasers = [
    supervisor.getFromDef("CHASER_1"),
    supervisor.getFromDef("CHASER_2"),
    supervisor.getFromDef("CHASER_3"),
]

if target is None:
    print("[SUPERVISOR] ERROR: No se encuentra DEF TARGET")
    exit(1)

for idx, c in enumerate(chasers, start=1):
    if c is None:
        print(f"[SUPERVISOR] ERROR: No se encuentra DEF CHASER_{idx}")
        exit(1)

print("[SUPERVISOR] Juego iniciado")

# Cambiar el nombre del dispositivo a 'super_emitter'
emitter = supervisor.getDevice("super_emitter")
if emitter is None:
    print("[SUPERVISOR] ERROR: No se encuentra el dispositivo Emitter")
    exit(1)

# Configurar el emisor
emitter.setChannel(1)

step_count = 0

# ===== LOOP =====
while supervisor.step(TIME_STEP) != -1:

    step_count += 1
    if step_count < START_DELAY_STEPS:
        continue

    tpos = target.getField("translation").getSFVec3f()
    cposes = [c.getField("translation").getSFVec3f() for c in chasers]

    # Enviar la posición del objetivo
    message = f"{tpos[0]},{tpos[1]},{tpos[2]}"
    emitter.send(message.encode("utf-8"))

    # Elegir plano correcto automáticamente
    i, j, up = pick_ground_plane([tpos] + cposes)

    # (Opcional) puedes descomentar para ver qué plano está usando:
    # axis_names = ["X", "Y", "Z"]
    # print(f"[SUPERVISOR] Plano suelo: {axis_names[i]}{axis_names[j]} (vertical {axis_names[up]})")

    for ch, cpos in zip(chasers, cposes):
        d = dist_2d(cpos, tpos, i, j)

        # Agregar mensaje de depuración para verificar la distancia
        print(f"[DEBUG] Distancia entre '{ch.getDef()}' y 'TARGET': {d:.4f} m")

        if d < CAPTURE_DISTANCE:
            print("\n=== CAPTURADO ===")
            print(f"El robot '{ch.getDef()}' ha capturado a 'TARGET'")
            print(f"Distancia en plano suelo: {d:.4f} m (umbral {CAPTURE_DISTANCE:.4f} m)")

            # Debug brutal para que NO haya dudas:
            print(f"Plano usado: ejes {i},{j} (vertical {up})")
            print(f"TARGET translation:  {tpos}")
            print(f"{ch.getDef()} translation: {cpos}")

            supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
            break
