from controller import Robot, Motor, DistanceSensor, PositionSensor
import math

def metros_a_grilla(x_m, y_m):
    columna = int((x_m + 1.0) / TAMANO_CELDA)
    fila = int((1.0 - y_m) / TAMANO_CELDA)
    columna = max(0, min(columna, GRILLA_ANCHO - 1))
    fila = max(0, min(fila, GRILLA_ALTO - 1))
    return fila, columna

def grilla_a_metros(fila, columna):
    x_m = (columna * TAMANO_CELDA) + (TAMANO_CELDA / 2.0) - 1.0
    y_m = 1.0 - (fila * TAMANO_CELDA) - (TAMANO_CELDA / 2.0)
    return x_m, y_m

def algoritmo_a_estrella(grilla, inicio, meta):
    movimientos = [
        (-1, 0), (1, 0), (0, -1), (0, 1),
        (-1, -1), (-1, 1), (1, -1), (1, 1)
    ]
    
    def heuristica(a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    nodo_inicial = inicio
    open_list = {nodo_inicial}
    came_from = {}
    g_score = {nodo_inicial: 0.0}
    f_score = {nodo_inicial: heuristica(nodo_inicial, meta)}

    while open_list:
        actual = min(open_list, key=lambda nodo: f_score.get(nodo, float('inf')))
        
        if actual == meta:
            camino = [actual]
            while actual in came_from:
                actual = came_from[actual]
                camino.append(actual)
            camino.reverse()
            return camino

        open_list.remove(actual)
        
        for mov in movimientos:
            vecino = (actual[0] + mov[0], actual[1] + mov[1])
            
            if not (0 <= vecino[0] < GRILLA_ALTO and 0 <= vecino[1] < GRILLA_ANCHO):
                continue
                
            if grilla[vecino[0]][vecino[1]] == 1:
                continue
            
            if mov[0] != 0 and mov[1] != 0:
                if grilla[actual[0] + mov[0]][actual[1]] == 1 or grilla[actual[0]][actual[1] + mov[1]] == 1:
                    continue
            
            costo_movimiento = 1.414 if (mov[0] != 0 and mov[1] != 0) else 1.0
            tentative_g_score = g_score[actual] + costo_movimiento
            
            if tentative_g_score < g_score.get(vecino, float('inf')):
                came_from[vecino] = actual
                g_score[vecino] = tentative_g_score
                f_score[vecino] = tentative_g_score + heuristica(vecino, meta)
                open_list.add(vecino)
                
    return None


robot = Robot()
timestep = int(robot.getBasicTimeStep())
ts = timestep / 1000.0 

RADIO_RUEDA = 0.0205
DISTANCIA_RUEDAS = 0.052

motor_izquierdo = robot.getDevice('left wheel motor')
motor_derecho = robot.getDevice('right wheel motor')
motor_izquierdo.setPosition(float('inf'))
motor_derecho.setPosition(float('inf'))
motor_izquierdo.setVelocity(0.0)
motor_derecho.setVelocity(0.0)

encoder_izquierdo = robot.getDevice('left wheel sensor')
encoder_derecho = robot.getDevice('right wheel sensor')
encoder_izquierdo.enable(timestep)
encoder_derecho.enable(timestep)

nombres_sensores = {
    'frontal_der': 'ps0',
    'frontal_izq': 'ps7',
    'lateral_der': 'ps2',
    'lateral_izq': 'ps5'
}
sensores_distancia = {}
for clave, nombre_webots in nombres_sensores.items():
    sensores_distancia[clave] = robot.getDevice(nombre_webots)
    sensores_distancia[clave].enable(timestep)

x_actual = 0.0      
y_actual = 0.0      
theta_actual = 0.0  
last_enc_izq = 0.0
last_enc_der = 0.0

distancia_total_recorrida = 0.0
contador_evasiones = 0
reporte_impreso = False

GRILLA_ANCHO = 20
GRILLA_ALTO = 20
TAMANO_CELDA = 0.1 

grilla = [[0 for _ in range(GRILLA_ANCHO)] for _ in range(GRILLA_ALTO)]

MODO_ESCENARIO = "COMPLEJO" 

if MODO_ESCENARIO == "COMPLEJO":
    for col in range(0, 14):
        grilla[12][col] = 1
    
    for col in range(7, 20):
        grilla[15][col] = 1

elif MODO_ESCENARIO == "SIMPLE":
    grilla[13][10] = 1
    grilla[13][9] = 1


celda_inicio = metros_a_grilla(x_actual, y_actual)
celda_meta = (18, 10)

print(f"Calculando ruta desde {celda_inicio} hasta {celda_meta}...")
ruta_global = algoritmo_a_estrella(grilla, celda_inicio, celda_meta)

longitud_ruta_planificada = 0.0
if ruta_global:
    for i in range(len(ruta_global) - 1):
        x1, y1 = grilla_a_metros(ruta_global[i][0], ruta_global[i][1])
        x2, y2 = grilla_a_metros(ruta_global[i+1][0], ruta_global[i+1][1])
        longitud_ruta_planificada += math.sqrt((x2-x1)**2 + (y2-y1)**2)
    print(f"Ruta encontrada con exito, Consta de {len(ruta_global)} puntos.")
    print(f"Longitud planificada: {longitud_ruta_planificada:.2f} m")
    print(f"Puntos de la ruta: {ruta_global}")
else:
    print("ERROR: No se pudo encontrar una ruta válida hacia la meta.")


distancia_estimada = 0.40  
P_kalman = 0.1             
Q_kalman = 0.0001          
R_kalman = 0.02            

UMBRAL_CRITICO = 0.16      
UMBRAL_HISTERESIS = 0.22   
estado_emergencia = False  


while robot.step(timestep) != -1:
    tiempo_simulacion = robot.getTime()
    
    enc_izq = encoder_izquierdo.getValue()
    enc_der = encoder_derecho.getValue()

    delta_th_izq = enc_izq - last_enc_izq
    delta_th_der = enc_der - last_enc_der

    delta_s_izq = RADIO_RUEDA * delta_th_izq
    delta_s_der = RADIO_RUEDA * delta_th_der

    delta_s = (delta_s_der + delta_s_izq) / 2.0
    delta_phi = (delta_s_der - delta_s_izq) / DISTANCIA_RUEDAS

    x_actual += delta_s * math.cos(theta_actual + (delta_phi / 2.0))
    y_actual += delta_s * math.sin(theta_actual + (delta_phi / 2.0))
    theta_actual += delta_phi
    theta_actual = math.atan2(math.sin(theta_actual), math.cos(theta_actual))
    
    distancia_total_recorrida += abs(delta_s)

    last_enc_izq = enc_izq
    last_enc_der = enc_der
    
    val_ps0 = sensores_distancia['frontal_der'].getValue()
    val_ps7 = sensores_distancia['frontal_izq'].getValue()
    val_ps2 = sensores_distancia['lateral_der'].getValue()
    val_ps5 = sensores_distancia['lateral_izq'].getValue()
    val_max = max(val_ps0, val_ps7, val_ps2, val_ps5)
    
    if val_max < 1.0: 
        val_max = 1.0
        
    z_k = 15.0 / (val_max ** 0.8)

    distancia_predicha = distancia_estimada - delta_s
    P_predicha = P_kalman + Q_kalman

    K_ganancia = P_predicha / (P_predicha + R_kalman)
    distancia_estimada = distancia_predicha + K_ganancia * (z_k - distancia_predicha)
    P_kalman = (1.0 - K_ganancia) * P_predicha

    if distancia_estimada < UMBRAL_CRITICO:
        if not estado_emergencia:
            print("ALERTA DE COLISION: Activando Navegacion Reactiva Local.")
            contador_evasiones += 1  
        estado_emergencia = True
    elif distancia_estimada > UMBRAL_HISTERESIS:
        if estado_emergencia:
            print("ZONA DESPEJADA: Reanudando Planificacion Global.")
        estado_emergencia = False

    if estado_emergencia:
        ps2_lateral_der = sensores_distancia['lateral_der'].getValue()
        ps5_lateral_izq = sensores_distancia['lateral_izq'].getValue()
        
        v_lineal = 0.0
        if ps5_lateral_izq > ps2_lateral_der:
            v_angular = -2.5 
        else:
            v_angular = 2.5
            
    elif ruta_global and len(ruta_global) > 0:
        objetivo_celda = ruta_global[0]
        x_target, y_target = grilla_a_metros(objetivo_celda[0], objetivo_celda[1])
        
        dx = x_target - x_actual
        dy = y_target - y_actual
        distancia_error = math.sqrt(dx**2 + dy**2)
        
        theta_target = math.atan2(dy, dx)
        theta_error = theta_target - theta_actual
        theta_error = math.atan2(math.sin(theta_error), math.cos(theta_error))
        
        if distancia_error < 0.08:
            ruta_global.pop(0)
            v_lineal = 0.0
            v_angular = 0.0
        else:
            Kp_angular = 5.0
            Kp_lineal = 3.0
            
            if abs(theta_error) > 0.4:
                v_lineal = 0.0
                v_angular = Kp_angular * theta_error
            else:
                v_lineal = min(Kp_lineal * distancia_error, 4.0)
                v_angular = Kp_angular * theta_error
    else:
        v_lineal = 0.0
        v_angular = 0.0
        if not reporte_impreso:
            print("\n=======================================================")
            print("META ALCANZADA CON EXITO")
            print(f"REPORTE DE MÉTRICAS:")
            print(f"- Tiempo total de viaje: {tiempo_simulacion:.2f} s")
            print(f"- Longitud ruta planificada (A*): {longitud_ruta_planificada:.2f} m")
            print(f"- Distancia real recorrida (Odometría): {distancia_total_recorrida:.2f} m")
            print(f"- Intervenciones reactivas (Colisiones): {contador_evasiones}")
            print("=======================================================\n")
            reporte_impreso = True

    w_izq = (v_lineal - (v_angular * DISTANCIA_RUEDAS / 2.0)) / RADIO_RUEDA
    w_der = (v_lineal + (v_angular * DISTANCIA_RUEDAS / 2.0)) / RADIO_RUEDA
    
    VEL_MAX = 6.28
    v_izq = max(-VEL_MAX, min(w_izq, VEL_MAX))
    v_der = max(-VEL_MAX, min(w_der, VEL_MAX))

    motor_izquierdo.setVelocity(v_izq)
    motor_derecho.setVelocity(v_der)
    
    if int(tiempo_simulacion / ts) % 30 == 0 and not reporte_impreso:
        fila, col = metros_a_grilla(x_actual, y_actual)
        target_print = ruta_global[0] if (ruta_global and len(ruta_global) > 0) else "META"
        print(f"[T: {tiempo_simulacion:.2f}s] Celda: M[{fila}][{col}] | Kalman: {distancia_estimada:.3f}m | Emergencia: {estado_emergencia} | Target: {target_print}")