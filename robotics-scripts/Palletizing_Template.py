# -----------------------------------------------------------
# Proyecto: Paletizado
# Robot: Doosan Robotics H2017
# Programado por: Mario Padilla
# Fecha de creación: 27/abril/2025
# Última modificación: 27/abril/2025
# Descripción: Paletizado enseñando APP, calculando PLACE y RET con variables independientes.
# -----------------------------------------------------------

from robodk import robolink, robomath
import time

# Conectar con RoboDK
RDK = robolink.Robolink()

# Seleccionar el robot
robot = RDK.Item('Doosan Robotics H2017', robolink.ITEM_TYPE_ROBOT)

# Seleccionar la herramienta
tool = RDK.Item('OnRobot VGP20 Vacuum Gripper', robolink.ITEM_TYPE_TOOL)

# Seleccionar marcos de referencia
frame_pick = RDK.Item('Conveyor_Frame', robolink.ITEM_TYPE_FRAME)
frame_place = RDK.Item('Pallet_Frame', robolink.ITEM_TYPE_FRAME)

# Variables globales de velocidad y aceleración
Global_Vel_Joint = 50     # Velocidad en Joints (deg/s)
Global_Acc_Joint = 50     # Aceleración en Joints (deg/s²)
Global_Vel_Linear = 300   # Velocidad lineal (mm/s)
Global_Acc_Linear = 300   # Aceleración lineal (mm/s²)

# Variables de cantidad de cajas y capas
Global_Num_Box_Layer = 2    # Cajas por cama
Global_Num_Layers = 4       # Número de camas

# Variables independientes de alturas y movimientos
Global_Box_Height = 200            # Altura real de cada caja (positivo)
Global_Approach_Distance = 200      # Bajada desde APP a PLACE (positivo porque Z apunta hacia abajo)
Global_Retreat_Distance = -50       # Retirada desde PLACE (negativo)

# Obtener posiciones auxiliares UP
pose_up_pallet1 = RDK.Item('Global_up1_Pallet').Joints()
pose_up_pallet2 = RDK.Item('Global_up2_Pallet').Joints()

if not pose_up_pallet1 or not pose_up_pallet2:
    raise Exception("Error: No se encontraron las posiciones UP de Pallet.")

# Cargar APP enseñados
pattern_a_apps = []
pattern_b_apps = []

for i in range(Global_Num_Box_Layer):
    item_app_a = RDK.Item(f'Global_App_P1_A_{i}')
    item_app_b = RDK.Item(f'Global_App_P1_B_{i}')
    
    if item_app_a.Valid():
        pattern_a_apps.append(item_app_a.Pose())
    else:
        print(f"Advertencia: No se encontró Global_App_P1_A_{i}")

    if item_app_b.Valid():
        pattern_b_apps.append(item_app_b.Pose())
    else:
        print(f"Advertencia: No se encontró Global_App_P1_B_{i}")

if not pattern_a_apps or not pattern_b_apps:
    raise Exception("Error: No se encontraron las posiciones de APP A o B.")

# Construir todas las posiciones de todas las camas
positions = []  # [(app_pose, place_pose, ret_pose)]

for layer in range(Global_Num_Layers):
    if layer % 2 == 0:
        base_apps = pattern_a_apps
    else:
        base_apps = pattern_b_apps

    # Altura acumulada: ¡Se resta porque Z apunta hacia abajo!
    altura_extra = Global_Box_Height * Global_Num_Box_Layer * (layer // 2)

    for i in range(Global_Num_Box_Layer):
        app_pose = base_apps[i]

        # RESTAR altura extra para subir
        final_app_pose = app_pose * robomath.transl(0, 0, -altura_extra)

        # Bajar desde APP para obtener PLACE
        place_pose = final_app_pose * robomath.transl(0, 0, Global_Approach_Distance)

        # Bajar más desde PLACE para obtener RET
        ret_pose = place_pose * robomath.transl(0, 0, Global_Retreat_Distance)

        positions.append((final_app_pose, place_pose, ret_pose))

# Obtener posiciones de Pick
joints_app_pick = RDK.Item('Global_app_Pick').Joints()
pose_pick = RDK.Item('Global_Pick').Pose()
pose_ret_pick = RDK.Item('Global_ret_Pick').Pose()

if not all([joints_app_pick, pose_pick, pose_ret_pick]):
    raise Exception("Error: No se encontraron los targets de Pick en RoboDK.")

# Función de Pick
def pick():
    robot.setPoseFrame(frame_pick)
    robot.setPoseTool(tool)
    robot.setSpeed(Global_Vel_Linear, Global_Acc_Linear, Global_Vel_Joint, Global_Acc_Joint)
    time.sleep(0.2)

    robot.MoveJ(joints_app_pick)
    robot.MoveL(pose_pick)
    print("Recogiendo caja...")
    time.sleep(0.5)
    RDK.RunProgram("Attach", True)  # Activar ventosa si quieres
    time.sleep(0.5)
    robot.MoveL(pose_ret_pick)

# Función de Place
def place(app_pose, place_pose, ret_pose, up_pose, index):
    robot.setPoseFrame(frame_place)
    robot.setPoseTool(tool)
    robot.setSpeed(Global_Vel_Linear, Global_Acc_Linear, Global_Vel_Joint, Global_Acc_Joint)
    time.sleep(0.2)

    robot.MoveJ(up_pose)
    print(f"Moviendo a APP_PLACE {index+1}")
    robot.MoveJ(app_pose)
    print(f"Moviendo a PLACE {index+1}")
    robot.MoveL(place_pose)
    time.sleep(0.5)
    RDK.RunProgram("Drop", True)  # Soltar ventosa si quieres
    time.sleep(0.5)
    print(f"Moviendo a RET_PLACE {index+1}")
    robot.MoveL(ret_pose)
    robot.MoveJ(up_pose)

# Programa principal
def run_pick_and_place():
    start_time = time.time()
    for i in range(Global_Num_Box_Layer * Global_Num_Layers):
        print(f"--- Caja {i+1} ---")
        pick()

        if (i % (Global_Num_Box_Layer * 2)) < Global_Num_Box_Layer:
            up_pose = pose_up_pallet1
        else:
            up_pose = pose_up_pallet2

        place(*positions[i], up_pose, i)

    end_time = time.time()
    total_time = end_time - start_time
    print(f"Tiempo total de ejecución: {total_time:.2f} segundos")
    RDK.ShowMessage(f"Tiempo total: {total_time:.2f} s", False)

# Esperar antes de iniciar
time.sleep(2)

# Ejecutar
run_pick_and_place()
