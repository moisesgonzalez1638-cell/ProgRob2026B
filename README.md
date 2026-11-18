# Celda Robótica Autónoma Pick-and-Place en ROS 2 Humble

**Autor:** Moisés Isaías González  
**Proyecto:** Programación de Robots 2026B — Clasificación Autónoma por Color (RGB)

Sistema de manipulación autónoma basado en **ROS 2 Humble** e **Ignition Gazebo** que detecta, clasifica por color (Rojo, Verde, Azul) y deposita objetos cilíndricos en sus bandejas mediante visión artificial con IA (**MLP**), cinemática inversa analítica y una máquina de estados finitos (**FSM**) con recuperación ante fallos.

## 1. Arquitectura del Workspace (`~/ros2_ws/src/ProgRob2026B`)
* **`my_robot_description`**: URDF/Xacro del manipulador de 4 GDL + pinza paralela, `ros2_control` (`JointTrajectoryController`), mundo `workcell.sdf`, cámara *Eye-to-Hand* y plugin `DetachableJoint`.
* **`mi_robot_vision`**: Pipeline OpenCV + `cv_bridge`, clasificación neuronal **MLP (`scikit-learn`)**, proyección 2D-a-3D (`TF2`), `/detected_objects` y `/vision/debug_image`.
* **`mi_robot_task`**: Cinemática inversa analítica, FSM de 9 estados, recuperación ante fallos (3 reintentos), marcadores `/fsm_markers` y registro en `metricas_corrida.txt`.
* **`mi_robot_bringup`**: Launch maestro (`pick_and_place.launch.py`) que orquesta Gazebo, RViz2 (`cell.rviz`), visión IA y FSM.

## 2. Cinemática Inversa Analítica (4 GDL)
Con altura de hombro $Z_0 = 0.400\text{ m}$, brazo $L_1 = 0.400\text{ m}$ y antebrazo + efector $L_2 = 0.445\text{ m}$:

$$q_1 = \operatorname{atan2}(y, x), \qquad r = \sqrt{x^2 + y^2}, \qquad \Delta z = z - Z_0$$

$$q_3 = \arccos\left(\frac{r^2 + \Delta z^2 - L_1^2 - L_2^2}{2 L_1 L_2}\right), \qquad q_2 = \operatorname{atan2}(r, \Delta z) - \operatorname{atan2}\big(L_2 \sin(q_3),\; L_1 + L_2 \cos(q_3)\big)$$

## 3. Percepción IA-2 (MLP vs. HSV) y FSM de 9 Estados
* **Visión IA:** Cámara cenital calibrada ($\text{FOV} = 1.45\text{ rad}$, $f_x = f_y = 361.193\text{ px}$). Red neuronal **MLP (`64-32`)** entrenada sobre características **HSV + LAB + RGB** con **100.00 % de exactitud** frente a variaciones de iluminación ($0.45\times$ a $1.35\times$) y brillo especular.
* **FSM (9 estados):** `INICIO -> DETECTAR -> SELECCIONAR -> APROXIMAR -> AGARRAR -> TRANSPORTAR -> SOLTAR -> VERIFICAR -> RECUPERAR / FIN`.

## 4. Compilación y Ejecución
```bash
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch mi_robot_bringup pick_and_place.launch.py
```

## 5. Benchmark de Percepción (IA vs. Visión Clásica)
```text
=== BENCHMARK: IA (MLP NEURAL NETWORK) VS VISION CLASICA (HSV) ===
Dataset de evaluacion: 225 muestras sinteticas con variacion de iluminacion (0.45x - 1.35x) y brillo especular.

1. Metodo Clasico (Segmentacion HSV por umbrales):
   - Exactitud (Accuracy): 100.00 %
   - Tiempo promedio por parche: 0.1340 ms

2. Componente de IA (Red Neuronal MLP 64-32 sobre caracteristicas HSV+LAB+RGB):
   - Exactitud (Accuracy): 100.00 %
   - Tiempo promedio por parche: 2.1101 ms

Reporte detallado IA:
              precision    recall  f1-score   support

         red       1.00      1.00      1.00        75
       green       1.00      1.00      1.00        75
        blue       1.00      1.00      1.00        75

    accuracy                           1.00       225
   macro avg       1.00      1.00      1.00       225
weighted avg       1.00      1.00      1.00       225

```
