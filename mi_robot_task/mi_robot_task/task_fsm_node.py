#!/usr/bin/env python3
import os
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from std_msgs.msg import String, Empty
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from visualization_msgs.msg import Marker, MarkerArray

from mi_robot_task.ik_solver import solve_ik

Q_HOME = [0.0, -0.45, 0.90, 0.0]

BIN_COORDS = {
    'red':   (0.15, -0.40),
    'green': (0.15,  0.40),
    'blue':  (-0.25, 0.35),
}

TRANSIT_COORDS = {
    'red':   (0.25, -0.22, 0.24),
    'green': (0.25,  0.22, 0.24),
    'blue':  (0.00,  0.32, 0.24),
}

GAZEBO_OBJECTS = {
    'obj_red_1':   {'color': 'red',   'x': 0.31, 'y': -0.15},
    'obj_green_1': {'color': 'green', 'x': 0.47, 'y': -0.13},
    'obj_blue_1':  {'color': 'blue',  'x': 0.31, 'y':  0.15},
    'obj_red_2':   {'color': 'red',   'x': 0.47, 'y':  0.13},
}

class TaskFSMNode(Node):
    def __init__(self):
        super().__init__('task_fsm_node')

        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        self.attach_pubs = {}
        self.detach_pubs = {}
        for name in GAZEBO_OBJECTS:
            self.attach_pubs[name] = self.create_publisher(Empty, f'/gripper/attach/{name}', 10)
            self.detach_pubs[name] = self.create_publisher(Empty, f'/gripper/detach/{name}', 10)

        self.latest_detections = []
        self.sub_det = self.create_subscription(String, '/detected_objects', self.det_cb, 10)

        self.pub_state = self.create_publisher(String, '/fsm_state', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/fsm_markers', 10)

        self.state = 'HOME'
        self.sub_step = 0
        self.goal_future = None
        self.result_future = None
        self.step_start_time = 0.0

        self.target_obj = None
        self.target_gz_name = None
        self.attached_gz_name = None
        self.picked_gz_models = set()
        self.bin_counts = {'red': 0, 'green': 0, 'blue': 0}

        self.retries = 0
        self.max_retries = 3
        self.failure_reason = 'Ninguna'
        self.run_start_time = time.time()
        self.obj_start_time = 0.0
        self.metrics_objects = []
        self.finished_reported = False

        self.get_logger().info('Esperando servidor de acción joint_trajectory_controller...')
        self.traj_client.wait_for_server()
        self.get_logger().info('Servidor ros2_control conectado. Iniciando FSM en estado HOME.')

        self.timer = self.create_timer(0.1, self.fsm_loop)

    def det_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.latest_detections = [d for d in data if d['x'] > 0.22 and abs(d['y']) < 0.25]
        except Exception:
            pass

    def transition(self, new_state, reason=''):
        msg = f'[FSM] {self.state} -> {new_state}'
        if reason:
            msg += f' ({reason})'
        self.get_logger().info(msg)
        self.state = new_state
        self.sub_step = 0
        self.goal_future = None
        self.result_future = None
        self.step_start_time = time.time()

    def send_trajectory(self, waypoints, durations_sec):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        for q, t_sec in zip(waypoints, durations_sec):
            pt = JointTrajectoryPoint()
            pt.positions = [float(val) for val in q]
            sec = int(t_sec)
            nanosec = int((t_sec - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
            goal.trajectory.points.append(pt)

        self.goal_future = self.traj_client.send_goal_async(goal)
        self.result_future = None
        self.step_start_time = time.time()

    def check_trajectory(self, timeout_sec=45.0):
        if self.goal_future is not None and self.result_future is None:
            if self.goal_future.done():
                gh = self.goal_future.result()
                if not gh or not gh.accepted:
                    return 'FAILED'
                self.result_future = gh.get_result_async()
            elif time.time() - self.step_start_time > timeout_sec:
                return 'TIMEOUT'
            return 'RUNNING'

        if self.result_future is not None:
            if self.result_future.done():
                return 'SUCCEEDED'
            elif time.time() - self.step_start_time > timeout_sec:
                return 'TIMEOUT'
            return 'RUNNING'

        return 'IDLE'

    def match_gazebo_model(self, x, y, color):
        best_name = None
        best_dist = 999.0
        for name, info in GAZEBO_OBJECTS.items():
            if name in self.picked_gz_models:
                continue
            d = math.hypot(info['x'] - x, info['y'] - y)
            if info['color'] == color:
                d -= 0.05
            if d < best_dist:
                best_dist = d
                best_name = name
        return best_name

    def publish_diagnostics(self):
        st_msg = String()
        st_msg.data = self.state
        self.pub_state.publish(st_msg)

        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        txt = Marker()
        txt.header.frame_id = 'world'
        txt.header.stamp = now
        txt.ns = 'fsm_status'
        txt.id = 0
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position.x = 0.0
        txt.pose.position.y = 0.0
        txt.pose.position.z = 0.95
        txt.scale.z = 0.08
        txt.color.r = 1.0
        txt.color.g = 1.0
        txt.color.b = 0.0
        txt.color.a = 1.0
        label = f'FSM: {self.state}'
        if self.target_obj:
            label += f" | Obj: {self.target_obj['color_ai'].upper()}"
        txt.text = label
        markers.markers.append(txt)

        sph = Marker()
        sph.header.frame_id = 'world'
        sph.header.stamp = now
        sph.ns = 'target_pose'
        sph.id = 1
        sph.type = Marker.SPHERE
        sph.scale.x = 0.07
        sph.scale.y = 0.07
        sph.scale.z = 0.07
        if self.target_obj:
            sph.action = Marker.ADD
            sph.pose.position.x = float(self.target_obj['x'])
            sph.pose.position.y = float(self.target_obj['y'])
            sph.pose.position.z = 0.06
            sph.color.r = 0.0
            sph.color.g = 1.0
            sph.color.b = 1.0
            sph.color.a = 0.85
        else:
            sph.action = Marker.DELETE
        markers.markers.append(sph)

        self.pub_markers.publish(markers)

    def fsm_loop(self):
        self.publish_diagnostics()

        if self.state == 'HOME':
            if self.sub_step == 0:
                self.send_trajectory([Q_HOME], [2.0])
                self.sub_step = 1
            elif self.sub_step == 1:
                res = self.check_trajectory()
                if res == 'SUCCEEDED':
                    self.step_start_time = time.time()
                    self.sub_step = 2
                elif res in ('FAILED', 'TIMEOUT'):
                    self.failure_reason = 'Falla al ir a postura HOME'
                    self.transition('RECUPERACIÓN', self.failure_reason)
            elif self.sub_step == 2:
                # Esperar 2.2s reales (~0.45s sim) para estabilizar cámara
                if time.time() - self.step_start_time >= 2.2:
                    self.transition('DETECTAR')

        elif self.state == 'DETECTAR':
            if len(self.latest_detections) > 0:
                ordered = sorted(self.latest_detections, key=lambda d: d['x'])
                cand = ordered[0]
                q_test, err_mm = solve_ik(cand['x'], cand['y'], 0.058)
                if q_test is None:
                    self.failure_reason = f"Objeto fuera de alcance IK ({cand['x']:.2f}, {cand['y']:.2f})"
                    self.transition('RECUPERACIÓN', self.failure_reason)
                    return

                self.target_obj = cand
                self.target_obj['ik_err_mm'] = err_mm
                self.target_gz_name = self.match_gazebo_model(cand['x'], cand['y'], cand['color_ai'])
                self.obj_start_time = time.time()
                self.transition(
                    'APROXIMAR',
                    f"Seleccionado {self.target_gz_name} ({cand['color_ai'].upper()}) en [{cand['x']:.2f}, {cand['y']:.2f}]"
                )
            else:
                if time.time() - self.step_start_time > 4.0:
                    if len(self.picked_gz_models) >= len(GAZEBO_OBJECTS):
                        self.transition('FIN', 'Zona de entrada vacía, todos los objetos clasificados')
                    else:
                        self.failure_reason = 'No se detectaron objetos pendientes en la cámara'
                        self.transition('RECUPERACIÓN', self.failure_reason)

        elif self.state == 'APROXIMAR':
            if self.sub_step == 0:
                tx, ty = self.target_obj['x'], self.target_obj['y']
                q_high, _ = solve_ik(tx, ty, 0.24)
                q_pre, _ = solve_ik(tx, ty, 0.14)
                if q_high is None or q_pre is None:
                    self.failure_reason = 'Error IK en trayectoria de aproximación'
                    self.transition('RECUPERACIÓN', self.failure_reason)
                    return
                self.send_trajectory([q_high, q_pre], [1.6, 2.8])
                self.sub_step = 1
            elif self.sub_step == 1:
                res = self.check_trajectory()
                if res == 'SUCCEEDED':
                    self.transition('TOMAR')
                elif res in ('FAILED', 'TIMEOUT'):
                    self.failure_reason = f'Falla de trayectoria en APROXIMAR ({res})'
                    self.transition('RECUPERACIÓN', self.failure_reason)

        elif self.state == 'TOMAR':
            tx, ty = self.target_obj['x'], self.target_obj['y']
            if self.sub_step == 0:
                q_grasp, _ = solve_ik(tx, ty, 0.058)
                self.send_trajectory([q_grasp], [1.4])
                self.sub_step = 1
            elif self.sub_step == 1:
                res = self.check_trajectory()
                if res == 'SUCCEEDED':
                    self.attached_gz_name = self.target_gz_name
                    self.step_start_time = time.time()
                    self.sub_step = 2
                elif res in ('FAILED', 'TIMEOUT'):
                    self.failure_reason = 'Falla al descender sobre objeto'
                    self.transition('RECUPERACIÓN', self.failure_reason)
            elif self.sub_step == 2:
                # Pulso continuo de attach durante 1.6s para asegurar captura en Gazebo lento
                if self.attached_gz_name in self.attach_pubs:
                    self.attach_pubs[self.attached_gz_name].publish(Empty())
                if time.time() - self.step_start_time >= 1.6:
                    q_lift, _ = solve_ik(tx, ty, 0.24)
                    self.send_trajectory([q_lift], [1.5])
                    self.sub_step = 3
            elif self.sub_step == 3:
                res = self.check_trajectory()
                if res == 'SUCCEEDED':
                    self.transition('TRANSPORTAR', f'Objeto {self.attached_gz_name} sujetado')
                elif res in ('FAILED', 'TIMEOUT'):
                    self.failure_reason = 'Falla al elevar objeto'
                    self.transition('RECUPERACIÓN', self.failure_reason)

        elif self.state == 'TRANSPORTAR':
            if self.sub_step == 0:
                color = self.target_obj['color_ai']
                bx, by = BIN_COORDS[color]
                offset = 0.035 * self.bin_counts[color]
                bx_adj = bx + offset
                by_adj = by
                self.target_bin_pos = (bx_adj, by_adj)

                mx, my, mz = TRANSIT_COORDS[color]
                q_mid, _ = solve_ik(mx, my, mz)
                q_bin_pre, _ = solve_ik(bx_adj, by_adj, 0.22)

                if q_mid is None or q_bin_pre is None:
                    self.failure_reason = f'Error IK hacia bandeja {color}'
                    self.transition('RECUPERACIÓN', self.failure_reason)
                    return

                self.send_trajectory([q_mid, q_bin_pre], [1.8, 3.4])
                self.sub_step = 1
            elif self.sub_step == 1:
                res = self.check_trajectory()
                if res == 'SUCCEEDED':
                    self.transition('SOLTAR')
                elif res in ('FAILED', 'TIMEOUT'):
                    self.failure_reason = 'Falla durante transporte a bandeja'
                    self.transition('RECUPERACIÓN', self.failure_reason)

        elif self.state == 'SOLTAR':
            bx, by = self.target_bin_pos
            if self.sub_step == 0:
                q_place, _ = solve_ik(bx, by, 0.086)
                self.send_trajectory([q_place], [1.4])
                self.sub_step = 1
            elif self.sub_step == 1:
                res = self.check_trajectory()
                if res == 'SUCCEEDED':
                    self.step_start_time = time.time()
                    self.sub_step = 2
                elif res in ('FAILED', 'TIMEOUT'):
                    self.failure_reason = 'Falla al descender en bandeja'
                    self.transition('RECUPERACIÓN', self.failure_reason)
            elif self.sub_step == 2:
                # Pulso continuo de detach durante 1.6s para asegurar liberación en la bandeja
                if self.attached_gz_name in self.detach_pubs:
                    self.detach_pubs[self.attached_gz_name].publish(Empty())
                if time.time() - self.step_start_time >= 1.6:
                    q_retreat, _ = solve_ik(bx, by, 0.24)
                    self.send_trajectory([q_retreat], [1.4])
                    self.sub_step = 3
            elif self.sub_step == 3:
                res = self.check_trajectory()
                if res == 'SUCCEEDED':
                    self.transition('VERIFICAR')
                elif res in ('FAILED', 'TIMEOUT'):
                    self.failure_reason = 'Falla al retirar efector'
                    self.transition('RECUPERACIÓN', self.failure_reason)

        elif self.state == 'VERIFICAR':
            if self.sub_step == 0:
                self.send_trajectory([Q_HOME], [2.0])
                self.sub_step = 1
            elif self.sub_step == 1:
                res = self.check_trajectory()
                if res == 'SUCCEEDED':
                    self.step_start_time = time.time()
                    self.sub_step = 2
                elif res in ('FAILED', 'TIMEOUT'):
                    self.failure_reason = 'Falla al regresar a HOME en VERIFICAR'
                    self.transition('RECUPERACIÓN', self.failure_reason)
            elif self.sub_step == 2:
                if time.time() - self.step_start_time >= 2.2:
                    tx, ty = self.target_obj['x'], self.target_obj['y']
                    still_there = any(math.hypot(d['x'] - tx, d['y'] - ty) < 0.04 for d in self.latest_detections)
                    if still_there:
                        self.failure_reason = 'Verificación fallida: objeto permanece en zona de entrada'
                        self.attached_gz_name = None
                        self.transition('RECUPERACIÓN', self.failure_reason)
                    else:
                        color = self.target_obj['color_ai']
                        self.bin_counts[color] += 1
                        if self.attached_gz_name:
                            self.picked_gz_models.add(self.attached_gz_name)
                        cycle_dt = time.time() - self.obj_start_time
                        self.metrics_objects.append({
                            'model': self.attached_gz_name,
                            'color': color,
                            'time_sec': round(cycle_dt, 2),
                            'ik_err_mm': round(self.target_obj['ik_err_mm'], 4),
                            'ai_conf': self.target_obj['confidence_ai']
                        })
                        self.attached_gz_name = None
                        self.target_obj = None

                        if len(self.latest_detections) > 0:
                            self.transition('DETECTAR', f'Quedan {len(self.latest_detections)} objetos')
                        else:
                            self.transition('FIN', 'Zona de entrada vacía')

        elif self.state == 'RECUPERACIÓN':
            if self.sub_step == 0:
                if self.attached_gz_name and self.attached_gz_name in self.detach_pubs:
                    self.detach_pubs[self.attached_gz_name].publish(Empty())
                self.attached_gz_name = None
                self.target_obj = None
                self.retries += 1
                self.get_logger().warn(
                    f'[RECUPERACIÓN] Intento {self.retries}/{self.max_retries} | Causa: {self.failure_reason}'
                )
                self.send_trajectory([Q_HOME], [2.2])
                self.sub_step = 1
            elif self.sub_step == 1:
                res = self.check_trajectory()
                if res in ('SUCCEEDED', 'FAILED', 'TIMEOUT'):
                    if self.retries <= self.max_retries:
                        self.transition('HOME', 'Reintentando ciclo tras recuperación')
                    else:
                        self.transition('FIN', 'Límite máximo de reintentos alcanzado')

        elif self.state == 'FIN':
            if not self.finished_reported:
                self.finished_reported = True
                total_time = round(time.time() - self.run_start_time, 2)
                n_ok = len(self.metrics_objects)
                total_objs = len(GAZEBO_OBJECTS)
                success_rate = (n_ok / total_objs) * 100.0
                avg_time = round(sum(m['time_sec'] for m in self.metrics_objects) / max(1, n_ok), 2)
                avg_err = round(sum(m['ik_err_mm'] for m in self.metrics_objects) / max(1, n_ok), 4)

                summary = (
                    "\n============================================================\n"
                    "          REPORTE FINAL DE CORRIDA AUTÓNOMA (FSM)           \n"
                    "============================================================\n"
                    f"  Objetos clasificados:      {n_ok} / {total_objs}\n"
                    f"  Tasa de éxito:             {success_rate:.1f} %\n"
                    f"  Tiempo total de corrida:   {total_time:.2f} s\n"
                    f"  Tiempo promedio / objeto:  {avg_time:.2f} s\n"
                    f"  Error promedio IK:         {avg_err:.4f} mm\n"
                    f"  Reintentos / Recuperación: {self.retries}\n"
                    "------------------------------------------------------------\n"
                    "  Detalle por objeto:\n"
                )
                for m in self.metrics_objects:
                    summary += (
                        f"   - {m['model']} [{m['color'].upper()}]: "
                        f"{m['time_sec']} s | Confianza IA: {m['ai_conf']*100:.1f}% | Error IK: {m['ik_err_mm']} mm\n"
                    )
                summary += "============================================================\n"

                self.get_logger().info(summary)
                metrics_path = os.path.expanduser('~/ros2_ws/metricas_corrida.txt')
                with open(metrics_path, 'w') as f:
                    f.write(summary)
                self.get_logger().info(f'Métricas guardadas en: {metrics_path}')

def main(args=None):
    rclpy.init(args=args)
    node = TaskFSMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
