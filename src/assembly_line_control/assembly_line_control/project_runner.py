#!/usr/bin/env python3

"""
Headless project runner for Assembly Line OS.

Runs the last opened project from disk when the physical Start button is pressed,
without requiring a browser. Stop / E-STOP cancels the run and publishes /estop.
"""

from __future__ import annotations

import json
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String


CLIENT_ID = 'project_runner'


def _projects_dir() -> Path:
    path = Path.home() / '.assembly_line_os' / 'projects'
    path.mkdir(parents=True, exist_ok=True)
    return path


def _last_project_path() -> Path:
    config_dir = Path.home() / '.assembly_line_os'
    config_dir.mkdir(parents=True, exist_ok=True)
    return config_dir / 'last_project.json'


def load_last_project_id() -> Optional[str]:
    path = _last_project_path()
    if path.exists():
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            project_id = data.get('project_id')
            if project_id:
                return str(project_id)
        except (OSError, json.JSONDecodeError, TypeError, ValueError):
            pass

    # Fallback: most recently modified project file
    newest = None
    newest_mtime = -1.0
    for project_file in _projects_dir().glob('*.json'):
        try:
            mtime = project_file.stat().st_mtime
        except OSError:
            continue
        if mtime > newest_mtime:
            newest_mtime = mtime
            newest = project_file.stem
    return newest


def load_project(project_id: str) -> Optional[dict]:
    project_file = _projects_dir() / f'{project_id}.json'
    if not project_file.exists():
        return None
    try:
        with open(project_file, 'r', encoding='utf-8') as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError):
        return None


class ProjectRunner(Node):
    """Execute green-flag workflows from the last opened project on ROS topics."""

    def __init__(self):
        super().__init__('project_runner')

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._is_executing = False
        self._run_thread: Optional[threading.Thread] = None
        self._current_project_id: Optional[str] = None
        self._motor_status: Dict[int, dict] = {1: {}, 2: {}}
        self._motor_speeds: Dict[int, float] = {1: 100.0, 2: 100.0}
        self._loop_break = threading.local()

        self.motor_cmd_pubs = {
            1: self.create_publisher(Int32, 'motor1/command', 10),
            2: self.create_publisher(Int32, 'motor2/command', 10),
        }
        self.motor_speed_pubs = {
            1: self.create_publisher(Float32, 'motor1/speed', 10),
            2: self.create_publisher(Float32, 'motor2/speed', 10),
        }
        self.relay_pub = self.create_publisher(String, 'relay/command', 10)
        self.estop_pub = self.create_publisher(String, 'estop', 10)
        self.execution_state_pub = self.create_publisher(String, 'assembly_line/execution_state', 10)
        self.execution_stop_request_pub = self.create_publisher(
            String, 'assembly_line/execution_stop_request', 10
        )

        self.create_subscription(String, 'assembly_line/hardware_button', self._hardware_button_cb, 10)
        self.create_subscription(String, 'estop', self._estop_cb, 10)
        self.create_subscription(
            String, 'assembly_line/execution_stop_request', self._stop_request_cb, 10
        )
        self.create_subscription(String, 'motor1/status', lambda msg: self._motor_status_cb(1, msg), 10)
        self.create_subscription(String, 'motor2/status', lambda msg: self._motor_status_cb(2, msg), 10)

        self._state_timer = self.create_timer(1.5, self._heartbeat_execution_state)

        self.get_logger().info('Project runner started (headless start/stop for last opened project)')

    # --- ROS callbacks ---

    def _hardware_button_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(data, dict) or data.get('type') != 'button':
            return
        action = data.get('action')
        if action == 'start':
            self.start_last_project()
        elif action == 'stop':
            self.stop_execution(publish_estop=False)  # Arduino/controller already estopped

    def _estop_cb(self, _msg: String):
        self.stop_execution(publish_estop=False)

    def _stop_request_cb(self, _msg: String):
        self.stop_execution(publish_estop=False)

    def _motor_status_cb(self, motor_id: int, msg: String):
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                self._motor_status[motor_id] = data
        except (json.JSONDecodeError, TypeError):
            pass

    def _heartbeat_execution_state(self):
        if self._is_executing:
            self._publish_execution_state(True, self._current_project_id)

    # --- Public control ---

    def start_last_project(self):
        # Stop any in-progress run outside the lock to avoid joining while holding it
        with self._lock:
            previous = self._run_thread if self._is_executing else None
            if previous:
                self._stop_event.set()
        if previous and previous.is_alive():
            previous.join(timeout=2.0)

        project_id = load_last_project_id()
        if not project_id:
            self.get_logger().error('No last opened project found; save/open a project first')
            return

        project = load_project(project_id)
        if not project:
            self.get_logger().error(f'Could not load project "{project_id}"')
            return

        with self._lock:
            self._stop_event.clear()
            self._is_executing = True
            self._current_project_id = project_id
            self._publish_execution_state(True, project_id)
            self.get_logger().info(f'Starting last opened project: {project_id}')

            self._run_thread = threading.Thread(
                target=self._run_project,
                args=(project, project_id),
                daemon=True,
            )
            self._run_thread.start()

    def stop_execution(self, publish_estop: bool = True):
        was_running = self._is_executing
        self._stop_event.set()
        self._is_executing = False
        if publish_estop:
            msg = String()
            msg.data = 'ESTOP'
            self.estop_pub.publish(msg)
        if was_running:
            stop_msg = String()
            stop_msg.data = '{}'
            self.execution_stop_request_pub.publish(stop_msg)
            self._publish_execution_state(False, self._current_project_id)
            self.get_logger().warn('Project execution stopped')
        else:
            self._publish_execution_state(False, None)

    def _publish_execution_state(self, running: bool, project_id: Optional[str]):
        msg = String()
        msg.data = json.dumps({
            'running': bool(running),
            'projectId': project_id if running else None,
            'clientId': CLIENT_ID,
        })
        self.execution_state_pub.publish(msg)

    # --- Execution engine ---

    def _run_project(self, project: dict, project_id: str):
        try:
            blocks, connections, motor_speeds = self._build_graph(project)
            self._motor_speeds = {
                1: float(motor_speeds.get(1, motor_speeds.get('1', 100.0))),
                2: float(motor_speeds.get(2, motor_speeds.get('2', 100.0))),
            }
            for mid, speed in self._motor_speeds.items():
                self._publish_motor_speed(mid, speed)

            startable_roots = self._get_startable_root_ids(project, blocks)
            if not startable_roots:
                self.get_logger().error('No green-flag workflows in project')
                return

            with ThreadPoolExecutor(max_workers=max(4, len(startable_roots))) as pool:
                futures = [
                    pool.submit(self._execute_from_root, root_id, blocks, connections)
                    for root_id in startable_roots
                ]
                for fut in as_completed(futures):
                    try:
                        fut.result()
                    except Exception as exc:
                        self.get_logger().error(f'Workflow error: {exc}')
        except Exception as exc:
            self.get_logger().error(f'Project run failed: {exc}')
        finally:
            with self._lock:
                self._is_executing = False
            self._publish_execution_state(False, project_id)
            self.get_logger().info(f'Project "{project_id}" finished')

    def _build_graph(self, project: dict):
        blocks: Dict[int, dict] = {}
        for block in project.get('blocks') or []:
            if not isinstance(block, dict):
                continue
            try:
                bid = int(block.get('id'))
            except (TypeError, ValueError):
                continue
            blocks[bid] = block

        connections: Dict[int, List[int]] = {}

        def add_next(from_id: int, next_ids):
            if next_ids is None:
                return
            if not isinstance(next_ids, list):
                next_ids = [next_ids]
            out = connections.setdefault(from_id, [])
            for nid in next_ids:
                try:
                    nid_int = int(nid)
                except (TypeError, ValueError):
                    continue
                if nid_int not in out:
                    out.append(nid_int)

        for block_id, block in blocks.items():
            conn = block.get('connections') or {}
            add_next(block_id, conn.get('next'))

        for conn in project.get('connections') or []:
            if not isinstance(conn, dict):
                continue
            try:
                from_id = int(conn.get('id'))
            except (TypeError, ValueError):
                continue
            add_next(from_id, conn.get('next'))

        motor_speeds = project.get('motorSpeeds') or {}
        return blocks, connections, motor_speeds

    def _get_startable_root_ids(self, project: dict, blocks: Dict[int, dict]) -> List[int]:
        roots: List[int] = []
        for workflow in project.get('workflows') or []:
            if not isinstance(workflow, dict):
                continue
            try:
                root_id = int(workflow.get('rootBlockId'))
            except (TypeError, ValueError):
                continue
            root = blocks.get(root_id)
            if root and root.get('type') == 'event' and root.get('eventType') == 'green-flag':
                if root_id not in roots:
                    roots.append(root_id)
        if not roots:
            for block_id, block in blocks.items():
                if block.get('type') == 'event' and block.get('eventType') == 'green-flag':
                    roots.append(block_id)
        return roots

    def _execute_from_root(self, root_id: int, blocks: Dict[int, dict], connections: Dict[int, List[int]]):
        next_ids = connections.get(root_id) or []
        if not next_ids:
            self.get_logger().warn(f'Green-flag block {root_id} has no connected blocks')
            return
        self._execute_parallel(next_ids, blocks, connections)

    def _get_next(self, block_id: int, connections: Dict[int, List[int]]) -> List[int]:
        return list(connections.get(block_id) or [])

    def _execute_parallel(self, block_ids: List[int], blocks: Dict[int, dict], connections: Dict[int, List[int]]):
        if not block_ids or self._stop_event.is_set():
            return
        if len(block_ids) == 1:
            self._execute_and_continue(block_ids[0], blocks, connections)
            return
        with ThreadPoolExecutor(max_workers=len(block_ids)) as pool:
            futures = [
                pool.submit(self._execute_and_continue, bid, blocks, connections)
                for bid in block_ids
            ]
            for fut in as_completed(futures):
                try:
                    fut.result()
                except Exception as exc:
                    self.get_logger().error(f'Parallel block error: {exc}')

    def _execute_and_continue(self, block_id: int, blocks: Dict[int, dict], connections: Dict[int, List[int]]):
        if self._stop_event.is_set():
            return
        block = blocks.get(block_id)
        if not block:
            return

        # Skip re-entering loop block from body connect-back
        loop_stack: List[dict] = getattr(self._loop_break, 'stack', None) or []
        if loop_stack:
            current = loop_stack[-1]
            if block_id == current.get('blockId') and block.get('type') in ('repeat', 'forever'):
                return

        self._execute_block(block_id, block, blocks, connections)
        if self._stop_event.is_set():
            return

        if loop_stack:
            current = loop_stack[-1]
            next_ids = self._get_next(block_id, connections)
            if current.get('blockId') in next_ids and not current.get('shouldBreak'):
                return

        next_ids = self._get_next(block_id, connections)
        if next_ids:
            self._execute_parallel(next_ids, blocks, connections)

    def _execute_block(self, block_id: int, block: dict, blocks: Dict[int, dict], connections: Dict[int, List[int]]):
        if self._stop_event.is_set():
            return

        btype = block.get('type')
        if btype == 'event':
            return
        if btype == 'motor':
            self._run_motor(block)
        elif btype == 'relay':
            self._run_relay(block)
        elif btype == 'delay':
            duration = float(block.get('duration') or 1.0)
            self._sleep(duration)
        elif btype == 'repeat':
            self._run_repeat(block_id, block, blocks, connections)
        elif btype == 'forever':
            self._run_forever(block_id, blocks, connections)
        elif btype == 'break':
            stack = getattr(self._loop_break, 'stack', None)
            if stack:
                stack[-1]['shouldBreak'] = True
        elif btype == 'pause':
            # Headless: treat pause as wait until stop (no UI resume)
            self.get_logger().warn(f'Pause block {block_id}: waiting until stop (no UI resume in headless mode)')
            while not self._stop_event.is_set():
                time.sleep(0.1)
        elif btype == 'wait-key':
            self.get_logger().warn(f'Wait-key block {block_id}: skipped in headless mode')
        elif btype in ('motor-speed-from-topic', 'subscribe-motor-speed-topic',
                       'unsubscribe-motor-speed-topic', 'ros-trigger', 'wait-sensor',
                       'read-sensor', 'try', 'catch', 'throw-error'):
            self.get_logger().warn(f'Block type "{btype}" ({block_id}) not fully supported headless; skipping')
        else:
            self.get_logger().warn(f'Unknown block type "{btype}" ({block_id}); skipping')

    def _ensure_loop_stack(self) -> List[dict]:
        stack = getattr(self._loop_break, 'stack', None)
        if stack is None:
            stack = []
            self._loop_break.stack = stack
        return stack

    def _run_repeat(self, block_id: int, block: dict, blocks: Dict[int, dict], connections: Dict[int, List[int]]):
        count = int(block.get('count') or 10)
        loop_state = {'blockId': block_id, 'type': 'repeat', 'shouldBreak': False}
        stack = self._ensure_loop_stack()
        stack.append(loop_state)
        try:
            body = self._get_next(block_id, connections)
            for _ in range(count):
                if self._stop_event.is_set() or loop_state['shouldBreak']:
                    break
                if body:
                    self._execute_parallel(body, blocks, connections)
                self._sleep(0.01)
        finally:
            if stack and stack[-1] is loop_state:
                stack.pop()

    def _run_forever(self, block_id: int, blocks: Dict[int, dict], connections: Dict[int, List[int]]):
        loop_state = {'blockId': block_id, 'type': 'forever', 'shouldBreak': False}
        stack = self._ensure_loop_stack()
        stack.append(loop_state)
        try:
            body = self._get_next(block_id, connections)
            while not self._stop_event.is_set() and not loop_state['shouldBreak']:
                if body:
                    self._execute_parallel(body, blocks, connections)
                self._sleep(0.01)
        finally:
            if stack and stack[-1] is loop_state:
                stack.pop()

    def _run_motor(self, block: dict):
        try:
            motor_id = int(block.get('motor_id') or 1)
        except (TypeError, ValueError):
            motor_id = 1
        if motor_id not in (1, 2):
            motor_id = 1

        speed = block.get('speed')
        if speed is None:
            speed = self._motor_speeds.get(motor_id, 100.0)
        try:
            speed = float(speed)
        except (TypeError, ValueError):
            speed = 100.0
        speed = max(1.0, min(6500.0, speed))
        self._motor_speeds[motor_id] = speed
        self._publish_motor_speed(motor_id, speed)

        try:
            steps = int(block.get('steps') or 0)
        except (TypeError, ValueError):
            steps = 0
        direction = block.get('direction')
        if direction == 'backward':
            steps = -abs(steps)
        elif direction == 'forward':
            steps = abs(steps)

        msg = Int32()
        msg.data = steps
        self.motor_cmd_pubs[motor_id].publish(msg)
        estimated = abs(steps) / speed if speed > 0 else 0.0
        timeout = max(60.0, estimated * 3.0)
        self._wait_motor_complete(motor_id, timeout)

    def _run_relay(self, block: dict):
        try:
            relay_id = int(block.get('relay_id') or 1)
        except (TypeError, ValueError):
            relay_id = 1
        state = block.get('state') or 'off'
        msg = String()
        msg.data = json.dumps({'relay_id': relay_id, 'state': state})
        self.relay_pub.publish(msg)

    def _publish_motor_speed(self, motor_id: int, speed: float):
        msg = Float32()
        msg.data = float(speed)
        self.motor_speed_pubs[motor_id].publish(msg)

    def _wait_motor_complete(self, motor_id: int, timeout_sec: float, poll: float = 0.1):
        deadline = time.monotonic() + timeout_sec
        move_started = False
        while not self._stop_event.is_set() and time.monotonic() < deadline:
            status = self._motor_status.get(motor_id) or {}
            remaining = status.get('steps_remaining')
            is_moving = status.get('is_moving')
            try:
                remaining = int(remaining) if remaining is not None else None
            except (TypeError, ValueError):
                remaining = None
            if remaining is not None and remaining > 0:
                move_started = True
            if is_moving:
                move_started = True
            if move_started and remaining == 0 and not is_moving:
                return
            if move_started and remaining == 0:
                return
            time.sleep(poll)

    def _sleep(self, seconds: float):
        end = time.monotonic() + max(0.0, seconds)
        while not self._stop_event.is_set() and time.monotonic() < end:
            time.sleep(min(0.05, end - time.monotonic()))


def main(args=None):
    rclpy.init(args=args)
    node = ProjectRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_execution(publish_estop=False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
