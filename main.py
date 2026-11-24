#!/usr/bin/env python3
"""High Piper green plant search and supervised navigation."""
import argparse
from contextlib import contextmanager
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import numpy as np
from search_geometry import approach, checked_matrix, mean_region_point, plant_map, rigid

ROOT = Path(__file__).resolve().parent
HOME = Path.home()
DRIVER = '/piper_test/piper_basic_grasp'
CAN = 'can_piper_high'


def save(path, data):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')
    temp.replace(path)


@contextmanager
def hardware_lock():
    path = HOME / '.local/state/vlm-jetson/hardware.lock'
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a') as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('HARDWARE_BUSY: exit other execution scripts')
        yield


def joints(cfg, key):
    from vlm_jetson.execution.arm.controller import joints_valid
    value = cfg.get(key)
    if (not isinstance(value, list) or any(type(x) not in (int, float) or not math.isfinite(x)
                                         for x in value) or not joints_valid(value)):
        raise ValueError('RECORD_OR_CHECK_' + key.upper())
    return value


def mount_matrix(cfg):
    xyz, rpy = (cfg[k] for k in ('arm_base_in_base_link_xyz_m', 'arm_base_in_base_link_rpy_deg'))
    if xyz is None or rpy is None:
        raise ValueError('SET_HIGH_ARM_TO_BASE_LINK_EXTRINSIC')
    for v in (xyz, rpy):
        if not isinstance(v, list) or len(v) != 3 or any(type(x) not in (int, float) or not math.isfinite(x) for x in v):
            raise ValueError('INVALID_HIGH_ARM_TO_BASE_LINK_EXTRINSIC')
    return checked_matrix(rigid(xyz, rpy))


class Search:
    def __init__(self, cfg, nav, directory, cancel, read_only=False):
        import rospy
        from fake_harvest_skills import ROSArm
        from piper_feedback_compat import detect_feedback_layout
        from ros_motion_smoke import Monitor, healthy
        self.ros, self.cfg, self.nav, self.cancel = rospy, cfg, nav, cancel
        self.directory, self.healthy = directory, healthy
        if not read_only and rospy.get_param(DRIVER + '/can_port', None) != CAN:
            raise RuntimeError('HIGH_ARM_DRIVER_REQUIRED: ' + CAN)
        offset, _, _ = detect_feedback_layout(CAN)
        self.monitor = Monitor(directory / 'can.jsonl', can_interface=CAN, feedback_offset=offset)
        try:
            self.arm = None if read_only else ROSArm(self.monitor, self.log)
        except BaseException:
            self.monitor.close()
            raise

    def log(self, event, **items):
        line = json.dumps(dict(event=event, time=time.time(), **items), allow_nan=False)
        print(line, flush=True)
        with (self.directory / 'events.jsonl').open('a') as f:
            f.write(line + '\n')

    def stopped_arm(self, require_can=True):
        end = time.monotonic() + 3.
        while True:
            if self.cancel.is_set():
                raise RuntimeError('CANCELLED')
            try:
                state = self.monitor.state()
                break
            except RuntimeError:
                if time.monotonic() >= end:
                    raise
                time.sleep(.05)
        self.healthy(state, require_can=require_can)
        if self.arm is None:
            # Recording needs only fresh, stable CAN telemetry.
            for _ in range(8):
                time.sleep(.06)
                new = self.monitor.state()
                self.healthy(new, require_can=False)
                if (self.cancel.is_set() or new['stamp'] <= state['stamp'] or
                        max(abs(a-b) for a,b in zip(new['joints'],state['joints'])) > .1):
                    raise RuntimeError('ARM_FEEDBACK_NOT_STATIONARY')
                state = new
        else:
            state = self.arm.stationary()
        self.healthy(state, require_can=require_can)
        if require_can and (state['motion_status'] != 0 or not all(state['enabled'])):
            raise RuntimeError('ARM_NOT_READY')
        return state

    def move_arm(self, target):
        self.stopped_arm()
        until = time.monotonic() + 3.
        while not self.arm.jp.get_num_connections():
            if self.cancel.is_set() or time.monotonic() > until:
                raise RuntimeError('ARM_COMMAND_SUBSCRIBER_MISSING_OR_CANCELLED')
            time.sleep(.05)
        speed = self.cfg['arm_speed_percent']
        if type(speed) is not int or not 1 <= speed <= 10:
            raise ValueError('INVALID_ARM_SPEED_PERCENT')
        status, code, effect = self.arm.move_joints(target, speed, 20000, self.cancel)
        self.log('arm_result', status=status, code=code, effect=effect)
        if status != 'SUCCEEDED' or effect.get('stopped') is not True:
            raise RuntimeError('ARM_MOVE_FAILED: {}'.format(code))
        return self.stopped_arm()

    def stationary_base(self):
        from nav_msgs.msg import Odometry
        m = self.ros.wait_for_message('/ranger_base_node/odom', Odometry, timeout=2.)
        v, w = m.twist.twist.linear, m.twist.twist.angular
        age = (self.ros.Time.now() - m.header.stamp).to_sec()
        if (m.header.frame_id != 'odom' or m.child_frame_id != 'base_link' or not 0 <= age <= .5 or
                not math.hypot(v.x, v.y) <= .01 or not abs(w.z) <= .02):
            raise RuntimeError('BASE_NOT_STATIONARY_OR_ODOM_STALE')
        p, q = m.pose.pose.position, m.pose.pose.orientation
        values = np.array([p.x, p.y, q.z, q.w])
        if not np.isfinite(values).all():
            raise RuntimeError('INVALID_ODOM')
        return values

    def map_state(self):
        from nav_msgs.msg import OccupancyGrid
        ros = self.ros
        if ros.get_param('/vlm_navigation/map_id', None) != self.nav['map_id']:
            raise RuntimeError('MAP_ID_MISMATCH')
        # Melodic provides its compiled TF bindings for system Python 2.
        try:
            raw = subprocess.check_output(['/usr/bin/python2', str(ROOT / 'read_map_pose.py'),
                                           str(self.nav['feedback_max_age_s'])],
                                          stderr=subprocess.PIPE, timeout=5.)
        except subprocess.TimeoutExpired:
            raise RuntimeError('MAP_TF_TIMEOUT')
        except subprocess.CalledProcessError as exc:
            raise RuntimeError(exc.stderr.decode('utf-8', errors='replace').strip()
                               or 'MAP_TF_UNAVAILABLE')
        pose, stamp = json.loads(raw)
        grid = ros.wait_for_message('/map', OccupancyGrid, timeout=2.)
        if grid.header.frame_id != 'map' or len(grid.data) != grid.info.width * grid.info.height or not grid.data:
            raise RuntimeError('INVALID_MAP')
        origin = grid.info.origin
        meta = [grid.info.width, grid.info.height, grid.info.resolution,
                origin.position.x, origin.position.y, origin.position.z,
                origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w]
        signature = hashlib.sha256(json.dumps(meta).encode() + np.asarray(grid.data, dtype=np.int8).tobytes()).hexdigest()
        return pose, signature, stamp, grid

    def capture(self, preview_only, auto=False):
        import cv2
        from locate_color_target import ColorTargetLocator, draw_location
        if not os.environ.get('DISPLAY'):
            raise RuntimeError('USE_GRAPHICAL_TERMINAL')
        handeye = checked_matrix(np.load(str(Path(self.cfg['handeye_path']).expanduser()), allow_pickle=False))
        mount = None if preview_only else mount_matrix(self.cfg)
        target = joints(self.cfg, 'observation_joints_deg')
        odom = self.stationary_base()
        if not preview_only:
            self.map_state()
        self.move_arm(target)
        camera = ColorTargetLocator(self.cfg)
        window = 'Tomato search: automatic capture / Q quit' if auto else 'Tomato search: S save / Q quit'
        deadline = time.monotonic() + 30. if auto else math.inf
        sample_window_s = 2.0
        min_samples = 3
        points = []
        started = None
        estimate = None
        last_stamp = None
        try:
            while not self.cancel.is_set():
                if time.monotonic() >= deadline:
                    raise RuntimeError('PLANT_CAPTURE_TIMEOUT: insufficient valid depth samples')
                bgr, depth, result = camera.read()
                frame = draw_location(bgr, result)
                stamp = result['camera_timestamp_s']
                now = time.monotonic()
                if not preview_only and result['valid'] and (last_stamp is None or stamp > last_stamp):
                    limits = self.cfg['camera']
                    mask = (result['mask'] > 0) & (camera._color_mask(bgr) > 0)
                    sample = mean_region_point(depth, mask, camera.intrinsics,
                                               limits['min_depth_m'], limits['max_depth_m'])
                    if sample is not None:
                        if started is None:
                            started = now
                        points.append(sample)
                if last_stamp is None or stamp > last_stamp:
                    last_stamp = stamp
                if started is not None and now - started >= sample_window_s:
                    if len(points) >= min_samples:
                        estimate = (np.mean(points, axis=0), len(points), now)
                    points.clear()
                    started = None
                if estimate is not None and now - estimate[2] > sample_window_s:
                    estimate = None
                if preview_only:
                    text = 'Preview only'
                elif estimate is not None:
                    text = 'Mean depth {:.3f} m | {} frames | {}'.format(
                        estimate[0][2], estimate[1], 'saving' if auto else 'S save')
                else:
                    text = 'Sampling plant depth | {} valid frames'.format(len(points))
                cv2.putText(frame, text, (12, 108), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 2)
                cv2.imshow(window, frame)
                key = cv2.waitKey(1) & 255
                if key in (27, ord('q'), ord('Q')) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                    return
                if preview_only or estimate is None or (not auto and key not in (ord('s'), ord('S'))):
                    continue
                rgbd, sample_count, _ = estimate
                try:
                    if np.linalg.norm(self.stationary_base() - odom) > .008:
                        raise RuntimeError('BASE_MOVED_DURING_CAPTURE')
                    arm = self.stopped_arm()
                    if max(abs(a-b) for a,b in zip(arm['joints'], target)) > .3:
                        raise RuntimeError('OBSERVATION_POSE_CHANGED')
                    base, signature, stamp, _ = self.map_state()
                    if np.linalg.norm(self.stationary_base() - odom) > .008:
                        raise RuntimeError('BASE_MOVED_DURING_CAPTURE')
                    after = self.monitor.state()
                    self.healthy(after)
                    if after['motion_status'] != 0 or max(abs(a-b) for a,b in zip(arm['joints'], after['joints'])) > .15:
                        raise RuntimeError('OBSERVATION_POSE_CHANGED')
                    point = plant_map(rgbd, arm['pose'], handeye, mount, base)
                    data = dict(map_id=self.nav['map_id'], map_signature=signature, map_tf_stamp=stamp,
                                saved_at=time.time(), camera_point_m=rgbd.tolist(), arm_pose_m_deg=arm['pose'],
                                mean_depth_m=float(rgbd[2]), depth_sample_frames=sample_count,
                                base_pose=base, plant_map_m=point.tolist(),
                                suggested_goal=approach(base, point, self.cfg['approach_standoff_m']))
                    save(self.directory / 'target.json', data)
                    self.log('plant_saved', path=str(self.directory / 'target.json'), **data)
                    return self.directory / 'target.json'
                except (ValueError, RuntimeError, self.ros.ROSException, self.ros.ServiceException) as exc:
                    self.log('capture_rejected', error=str(exc))
                    if auto:
                        raise
                    estimate = None
                    points.clear()
                    started = None
        finally:
            try:
                camera.stop()
            finally:
                cv2.destroyAllWindows()

    def check_navigation_adapter(self):
        if not Path(self.nav['socket_path']).is_socket():
            raise RuntimeError('NAVIGATION_ADAPTER_SOCKET_MISSING')

    def run_pipeline(self):
        joints(self.cfg, 'travel_joints_deg')
        self.check_navigation_adapter()
        path = self.capture(preview_only=False, auto=True)
        if path is not None and not self.cancel.is_set():
            self.navigate(path)

    def navigate(self, path):
        from vlm_jetson.execution.navigation.client import NavigationController
        from vlm_jetson.execution.skills import SkillExecutionError
        data = json.loads(path.read_text())
        travel = joints(self.cfg, 'travel_joints_deg')

        def check_target(base, signature, grid):
            if (data['map_id'] != self.nav['map_id'] or data['map_signature'] != signature or
                    not 0 <= time.time() - data['saved_at'] <= self.cfg['record_max_age_s']):
                raise RuntimeError('MAP_CHANGED_OR_TARGET_EXPIRED: recapture')
            goal = approach(base, data['plant_map_m'], self.cfg['approach_standoff_m'])
            if goal is None:
                raise RuntimeError('ALREADY_NEAR_PLANT: no motion requested')
            origin = grid.info.origin
            q = origin.orientation
            if max(abs(q.x), abs(q.y), abs(q.z)) > 1e-6 or not abs(abs(q.w)-1) < 1e-6:
                raise RuntimeError('UNSUPPORTED_ROTATED_MAP_ORIGIN')
            x = int(math.floor((goal[0] - origin.position.x) / grid.info.resolution))
            y = int(math.floor((goal[1] - origin.position.y) / grid.info.resolution))
            if not (0 <= x < grid.info.width and 0 <= y < grid.info.height) or grid.data[y*grid.info.width+x] != 0:
                raise RuntimeError('APPROACH_CELL_NOT_FREE')
            return goal

        self.check_navigation_adapter()
        odom = self.stationary_base()
        base, signature, _, grid = self.map_state()
        candidate = check_target(base, signature, grid)
        self.log('approach_proposed', plant_map_m=data['plant_map_m'], goal=candidate)
        if input('Check the arm path and approach in RViz. Type GO to stow and navigate: ').strip() != 'GO':
            return
        if self.cancel.is_set() or np.linalg.norm(self.stationary_base() - odom) > .008:
            raise RuntimeError('CANCELLED_OR_BASE_MOVED')
        self.move_arm(travel)
        self.check_navigation_adapter()
        base, signature, _, grid = self.map_state()
        goal = check_target(base, signature, grid)
        if np.linalg.norm(np.asarray(goal[:2]) - candidate[:2]) > .03 or abs(math.atan2(
                math.sin(goal[2]-candidate[2]), math.cos(goal[2]-candidate[2]))) > .05:
            raise RuntimeError('APPROACH_CHANGED_AFTER_CONFIRMATION')
        if self.cancel.is_set() or np.linalg.norm(self.stationary_base() - odom) > .008:
            raise RuntimeError('CANCELLED_OR_BASE_MOVED')
        try:
            status, code, effect = NavigationController(self.nav).execute(
                dict(map_id=self.nav['map_id'], target_pose=goal), 180000, self.cancel)
        except SkillExecutionError as exc:
            self.log('navigation_uncertain', error=str(exc))
            raise
        self.log('navigation_result', status=status, code=code, effect=effect)
        if status != 'SUCCEEDED' or effect.get('stopped') is not True:
            raise RuntimeError('NAVIGATION_NOT_CONFIRMED: {}'.format(code))


def camera_preview(cfg, check=False):
    import cv2
    from locate_color_target import ColorTargetLocator, draw_location
    if not check and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        raise RuntimeError('USE_GRAPHICAL_TERMINAL')
    camera = ColorTargetLocator(cfg)
    window = 'Piper High D435i | Q / Esc'
    try:
        if check:
            bgr, _, _ = camera.read()
            print('CAMERA_OK: {}x{}'.format(bgr.shape[1], bgr.shape[0]))
            return
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        while True:
            bgr, _, result = camera.read()
            cv2.imshow(window, np.hstack((bgr, draw_location(bgr, result))))
            if (cv2.waitKey(1) & 255 in (27, ord('q'), ord('Q')) or
                    cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1):
                break
    finally:
        camera.stop()
        if not check:
            cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config.json')
    sub = parser.add_subparsers(dest='mode')
    preview = sub.add_parser('camera-preview', help='show live RGB and green detection without ROS or motion')
    preview.add_argument('--check', action='store_true', help='read one frame without opening a window')
    teach = sub.add_parser('record-pose')
    teach.add_argument('pose', choices=['observation', 'travel'])
    for name in ('capture', 'navigate', 'run'):
        cmd = sub.add_parser(name)
        cmd.add_argument('--execute', action='store_true', required=True)
        if name == 'capture':
            cmd.add_argument('--preview-only', action='store_true')
        elif name == 'navigate':
            cmd.add_argument('target', type=Path)
    args = parser.parse_args()
    if args.mode is None:
        parser.error('select a subcommand')
    cfg = json.loads(args.config.read_text())
    if cfg['camera']['serial'] != '317622073937':
        raise ValueError('HIGH_D435I_REQUIRED')
    sys.path[:0] = [str(HOME / p) for p in ('vlm-jetson', 'piper_ws/src/piper_ros/local_tools',
                     'piper_ws/src/piper_ros/src/piper/scripts', 'robot_project/VLM-to-Robot/visual_servo')]
    if args.mode == 'camera-preview':
        with hardware_lock():
            camera_preview(cfg, args.check)
        return
    nav = json.loads(Path(cfg['navigation_config']).expanduser().read_text())
    import rospy
    rospy.init_node('tomato_search', anonymous=True, disable_signals=True)
    cancel = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: cancel.set())
    with hardware_lock():
        directory = ROOT/'runs'/str(int(time.time()*1e6))
        directory.mkdir(parents=True)
        search = Search(cfg, nav, directory, cancel, read_only=args.mode == 'record-pose')
        try:
            if args.mode == 'record-pose':
                state = search.stopped_arm(require_can=False)
                key = args.pose + '_joints_deg'
                cfg[key] = state['joints']
                cfg[args.pose + '_pose_m_deg'] = state['pose']
                joints(cfg, key)
                save(args.config, cfg)
                search.log('pose_saved', name=args.pose, joints_deg=state['joints'], pose_m_deg=state['pose'])
            elif args.mode == 'capture':
                search.capture(args.preview_only)
            elif args.mode == 'run':
                search.run_pipeline()
            else:
                search.navigate(args.target)
        finally:
            search.monitor.close()


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        sys.exit(str(error))
