#!/usr/bin/env python3
"""ARM1 J7: position -10 deg, then repeat (+20 deg, -20 deg) five times at 20%.

REAL MOTION PROGRAM: running this file immediately requests hardware execution.
Right arm J7: position -10 deg, then (+20 deg, -20 deg) five times at 20%.
--preview is the only offline/no-connection mode. There is no menu.
Default execution: acquire live feedback and validate the ENTIRE sequence before moving.
The five cycles return to the positioned lower endpoint without accumulating drift.
The final endpoint is 10 deg below the execution start; no extra return is added.
Hard limits and a two-degree J7 margin remain enforced.
After one startup preflight, send the next segment immediately on the previous
controller completion result; no extra status polling, dwell or stable counters.
"""
import argparse
import importlib
import json
import math
import platform
from pathlib import Path
import shutil
import subprocess
import sys
import time
import uuid

SPEED_PERCENT = 20.0
ACCELERATION_PERCENT = 20.0

def log(event, **values):
    if event == 'EXPLICIT_REMOTE_EXECUTION':
        print('正在连接机器人……', flush=True)
        return
    if event == 'REAL_MOTION_START':
        print('已连接：右臂 J7，5 次往复，目标速度及加速度比例 20%。', flush=True)
        return
    if event == 'live_preflight':
        start, plan = values['start'], values['plan']
        angles = [s['target_joints_deg'][6] for s in plan]
        print(f"当前 J7：{start[6]:.3f}°；计划范围：{min(angles):.3f}°～{max(angles):.3f}°。", flush=True)
        return
    if event in ('segment_start', 'segment_verified'):
        cycle, phase = values['cycle'], values['phase']
        stage = '初始定位' if cycle == 0 else f"第 {cycle}/5 次" + (' 正向' if phase == 'forward' else ' 反向')
        if event == 'segment_start':
            print(f"{stage} → {values['target'][6]:.3f}°", flush=True)
        else:
            print(f"  运动完成回执：J7={values['joints'][6]:.3f}°", flush=True)
        return
    if event == 'motion_result':
        return  # The action's first result may arrive before measured velocity settles.
    if event == 'five_cycles_complete':
        print(f"5 次往复已完成，最终 J7={values['final_joints'][6]:.3f}°，保持当前位置。", flush=True)
        return
    if event == 'handoff':
        print('测试控制权已交还。', flush=True)
        return
    if event == 'stop_after_failure':
        print('异常停止右臂：' + ('成功。' if values['success'] else '未确认成功，请检查现场。'), flush=True)
        return
    if event in ('rejected_or_failed', 'cleanup_error'):
        print('未完成：' + str(values['message']), flush=True)
        return
    print(json.dumps({'event':event, **values},ensure_ascii=False),flush=True)

def load_robot_runtime():
    """Load optional, robot-side ROS modules only when live mode is selected."""
    try:
        ros = importlib.import_module('rclpy')
        actions = importlib.import_module('rclpy.action')
        services = importlib.import_module('gento_unified_interfaces.srv')
        interfaces = importlib.import_module('gento_unified_interfaces.action')
    except (ImportError, OSError) as exc:
        raise RuntimeError(
            '机器人 ROS 2 环境未加载。请先执行：source '
            '/home/nvidia/gento-test-baseline/gento-test-env.sh；'
            'Windows 只需标准 Python 和 SSH，无需安装这些 ROS 包。'
        ) from exc
    return ros, actions, services, interfaces

def execute_via_ssh():
    """Only called after an explicit --execute on Windows; no stored password."""
    ssh = shutil.which('ssh')
    if not ssh:
        raise RuntimeError('未找到 Windows OpenSSH 客户端 ssh.exe')
    source = Path(__file__).resolve().read_text(encoding='utf-8')
    command = [ssh, '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
               'gentoluna',
               'source /home/nvidia/gento-test-baseline/gento-test-env.sh '
               '&& exec python3 -']
    log('EXPLICIT_REMOTE_EXECUTION', host='gentoluna', full_path_check=True)
    result = subprocess.run(command, input=source, text=True, encoding='utf-8', check=False)
    if result.returncode:
        if result.returncode == 255:
            raise RuntimeError('SSH 连接失败，请检查机器人网络和 gentoluna 登录设置。')
        raise SystemExit(result.returncode)  # Robot already printed the concrete error.

REFERENCE = {'joints_deg': [-59.49, -60.847, 60.365, -113.859, 23.839, 35.713, 18.632], 'joint_limits_deg': [[-173.0, 173.0], [-118.0, 118.0], [-173.0, 173.0], [-145.0, 45.0], [-173.0, 173.0], [-60.0, 60.0], [-90.0, 90.0]], 'state': 'Position', 'error': 0, 'velocities_deg_s': [0.008, 0.041, 0.013, 0.021, 0.03, 0.007, 0.006], 'velocity_ratio': 5.0, 'acceleration_ratio': 5.0}

class Session:
    def __init__(self):
        rclpy, actions, services, interfaces = load_robot_runtime()
        self.ros = rclpy
        self.Lease = services.ManageHardwareConnection
        self.Status = services.GetRobotStatus
        self.Stop = services.StopRobot
        self.Motion = interfaces.ArmJointMotion
        rclpy.init()
        self.node = rclpy.create_node('right_arm_j7_bounded_test')
        self.owner = 'right-j7-' + uuid.uuid4().hex
        self.lease = self.node.create_client(self.Lease, '/gento/hardware/manage_connection')
        self.status = self.node.create_client(self.Status, '/gento/get_status')
        self.stop = self.node.create_client(self.Stop, '/gento/stop')
        self.action = actions.ActionClient(self.node, self.Motion, '/gento/arm/joint_motion')
        self.acquired = False
        self.renew_error = None
        self.renew_future = None
        self.goal_handle = None
        self.submitted = False
        self.completed = False
        self.timer = None

    def wait(self, future, timeout=15, check_lease=True):
        end = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() >= end:
                raise TimeoutError('ROS request timed out')
            if check_lease and self.renew_error:
                raise RuntimeError(self.renew_error)
            self.ros.spin_once(self.node, timeout_sec=0.1)
        return future.result()

    def call(self, client, request, check_lease=True):
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError('Required ROS service is unavailable')
        return self.wait(client.call_async(request), check_lease=check_lease)

    def lease_request(self, operation):
        req = self.Lease.Request()
        req.operation, req.client_id = operation, self.owner
        req.lease_seconds, req.stop_before_release = 30.0, False
        return req

    def acquire(self):
        inspected = self.call(self.lease, self.lease_request('inspect'))
        if inspected.owner_client_id and inspected.expires_in > 0:
            raise RuntimeError('Another client owns the controller; no takeover attempted')
        result = self.call(self.lease, self.lease_request('acquire'))
        if not result.success:
            raise RuntimeError(result.message)
        self.acquired = True
        self.timer = self.node.create_timer(2.0, self.renew)

    def renew(self):
        if self.renew_future is not None and not self.renew_future.done():
            return
        self.renew_future = self.lease.call_async(self.lease_request('renew'))
        def checked(future):
            try:
                if not future.result().success:
                    self.renew_error = 'Controller lease renewal failed'
            except Exception as exc:
                self.renew_error = str(exc)
        self.renew_future.add_done_callback(checked)

    def read(self, component='arm1'):
        request = self.Status.Request()
        request.component = component
        response = self.call(self.status, request)
        if not response.success:
            raise RuntimeError(response.message)
        return json.loads(response.status_json)

    def move(self, target):
        self.completed = False
        self.submitted = False
        self.goal_handle = None
        if not self.action.wait_for_server(timeout_sec=4):
            raise RuntimeError('Motion action server unavailable')
        goal = self.Motion.Goal()
        goal.execute, goal.arm, goal.relative = True, 1, False
        goal.joints_deg = target
        goal.planner_velocity = SPEED_PERCENT
        goal.planner_acceleration = ACCELERATION_PERCENT
        goal.frequency_hz, goal.timeout = 50, 60.0
        goal.max_joint_delta_deg, goal.reset_error = 20.5, False
        self.submitted = True
        self.goal_handle = self.wait(self.action.send_goal_async(goal))
        if not self.goal_handle.accepted:
            self.submitted = False
            raise RuntimeError('Motion goal rejected')
        result = self.wait(self.goal_handle.get_result_async(), timeout=70)
        if result.status != 4 or not result.result.success:
            raise RuntimeError(result.result.message)
        final = list(result.result.final_joints_deg)
        if len(final) != 7 or not all(math.isfinite(v) for v in final):
            raise RuntimeError('Invalid joint feedback in controller completion result')
        if max(abs(a-b) for a,b in zip(final,target)) > 0.5:
            raise RuntimeError('Controller completion result exceeds 0.5 degree tolerance')
        self.completed = True
        log('motion_result', success=True, target=list(result.result.target_joints_deg),
            final=final)
        return final

    def close(self):
        if self.timer:
            self.timer.cancel()
        if self.acquired:
            try:
                if self.submitted and not self.completed:
                    if self.goal_handle is not None:
                        self.goal_handle.cancel_goal_async()
                    request = self.Stop.Request()
                    request.component, request.execute, request.emergency = 'arm1', True, False
                    result = self.call(self.stop, request, check_lease=False)
                    log('stop_after_failure', success=result.success, message=result.message)
                result = self.call(self.lease, self.lease_request('handoff'), check_lease=False)
                if not result.success:
                    raise RuntimeError(result.message)
                log('handoff', success=True)
            except Exception as exc:
                log('cleanup_error', message=str(exc))
                raise
            finally:
                self.node.destroy_node()
                self.ros.shutdown()
        else:
            self.node.destroy_node()
            self.ros.shutdown()


def check_status(status, check_ratio=True):
    if status.get('state') not in ('IDLE','Position') or status.get('error') != 0:
        raise ValueError('Right arm must be IDLE/Position and error-free')
    if check_ratio and status.get('state') != 'Position':
        raise ValueError('20%程序要求右臂已处于 Position 模式并设置为20%倍率，避免网关进入模式时重新使用原5%配置。')
    for name in ('joints_deg','velocities_deg_s'):
        if len(status[name]) != 7 or not all(math.isfinite(float(v)) for v in status[name]):
            raise ValueError('Invalid seven-axis feedback')
    if max(abs(v) for v in status['velocities_deg_s']) > .5:
        raise ValueError('Arm has not settled below 0.5 deg/s')
    for key,expected in (('velocity_ratio',SPEED_PERCENT),('acceleration_ratio',ACCELERATION_PERCENT)):
        value=float(status[key])
        if check_ratio and (not math.isfinite(value) or abs(value-expected)>.1):
            label='速度' if key=='velocity_ratio' else '加速度'
            raise ValueError(f'控制器{label}倍率为 {value:g}%，程序要求 {expected:g}%。请先配置控制器倍率；本程序不修改倍率，也不会按旧倍率继续执行。')

def make_plan(status, check_ratio=True):
    check_status(status, check_ratio=check_ratio)
    start=[float(v) for v in status['joints_deg']]
    limits=status['joint_limits_deg']
    if len(limits)!=7:
        raise ValueError('Expected seven joint limit pairs')
    for angle,pair in zip(start,limits):
        low,high=map(float,pair)
        if not all(math.isfinite(v) for v in (low,high)) or not low<high or not low<=angle<=high:
            raise ValueError('Invalid current position or joint limits')
    result=[]
    offset=0.0
    sequence=[(0,'positioning',-10.0)]
    sequence.extend((cycle,phase,delta) for cycle in range(1,6)
                    for phase,delta in (('forward',20.0),('reverse',-20.0)))
    for cycle,phase,delta in sequence:
        offset+=delta
        target=start.copy()
        target[6]+=offset
        low,high=map(float,limits[6])
        within=low<=target[6]<=high
        margin=low+2<=target[6]<=high-2
        result.append(dict(cycle=cycle,phase=phase,delta_deg=delta,
                           target_joints_deg=target,within_limits=within,
                           margin_ok=margin))
    return start,result

def require_valid_plan(plan):
    for step in plan:
        if not step['within_limits'] or not step['margin_ok']:
            raise ValueError('Entire sequence rejected before motion: cycle %d %s J7=%.3f deg violates limits or 2-degree margin'
                             %(step['cycle'],step['phase'],step['target_joints_deg'][6]))

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preview',action='store_true',help='Offline preview only; do not connect or move')
    parser.add_argument('--check-environment',action='store_true',help='Check local execution mode without any robot connection')
    args=parser.parse_args()
    args.execute = not args.preview and not args.check_environment
    if args.check_environment:
        log('local_environment',system=platform.system(),offline_preview_supported=True,
            ssh_client_available=shutil.which('ssh') is not None,
            robot_connected=False,mode='Windows SSH launcher / robot ROS 2 backend')
        return
    if not args.execute:
        start,plan=make_plan(REFERENCE,check_ratio=False)
        log('OFFLINE_PREVIEW_NO_CONNECTION',historical_start=start,
            net_change_deg=-10,cycles=5,segments=len(plan),
            target_speed_percent=SPEED_PERCENT,target_acceleration_percent=ACCELERATION_PERCENT,
            controller_ratios_verified=False,
            j7_range_deg=[round(min(s['target_joints_deg'][6] for s in plan),3),
                          round(max(s['target_joints_deg'][6] for s in plan),3)])
        print('轮次  阶段       J7目标角度(°)',flush=True)
        phases={'positioning':'初始定位','forward':'正向旋转','reverse':'反向旋转'}
        for step in plan:
            print(f"{step['cycle']:>2}    {phases[step['phase']]}   {step['target_joints_deg'][6]:.3f}",flush=True)
        try:
            require_valid_plan(plan)
        except ValueError as exc:
            log('preview_blocked',executed=False,valid_for_execution=False,message=str(exc))
            return
        log('preview_valid',executed=False,path_valid=True,valid_for_execution=False,live_preflight_required=True)
        return
    if platform.system()=='Windows':
        print('实际运动：右臂 J7，先反向 10°，再正向/反向 20°往复 5 次，目标速度及加速度 20%。',flush=True)
        execute_via_ssh()
        return
    # SSH disconnection/termination must enter cleanup instead of abandoning a goal.
    import signal
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    log('REAL_MOTION_START', arm='ARM1', joint=7, cycles=5, speed_percent=SPEED_PERCENT)
    session=Session()
    try:
        session.acquire()
        current=session.read()
        start,plan=make_plan(current)
        log('live_preflight',start=start,plan=plan)
        require_valid_plan(plan)  # Must precede EVERY motion submission.
        final=start
        for step in plan:
            target=step['target_joints_deg']
            log('segment_start',cycle=step['cycle'],phase=step['phase'],target=target)
            final=session.move(target)
            log('segment_verified',cycle=step['cycle'],phase=step['phase'],joints=final)
        log('five_cycles_complete',final_joints=final)
    finally:
        session.close()

if __name__=='__main__':
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    try:
        main()
    except KeyboardInterrupt:
        log('interrupted')
        raise SystemExit(130)
    except Exception as exc:
        log('rejected_or_failed',message=str(exc))
        raise SystemExit(1)
