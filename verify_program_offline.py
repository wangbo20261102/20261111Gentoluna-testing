"""Offline checks only: ROS, SSH and real motion are never called."""
import contextlib
import copy
import importlib.util
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.dont_write_bytecode = True
path = Path(__file__).with_name('run_right_arm_5cycles.py')
source = path.read_text(encoding='utf-8')
compile(source, str(path), 'exec')
spec = importlib.util.spec_from_file_location('delivered_motion', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
state = copy.deepcopy(module.REFERENCE)
state['velocity_ratio'] = state['acceleration_ratio'] = 20.0
start, plan = module.make_plan(state)
module.require_valid_plan(plan)
assert len(plan) == 11
assert [step['delta_deg'] for step in plan] == [-10] + [20, -20] * 5
assert all(step['target_joints_deg'][:6] == start[:6] for step in plan)
assert abs(plan[-1]['target_joints_deg'][6] - (start[6] - 10)) < 1e-9
for position in (-78.01, 78.01):
    invalid = copy.deepcopy(state)
    invalid['joints_deg'][6] = position
    try:
        module.require_valid_plan(module.make_plan(invalid)[1])
    except ValueError:
        pass
    else:
        raise AssertionError('Out-of-range sequence was accepted')
try:
    module.make_plan(module.REFERENCE)  # Historical snapshot still has real 5% values.
except ValueError:
    pass
else:
    raise AssertionError('Live 5% feedback must not pass 20% preflight')

class FakeSession:
    def __init__(self, fail_at=None):
        self.reads, self.moves, self.closed, self.fail_at = 0, [], False, fail_at
    def acquire(self):
        pass
    def read(self):
        self.reads += 1
        assert self.reads == 1
        return copy.deepcopy(state)
    def move(self, target):
        self.moves.append(list(target))
        if len(self.moves) == self.fail_at:
            raise RuntimeError('Simulated controller failure')
        return list(target)
    def close(self):
        self.closed = True

for failure in (None, 4):
    session = FakeSession(failure)
    with patch.object(sys, 'argv', [str(path)]), patch.object(module.platform, 'system', return_value='Linux'), patch.object(module, 'Session', return_value=session), patch('signal.signal'), patch('signal.SIGHUP', 1, create=True), contextlib.redirect_stdout(io.StringIO()):
        try:
            module.main()
        except RuntimeError:
            assert failure is not None
        else:
            assert failure is None
    assert session.reads == 1 and session.closed
    assert len(session.moves) == (4 if failure else 11)

with patch.object(sys, 'argv', [str(path)]), patch.object(module.platform, 'system', return_value='Windows'), patch.object(module.shutil, 'which', return_value='ssh.exe'), patch.object(module.subprocess, 'run', return_value=SimpleNamespace(returncode=0)) as remote, contextlib.redirect_stdout(io.StringIO()):
    module.main()
assert remote.call_count == 1 and 'gentoluna' in remote.call_args.args[0]

output = io.StringIO()
with patch.object(sys, 'argv', [str(path), '--preview']), patch.object(module, 'Session', side_effect=AssertionError('Unexpected hardware session')), patch.object(module.subprocess, 'run', side_effect=AssertionError('Unexpected SSH connection')), contextlib.redirect_stdout(output):
    module.main()
assert json.loads(output.getvalue().splitlines()[-1])['executed'] is False

session = object.__new__(module.Session)
session.Motion = SimpleNamespace(Goal=SimpleNamespace)
target = plan[0]['target_joints_deg']
handle = SimpleNamespace(accepted=True, get_result_async=lambda: None)
receipt = SimpleNamespace(status=4, result=SimpleNamespace(success=True, target_joints_deg=target, final_joints_deg=target))
session.action = SimpleNamespace(wait_for_server=lambda **kwargs: True, send_goal_async=Mock())
session.wait = Mock(side_effect=[handle, receipt])
with contextlib.redirect_stdout(io.StringIO()):
    assert session.move(target) == target
goal = session.action.send_goal_async.call_args.args[0]
assert goal.arm == 1 and goal.execute
assert goal.planner_velocity == goal.planner_acceleration == 20.0
assert not hasattr(module, 'wait_settled')
print('PASS: five cycles; 20% parameters; full-path limits; single startup read; no added dwell; error interruption; Windows dispatch mocked; offline preview. No robot connection or motion.')
