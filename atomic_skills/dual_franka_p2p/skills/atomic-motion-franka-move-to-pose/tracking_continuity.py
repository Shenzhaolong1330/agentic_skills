"""References tied to exact acknowledged robot state.

No command is sent here. A receipt cannot reset compensation, change caps or
authorize motion. Fresh health, pose and normal transport checks remain required.
"""
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import tempfile

ENV = 'FRANKA_P2P_CONTINUITY_FILE'
ATTRIBUTE = '_confirmed_p2p_reference'


def verified_interruption(previous, progress, failure, *, session, count, saved,
                          measured, config, nominal_rotation, geometry):
    """Reconcile a fully acknowledged timeout, without restarting any motion.

    For explicit recovery under the same arms lease only. The caller must read
    fresh stationary healthy feedback and retain the original failed action.
    An incomplete log, unknown RPC outcome, changed count/session/target, or
    invalid reference cannot create a receipt. This is not task completion.
    """
    commands = failure.get('confirmed_commands')
    if (failure.get('event') != 'p2p_servo_failed' or failure.get('phase') != 'servo'
            or not failure.get('message', '').startswith('External tracking budget exhausted;')
            or type(commands) is not int or commands <= 0
            or failure.get('dispatch_attempts') != commands
            or progress.get('event') != 'p2p_servo_progress'
            or progress.get('commands') != commands or progress.get('step_count') != count
            or previous.get('server_session_id') != session
            or type(previous.get('step_count')) is not int
            or previous['step_count'] + commands != count):
        raise ValueError('Interrupted motion lacks a complete acknowledged command chain')
    records = {}
    if set(previous.get('arms', {})) != set(saved):
        raise ValueError('Interrupted motion arm set changed')
    for side in saved:
        update = progress.get('servo', {}).get(side)
        if update is not None:
            recorded_saved = geometry.Pose(**update['command'])
            reference_data = progress['tracking'][side]['reference']
            if reference_data != failure.get('tracking', {}).get(side, {}).get('reference'):
                raise ValueError('Interrupted reference does not match final failure')
            reference = geometry.Pose(**reference_data)
        else:
            recorded_saved = geometry.Pose(**previous['arms'][side]['saved'])
            data = previous['arms'][side]['reference']
            reference = geometry.Pose(**data) if data else None
        gap = geometry._error(recorded_saved, saved[side])
        if gap['translation_m'] > 1e-6 or gap['rotation_rad'] > 1e-6:
            raise ValueError('Fresh saved command differs from acknowledged interruption')
        records[side] = {'saved': asdict(saved[side]),
                         'reference': asdict(reference) if reference else None}
    receipt = {'version': 1, 'kind': 'verified_interruption', 'task_completed': False,
               'automatic_retry': False, 'server_session_id': session,
               'step_count': count, 'arms': records,
               'accepted_rotation_tolerance_rad': min(config.rotation_tolerance_rad, nominal_rotation)}
    references = matching_references(receipt, session=session, count=count, saved=saved,
        measured=measured, config=config, nominal_rotation=nominal_rotation, geometry=geometry)
    active = set(progress.get('servo', {}))
    if not active or not active.issubset(references):
        raise ValueError('Interrupted reference exceeds fresh measured tolerance or compensation cap')
    return receipt


def read(client):
    receipt = getattr(client, ATTRIBUTE, None)
    path = os.environ.get(ENV)
    if receipt is None and path:
        p = Path(path)
        if p.is_symlink():
            raise ValueError('P2P continuity receipt must not be a symlink')
        if p.exists():
            if p.stat().st_size > 65536:
                raise ValueError('P2P continuity receipt is oversized')
            receipt = json.loads(p.read_text())
    return receipt if isinstance(receipt, dict) else {}


def matching_references(receipt, *, session, count, saved, measured, config, nominal_rotation, geometry):
    if (receipt.get('version') != 1 or receipt.get('server_session_id') != session
            or type(receipt.get('step_count')) is not int or receipt['step_count'] != count):
        return {}
    records = receipt.get('arms', {})
    if not isinstance(records, dict) or set(records) != set(saved):
        return {}
    result = {}
    tolerance = reference_tolerance(receipt, config, nominal_rotation)
    for side in saved:
        item = records[side]
        previous = geometry.Pose(**item['saved'])
        gap = geometry._error(previous, saved[side])
        if gap['translation_m'] > 1e-6 or gap['rotation_rad'] > 1e-6:
            return {}  # Both arms must match, including the stationary arm.
        if item.get('reference') is None:
            continue
        reference = geometry.Pose(**item['reference'])
        if (geometry._error(reference, saved[side])['rotation_rad'] <= config.max_rotation_bias_rad + 4e-12
                and geometry._error(reference, measured[side])['rotation_rad'] <= tolerance):
            result[side] = reference.quaternion_xyzw
    return result


def reference_tolerance(receipt, config, nominal_rotation):
    accepted = receipt.get('accepted_rotation_tolerance_rad', nominal_rotation)
    if type(accepted) not in (int, float) or not math.isfinite(accepted) or accepted <= 0:
        raise ValueError('Invalid acknowledged arrival tolerance')
    # This binds the START reference to the preceding successful call. A later
    # precision stage may tighten its ENDPOINT without invalidating the old
    # compensation centre. Legacy receipts have no such source-bound tolerance.
    return (accepted if 'accepted_rotation_tolerance_rad' in receipt
            else min(config.rotation_tolerance_rad, accepted))


def write(client, receipt):
    path = os.environ.get(ENV)
    if path:
        p = Path(path)
        if p.is_symlink():
            raise ValueError('P2P continuity receipt must not be a symlink')
        # The console provides a run-owned parent under its arms lease. Never
        # create a global state directory or import a previous run implicitly.
        fd, temporary = tempfile.mkstemp(prefix='.p2p-reference-', dir=p.parent)
        try:
            with os.fdopen(fd, 'w') as stream:
                json.dump(receipt, stream, allow_nan=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, p)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
    setattr(client, ATTRIBUTE, receipt)


def after_gripper(client, receipt, response, geometry):
    """Advance a success receipt across one confirmed gripper-only RPC.

    Gripper commands increment the server's shared count. Never carry a receipt
    across unknown commands or a changed saved arm target.
    """
    if (not receipt or type(receipt.get('step_count')) is not int
            or not isinstance(response, dict) or response.get('ok') is False
            or response.get('error') or response.get('done') is True
            or (response.get('info') or {}).get('error')
            or response.get('step_count') != receipt['step_count'] + 1):
        return
    observation = client.get_observation()
    state = observation.get('command_target_state', {})
    if (state.get('available') is not True or state.get('server_session_id') != receipt.get('server_session_id')
            or state.get('step_count') != response['step_count']):
        return
    records = receipt.get('arms', {})
    if set(records) != {'left_arm', 'right_arm'}:
        return
    for side, item in records.items():
        actual = state.get('arms', {}).get(side, {})
        if actual.get('frame') != 'torso':
            return
        error = geometry._error(geometry.Pose(**item['saved']),
            geometry.Pose(actual.get('position_m'), actual.get('quaternion_xyzw')))
        if error['translation_m'] > 1e-6 or error['rotation_rad'] > 1e-6:
            return
    write(client, {**receipt, 'step_count': state['step_count']})
