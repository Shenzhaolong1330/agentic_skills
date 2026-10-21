import importlib.util,io,json,os,unittest
from pathlib import Path
from unittest.mock import Mock,patch
from contextlib import redirect_stdout
import numpy as np
P=Path(__file__).resolve().parents[1]/'task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/place_held_object_on_table.py'
s=importlib.util.spec_from_file_location('phase_place',P);m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
class PhaseTests(unittest.TestCase):
 def run_failure(self,phase):
  client=Mock();rpc=Mock();rpc.DualFrankaRobotiqRpcClient.return_value=client
  def move(*a,**k):
   if k['stage']==phase:raise RuntimeError('force guard')
   return {'stage':k['stage'],'result':{'ok':True}}
  if phase=='open_gripper':client.open_gripper.side_effect=TimeoutError('unknown ACK')
  out=io.StringIO()
  with patch.dict(os.environ,FRANKA_P2P_CONTROLLER='external_tracking'),patch.object(m,'_load_rpc_module',return_value=rpc),patch.object(m,'_pose_from_observation',return_value=np.array([.5,.1,.1,3.,0.,0.])),patch.object(m,'_move',side_effect=move),redirect_stdout(out):
   with self.assertRaises((RuntimeError,TimeoutError)):m.main(['--side','right','--place-xyz','[0.4,0.2,0]','--execute','--keep-current-rotation','--retract-distance-m','0'])
  return json.loads(out.getvalue()),client
 def test_motion_failures_preserve_stage_and_never_release(self):
  for phase in ['move_xy_keep_current_z','move_to_release_xyz']:
   with self.subTest(phase=phase):
    r,c=self.run_failure(phase);self.assertEqual(r['failure_stage'],phase);self.assertEqual(r['release_dispatch_state'],'not_dispatched');self.assertFalse(r['completion_flag']);self.assertFalse(r['physical_verified']);c.open_gripper.assert_not_called();c.go_home.assert_not_called()
 def test_missing_open_ack_is_attempted_never_claimed_not_dispatched(self):
  r,c=self.run_failure('open_gripper');self.assertEqual(r['release_dispatch_state'],'attempted');self.assertEqual(r['failure_stage'],'open_gripper');c.open_gripper.assert_called_once()
