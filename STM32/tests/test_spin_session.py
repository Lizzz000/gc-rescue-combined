import math
from pathlib import Path
import struct
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/live_tuning'))
from spin_session import SPIN_NAMES,PLAN,check_complete,check_sample,new_spin_running
from uart_telemetry import FrameDecoder
import swd_session as swd


class SpinSessionTest(unittest.TestCase):
    @staticmethod
    def sample():
        sample=dict.fromkeys(SPIN_NAMES,0.)
        sample.update(tick_ms=1000,phase=4,enabled=1,odom_valid=1,odom_age_ms=4,
            state_1=1,state_2=1,state_3=1,end_stopped=1,end_valid=1,
            spin_stage=4,spin_yaw_rad=30*math.tau,spin_encoder_rad=30*math.tau,spin_target_rad=30*math.tau)
        return sample

    def test_shape_and_yaw_priority(self):
        self.assertEqual(len(SPIN_NAMES),64)
        self.assertEqual(len(PLAN['stages_rad_s'])*PLAN['turns_per_stage'],30)
        self.assertEqual(PLAN['counter'],'AHRS_YAW')
        s=self.sample();s['spin_encoder_rad']=30*math.tau+.4
        check_complete(s,[0,0]) # 轮角偏差只作诊断，以YAW判断圈数。
        data=struct.pack('<64f',*(s[k] for k in SPIN_NAMES))+FrameDecoder.TAIL
        decoder=FrameDecoder(SPIN_NAMES)
        self.assertEqual(decoder.feed(data[:217]),[])
        result=decoder.feed(data[217:])[0]
        self.assertAlmostEqual(result['spin_yaw_rad'],s['spin_yaw_rad'],places=4)

    def test_faults_and_endpoint_rejected(self):
        for field,value in [('error',5),('enabled',0),('state_2',0),('target_3',1),
                            ('odom_valid',0),('odom_age_ms',51),('end_stopped',0),
                            ('spin_yaw_rad',30*math.tau-.3),('odom_x_m',.36)]:
            s=self.sample();s[field]=value
            with self.subTest(field=field),self.assertRaises(swd.SessionError):
                check_complete(s,[0,0])

    def test_old_done_frame_cannot_complete_new_request(self):
        s=self.sample();s['spin_yaw_rad']=33*math.tau
        self.assertFalse(new_spin_running(s))
        s['phase']=2
        with self.assertRaises(swd.SessionError):new_spin_running(s)
        s['spin_yaw_rad']=.005
        self.assertTrue(new_spin_running(s))

if __name__=='__main__':unittest.main()
