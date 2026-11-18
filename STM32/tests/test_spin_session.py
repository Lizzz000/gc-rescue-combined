import math
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/live_tuning'))
from spin_session import SPIN_NAMES,make_plan,check_complete,check_sample,new_spin_running,stop
from uart_telemetry import FrameDecoder
import swd_session as swd


class SpinSessionTest(unittest.TestCase):
    @staticmethod
    def sample(turns=30):
        sample=dict.fromkeys(SPIN_NAMES,0.)
        sample.update(tick_ms=1000,phase=4,enabled=1,odom_valid=1,odom_age_ms=4,
            state_1=1,state_2=1,state_3=1,end_stopped=1,end_valid=1,
            spin_stage=4,spin_yaw_rad=turns*math.tau,spin_encoder_rad=turns*math.tau,spin_target_rad=turns*math.tau)
        return sample

    def test_shape_and_yaw_priority(self):
        self.assertEqual(len(SPIN_NAMES),64)
        for turns in (30,35):
            plan=make_plan(turns)
            self.assertEqual(len(plan['stages_rad_s'])*plan['turns_per_stage'],turns)
            self.assertEqual(plan['counter'],'AHRS_YAW')
            check_complete(self.sample(turns),[0,0],turns)
        with self.assertRaises(swd.SessionError):
            check_complete(self.sample(30),[0,0],35)
        with self.assertRaises(ValueError):
            make_plan(34)
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

    def test_stop_checks_ack_and_new_frames_in_either_layout(self):
        s=self.sample(35)
        s.update(enabled=0, state_1=0, state_2=0, state_3=0)
        decoder=FrameDecoder(swd.SAMPLE_NAMES, SPIN_NAMES)
        data=bytearray()
        for tick, names in ((100,swd.SAMPLE_NAMES),(140,SPIN_NAMES),(180,SPIN_NAMES)):
            s['tick_ms']=tick
            data.extend(struct.pack(f'<{len(names)}f',*(s[k] for k in names))+FrameDecoder.TAIL)
        frames=decoder.feed(data)
        self.assertEqual([s['tick_ms'] for s in frames], [100,140,180])
        session=Mock(); session.publish.return_value=4
        session.read.side_effect=[3,4,4]
        telemetry=Mock(); telemetry.latest.side_effect=frames
        stop(session,telemetry)
        telemetry.set_names.assert_called_once_with(swd.SAMPLE_NAMES,SPIN_NAMES)
        self.assertEqual(telemetry.latest.call_count,3)

if __name__=='__main__':unittest.main()
