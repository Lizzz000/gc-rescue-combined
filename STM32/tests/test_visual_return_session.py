import math
from pathlib import Path
import sys
import struct
import unittest
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"tools/live_tuning"))
from visual_return_session import image_center
from uart_telemetry import FrameDecoder, visual_snapshot
import swd_session as swd


class UartFrames(unittest.TestCase):
    @staticmethod
    def frame(tick, phase=2.):
        values=[0.]*len(swd.SAMPLE_NAMES)
        values[0:4]=[tick,phase,0.,1.]
        values[40:43]=[1.2,.6,1.]
        values[46:48]=[1.,5.]
        return struct.pack('<56f',*values)+FrameDecoder.TAIL

    def test_fragmentation_resync_and_newest_frame(self):
        decoder=FrameDecoder()
        data=b'partial-old-frame'+FrameDecoder.TAIL+self.frame(100.)+self.frame(140.)
        found=[]
        for i in range(0,len(data),13): found.extend(decoder.feed(data[i:i+13]))
        self.assertEqual([x['tick_ms'] for x in found],[100.,140.])
        self.assertAlmostEqual(found[-1]['odom_x_m'],1.2)

    def test_invalid_values_and_wrong_width_are_not_sources(self):
        decoder=FrameDecoder()
        data=self.frame(float('nan'))+self.frame(100.,7.)+b'bad'+FrameDecoder.TAIL
        self.assertEqual(decoder.feed(data),[])
        self.assertEqual(decoder.feed(self.frame(140.))[0]['tick_ms'],140.)

    def test_long_frame_cannot_supply_a_plausible_56_channel_suffix(self):
        wrong_width = bytes(8 * 4) + self.frame(100.)
        stream = wrong_width + self.frame(140.)
        for split in range(len(wrong_width) + 1):
            with self.subTest(split=split):
                decoder = FrameDecoder()
                samples = decoder.feed(stream[:split]) + decoder.feed(stream[split:])
                self.assertEqual([sample["tick_ms"] for sample in samples], [140.])

    def test_uart_snapshot_keeps_real_command_identity(self):
        session=Mock()
        fields=['request_seq','command','heartbeat','ack_seq','phase','error']
        session.symbols={'tuning_'+key:0x20000000+i*4 for i,key in enumerate(fields)}
        session.symbols['tuning_sample']=0x20000100
        session.client.parse_words=swd.TclClient.parse_words
        session.client.evaluate.return_value=' '.join(map(str,[3,8,20,3,2,0,swd.float_word(100.)]))
        telemetry=Mock()
        telemetry.latest.return_value=FrameDecoder().feed(self.frame(100.))[0]
        state=visual_snapshot(session,telemetry)
        self.assertEqual((state['command'],state['ack_seq'],state['telemetry_tick_ms']),(8,3,100))
        self.assertIsNone(state['sample_seq'])
        self.assertEqual(session.client.evaluate.call_count,1)

    def test_bridge_buffered_old_frame_is_discarded(self):
        session=Mock()
        fields=['request_seq','command','heartbeat','ack_seq','phase','error']
        session.symbols={'tuning_'+key:0x20000000+i*4 for i,key in enumerate(fields)}
        session.symbols['tuning_sample']=0x20000100
        session.client.parse_words=swd.TclClient.parse_words
        session.client.evaluate.return_value=' '.join(map(str,[3,8,20,3,2,0,swd.float_word(10000.)]))
        telemetry=Mock()
        telemetry.latest.side_effect=[FrameDecoder().feed(self.frame(t))[0] for t in (100.,9960.)]
        state=visual_snapshot(session,telemetry)
        self.assertEqual(state['telemetry_tick_ms'],9960)
        self.assertEqual(session.client.evaluate.call_count,2)
        self.assertEqual(session.log.emit.call_args.args[0],'uart_stale_discarded')

    def test_read_only_uart_never_starts_heartbeat(self):
        session=Mock()
        session.heartbeat=None
        fields=['request_seq','command','heartbeat','ack_seq','phase','error']
        session.symbols={'tuning_'+key:0x20000000+i*4 for i,key in enumerate(fields)}
        session.symbols['tuning_sample']=0x20000100
        session.client.parse_words=swd.TclClient.parse_words
        session.client.evaluate.return_value=' '.join(map(str,[3,8,20,3,2,0,swd.float_word(100.)]))
        telemetry=Mock()
        telemetry.latest.return_value=FrameDecoder().feed(self.frame(100.))[0]
        visual_snapshot(session,telemetry)
        session.beat.assert_not_called()

    def test_running_uart_renews_lease_after_serial_wait(self):
        order=[]
        session=Mock()
        session.heartbeat=20
        fields=['request_seq','command','heartbeat','ack_seq','phase','error']
        session.symbols={'tuning_'+key:0x20000000+i*4 for i,key in enumerate(fields)}
        session.symbols['tuning_sample']=0x20000100
        session.client.parse_words=swd.TclClient.parse_words
        session.beat.side_effect=lambda:order.append('heartbeat')
        session.client.evaluate.side_effect=lambda command:order.append('read') or ' '.join(map(str,[3,8,20,3,2,0,swd.float_word(100.)]))
        telemetry=Mock()
        telemetry.latest.side_effect=lambda **kw:order.append('serial') or FrameDecoder().feed(self.frame(100.))[0]
        visual_snapshot(session,telemetry)
        self.assertEqual(order,['serial','heartbeat','read'])

class VisualProjection(unittest.TestCase):
    def test_axes_and_rotated_body(self):
        for yaw in (0,math.pi/2,-math.pi/2,math.pi):
            pose=[.12,-.13,yaw]
            goal=[pose[0]+.6*math.cos(yaw)-.1*math.sin(yaw),
                  pose[1]+.6*math.sin(yaw)+.1*math.cos(yaw),yaw]
            raw=image_center(pose,goal,0)
            self.assertAlmostEqual(raw[0],6015)
            self.assertAlmostEqual(raw[1],1475)
    def test_noise_is_zero_mean(self):
        a=image_center([0,0,0],[0,0,0],0)
        b=image_center([0,0,0],[0,0,0],1)
        self.assertEqual([(x+y)/2 for x,y in zip(a,b)],[5000,7500])

if __name__=="__main__": unittest.main()
