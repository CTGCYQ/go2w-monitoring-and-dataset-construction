import io
import json
import struct
import time
import unittest
from depth_stream import read_frame, summarize, DepthReceiver, packet_with_age

class Channel:
    def __init__(self,data):self.stream=io.BytesIO(data)
    def recv(self,n):return self.stream.read(min(n,7))

def packet(**changes):
    h=dict(version=1,format='Z16_LE',alignment='color',width=5,height=5,payload_bytes=50,
           frame_id="realsense_color_optical_frame",coefficients=[0]*5,frame_number=1,meters_per_unit=.001,source_age_ms=5,captured_host_time_s=100,
           device_timestamp_ms=20,color_timestamp_ms=20,fx=3,fy=3,ppx=2,ppy=2)
    h.update(changes)
    b=json.dumps(h).encode()
    return struct.pack('!I',len(b))+b+struct.pack('<25H',*([2000]*25))

class DepthTests(unittest.TestCase):
    def test_partial_transport_and_metric(self):
        h,p,b=read_frame(Channel(packet()))
        self.assertEqual(b,packet())
        self.assertEqual(summarize(h,p)['center_depth_m'],2)
    def test_zero_invalid(self):
        h,p,b=read_frame(Channel(packet()))
        self.assertIsNone(summarize(h,bytes(50))['center_depth_m'])
    def test_bad_metadata(self):
        for changes in [dict(format='JPEG'),dict(alignment='native'),dict(width=True),
                        dict(payload_bytes=51),dict(meters_per_unit=0),dict(fx=float('nan')),
                        dict(source_age_ms=1501),dict(source_age_ms=-1),dict(ppx=5),dict(version=True),dict(frame_id="base_link"),dict(coefficients=[float("nan")]*5)]:
            with self.subTest(changes=changes),self.assertRaises(ValueError):
                read_frame(Channel(packet(**changes)))
    def test_truncated(self):
        with self.assertRaises(EOFError):read_frame(Channel(packet()[:-1]))
    def test_size_limit(self):
        with self.assertRaises(ValueError):read_frame(Channel(struct.pack('!I',999999)))
    def test_http_packet_includes_receiver_age(self):
        h, _, _ = read_frame(Channel(packet_with_age(packet(), .4)))
        self.assertEqual(h['source_age_ms'], 405)

    def test_snapshot_expiry(self):
        r=DepthReceiver('unused','unused','unused')
        self.assertFalse(r.snapshot()[0]['online'])
        r.frame=({'online':True,'source_age_ms':0},b'packet',time.monotonic())
        self.assertEqual(r.snapshot()[1],b'packet')
        r.frame=({'online':True,'source_age_ms':0},b'packet',time.monotonic()-2)
        self.assertFalse(r.snapshot()[0]['online'])
        self.assertIsNone(r.snapshot()[1])

if __name__=='__main__':unittest.main()
