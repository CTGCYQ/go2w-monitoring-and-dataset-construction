import json,math,tempfile,unittest
from pathlib import Path
from display_alignment import metadata,valid_rotation
from tools.calibrate_display_direction import solve

class AlignmentTests(unittest.TestCase):
    def test_reflections_invalid(self):
        self.assertFalse(valid_rotation([-1,0,0,0,1,0,0,0,1]));self.assertFalse(valid_rotation([float('nan')]*9))
    def test_manual_forward_finds_rotation(self):
        samples=[(i,(0,y,0),(0,0,0,1)) for i,y in enumerate([0,.1,.2,.3,.4,.3,.2,.1,0]*3)]
        result=solve(samples,[0,0,9.81]);self.assertEqual(result['rotation_row_major'],[0.,1.,0.,-1.,0.,0.,0.,0.,1.])
    def test_short_excursion_rejected(self):
        with self.assertRaises(ValueError):solve([(i,(0,0,0),(0,0,0,1)) for i in range(25)],[0,0,9.81])
    def test_metadata_requires_validated_matching_frame(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'alignment.json';p.write_text(json.dumps({'validated':True,'source_frame':'utlidar_lidar','calibration_id':'test','rotation_row_major':[1,0,0,0,1,0,0,0,1]}))
            self.assertEqual(metadata(p,{'frame_id':'other'}),{})
            self.assertTrue(metadata(p,{'frame_id':'utlidar_lidar'})['display_direction_verified'])
            self.assertFalse(metadata(p,{'frame_id':'utlidar_lidar'})['body_alignment_verified'])
            self.assertEqual(metadata(p,{'mapping_version':1},True),{})
    def test_map_rotation_preserves_gravity(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'alignment.json';p.write_text(json.dumps({'validated':True,'source_frame':'utlidar_lidar','calibration_id':'test','rotation_row_major':[1,0,0,0,1,0,0,0,1]}))
            result=metadata(p,{'mapping_version':2,'anchor_orientation_xyzw':[0,0,math.sin(.4),math.cos(.4)]},True)
            for a,b in zip(result['display_rotation_row_major'],[1,0,0,0,1,0,0,0,1]):self.assertAlmostEqual(a,b)
    def test_missing_or_unverified_file(self):
        self.assertEqual(metadata('missing',{}),{});self.assertEqual(metadata('missing',[]),{})

if __name__=='__main__':unittest.main()
