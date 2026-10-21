import unittest
from sensor_freshness import cloud_is_fresh

class FreshnessTests(unittest.TestCase):
    def test_fresh(self):
        self.assertTrue(cloud_is_fresh(dict(online=True, received_at=99), now=100))
    def test_stale_even_if_rewritten(self):
        self.assertFalse(cloud_is_fresh(dict(online=True, received_at=90, ts=100, written_at=100), now=100))
    def test_worker_stopped(self):
        self.assertFalse(cloud_is_fresh(dict(online=True, received_at=98), now=100))
    def test_legacy_cache_rejected(self):
        self.assertFalse(cloud_is_fresh(dict(online=True, ts=100), now=100))
    def test_invalid(self):
        for stamp in (None, True, '100', float('nan'), float('inf'), 101):
            self.assertFalse(cloud_is_fresh(dict(online=True, received_at=stamp), now=100))
    def test_offline(self):
        self.assertFalse(cloud_is_fresh(dict(online=False, received_at=100), now=100))

if __name__ == '__main__':
    unittest.main()
