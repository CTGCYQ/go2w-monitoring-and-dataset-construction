"""Validate receipt timestamps; never use cache write time as sensor freshness."""
import math
import time


def cloud_is_fresh(cache, now=None, max_age=1.5):
    if not isinstance(cache, dict) or cache.get("online") is not True:
        return False
    stamp = cache.get("received_at")
    if isinstance(stamp, bool) or not isinstance(stamp, (int, float)):
        return False
    age = (time.time() if now is None else now) - stamp
    return math.isfinite(age) and 0 <= age <= max_age
