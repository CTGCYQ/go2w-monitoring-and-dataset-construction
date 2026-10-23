"""Freshness checks for live camera endpoints, independent of clock on the dock."""
import math,time,threading
from pathlib import Path

def live_front(path,state,now=None):
    now=time.time() if now is None else now
    try:
        age=now-float(state['last_seen_s']);file_age=now-Path(path).stat().st_mtime
        if not math.isfinite(age) or not 0<=age<=3 or not 0<=file_age<=3:return None
        return Path(path).read_bytes()
    except (OSError,KeyError,ValueError,TypeError):return None

class FrameProgress:
    def __init__(self):self.items={};self.lock=threading.Lock()
    def fresh(self,name,fingerprint,now=None):
        now=time.monotonic() if now is None else now
        with self.lock:
            old=self.items.get(name)
            if old is None or old[0]!=fingerprint:self.items[name]=(fingerprint,now);return True
            return 0<=now-old[1]<=3
