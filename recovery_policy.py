"""Pure link debounce and bounded restart scheduling; no motion decisions."""
class LinkGate:
    def __init__(self,stable_seconds=3.,offline_seconds=3.):
        self.stable_seconds=stable_seconds;self.offline_seconds=offline_seconds
        self.since=None;self.last_good=None;self.key=None;self.distinct=0
    def observe(self,now,fresh,key):
        if not fresh:
            self.since=None;self.distinct=0;self.key=None;return False
        self.last_good=now
        if self.since is None:self.since=now
        if key!=self.key:self.distinct+=1;self.key=key
        return self.distinct>=3 and now-self.since>=self.stable_seconds
    def offline(self,now):return self.last_good is None or now-self.last_good>=self.offline_seconds

class Backoff:
    def __init__(self):self.next=0.;self.failures=0
    def ready(self,now):return now>=self.next
    def fail(self,now):self.failures+=1;self.next=now+min(60,3*2**min(5,self.failures-1))
    def success(self):self.failures=0;self.next=0.


def supervisor_health(path,now=None):
    import json,math,time
    from pathlib import Path
    now=time.time() if now is None else now
    try:
        status=json.loads(Path(path).read_text())
        age=now-float(status['written_at'])
        if not isinstance(status,dict) or not math.isfinite(age) or not 0<=age<=8:return {'online':False,'reason':'supervisor status stale'}
        return {**status,'online':True,'age_s':round(age,3)}
    except (OSError,ValueError,TypeError,KeyError):return {'online':False,'reason':'supervisor unavailable'}


def wait_for_process(probe,timeout=3.,clock=None,sleep=None):
    import time
    clock=time.monotonic if clock is None else clock;sleep=time.sleep if sleep is None else sleep
    deadline=clock()+timeout
    while True:
        pid=probe()
        if pid:return pid
        if clock()>=deadline:return None
        sleep(.1)


def telemetry_status(state,now=None):
    import math,time
    now=time.time() if now is None else now
    if not isinstance(state,dict):return {'online':False,'reason':'invalid telemetry cache'}
    try:age=now-float(state['last_seen_s'])
    except (KeyError,TypeError,ValueError):age=None
    valid=age is not None and math.isfinite(age) and 0<=age<=3
    return {**state,'online':bool(state.get('online') and valid),'telemetry_age_s':round(age,3) if valid else None}
