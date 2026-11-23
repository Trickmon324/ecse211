"""Synchronous differential-drive odometry and grid correction, adapted from Lab 4."""
import math
import time
from collections import deque
from statistics import median
import settings as S


def axes(h):
    a = math.radians(h)
    return (math.sin(a), math.cos(a)), (math.cos(a), -math.sin(a))


def dot(a, b):
    return sum(x*y for x, y in zip(a,b))


def wrap(a):
    return (a+180) % 360-180


def clamp(x, limit):
    return max(-limit,min(x,limit))


class Hardware:
    def __init__(self):
        from utils.brick import Motor, EV3ColorSensor, TouchSensor, EV3UltrasonicSensor, wait_ready_sensors
        self.motors = [Motor(p) for p in (S.LEFT_PORT,S.RIGHT_PORT)]
        self.lights = [EV3ColorSensor(p) for p in (S.LEFT_LIGHT_PORT,S.RIGHT_LIGHT_PORT)]
        self.touch = TouchSensor(S.TOUCH_PORT)
        self.us = EV3UltrasonicSensor(S.US_PORT)
        wait_ready_sensors()
    def enc(self):
        return [m.get_encoder()*s for m,s in zip(self.motors,(S.LEFT_SIGN,S.RIGHT_SIGN))]
    def power(self,l,r):
        for m,p,s,t in zip(self.motors,(l,r),(S.LEFT_SIGN,S.RIGHT_SIGN),(S.LEFT_TRIM,S.RIGHT_TRIM)):
            m.set_power(int(s*math.copysign(min(100,max(0,abs(p)+t)),p)) if p else 0)
    def pressed(self):
        v = self.touch.get_value()
        if v not in (0,1):
            raise RuntimeError("Touch sensor unavailable")
        return bool(v)
    def light(self):
        out = []
        for sensor in self.lights:
            rgb = sensor.get_value()
            if not rgb or len(rgb)<3 or any(v is None for v in rgb[:3]):
                raise RuntimeError("Light sensor unavailable")
            out.append(sum(rgb[:3]))
        return out
    def side(self):
        v = self.us.get_cm()
        if not isinstance(v,(float,int)) or not 0 < v <= 255:
            raise RuntimeError("Ultrasonic sensor unavailable")
        return v
    now = staticmethod(time.monotonic)
    sleep = staticmethod(time.sleep)


class Robot:
    def __init__(self,hw,seconds):
        self.hw=hw
        self.deadline=hw.now()+seconds
        self.x=self.y=self.travel=0.
        self.last_alignment=None
        self.h=S.START_HEADING
        self.enc=hw.enc()
        self.last_move=hw.now()
        self.reset_filters()

    def reset_filters(self):
        self.lightbuf=[deque(maxlen=S.LIGHT_MEDIAN) for _ in range(2)]
        self.black=[False,False]
        self.sidebuf=deque(maxlen=S.US_MEDIAN)
        self.side_time=-math.inf
        self.side_value=None
        self.last_line=None
        self.line_travel=self.travel
        self.armed=False  # first see clear floor

    def tick(self):
        self.hw.sleep(S.SAMPLE_S)
        now=self.hw.now()
        if now>=self.deadline:
            raise RuntimeError("Run deadline reached")
        enc=self.hw.enc()
        dl,dr=[math.radians(e-old)*r for e,old,r in zip(enc,self.enc,(S.LEFT_RADIUS,S.RIGHT_RADIUS))]
        self.enc=enc
        delta=math.degrees((dl-dr)/S.TRACK)
        delta/=S.RIGHT_TURN_FACTOR if delta>=0 else S.LEFT_TURN_FACTOR
        f,_=axes(self.h+delta/2)
        self.x+=(dl+dr)/2*f[0]
        self.y+=(dl+dr)/2*f[1]
        self.travel+=(abs(dl)+abs(dr))/2
        self.h+=delta  # deliberately unwrapped for multi-revolution turns
        if dl or dr:
            self.last_move=now
        if now-self.last_move>S.STALL_SECONDS:
            raise RuntimeError("Wheels stalled")

    def stop(self):
        self.hw.power(0,0)

    def turn(self,angle,power=S.TURN):
        target=self.h+angle
        self.stop()
        self.last_move=self.hw.now()
        # Only encoder and bumper reads here. Side history belongs to a straight leg.
        while abs(target-self.h)>S.TURN_TOLERANCE:
            if self.hw.pressed():
                raise RuntimeError("Contact during turn")
            e=target-self.h
            p=math.copysign(min(power,max(min(S.TURN_MIN,power),power*abs(e)/S.TURN_SLOW_ZONE)),e)
            self.hw.power(p,-p)
            self.tick()
        self.stop()
        self.hw.sleep(S.TURN_SETTLE)
        self.tick()
        self.reset_filters()

    def face(self,h):
        if abs(wrap(h-self.h))>S.TURN_TOLERANCE:
            self.turn(wrap(h-self.h))

    def light_state(self):
        values=self.hw.light()
        for i,v in enumerate(values):
            buf=self.lightbuf[i]
            buf.append(v)
            if len(buf)==buf.maxlen:
                self.black[i]=median(buf)<(S.BLACK_EXIT[i] if self.black[i] else S.BLACK_ENTER[i])
        return self.black

    def side_distance(self):
        if self.hw.now()-self.side_time>=S.US_SAMPLE_S:
            self.sidebuf.append(self.hw.side())
            self.side_time=self.hw.now()
            if len(self.sidebuf)==self.sidebuf.maxlen:
                self.side_value=median(self.sidebuf)
        return self.side_value

    def position(self,h):
        return dot((self.x,self.y),axes(h)[0])

    def align(self,h,line):
        first=0 if self.black[0] else 1
        second=1-first
        h0=self.h
        d=self.travel-self.line_travel
        before=(self.x,self.y,self.h)
        filtered=[median(buf) for buf in self.lightbuf]
        start=self.hw.now()
        both=all(self.black)
        while not all(self.black):
            if self.hw.pressed():
                self.stop()
                return False
            if self.hw.now()-start>S.ALIGN_TIMEOUT or abs(self.h-h0)>S.MAX_PIVOT:
                raise RuntimeError("Grid-line alignment exceeded limit")
            self.hw.power(0 if first==0 else S.ALIGN,S.ALIGN if first==0 else 0)
            self.tick()
            self.light_state()
            self.black[first]=True
        self.stop()
        self.tick()
        p=math.radians(self.h-h0)
        f1,r1=S.LIGHTS[first]
        f2,r2=S.LIGHTS[second]
        wheel=(-1 if first==0 else 1)*S.TRACK/2
        # Rotate the second sensor around the stopped wheel. Both trigger points
        # lie on the same transverse line, which fixes the initial approach angle.
        df=f2*math.cos(p)-(r2-wheel)*math.sin(p)-f1
        dr=wheel+f2*math.sin(p)+(r2-wheel)*math.cos(p)-r1
        actual=math.atan2(df,dr)
        actual=(actual+math.pi/2)%math.pi-math.pi/2
        final=actual+p
        residual=wrap(math.degrees(actual)-wrap(h0-h))
        drift=clamp(S.DRIFT_GAIN*d*math.sin(math.radians(residual)),S.MAX_DRIFT)
        f,right=axes(h)
        shift=line-(f2*math.cos(final)-r2*math.sin(final))-self.position(h)
        self.x+=shift*f[0]+drift*right[0]
        self.y+=shift*f[1]+drift*right[1]
        self.h=h+math.degrees(final)
        print("line %.2f first=%s pivot=%.1f heading error=%.1f lateral correction=%.2f pose=(%.1f,%.1f)" %
              (line,"both" if both else ("L","R")[first],math.degrees(p),residual,drift,self.x,self.y))
        self.face(h)
        self.sidebuf.clear()
        self.side_value=None
        self.last_line=line
        self.line_travel=self.travel
        self.armed=False
        self.last_alignment=dict(line=line,pose_before=before,pose_after=(self.x,self.y,self.h),
                                 first="both" if both else ("left","right")[first],
                                 rgb_medians=filtered,pivot_deg=math.degrees(p),
                                 heading_discrepancy_deg=residual,lateral_cm=drift)
        return True
