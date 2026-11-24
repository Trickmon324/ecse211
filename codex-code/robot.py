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


class MotorDrive:
    """Direct BrickPi power commands with encoder feedback in Robot."""
    def __init__(self):
        from utils.brick import Motor

        self.ports = (S.LEFT_PORT, S.RIGHT_PORT)
        self.signs = (S.LEFT_SIGN, S.RIGHT_SIGN)
        self.trims = (S.LEFT_TRIM, S.RIGHT_TRIM)
        self.motors = [Motor(port) for port in self.ports]
        self.commands = [None, None]
        self.stop()

    def enc(self):
        values = []
        for motor, port, sign in zip(self.motors, self.ports, self.signs):
            value = motor.get_encoder()
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise RuntimeError(f"Motor {port} encoder reading unavailable")
            values.append(value * sign)
        return values

    def power(self, left, right):
        for i, (motor, demand) in enumerate(zip(self.motors, (left, right))):
            magnitude = max(0, abs(demand) + self.trims[i]) if demand else 0
            limit = S.MAX_MOTOR_POWER
            signed = int(math.copysign(round(min(limit, magnitude)), demand)) * self.signs[i]
            if signed != self.commands[i]:
                motor.set_power(signed)
                self.commands[i] = signed

    def stop(self):
        # Try the other wheel even if the first command fails.
        try:
            self.motors[0].set_power(0)
        finally:
            self.commands = [None, None]
            self.motors[1].set_power(0)

    def describe(self, i):
        side = ("left", "right")[i]
        flags, output, encoder, dps = self.motors[i].get_status()
        label = f"{side} motor {self.ports[i]}"
        if flags is None:
            return f"{label}: BrickPi status unavailable"
        if flags & 1:
            return f"{label}: LOW VOLTAGE (BrickPi disabled motor output)"
        return (f"{label}: asked {self.commands[i]}%, BrickPi output {output}%, "
                f"encoder {encoder} deg, speed {dps} deg/s")

    def bench_test(self):
        """Exercise the same direct power commands as the mapping run."""
        print("Testing each wheel by itself. Look for forward rotation.")
        passed = True
        try:
            for i, port in enumerate(self.ports):
                before = self.enc()[i]
                try:
                    demand = [0, 0]
                    demand[i] = S.DRIVE
                    self.power(*demand)
                    time.sleep(.55)
                    delta = self.enc()[i] - before
                    status = self.describe(i)
                finally:
                    self.stop()
                if delta < 3:
                    passed = False
                print(f"{port}: encoder change {delta:+.0f} deg | {status}")
            print("Motor test passed." if passed else
                  "Motor test failed. Check the output and encoder direction printed above.")
        finally:
            self.stop()


class Hardware(MotorDrive):
    def __init__(self, *, use_bumper=True):
        from utils.brick import EV3ColorSensor, TouchSensor, EV3UltrasonicSensor

        super().__init__()
        self.lights = [EV3ColorSensor(p) for p in (S.LEFT_LIGHT_PORT, S.RIGHT_LIGHT_PORT)]
        self.touch = TouchSensor(S.TOUCH_PORT) if use_bumper else None
        self.us = EV3UltrasonicSensor(S.US_PORT)
        for sensor in [*self.lights, self.us, self.touch]:
            if sensor is not None:
                sensor.wait_ready()

    def pressed(self):
        if self.touch is None:
            return False
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
        self.wheel_commands = (0, 0)
        self.last_move = [hw.now(), hw.now()]
        self.motion = "idle"
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
        stalled = []
        for i, (demand, distance) in enumerate(zip(self.wheel_commands, (dl, dr))):
            if not demand or distance:
                self.last_move[i] = now
            elif now - self.last_move[i] >= S.STALL_SECONDS:
                stalled.append(i)
        if stalled:
            reporter = getattr(self.hw, "describe", None)
            details = [reporter(i) if reporter else ("left wheel", "right wheel")[i]
                       for i in stalled]
            self.stop()
            raise RuntimeError(f"No encoder movement during {self.motion}. " + " | ".join(details))

    def command(self, left, right, *, motion):
        now = self.hw.now()
        for i, demand in enumerate((left, right)):
            if demand * self.wheel_commands[i] <= 0:
                self.last_move[i] = now
        self.wheel_commands = (left, right)
        self.motion = motion
        self.hw.power(left, right)

    def stop(self):
        self.wheel_commands = (0, 0)
        self.hw.power(0, 0)

    def turn(self, angle, power=S.TURN):
        target = self.h + angle
        self.stop()
        # Keep sonar readings for straight driving only.
        pulse_seconds = S.TURN_PULSE_SECONDS
        best_error = S.TURN_FINE_ZONE
        remaining_pulses = S.TURN_MAX_PULSES
        try:
            while True:
                error = target - self.h
                if abs(error) <= S.TURN_TOLERANCE:
                    break
                if self.hw.pressed():
                    raise RuntimeError("Contact during turn")

                # Full power gets the turn moving. Close to the target, use
                # short pulses at usable torque and re-read after braking.
                fine = abs(error) <= S.TURN_FINE_ZONE
                magnitude = min(power, S.TURN_MIN) if fine else power
                demand = math.copysign(magnitude, error)
                self.command(demand, -demand, motion="turn")
                self.tick()
                if fine:
                    elapsed = S.SAMPLE_S
                    while elapsed + S.SAMPLE_S / 2 < pulse_seconds:
                        if self.hw.pressed():
                            raise RuntimeError("Contact during turn")
                        self.tick()
                        elapsed += S.SAMPLE_S
                    self.stop()
                    self.hw.sleep(S.TURN_SETTLE)
                    self.tick()
                    remaining = target - self.h
                    if remaining * error < 0:
                        pulse_seconds = max(S.SAMPLE_S, pulse_seconds / 2)
                    elif abs(remaining - error) < .2:
                        pulse_seconds = min(.12, pulse_seconds * 1.5)
                    if abs(remaining) < best_error - .2:
                        best_error = abs(remaining)
                        remaining_pulses = S.TURN_MAX_PULSES
                    else:
                        remaining_pulses -= 1
                    if not remaining_pulses and abs(remaining) > S.TURN_TOLERANCE:
                        raise RuntimeError(f"Could not finish turn. Remaining angle {remaining:+.1f} deg.")
        finally:
            self.stop()
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
            self.command(0 if first == 0 else S.ALIGN,
                         S.ALIGN if first == 0 else 0, motion="line squaring")
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
