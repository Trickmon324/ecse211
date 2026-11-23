"""Run on the BrickPi with the course utils package. See README.md."""
import argparse
import json
import math
import os
import signal
from pathlib import Path
from statistics import median
import settings as S
from robot import Hardware, Robot, axes, clamp, dot, wrap


def offset(p,h,d):
    f,_=axes(h)
    return [p[i]+f[i]*d for i in range(2)]


def inside(x,y,margin):
    return (-S.TILE+margin <= x <= 8*S.TILE-margin and
            -S.TILE+margin <= y <= 4*S.TILE-margin)


class Mission:
    def __init__(self,r):
        self.r=r
        self.side_sign={"right":1,"left":-1}[S.SIDE_SENSOR_ORIENTATION]

    def advance(self,h,target=None,monitor=None,raw=False,lines=None,speed=None):
        r=self.r
        r.face(h)
        r.last_move=r.hw.now()
        speed=speed or S.DRIVE
        _,right=axes(h)
        cross=dot((r.x,r.y),right)
        count=0
        clear_line=None
        try:
            while True:
                r.tick()
                if r.hw.pressed():
                    return "contact"
                pos=r.position(h)
                if target is not None and pos>=target-.2:
                    return "arrived"
                v=r.side_distance() if monitor else None
                if monitor:
                    event=monitor(pos,v)
                    if event:
                        return event
                if not raw:
                    black=r.light_state()
                    sensor_pos=pos+sum(a for a,b in S.LIGHTS)/2
                    line=round(sensor_pos/S.TILE)*S.TILE
                    if not any(black) and (r.last_line is None or r.travel-r.line_travel>S.REARM_CM):
                        r.armed=True
                    if not any(black) and abs(sensor_pos-line)<S.LINE_GATE_CM:
                        clear_line=line
                    if r.armed and any(black) and clear_line==line and abs(sensor_pos-line)<S.LINE_GATE_CM and line!=r.last_line:
                        if not r.align(h,line):
                            return "contact"
                        if lines is not None:
                            print(r.last_alignment)
                        count+=1
                        if count==lines:
                            return "lines"
                        continue
                x,y=offset((r.x,r.y),h,S.FRONT+S.MARGIN)
                if not inside(x,y,0):
                    raise RuntimeError("Board boundary reached before expected event")
                correction=0 if raw else S.HEADING_GAIN*wrap(h-r.h)+S.CENTER_GAIN*(cross-dot((r.x,r.y),right))
                if v is not None and v<=S.WALL_NEAR:
                    correction+=self.side_sign*(S.WALL_GAIN*(v-S.US_BAND_CENTER-S.WALL_TRIM) if v>S.US_TOO_CLOSE else -S.MAX_STEER)
                correction=clamp(correction,S.MAX_STEER)
                r.hw.power(speed+correction,speed-correction)
        finally:
            r.stop()

    def goto(self,p,order=(0,1)):
        r=self.r
        for axis in order:
            diff=p[axis]-(r.x,r.y)[axis]
            if abs(diff)<.7:
                continue
            h=((90 if diff>0 else 270) if axis==0 else (0 if diff>0 else 180))
            if self.advance(h,dot(p,axes(h)[0]))!="arrived":
                raise RuntimeError("Unexpected bumper contact on transfer route")

    def reverse_to(self,h,target):
        r=self.r
        try:
            while r.position(h)>target+.15:
                x,y=offset((r.x,r.y),h,-S.REAR-S.MARGIN)
                if not inside(x,y,0):
                    raise RuntimeError("Insufficient backing clearance")
                r.hw.power(-S.BACKUP,-S.BACKUP)
                r.tick()
            if r.hw.pressed():
                raise RuntimeError("Bumper did not release after backup")
        finally:
            r.stop()

    def entrance(self,a,b):
        if sum(abs(x-y) for x,y in zip(a,b))!=1 or not all(inside(x*S.TILE,y*S.TILE,0) for x,y in (a,b)):
            raise ValueError("Entrance must be two adjacent grid intersections")
        mid=[(x+y)*S.TILE/2 for x,y in zip(a,b)]
        axis=0 if a[0]==b[0] else 1
        h=(90 if mid[axis]>=0 else 270) if axis==0 else (0 if mid[axis]>=0 else 180)
        approach=offset(mid,h,-S.TILE/2)
        order=(axis,1-axis)
        self.goto(approach,order)
        # Check both flanks from rest if this approach appears to be inside the
        # corridor. A settled right-looking sensor can inspect either flank.
        def near(heading):
            self.r.face(heading)
            samples=[]
            for _ in range(S.US_MEDIAN):
                self.r.hw.sleep(S.US_SAMPLE_S)
                samples.append(self.r.hw.side())
            return median(samples)<S.WALL_NEAR
        if near(h) or near(h+180):
            old=approach
            h=(h+180)%360
            approach=offset(mid,h,-S.TILE/2)
            self.goto(approach)
            return mid,h,approach,[old,approach],order
        return mid,h,approach,[approach],order

    def map(self,a,b):
        r=self.r
        mid,h,approach,transfers,order=self.entrance(a,b)
        self.go_along(mid,h)
        corners=[]
        seen=set()
        while True:
            r.face(h)
            # Entering at mid-line puts forward-mounted sensors past that line.
            # The supplied entrance is also a known first-turn landmark.
            opening=None
            def observe(pos,v):
                nonlocal opening
                if v is None:
                    return
                if v<=S.WALL_NEAR:
                    opening=None
                elif v>=S.WALL_OPEN:
                    if opening is None:
                        opening=pos+S.US[0]
                    opening_line=round(opening/S.TILE)*S.TILE
                    confirm=opening_line+S.TILE-S.TOUCH[0]+S.EXIT_EXTRA_CM
                    end=offset((r.x,r.y),h,opening_line+S.TILE-pos+.5)
                    # A front box beyond the board is impossible. Confirm on
                    # entering the final open cell, leaving room for a pivot.
                    boundary_exit=not inside(*end,0) and pos>=opening_line+S.MARGIN
                    if pos>=confirm or boundary_exit:
                        return "exit"
            event=self.advance(h,monitor=observe,speed=S.APPROACH)
            pos=r.position(h)
            if event=="exit":
                line=round(opening/S.TILE)*S.TILE
                exit_point=offset((r.x,r.y),h,line-pos-S.TILE/2)
                # Back onto the surveyed centreline before reversing direction.
                self.reverse_to(h,line-S.TILE/2)
                break
            # Robot stationary against the front wall. Sample before any reverse motion.
            readings=[]
            for _ in range(S.US_MEDIAN):
                readings.append(r.hw.side())
                r.hw.sleep(S.US_SAMPLE_S)
            side_cm=median(readings)
            direction=self.side_sign*(-90 if side_cm<=S.WALL_NEAR else 90)
            # Correct the front-wall coordinate at the moment of bumper contact.
            f,_=axes(h)
            e=math.radians(wrap(r.h-h))
            reach=S.TOUCH[0]*math.cos(e)-S.TOUCH[1]*math.sin(e)
            wall=round((pos+reach)/S.TILE)*S.TILE
            if abs(wall-pos-reach)>S.LINE_GATE_CM:
                raise RuntimeError("Bumper position disagrees with expected grid wall")
            shift=wall-reach-pos
            r.x+=f[0]*shift
            r.y+=f[1]*shift
            pivot=wall-S.TILE/2-S.BACKUP_TRIM_CM
            # Back away at least the configured release distance, then creep
            # forward again if the measured pivot is closer to the wall.
            self.reverse_to(h,min(pivot,r.position(h)-S.BUMP_BACKUP_CM))
            if r.position(h)<pivot-.2:
                if self.advance(h,pivot,raw=True,speed=S.ALIGN)!="arrived":
                    raise RuntimeError("Bumper contacted during pivot positioning")
            line=wall-S.TILE
            if r.last_line!=line and not (not corners and abs(dot(mid,f)-line)<.1):
                raise RuntimeError("Missing final grid line before corner")
            key=(round(r.x/S.TILE,1),round(r.y/S.TILE,1),h)
            if key in seen:
                raise RuntimeError("Repeated corner")
            seen.add(key)
            corners.append(dict(point=[r.x,r.y],heading=h,line_cm=line,turn=direction,wall_cm=wall,contact_side_cm=side_cm))
            h=(h+direction)%360
            r.face(h)
        result=dict(entrance=[a,b],approach=approach,midpoint=mid,transfers=transfers,
                    order=order,entry_heading=corners[0]["heading"] if corners else h,
                    corners=corners,exit_heading=h,exit_line_cm=line,exit_inside=exit_point)
        self.return_home(result)
        return result

    def return_home(self,m):
        # Retrace surveyed straight sections, keeping within the street on turns.
        for c in reversed(m["corners"]):
            self.go_along(c["point"],(c["heading"]+c["turn"]+180)%360)
        self.go_along(m["approach"],(m["entry_heading"]+180)%360)
        for point in reversed(m["transfers"][:-1]):
            self.goto(point)
        self.goto([0,0],reversed(m["order"]))
        self.r.face(S.START_HEADING)

    def go_along(self,p,h):
        if self.advance(h,dot(p,axes(h)[0]))!="arrived":
            raise RuntimeError("Unexpected contact on mapped route")

    def replay(self,m):
        r=self.r
        self.goto(m["transfers"][0],m["order"])
        for p in m["transfers"][1:]:
            self.goto(p)
        for c in m["corners"]:
            h=c["heading"]
            pivot_at=c["line_cm"]-sum(f for f,_ in S.LIGHTS)/2+S.POST_LINE_FORWARD_CM
            radius=math.hypot(max(S.FRONT,S.REAR),max(S.LEFT,S.RIGHT))
            if abs(pivot_at-dot(c["point"],axes(h)[0]))+radius+S.MARGIN>S.TILE/2:
                raise ValueError("Post-line distance places pivot outside the measured turn clearance")
            # Arm at the stored line, then measure the adjustable axle travel
            # from the completed physical alignment, not from an odometry guess.
            start=None
            def trigger(pos,v):
                nonlocal start
                if start is None and r.last_line==c["line_cm"]:
                    start=pos
                if start is not None and pos>=start+S.POST_LINE_FORWARD_CM:
                    return "pivot"
                if pos+S.FRONT+S.MARGIN>=c["line_cm"]+S.TILE:
                    raise RuntimeError("Turn landmark missed or configured advance too long")
            if self.advance(h,monitor=trigger)!="pivot":
                raise RuntimeError("Unexpected contact during replay")
            r.face((h+c["turn"])%360)
        self.go_along(m["exit_inside"],m["exit_heading"])
        self.return_home(m)


def calibrate_rgb(hw, output):
    # Move the whole robot by hand; sample both mounted sensors together.
    hw.power(0,0)
    groups=[]
    for surface in ("light blue background","dark blue line"):
        input("Place both light sensors over the "+surface+" and press Enter: ")
        readings=[[],[]]
        for _ in range(50):
            for channel,value in zip(readings,hw.light()):
                channel.append(value)
            hw.sleep(S.SAMPLE_S)
        groups.append(readings)
    enter=[]
    release=[]
    for floor,line in zip(*groups):
        gap=min(floor)-max(line)
        if gap<=0:
            raise ValueError("Readings overlap; reposition sensors and repeat")
        threshold=(min(floor)+max(line))/2
        enter.append(threshold)
        release.append(threshold+gap*.1)
    result=dict(enter=enter,release=release,ports=[S.LEFT_LIGHT_PORT,S.RIGHT_LIGHT_PORT])
    temp=output.with_suffix(".tmp")
    temp.write_text(json.dumps(result,indent=2))
    os.replace(temp,output)
    print("Saved left/right enter thresholds",enter,"release",release)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("mode",nargs="?",default=S.MODE,choices=["map","replay","straight","turn","lines","rgb"])
    for p in ("s1","s2"):
        parser.add_argument("--"+p,type=int,nargs=2)
    parser.add_argument("--angle",type=float,default=S.CAL_ANGLE)
    parser.add_argument("--power",type=float)
    parser.add_argument("--distance",type=float,default=S.CAL_DISTANCE)
    args=parser.parse_args()
    if not math.isfinite(args.angle) or not math.isfinite(args.distance) or args.distance<=0:
        parser.error("Angle must be finite and distance positive")
    if len({S.LEFT_LIGHT_PORT,S.RIGHT_LIGHT_PORT,S.TOUCH_PORT,S.US_PORT})!=4 or S.LEFT_PORT==S.RIGHT_PORT:
        parser.error("Check the sensor and motor ports in settings.py")
    if args.mode=="map" and args.s1 is None and args.s2 is None:
        args.s1=list(map(int,input("Entrance S1 (x y): ").split()))
        args.s2=list(map(int,input("Entrance S2 (x y): ").split()))
        if len(args.s1)!=2 or len(args.s2)!=2:
            parser.error("Each entrance point needs two coordinates")
    if args.mode=="map" and (args.s1 is None or args.s2 is None):
        parser.error("map requires --s1 X Y --s2 X Y")
    if args.power is not None and not 0<args.power<=100:
        parser.error("power must be in (0,100]")
    if args.mode in ("map","replay"):
        radius=math.hypot(max(S.FRONT,S.REAR),max(S.LEFT,S.RIGHT))
        if radius+S.MARGIN>=S.TILE/2:
            parser.error("Measured footprint needs a wider swept turn than the centered pivot supports")
    def interrupt(signum,frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupt)
    from utils.brick import reset_brick
    try:
        hw=Hardware()
        calibration=Path("snow_light_calibration.json")
        if args.mode=="rgb":
            calibrate_rgb(hw,calibration)
            return
        if calibration.exists():
            data=json.loads(calibration.read_text())
            if data["ports"]!=[S.LEFT_LIGHT_PORT,S.RIGHT_LIGHT_PORT]:
                raise ValueError("Light ports changed; run rgb calibration again")
            S.BLACK_ENTER,S.BLACK_EXIT=data["enter"],data["release"]
        r=Robot(hw,S.PHASE_SECONDS if args.mode in ("map","replay") else S.CAL_TIMEOUT_S)
        m=Mission(r)
        path=Path(S.MAP_FILE)
        if args.mode=="map":
            result=m.map(args.s1,args.s2)
            r.stop()
            tmp=path.with_suffix(".tmp")
            tmp.write_text(json.dumps(result,indent=2))
            os.replace(tmp,path)
            print("Mapping complete. Returned home.",path)
        elif args.mode=="replay":
            m.replay(json.loads(path.read_text()))
        else:
            for _ in range(S.CAL_REPEATS):
                start=r.enc[:]
                if args.mode=="turn":
                    h=r.h
                    r.turn(args.angle,args.power or S.CAL_TURN_POWER)
                    print("Encoder angle:",r.h-h)
                    print("For a measured physical angle, multiply the direction factor by abs(encoder angle / measured angle).")
                else:
                    h=round(r.h/90)*90
                    target=r.position(h)+args.distance if args.mode=="straight" else None
                    event=m.advance(h,target,raw=args.mode=="straight",lines=S.CAL_LINES if args.mode=="lines" else None,
                                    speed=args.power or S.CAL_DRIVE_POWER)
                    if event=="contact":
                        raise RuntimeError("Calibration stopped by bumper")
                dl,dr=[math.radians(b-a)*radius for a,b,radius in zip(start,r.enc,(S.LEFT_RADIUS,S.RIGHT_RADIUS))]
                print("Left/right wheel travel (cm):",round(dl,2),round(dr,2))
                if args.mode=="straight" and dr>0:
                    print("Left/right travel ratio:",round(dl/dr,4),
                          "Check physical drift before changing motor trim.")
    finally:
        reset_brick()


if __name__=="__main__":
    main()
