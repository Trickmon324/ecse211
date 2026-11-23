"""Drive both phases using simulated encoders, dark grid tape, switch and side sonar."""
import contextlib
import io
import math
import unittest
import settings as S
from robot import Robot, axes
from mapping import Mission


class Arena:
    def __init__(self,boxes):
        self.boxes=set(boxes)
        self.x=self.y=self.h=self.clock=0.
        self.ticks=[0.,0.]
        self.p=(0,0)
        self.contacts=0
    def power(self,l,r):
        self.p=l,r
    def sleep(self,t):
        self.clock+=t
        dl,dr=(v*.9*t for v in self.p)
        dh=math.degrees((dl-dr)/S.TRACK)
        f,_=axes(self.h+dh/2)
        self.x+=(dl+dr)*f[0]/2
        self.y+=(dl+dr)*f[1]/2
        self.h+=dh
        for i,d in enumerate((dl,dr)):
            self.ticks[i]+=math.degrees(d/S.LEFT_RADIUS)
    def enc(self):
        return self.ticks[:]
    def now(self):
        return self.clock
    def point(self,fwd,right):
        f,r=axes(self.h)
        return (self.x+fwd*f[0]+right*r[0],self.y+fwd*f[1]+right*r[1])
    def box(self,p):
        return tuple(math.floor(a/S.TILE) for a in p) in self.boxes
    def pressed(self):
        hit=self.box(self.point(*S.TOUCH))
        self.contacts+=int(hit)
        return hit
    def light(self):
        values=[]
        for mount in S.LIGHTS:
            p=self.point(*mount)
            values.append(220 if any(abs(v-round(v/S.TILE)*S.TILE)<.8 for v in p) else 575)
        return values
    def side(self):
        assert self.p[0]*self.p[1]>=0, "side sonar sampled in a turn"
        for dist in range(1,251):
            if self.box(self.point(S.US[0],S.US[1]+dist)):
                return dist
        return 255


class MappingTests(unittest.TestCase):
    def run_layout(self,boxes,expected,entrance=((1,3),(1,2))):
        sim=Arena(boxes)
        m=Mission(Robot(sim,120))
        with contextlib.redirect_stdout(io.StringIO()):
            record=m.map(*entrance)
        self.assertEqual([c["turn"] for c in record["corners"]],expected)
        self.assertLess(math.hypot(sim.x,sim.y),4)
        sim=Arena(boxes)
        with contextlib.redirect_stdout(io.StringIO()):
            Mission(Robot(sim,120)).replay(record)
        self.assertEqual(sim.contacts,0)
        self.assertLess(math.hypot(sim.x,sim.y),4)
    def test_sample(self):
        boxes={(x,3) for x in range(1,7)}|{(x,1) for x in range(1,5)}
        self.run_layout(boxes|{(6,y) for y in range(3)}|{(4,0)},[90])
    def test_straight(self):
        self.run_layout({(x,y) for x in range(1,4) for y in (1,3)},[])
    def test_left(self):
        self.run_layout({(x,0) for x in range(1,5)}|{(1,2),(2,2),(4,1),(4,2)},[-90],((1,1),(1,2)))
    def test_skewed_lines(self):
        for skew in (4,-4):
            sim=Arena([])
            sim.x=S.TILE/2
            sim.h=skew
            r=Robot(sim,120)
            r.x=sim.x
            with contextlib.redirect_stdout(io.StringIO()) as log:
                self.assertEqual(Mission(r).advance(0,lines=2),"lines")
            self.assertIn("first="+("L" if skew>0 else "R"),log.getvalue())
            self.assertEqual(r.last_line,2*S.TILE)
            self.assertLess(abs(sim.h),abs(skew))
    def test_straight_calibration(self):
        sim=Arena([])
        r=Robot(sim,120)
        self.assertEqual(Mission(r).advance(0,60.96,raw=True),"arrived")
        self.assertAlmostEqual(sim.y,60.96,delta=.4)
        self.assertAlmostEqual(sim.x,0)

    def test_multiple_turns(self):
        sim=Arena([])
        r=Robot(sim,120)
        r.turn(-720,20)
        self.assertAlmostEqual(sim.h,-720,delta=1.5)
        r.turn(1080,23)
        self.assertAlmostEqual(sim.h,360,delta=2)


if __name__=="__main__":
    unittest.main()
