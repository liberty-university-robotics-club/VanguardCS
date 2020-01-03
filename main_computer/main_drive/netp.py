#!/usr/bin/env python3
import p
import socket
import time
import threading
import sys
from math import atan
from math import atan2
from math import sin
from math import cos


class netp:
    def __init__(self, name, d={}, src="0.0.0.0", cont=lambda: True):
        self.socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        print("listen "+src)
        self.socket.bind((src, 9000))
        # self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.p = p.stlendpoint(name, d, tx=lambda m, h: self.socket.sendto(m.encode("utf-8"), (h, 9000)))
        self.cont = cont
        self.rxthread = threading.Thread(target=self.rxloop)
        self.rxthread.start()

    def rxloop(self):
        while(self.cont()):
            data, host = self.socket.recvfrom(1024)
            self.p.recl(data.decode("utf-8"), host)
            print("rec"+data.decode("utf-8"))


class turtle:
    def __init__(self, name="turtle", wdiam=1, width=1, initv=(0, 0, 0), tstep=1, src="0.0.0.0", dest="0.0.0.0"):
        # a turtle is an arbitrarily short triangle with powered weels
        # of diameter wdiam on either end of the width long edge and a caster
        # at the point (which we'll ignore.)
        self.src = src
        print("turtle src"+self.src)
        self.netp = netp(name, src=self.src)
        self.wdiam = wdiam
        self.width = width
        self.tstep = tstep
        self.controller = dest
        self.netp.p.d["posx"] = str(initv[0])
        self.netp.p.d["posy"] = str(initv[1])
        self.netp.p.d["theta"] = str(initv[2])
        self.netp.p.d["odometer"] = str(0)
        self.netp.p.d["power_left"] = str(0)
        self.netp.p.d["power_right"] = str(0)
        self.netp.p.d["__PROCESS_STATE__"] = "posx posy"
        self.netp.p.d["__REPORT_PERIOD__"] = 3

    def simc(self):
        self.netp.p.d["posy"] = str(sin(float(time.time()))*10)

    def sim(self):
        v = (float(self.netp.p.d["posx"]), float(
            self.netp.p.d["posy"]), float(self.netp.p.d["theta"]))
        l = float(self.netp.p.d["power_left"])
        r = float(self.netp.p.d["power_right"])
        # TODO: clamp power between 0 and 1
        pi = 4*atan(1)  # heh
        if(r == l):  # Simple case \inf diameter
            self.netp.p.d["posx"] = str(v[0]+sin(v[2])*r*pi*self.wdiam)
            self.netp.p.d["posy"] = str(v[1]+cos(v[2])*r*pi*self.wdiam)
        # wheels must traverse \pi width to go a circle diameter 0
        # wheels traverse wdiam 2 /\pi every rotation
        # wheels rotate once per cycle at full power
        elif(r == -l):  # next simplest: diameter 0
            self.netp.p.d["theta"] = str(v[2]+2*pi*r)
        else:  # least simple case: diameter is a thing
            # maybe it works maybe not, I don't care it's a demo
            diam = (1/(1-abs(l-r)))/self.wdiam
            center = (v[0]-sin(v[2]+pi/4)*diam, v[1]-cos(v[2]+pi/4)*diam)
            cpos = (center[0]-v[0], center[1]-v[1])
            a = atan2(cpos[0], cpos[1])
            a = a+(2*pi*diam/(r*pi*self.wdiam))
            cpos = (sin(a)*diam, cos(a)*diam)
            self.netp.p.d["posx"] = str(center[0]+cpos[0])
            self.netp.p.d["posy"] = str(center[1]+cpos[1])
            self.netp.p.d["theta"] = str(a-pi/4)

    def dostuff(self):
        while(self.netp.cont()):
            print("tx")
            self.sim()
            self.netp.p.report(self.controller)
            time.sleep(0.200)


class turtlecons:
    # the turtle console lets you drive a turtle around.
    def __init__(self, name="console", src="0.0.0.0", dest="0.0.0.0"):
        self.src = src
        print("cons src"+self.src)
        self.netp = netp(name, src=self.src)
        self.netp.p.onset(self.drawturtle)
        self.turtle = dest
        self.x = 0
        self.y = 0

    def txjoy(self):
        #	joyv=tuple(int(x.strip()) for x in raw_input().split(','))
        joyv = (0.2, 0.3)
        time.sleep(0.200)
        theta = joyv[0]
        power = joyv[1]
        r = joyv[0]+joyv[1]
        l = -joyv[0]+joyv[1]  # meh
        self.netp.p.send("power_left", l, self.turtle)
        self.netp.p.send("power_right", r, self.turtle)

    def drawturtle(self, k, v):  # wrong
        if(k == "posx"):
            self.x = (float(v)+10) % 20
        if(k == "posy"):
            self.y = (float(v)+10) % 20
        print("\x1B[2J\x1B[;H")
        for i in range(int(self.y)):
            print(" ")
        for i in range(int(self.x)):
            print("", end=" ")
        # print("\x1B["+str(int(self.x))+";"+str(int(self.y))+"H")
        print("@")
        print("x y"+str(int(self.x))+" "+str(int(self.y)))


def main():
    argv = sys.argv
    print(argv)
    if(argv[1] == "t"):
        t = turtle(src=argv[2], dest=argv[3])
        turtlethread = threading.Thread(target=t.dostuff())
        turtlethread.start()
    else:
        c = turtlecons(src=argv[2], dest=argv[3])
        while(True):
            c.txjoy()


if(__name__ == "__main__"):
    main()
