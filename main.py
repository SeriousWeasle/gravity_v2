#dependencies
from catsnake import *
import math
import random
from PIL import Image, ImageDraw
import time
from colorama import Fore, Back, Style

#global variables
dt = 0.1              #timestep size
G = 6.67408e-11     #gravitational constant
framecount = 25000

#root for higher power numbers
def powroot(num, pow):
    return num ** (1 / pow)

#physics object
class physobject:
    def __init__(self, mass:float, density:float, col:color, position:vector2):
        #store params
        self.m = mass
        self.d = density
        self.c = col
        self.p = position
        self.v = vector2(0, 0)
        #calculate volume using V=mass/density
        self.s = self.m / self.d
        #calculate radius of sphere for drawing circle
        self.r = powroot(self.s/((4/3)*math.pi), 3)

    #adds a force for 1 timestep
    def addForce(self, forcevector:vector2):
        #F = m * a rewritten to A = F/m
        accel = forcevector / self.m
        #v = a * t
        self.v += accel * dt

    def tick(self):
        #s = v * t
        self.p += self.v * dt
    
    def setspeed(self, speed:vector2):
        self.v = speed

def addphysobjs(o1:physobject, o2: physobject):
    #calculate new mass
    mass = o1.m + o2.m
    #calculate fractions for other properties
    frac1 = o1.m/mass
    frac2 = o2.m/mass
    density = (o1.d * frac1) + (o2.d * frac2)
    col = ((o1.c * frac1) + (o2.c * frac2)).toint()
    vel = ((o1.v * o1.m) + (o2.v * o2.m)) / (o1.m + o2.m)
    pos = (o1.p * frac1) + (o2.p * frac2)
    po = physobject(mass, density, col, pos)
    po.setspeed(vel)
    return po

class simulationhandler:
    def __init__(self, objcount, min_sm, max_sm, min_d, max_d, min_x, max_x, min_y, max_y):
        #make empty array for storing objects
        self.objs = []
        #make n objects
        for i in range(objcount):
            #choose random mass
            mass = scaleLinear(random.random(), 0, 1, min_sm, max_sm)
            #choose random density
            density = min_d
            #choose random color
            col = color(random.randint(0, 255), random.randint (0, 255), random.randint(0, 255))
            #choose random position
            pos = point2(scaleLinear(random.random(), 0, 1, min_x, max_x), scaleLinear(random.random(), 0, 1, min_y, max_y))
            #add new object with set parameters
            self.objs.append(physobject(mass, density, col, pos))
    #add a random start force to each object
    def addStartForce(self, min_f, max_f):
        #go over objects
        for o in self.objs:
            #make random force beteween set bounds
            fv = vector2(scaleLinear(random.random(), 0, 1, -1, 1), scaleLinear(random.random(), 0, 1, -1, 1)).clamp(scaleLinear(random.random(), 0, 1, min_f, max_f))
            #add this force vector to object
            o.addForce(fv)
    #step forward one tick
    def calculateforces(self):
        #calculate interactions; loop over every object
        for affectedobject in self.objs:
            #make force vector to apply
            force = vector2(0, 0)
            #loop over every object
            for other in self.objs:
                #if it is not itself
                if affectedobject != other:
                    #calculate distance
                    distvec = other.p - affectedobject.p
                    #calculate force based on gravitational formula
                    forcelength = G + ((affectedobject.m * other.m)/(distvec.length() ** 2))
                    #add force to composite force
                    force += distvec.clamp(forcelength)
            #add force exerted by every other object to object
            affectedobject.addForce(force)
    
    def calculatecollisions(self, fn):
        colcount = 0
        runcount = 0
        #variable for checking if collisions should be recalculated
        runcheck = True
        while runcheck:
            runcount += 1
            #set to false in the hope of not finding a collision
            runcheck = False
            #list of collided object indexes
            collided = []
            #new list of objects
            newobjs = []
            #go over every object
            for i, affectedobject in enumerate(self.objs):
                #compare to every other object
                for j, other in enumerate(self.objs):
                    #if it is not itself
                    if affectedobject != other:
                        #calculate distance
                        dist = (affectedobject.p - other.p).length()
                        #calculate combined radii
                        coldist = affectedobject.r + other.r
                        #if distance is smaller than combined radii
                        if dist <= coldist and i not in collided and j not in collided:
                            #signify that the collision checker should run again
                            runcheck = True
                            #add both objects to the collided collection
                            collided.append(i)
                            collided.append(j)
                            #add combined of both to new objects
                            newobjs.append(addphysobjs(affectedobject, other))
                            colcount += 1
                #if the object has not collided, add to new objects
                if i not in collided:
                    newobjs.append(affectedobject)
            #set new objects as current objects and run collision checker again if needed
            self.objs = newobjs
        print(Fore.YELLOW + "[" + str(fn+1), "/", str(framecount) + "] Calculated", colcount, "collision(s) in", runcount, "run(s)" + Fore.RESET)
        print(Fore.WHITE + "[" + str(fn+1), "/", str(framecount) + "]", str(len(self.objs)), "objects left" + Fore.RESET)

    #move every object 1 tick forward in time
    def advancetime(self):
        #go over each object
        for o in self.objs:
            #move it
            o.tick()

    def drawframe(self, fn):
        frame = Image.new("RGB", (4096 , 4096), "black")
        offset = vector2(2048, 2048)
        framedraw = ImageDraw.Draw(frame)
        for o in self.objs:
            bl_x = int(offset.x() + o.p.x() - o.r)
            bl_y = int(offset.y() + o.p.y() - o.r)
            tr_x = int(offset.x() + o.p.x() + o.r)
            tr_y = int(offset.y() + o.p.y() + o.r)
            if bl_x > 0 and bl_x < 4096 and bl_y > 0 and bl_y < 4096 and tr_x > 0 and tr_x < 4096 and tr_y > 0 and tr_y < 4096:
                framedraw.ellipse([(bl_x, bl_y), (tr_x, tr_y)], fill=(o.c.r(), o.c.g(), o.c.b()), outline=(o.c.r(), o.c.g(), o.c.b()))
        frame.save("./frames/" + str(fn) + ".png")

        with open("./framedata/" + str(fn) + ".csv", "w") as fd:
            fd.write("x;y;vx;vy;m;d;r;g;b\n")
            for o in self.objs:
                fd.write(str(o.p.x()) + ";" + str(o.p.y()) + ";" + str(o.v.x()) + ";" + str(o.v.y()) + ";" + str(o.m) + ";" + str(o.r * 2) + ";" + str(o.c.r()) + ";" + str(o.c.g()) + ";" + str(o.c.b()) + "\n")

    #advance time forward by one timestep
    def tick(self, fn):
        print(Fore.RED + "[" + str(fn+1),"/", str(framecount) + "] Start force calculation" + Fore.RESET)
        st = time.time()
        self.calculateforces()
        print(Fore.GREEN + "[" + str(fn+1),"/", str(framecount) + "] calculated forces in", round(time.time() - st, 3), "seconds" + Fore.RESET)
        print(Fore.MAGENTA + "[" + str(fn+1),"/", str(framecount) + "] Start collision calculation" + Fore.RESET)
        st = time.time()
        self.calculatecollisions(fn)
        print(Fore.BLUE + "[" + str(fn+1),"/", str(framecount) + "] calculated collisions in", round(time.time() - st, 3), "seconds" + Fore.RESET)
        self.advancetime()
        self.drawframe(fn)

sim = simulationhandler(4096, 0.1, 1000, 1, 1, -2048, 2048, -2048, 2048)
sim.addStartForce(0, 1000)
for i in range(framecount):
    sim.tick(i)