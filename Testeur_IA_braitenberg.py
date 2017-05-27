

#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# multirobot.py
# Contact (ce fichier uniquement): nicolas.bredeche(at)upmc.fr
# 
# Description:
#   Template pour simulation mono- et multi-robots type khepera/e-puck/thymio
#   Ce code utilise pySpriteWorld, développé par Yann Chevaleyre (U. Paris 13)
# 
# Dépendances:
#   Python 2.x
#   Matplotlib
#   Pygame
# 
# Historique: 
#   2016-03-28__23:23 - template pour 3i025 (IA&RO, UPMC, licence info)
#
# Aide: code utile
#   - Partie "variables globales"
#   - La méthode "step" de la classe Agent
#   - La fonction setupAgents (permet de placer les robots au début de la simulation)
#   - La fonction setupArena (permet de placer des obstacles au début de la simulation)
#   - il n'est pas conseillé de modifier les autres parties du code.
# 

from robosim import *
from random import random, shuffle
import time
import sys
import atexit
from itertools import count
import math


'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''
'''  Aide                 '''
'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''

#game.setMaxTranslationSpeed(3) # entre -3 et 3
# size of arena: 
#   screenw,screenh = taille_terrain()
#   OU: screen_width,screen_height

'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''
'''  variables globales   '''
'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''

game = Game()

agents = []
screen_width=512 #512,768,... -- multiples de 32  
screen_height=512 #512,768,... -- multiples de 32
nbAgents = 8

maxSensorDistance = 30              # utilisé localement.
maxRotationSpeed = 5
maxTranslationSpeed = 1

SensorBelt = [-170,-80,-40,-20,+20,40,80,+170]  # angles en degres des senseurs (ordre clockwise)

maxIterations = -2000 # infinite: -1

showSensors = True
frameskip = 0   # 0: no-skip. >1: skip n-1 frames
verbose = True

'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''
'''  Classe Agent/Robot   '''
'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''

class Agent(object):
    
    agentIdCounter = 0 # use as static
    id = -1
    robot = -1
    name = "Equipe Alpha" # A modifier avec le nom de votre équipe
    params=[0.05265491446035409, 0.9923297145638675, -0.9882767912201784, -0.9253132047705936, -0.9959532290633177, -0.9998660349745442, -1, 0.079796612613471]
#    params=[-0.03548389690077048, -0.9822629103908489, -0.9890044536660909, -0.993469997839344, -1, -1, -0.9807045820270879, 0.9874256496707055]
    
#    params=[0.9406132937524698, 0.9914268447087623, 0.9670831670637168, -0.9757575060229612, -0.9811419957052052, -0.9958482501040135, -0.9747758471832151, 0.9801930678710344]
#    params=[1, 0.9276493902310463, 0.9454551258298026, 0.9441142182476501, 1, 0.998510172489955, -0.9789801227029912, -0.02094692949715693]


#    params=[0.9890248751383272, 0.0491143353806641, -1, -0.9983561121083748, -0.9401295904613223, -1, -1, -0.03605404967076988] #1600
#    params=[-0.008249147377788573, -0.024587007201769262, -0.9561752619875612, -0.9439809284360065, -0.9608109844994593, -1, -0.9510761424240844, -0.02064631087223302]#1580
#    params=[0.04423182017936696, 0.934528997030673, 1, -0.9763015264091525, -0.9760219981611424, -0.9485416005130439, -0.92872540912186, 0.01677811524322887]#1960
#    params=[0.07078766219243485, 0.967535151728737, 0.9375656202414974, -0.9755058188842111, -0.9383888102864946, -0.9468519126483007, -0.961512204242919, 0.030653904853620483]#1994
#    params=[0.9678461245663448, 0.9866195903738716, 0.9977037720845559, 0.9608544978654356, -0.9911859653351214, -0.9543537814045772, -0.9874777750988968, -0.9660823306315498]#2101
#    params=[-0.05886014950135228, 0.9650979775090678, 0.9842679462439766, -0.9795896031621049, -0.9511106350344977, -0.9343985042162506, -0.9725981458114215, -0.004659627276041908]#1971
#    params=[0.9730807682013304, 1, 0.9751484572000395, 1, 0.9844652496119043, -0.9926584516272665, -0.9590796344985832, -0.05130215192056453]#2034
    
#    params=[-0.9641602516894809, 0.9891347525809993, 0.9458547460027062, 0.9992965445727523, 0.9919598154213621, 0.9336711394730003, 0.9648199833846114, 0.9896805329286398, -0.9873728912672346]


#    params=[-1, 0.9807479894343233, 1, 0.975964293478959, 0.9845282227227201, 0.9575817213974537, 0.9592930271443981, -0.9971046098100891, -0.04087250372605146]
    def __init__(self,robot):
        self.id = Agent.agentIdCounter
        Agent.agentIdCounter = Agent.agentIdCounter + 1
        #print "robot #", self.id, " -- init"
        self.robot = robot
        
    def getRobot(self):
        return self.robot

    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    def step(self):
        color( (255,0,0) )
        circle( *self.getRobot().get_centroid() , r = 22)
        p = self.robot
#        if iteration==0:
#            self.params=[1,1,1,1,-1,-1,-1,-1]


        evite_obstacle=False
        mes_sensors=sensors[p]
        mes_sensor_dist=[]

        for i in range( len (SensorBelt)):
            if mes_sensors[i].dist_from_border > maxSensorDistance:
                mes_sensor_dist.append(maxSensorDistance)
            else:
                mes_sensor_dist.append(mes_sensors[i].dist_from_border)
            
        sensor_actif=mes_sensor_dist.index(min(mes_sensor_dist))
        
        for dist in (sorted(mes_sensor_dist)):
            total=(dist/maxSensorDistance)
            if dist< maxSensorDistance :#and mes_sensors[mes_sensor_dist.index(dist)].layer!='joueur' :
                evite_obstacle=True
                sensor_actif=mes_sensor_dist.index(dist)
                break

                
                
        if evite_obstacle==True :
            self.setRotationValue(self.params[sensor_actif])
            self.setTranslationValue(1)
        else :
             self.setRotationValue(0)
             self.setTranslationValue(1)
                        
        return
        


        return
            
            
            
        return

    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    def setTranslationValue(self,value):
        if value > 1:
            #print "[WARNING] translation value not in [-1,+1]. Normalizing."
            value = maxTranslationSpeed
        elif value < -1:
            #print "[WARNING] translation value not in [-1,+1]. Normalizing."
            value = -maxTranslationSpeed
        else:
            value = value * maxTranslationSpeed
        self.robot.forward(value)

    def setRotationValue(self,value):
        if value > 1:
            #print "[WARNING] translation value not in [-1,+1]. Normalizing."
            value = maxRotationSpeed
        elif value < -1:
            #print "[WARNING] translation value not in [-1,+1]. Normalizing."
            value = -maxRotationSpeed
        else:
            value = value * maxRotationSpeed
        self.robot.rotate(value)


'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''
'''  Fonctions init/step  '''
'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''

def setupAgents():
    global screen_width, screen_height, nbAgents, agents, game

    # Make agents
    nbAgentsCreated = 0
    for i in range(nbAgents):
        while True:
            p = -1
            while p == -1: # p renvoi -1 s'il n'est pas possible de placer le robot ici (obstacle)
                p = game.add_players( (random()*screen_width , random()*screen_height) , None , tiled=False)
            if p:
                p.oriente( random()*360 )
                p.numero = nbAgentsCreated
                nbAgentsCreated = nbAgentsCreated + 1
                agents.append(Agent(p))
                break
    game.mainiteration()


def setupArena():
    for i in range(6,13):
        addObstacle(row=3,col=i)
    for i in range(3,10):
        addObstacle(row=12,col=i)
    addObstacle(row=4,col=12)
    addObstacle(row=5,col=12)
    addObstacle(row=6,col=12)
    addObstacle(row=11,col=3)
    addObstacle(row=10,col=3)
    addObstacle(row=9,col=3)
    return



def stepWorld():
    # chaque agent se met à jour. L'ordre de mise à jour change à chaque fois (permet d'éviter des effets d'ordre).
    shuffledIndexes = [i for i in range(len(agents))]
    shuffle(shuffledIndexes)     ### TODO: erreur sur macosx
    for i in range(len(agents)):
        agents[shuffledIndexes[i]].step()
    return


'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''
'''  Fonctions internes   '''
'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''

def addObstacle(row,col):
    # le sprite situe colone 13, ligne 0 sur le spritesheet
    game.add_new_sprite('obstacle',tileid=(0,13),xy=(col,row),tiled=True)

class MyTurtle(Turtle): # also: limit robot speed through this derived class
    maxRotationSpeed = maxRotationSpeed # 10, 10000, etc.
    def rotate(self,a):
        mx = MyTurtle.maxRotationSpeed
        Turtle.rotate(self, max(-mx,min(a,mx)))

def onExit():
    print "\n[Terminated]"

'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''
'''  Main loop            '''
'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''

init('vide3',MyTurtle,screen_width,screen_height) # display is re-dimensioned, turtle acts as a template to create new players/robots
game.auto_refresh = False # display will be updated only if game.mainiteration() is called
game.frameskip = frameskip
atexit.register(onExit)

setupArena()
setupAgents()
game.mainiteration()

iteration = 0
while iteration != maxIterations:
    # c'est plus rapide d'appeler cette fonction une fois pour toute car elle doit recalculer le masque de collision,
    # ce qui est lourd....
    sensors = throw_rays_for_many_players(game,game.layers['joueur'],SensorBelt,max_radius = maxSensorDistance+game.player.diametre_robot() , show_rays=showSensors)
    stepWorld()
    game.mainiteration()
    iteration = iteration + 1
