#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# multirobot.py
# Contact (ce fichier uniquement): nicolas.bredeche(at)upmc.fr
# 
# Description:
#   Template pour robotique evolutionniste simple 
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
from random import random, shuffle, randint,gauss
import math
import time
import sys
import atexit
from itertools import count
import copy


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
nbAgents = 1

maxSensorDistance = 30              # utilisé localement.
maxRotationSpeed = 5
maxTranslationSpeed = 3
SensorBelt = [-170,-80,-40,-20,+20,40,80,+170]  # angles en degres des senseurs

maxIterations = -1000 # infinite: -1

showSensors = True
frameskip = 1000   # 0: no-skip. >1: skip n-1 frames
verbose = True
nbIteration=1000
nbPopu=3000000000000000000000000000000000000000000000000000000000
fichier="Param_gauss_avec_biais.txt"
ecrire_fichier=False

occupancyGrid = []
for y in range(screen_height/16):
    l = []
    for x in range(screen_width/16):
        l.append("_")
    occupancyGrid.append(l)
def resetOccupancyGrid():
    for i,x in enumerate(occupancyGrid):
          for j,y in enumerate(x):
              occupancyGrid[i][j]="_"

'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''
'''  Classe Agent/Robot   '''
'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''

class Agent(object):
    
    agentIdCounter = 0 # use as static
    id = -1
    robot = -1
    name = "Equipe Evol" # A modifier avec le nom de votre équipe
    params = bestParams = []
    fitness = bestFitness = 0
    pos=[(screen_width/2,screen_height/2),(screen_width/2,screen_height/2)]
    rot=[0,0]
    rotation=0
    sigma=0.01
    agentType="A"

    
    def __init__(self,robot):
        self.id = Agent.agentIdCounter
        Agent.agentIdCounter = Agent.agentIdCounter + 1
        #print "robot #", self.id, " -- init"
        self.robot = robot

        
    def getType(self):
        return self.agentType


    def getRobot(self):
        return self.robot

    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    def step(self):
        
        self.stepSearchMethod()
        self.stepController()


    def stepSearchMethod(self): # random search
        if iteration ==0 :
            for i in range(len(SensorBelt)*2+2):
                self.params.append((random()*20-10))

        if iteration % nbIteration == 0:

            # affiche la performance (et les valeurs de parametres)
            if iteration != 0 and iteration < nbIteration * nbPopu:
                if self.bestFitness < self.fitness:
                    self.bestFitness = self.fitness
                    self.bestParams = copy.deepcopy(self.params)
                    if ecrire_fichier==True :
                        f=open(fichier,"w")
                        s="fitness = " + str(self.bestFitness) + "params = " +str(self.bestParams)
                        f.write(s)
                        f.close()
                    self.sigma=0.01
                    if self.sigma < 0.01 :
                        self.sigma=0.01
                    print "if"
                else:
                   
                    self.sigma*=2
                    if self.sigma >5:
                        self.sigma =5
                    if self.sigma>=  5 and self.bestFitness >100:
                        self.sigma=0.01
                    if self.bestFitness >200:
                        self.sigma=0.01
                    print "else"
                    #self.sigma*=0.125
            print "sigma :",str(self.sigma) 
            print "Fitness:",self.fitness, "(best:", self.bestFitness,")"
            print self.bestParams
            print "Parameters:", str(self.params)
            print "Evaluation no.", int(iteration/nbIteration)
            
            # repositionne le robot
            p = self.robot
            p.set_position(256,250)
            p.oriente( 0 )
            
            # genere un nouveau jeu de paramètres
            if self.bestFitness != 0 :
                self.params = copy.deepcopy(self.bestParams)
                for i in range(len(self.params)):
                    self.params[i]+= self.sigma * gauss(0,1)
            else :
                print "random"
                for i in range(len(self.params)):
                    self.params[i]=((random()*20-10))
                
            if iteration>=nbIteration * nbPopu:
                print "best params"
                print self.bestParams
                print self.bestFitness
                self.params=copy.deepcopy(self.bestParams)
            # remet la fitness à zéro
            self.resetFitness()


    def stepController(self):



        #print "robot #", self.id, " -- step"
        p = self.robot
        sensor_infos = sensors[p]
        
        translation = 0
        rotation = 0

        k = 0
        
        for i in range(len(SensorBelt)):
            dist = sensor_infos[i].dist_from_border/maxSensorDistance
            translation += dist * self.params[k]
            k = k + 1
        translation+= 1.0* self.params[k]
        k=k+1

        for i in range(len(SensorBelt)):
            dist = sensor_infos[i].dist_from_border/maxSensorDistance
            rotation += dist * self.params[k]
            k = k + 1
        rotation+=1.0 * self.params[k]
        k+=1
            
            
            
        translation = math.tanh(translation)
        rotation = math.tanh(rotation)
        #print "r =",rotation," - t =",translation
        self.rotation=rotation
        self.setRotationValue( rotation )
        self.setTranslationValue( translation )
        
        self.pos[iteration%2]=self.robot.get_centroid()
        self.rot[iteration%2]=self.robot.orientation
        self.updateFitness()

        return

    def resetFitness(self):
        self.fitness = 0

    def updateFitness(self):
        tab_min=[]
        for i in range(len(SensorBelt)):
            tab_min.append(sensors[self.robot][i].dist_from_border)
        mindist=min(tab_min)        
        if mindist>maxSensorDistance:
            mindist=maxSensorDistance*1.0
        mindist/=maxSensorDistance
        tmp_occupation=occupation()
        tmp_occupation=tmp_occupation*1.0 / ((screen_height/16)*(screen_width/16))
        tmp_trans=math.sqrt((self.pos[iteration%2][0]-self.pos[(iteration+1)%2][0])**2 + (self.pos[iteration%2][1]-self.pos[(iteration+1)%2][1])**2)/maxTranslationSpeed
        self.fitness+= (1-abs(self.rotation)) * tmp_trans * mindist * tmp_occupation
        #print ((1-abs(self.rotation)) * tmp_trans * mindist)
        
    
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    def setTranslationValue(self,value):
        if value > 1:
            print "[WARNING] translation value not in [-1,+1]. Normalizing."
            value = maxTranslationSpeed
        elif value < -1:
            print "[WARNING] translation value not in [-1,+1]. Normalizing."
            value = -maxTranslationSpeed
        else:
            value = value * maxTranslationSpeed
            self.robot.forward(value)

    def setRotationValue(self,value):
        if value > 1:
            print "[WARNING] translation value not in [-1,+1]. Normalizing."
            value = maxRotationSpeed
        elif value < -1:
            print "[WARNING] translation value not in [-1,+1]. Normalizing."
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
#    for i in range(6,13):
#        addObstacle(row=3,col=i)
#    for i in range(3,10):
#        addObstacle(row=12,col=i)
#    addObstacle(row=4,col=12)
#    addObstacle(row=5,col=12)
#    addObstacle(row=6,col=12)
#    addObstacle(row=11,col=3)
#    addObstacle(row=10,col=3)
#    addObstacle(row=9,col=3)
    for i in range(4,10):
        addObstacle(row=i,col=10)
        addObstacle(row=i,col=5)

    return



def stepWorld():
    # chaque agent se met à jour. L'ordre de mise à jour change à chaque fois (permet d'éviter des effets d'ordre).
    shuffledIndexes = [i for i in range(len(agents))]
    shuffle(shuffledIndexes)     ### TODO: erreur sur macosx
    for i in range(len(agents)):
        agents[shuffledIndexes[i]].step()
        coord = agents[shuffledIndexes[i]].getRobot().get_centroid()
        occupancyGrid[int(coord[0])/16][int(coord[1])/16] = agents[shuffledIndexes[i]].getType() #
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
    
def occupation():
    nbA = 0
    for y in range(screen_height/16):
        for x in range(screen_width/16):
            if occupancyGrid[x][y] == "A":
                nbA = nbA+1

    return nbA

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
