#!/usr/bin/env python
# -*- coding: utf-8 -*-

from robosim import *
from random import random, shuffle
import time
import sys
import atexit

from teamwars_parameters import *
from teamwars import *

class AgentTypeA(object):
    
    agentIdCounter = 0 # use as static
    id = -1
    robot = -1
    teamname = "A"
    robotname = ""
    
    def __init__(self,robot):
        self.id = AgentTypeA.agentIdCounter
        AgentTypeA.agentIdCounter = AgentTypeA.agentIdCounter + 1
        self.name = str(self.teamname)+str(self.id)
        self.robot = robot
    
    def getTeamname(self):
        return self.teamname
    
    def getRobotname(self):
        return self.robotname
    
    def getRobot(self):
        return self.robot
    
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-= JOUEUR A -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    
    teamname = "Equipe Alpha" # A modifier avec le nom de votre équipe
    
    def step(self,sensors):
        
        if self.id==0 :
            color( (255,0,0) )
            circle( *self.getRobot().get_centroid() , r = 22)
        elif self.id==1:
            color( (255,255,0) )
            circle( *self.getRobot().get_centroid() , r = 22)
        elif self.id ==2:
            color( (255,0,210) )
            circle( *self.getRobot().get_centroid() , r = 22)
        else :
            color( (255,255,255) )
            circle( *self.getRobot().get_centroid() , r = 22)
        
        if (iteration==0):
                self.get_stuck=[0,1,2,3,4,5,6,7,8,9]
                self.get_stuck_sol=0
                print "test"

        p = self.robot
        self.get_stuck[iteration%10]=(int(self.getRobot().get_centroid()[0]),int(self.getRobot().get_centroid()[1])) #SE DECOINCER
        if (self.get_stuck.count(self.get_stuck[0])==10) or self.get_stuck_sol!=0:
            if self.get_stuck_sol==0:
                self.get_stuck_sol=20
            self.setTranslationValue(-1)
            self.setRotationValue(0.5)
            self.get_stuck_sol-=1
            return


        evite_joueur=False
        evite_obstacle=False
        suis_joueur=False
        mes_sensors=sensors[p]
        mes_sensor_dist=[]

        for i in range( len (SensorBelt)):
            if mes_sensors[i].dist_from_border > maxSensorDistance:
                mes_sensor_dist.append(maxSensorDistance)
            else:
                mes_sensor_dist.append(mes_sensors[i].dist_from_border)
            
        sensor_actif=mes_sensor_dist.index(min(mes_sensor_dist))
        
        for dist in (sorted(mes_sensor_dist)):
            print agents
            if  dist < maxSensorDistance/2 and mes_sensors[mes_sensor_dist.index(dist)].layer=='joueur' and  agents[mes_sensors[mes_sensor_dist.index(dist)].sprite.numero].teamname==self.teamname:
                sensor_actif=mes_sensor_dist.index(dist)
                evite_joueur=True
                break
            elif dist< maxSensorDistance and mes_sensors[mes_sensor_dist.index(dist)].layer!='joueur' :
                evite_obstacle=True
                sensor_actif=mes_sensor_dist.index(dist)
                break
            elif  (self.id==0 or self.id==1 or self.id==2 or self.id==3) and dist < maxSensorDistance and mes_sensors[mes_sensor_dist.index(dist)].layer=='joueur' and agents[mes_sensors[mes_sensor_dist.index(dist)].sprite.numero].teamname != self.teamname:
                suis_joueur=True
                
                
        if evite_joueur==True :
            self.setRotationValue(-SensorBelt[sensor_actif])
            self.setTranslationValue(1)
        elif evite_obstacle==True :
            self.setRotationValue(-SensorBelt[sensor_actif])
            self.setTranslationValue(1)
        elif suis_joueur==True:
            self.setRotationValue(SensorBelt[sensor_actif])
            self.setTranslationValue(1)
        else :
             self.setRotationValue(0)
             self.setTranslationValue(1)
            
            
            
            
        return
    
    
    def displayInfo(self,sensors):
        
        p = self.robot
        

        # monitoring - affiche diverses informations sur l'agent et ce qu'il voit.
        # pour ne pas surcharger l'affichage, je ne fais ca que pour le player 1
        if verbose == True and self.id == 0:
            
            efface()    # j'efface le cercle bleu de l'image d'avant
            color( (0,0,255) )
            circle( *game.player.get_centroid() , r = 22) # je dessine un rond bleu autour de ce robot
            
            print "\n# Current robot at " + str(p.get_centroid()) + " with orientation " + str(p.orientation())
            
            sensor_infos = sensors[p] # sensor_infos est une liste de namedtuple (un par capteur).
            for i,impact in enumerate(sensors[p]):  # impact est donc un namedtuple avec plein d'infos sur l'impact: namedtuple('RayImpactTuple', ['sprite','layer','x', 'y','dist_from_border','dist_from_center','rel_angle_degree','abs_angle_degree'])
                if impact.dist_from_border > maxSensorDistance:
                    print "- sensor #" + str(i) + " touches nothing"
                else:
                    print "- sensor #" + str(i) + " touches something at distance " + str(impact.dist_from_border)
                    if impact.layer == 'joueur':
                        playerTMP = impact.sprite
                        print "  - type: robot no." + str(playerTMP.numero) + " with name "
                        print "    - x,y = " + str( playerTMP.get_centroid() ) + ")" # renvoi un tuple
                        print "    - orientation = " + str( playerTMP.orientation() ) + ")" # p/r au "nord"
                    elif impact.layer == 'obstacle':
                        print "  - type obstacle"
                    else:
                        print "  - type boundary of window"
        return

    
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
    
    def getDistanceFromSensor(self, index):
        return max(sensor_infos[2].dist_from_border,maxSensorDistance)
    
    def getTeamnameFromRobot(self,agent):
        return agents[agent.sprite.numero].teamname
