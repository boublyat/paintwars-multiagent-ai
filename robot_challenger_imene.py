import math
import random
from robot import *

nb_robots = 0

class Robot_player(Robot):

    team_name = "imene"  
    robot_id = -1
    memory = 0

    def __init__(self, x_0, y_0, theta_0, name="n/a", team="n/a"):
        global nb_robots
        self.robot_id = nb_robots
        nb_robots += 1
        super().__init__(x_0, y_0, theta_0, name="Robot "+str(self.robot_id), team=self.team_name)

    def step(self, sensors, sensor_view=None, sensor_robot=None, sensor_team=None):
        
        
        translation = 0
        rotation = 0
        
        # Lecture capteurs
        front = sensors[sensor_front]
        front_left = sensors[sensor_front_left]
        front_right = sensors[sensor_front_right]
        left = sensors[sensor_left]
        right = sensors[sensor_right]

        # ==============================================================================
        # ROBOT 0 et 3 : Braitenberg Avoider 
        # ==============================================================================
        if self.robot_id == 0 or self.robot_id == 3:
            # Comportement déblocage (Si memory < 0, on recule)
            if self.memory < 0:
                self.memory += 1
                translation = 0.2
                rotation = left - right + (random.random()-0.5)*0.15
                return translation, rotation, False
            
            # Détection d'obstacle pour incrémenter le compteur
            if front < 0.5:
                self.memory += 1
            else :
                self.memory = max(0, self.memory - 1)
            
            # Activation du mode déblocage
            if self.memory > 5:
                self.memory = -10 

            # Comportement normal (Avoider)
            translation = front*0.3 + 0.7
            rotation = 0.3*(1-front) + 0.4*(front_left - front_right) + 0.3*(left - right) + (random.random()-0.5)*0.2
        
        # ==============================================================================
        # ROBOT 1 : Love Wall 
        # ==============================================================================
        elif self.robot_id == 1:
            # 1. Gestion Timer
            if front < 0.3:
                self.memory = 15
            else:
                self.memory = max(0, self.memory - 1)

            # 2. Calcul Mouvement
            if self.memory > 0: # Mode Dégagement pour qu'il decale du mur un peu 
                translation = -0.2 
                rotation = -1.0 
            else: # Mode Suivi
                translation = 1.0 - (0.7 * front) #
                target = 0.2
                wall_hug = (left - target) * 2.0 #si le mur est trop loin , resultat positif tourne a gauche  , si resultat negatif tourne a droite pour glisser le long du mur parfaitement 
                avoid_front = (1.0 - front) * -2.5
                rotation = wall_hug + avoid_front
                
                if left > 0.9 and front > 0.9:
            
                    rotation = 0.6
        
        # ==============================================================================
        # ROBOT 2 : Algorithme Génétique + Subsomption
        # ==============================================================================
        elif self.robot_id == 2:
            # Paramètres optimisés
            bestParam = [1, 0, 1, 1, -1, 1, 1, -1] #score 976

            # Analyse des capteurs pour la subsomption
            sensor_to_wall = []
            sensor_to_robot = []
            for i in range(0, 8):
                if sensor_view[i] == 1: # Mur
                    sensor_to_wall.append(sensors[i])
                    sensor_to_robot.append(1.0)
                elif sensor_view[i] == 2: # Robot
                    sensor_to_wall.append(1.0)
                    sensor_to_robot.append(sensors[i])
                else: # Rien
                    sensor_to_wall.append(1.0)
                    sensor_to_robot.append(1.0)

            near_wall = max(1 - sensor_to_wall[sensor_front], 1 - sensor_to_wall[sensor_front_left], 1 - sensor_to_wall[sensor_front_right])
            near_robot = max(1 - sensor_to_robot[sensor_front], 1 - sensor_to_robot[sensor_front_left], 1 - sensor_to_robot[sensor_front_right])

            # ---------- PRIORITÉ 1 : DÉBLOCAGE ----------
            if self.memory > 10:
                translation = -0.2  #recule un peu plutôt que d'avancer
                rotation = left - right + (random.random() - 0.5)
                self.memory = 0 # Reset immédiat 

            # ---------- PRIORITÉ 2 : HATE WALL ----------
            elif near_wall > 0.5:
                translation = sensor_to_wall[sensor_front] * 0.3 + 0.6
                rotation = (
                    0.2 * (1 - sensor_to_wall[sensor_front])
                    + 0.4 * (sensor_to_wall[sensor_front_left] - sensor_to_wall[sensor_front_right])
                    + 0.4 * (sensor_to_wall[sensor_left] - sensor_to_wall[sensor_right])
                    + (random.random() - 0.5) * 0.2
                )

            # ---------- PRIORITÉ 3 : HATE BOT ----------
            elif near_robot > 0.5:
                translation = sensor_to_robot[sensor_front] * 0.3 + 0.6
                rotation = (
                    0.2 * (1 - sensor_to_robot[sensor_front])
                    + 0.4 * (sensor_to_robot[sensor_front_left] - sensor_to_robot[sensor_front_right])
                    + 0.4 * (sensor_to_robot[sensor_left] - sensor_to_robot[sensor_right])
                    + (random.random() - 0.5) * 0.2
                )

            # ---------- PRIORITÉ 4 : GENETIC  ----------
            else:
                translation = math.tanh(bestParam[0] + bestParam[1] * front_left + bestParam[2] * front + bestParam[3] * front_right)
                rotation = math.tanh(bestParam[4] + bestParam[5] * front_left + bestParam[6] * front + bestParam[7] * front_right)

            # ---------- MISE A JOUR MÉMOIRE 
            danger_wall = 1.0 - sensor_to_wall[sensor_front]
            danger_robot = 1.0 - sensor_to_robot[sensor_front]
            danger = max(danger_wall, danger_robot)

            
            if danger > 0.8: #il avance trop longtement on concidere qu'il est bloqué 
                self.memory += 1
            else:
                self.memory = max(0, self.memory - 1)

        # ==============================================================================
        # RETOUR FINAL 
        # ==============================================================================
        return translation, rotation, False