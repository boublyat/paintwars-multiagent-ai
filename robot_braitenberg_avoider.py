from robot import *
import random

nb_robots = 0
debug = True


def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))


class Robot_player(Robot):
    team_name = "BraitAvoid"
    robot_id = -1
    iteration = 0

    def __init__(self, x_0, y_0, theta_0, name="n/a", team="n/a"):
        global nb_robots
        self.robot_id = nb_robots
        nb_robots += 1
        super().__init__(x_0, y_0, theta_0, name=name, team=team)

    def step(self, sensors, sensor_view=None, sensor_robot=None, sensor_team=None):
        # Décomposition comme dans robot_dumb.py
        sensor_to_wall = []
        sensor_to_robot = []
        for i in range(0, 8):
            if sensor_view[i] == 1:  # wall
                sensor_to_wall.append(sensors[i])
                sensor_to_robot.append(1.0)
            elif sensor_view[i] == 2:  # robot
                sensor_to_wall.append(1.0)
                sensor_to_robot.append(sensors[i])
            else:  # empty
                sensor_to_wall.append(1.0)
                sensor_to_robot.append(1.0)

        # Capteur "obstacle" unique : min(distance mur, distance robot)
        d = [min(sensor_to_wall[i], sensor_to_robot[i]) for i in range(8)]

        if debug and self.iteration % 100 == 0:
            print("Robot", self.robot_id, "(team " + str(self.team_name) + ")", "at step", self.iteration, ":")
            print("\tsensors (distance, max is 1.0)  =", sensors)
            print("\t\tsensors to wall  =", sensor_to_wall)
            print("\t\tsensors to robot =", sensor_to_robot)
            print("\t\tobstacle (min)   =", d)
            print("\ttype (0:empty, 1:wall, 2:robot) =", sensor_view)

        # ----- Braitenberg (pas de if/else) -----
        # Rotation : tourne vers le côté le plus libre
        # (plus d[i] est grand => plus c'est libre)
            
        danger = 1.0 - min(d)

        left_free  = d[sensor_front_left]  + 0.25*d[sensor_left]  + 0.1*d[sensor_rear_left]
        right_free = d[sensor_front_right] + 0.25*d[sensor_right] + 0.1*d[sensor_rear_right]
        rotation = left_free - right_free


        front_free = 0.6*d[sensor_front] + 0.2*d[sensor_front_left] + 0.2*d[sensor_front_right]
        translation = 0.4 + 0.6 * front_free

        # <-- AJOUT (casse la symétrie quand l'avant est bouché)
        sym = 1.0 - min(1.0, abs(rotation) / 0.05)  # ≈1 si rotation≈0
        rotation += 0.05 * danger * sym * (2.0*random.random() - 1.0)   


        translation = clamp(translation)
        rotation = clamp(rotation)

        self.iteration += 1
        return translation, rotation, False