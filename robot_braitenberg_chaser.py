from robot import *
import random

nb_robots = 0

# =========================
# DEBUG SETTINGS
# =========================
DEBUG = True
PRINT_EVERY = 20          # 1 => chaque itération ; 20 => toutes les 20
PRINT_ON_ENEMY = True     # print immédiat si ennemi détecté

def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))


class Robot_player(Robot):

    team_name = "Chaser"
    robot_id = -1
    iteration = 0

    def __init__(self, x_0, y_0, theta_0, name="n/a", team="n/a"):
        global nb_robots
        self.robot_id = nb_robots
        nb_robots += 1
        super().__init__(x_0, y_0, theta_0, name=name, team=team)

    def step(self, sensors, sensor_view=None, sensor_robot=None, sensor_team=None):

        # --- séparation murs / ennemis ---
        sensor_to_wall  = [1.0] * 8
        sensor_to_enemy = [1.0] * 8

        for i in range(8):
            if sensor_view[i] == 1:  # wall
                sensor_to_wall[i] = sensors[i]
                sensor_to_enemy[i] = 1.0

            elif sensor_view[i] == 2:  # robot
                sensor_to_wall[i] = 1.0
                # ennemi si team différente
                if sensor_team[i] != self.team:
                    sensor_to_enemy[i] = sensors[i]
                else:
                    sensor_to_enemy[i] = 1.0
            else:
                sensor_to_wall[i] = 1.0
                sensor_to_enemy[i] = 1.0

        # =========================================================
        # 1) AVOID WALL
        # =========================================================
        left_free = (sensor_to_wall[sensor_front_left] +
                     0.25 * sensor_to_wall[sensor_left] +
                     0.10 * sensor_to_wall[sensor_rear_left])

        right_free = (sensor_to_wall[sensor_front_right] +
                      0.25 * sensor_to_wall[sensor_right] +
                      0.10 * sensor_to_wall[sensor_rear_right])

        rot_avoid = left_free - right_free

        front_free = (0.6 * sensor_to_wall[sensor_front] +
                      0.2 * sensor_to_wall[sensor_front_left] +
                      0.2 * sensor_to_wall[sensor_front_right])

        trans_avoid = 0.25 + 0.75 * front_free


        # =========================================================
        # 2) CHASE ENEMY (ARGMAX direction) - SIGN FIXED
        # =========================================================
        enemy_close = [1.0 - sensor_to_enemy[i] for i in range(8)]

        # convention de signe (FIXED) :
        dir_rot = [0.0, +0.7, +1.0, +0.7, 0.0, -0.7, -1.0, -0.7]

        rot_chase = 0.0
        for i in range(8):
            rot_chase += dir_rot[i] * enemy_close[i]

        # renforce l'effet
        rot_chase *= 1.8
        rot_chase = clamp(rot_chase, -1.0, 1.0)

        front_enemy = enemy_close[sensor_front] + 0.5*(enemy_close[sensor_front_left] + enemy_close[sensor_front_right])
        rear_enemy  = enemy_close[sensor_rear]  + 0.5*(enemy_close[sensor_rear_left]  + enemy_close[sensor_rear_right])

        trans_chase = 0.25 + 0.50*front_free + 0.75*front_enemy - 0.35*rear_enemy
        trans_chase = clamp(trans_chase, 0.0, 1.0)

        # force de chase continue (aucun if) :
        m = max(enemy_close)
        chase_strength = clamp(2.0*m, 0.0, 1.0)


        # =========================================================
        # 3) MIX + exploration + anti-cycle
        # =========================================================
        rotation = (1.0 - chase_strength)*rot_avoid + chase_strength*rot_chase
        translation = (1.0 - chase_strength)*trans_avoid + chase_strength*trans_chase

        # scan seulement si aucun ennemi
        if m == 0.0:
            rotation += 0.12

        # bruit anti-cycle proche mur si rotation ~0
        danger = 1.0 - min(sensor_to_wall)
        sym = 1.0 - min(1.0, abs(rotation) / 0.05)
        rotation += 0.03 * danger * sym * (2.0 * random.random() - 1.0)

        # ralentir si obstacle devant
        translation *= sensors[sensor_front]

        # vitesse globale (si tu trouves encore trop rapide, mets 0.45)
        translation *= 0.55

        translation = clamp(translation, 0.0, 1.0)
        rotation = clamp(rotation, -1.0, 1.0)


        # =========================================================
        # DEBUG PRINTS (avec ALLY/ENEMY)
        # =========================================================
        do_print = False
        if DEBUG and (self.iteration % PRINT_EVERY == 0):
            do_print = True
        if DEBUG and PRINT_ON_ENEMY and (m > 0.0):
            do_print = True

        if do_print:
            print("\n--- ITER", self.iteration, "| robot_id", self.robot_id, "| team", self.team, "---")
            # affiche robots vus + classification
            for i in range(8):
                if sensor_view[i] == 2:
                    tag = "ENEMY" if sensor_team[i] != self.team else "ALLY"
                    print(" sees", tag, "on", i, "team=", sensor_team[i], "dist=%.3f" % sensors[i])

            print("enemy_close =", ["%.2f" % x for x in enemy_close],  "m=%.3f" % m)
            print("rot_avoid=%.3f rot_chase=%.3f chase_strength=%.3f -> rotation=%.3f" %
                  (rot_avoid, rot_chase, chase_strength, rotation))
            print("trans_avoid=%.3f trans_chase=%.3f -> translation=%.3f" %
                  (trans_avoid, trans_chase, translation))

        self.iteration += 1
        return translation, rotation, False