# ============================================================
# Projet "robotique" IA&Jeux 2025
# Robot controller: 4 robots / 4 comportements (rid = robot_id % 4)
#
# Convention capteurs:
# - sensors[i] dans [0.0, 1.0]  (1.0 = loin / rien devant ; 0.0 = très proche)
# Convention moteurs:
# - translation dans [-1, +1]   (avance / recule)
# - rotation    dans [-1, +1]   (tourne droite / gauche selon conventions projet)
# ============================================================

from robot import *
import math
import random
#import statistics as st

# Compteur global pour attribuer un id unique à chaque robot instancié
nb_robots = 0

# ============================================================
# 1) Paramètres GA (pré-calculés à l'avance)
# ------------------------------------------------------------
# Chaque BEST_PARAM_k est un vecteur de 8 réels (poids d'un "perceptron")
# qui pilote:
#   translation = tanh( w0 + w1*d_fl + w2*d_f + w3*d_fr )
#   rotation    = tanh( w4 + w5*d_fl + w6*d_f + w7*d_fr )
#
# Ici tu as 4 ensembles de paramètres (trouvés sur 4 arènes / runs)
# et tu fais une stratégie "robuste" en prenant la médiane coordonnée par coordonnée.
# ============================================================

# BEST_PARAM_1 = [-1.217043,  1.588766,  2.776777, -1.095726,  1.323706,  1.941396, -3.0,     -1.070865]
# BEST_PARAM_2 = [ 2.966807,  0.177096,  0.17556,   0.897079,  0.617804,  2.043894, -1.079349, -0.829011]
# BEST_PARAM_3 = [ 2.709788, -1.268477, -0.153818,  1.687339, -2.309589,  0.487075,  1.00042,   0.823303]

# BEST_PARAM_4 = [ 0.992237,  1.086936,  2.191976, -1.764676, -2.191082,  0.416231,  2.168867, -0.378178]

# params = [BEST_PARAM_1, BEST_PARAM_2, BEST_PARAM_3, BEST_PARAM_4]

# # Stratégie "robuste" = médiane de chaque composante
# ROBUST = [st.median([p[i] for p in params]) for i in range(8)]


class Robot_player(Robot):

    team_name = "Team"
    robot_id = -1
    # Contrainte projet: 1 seule case mémoire, obligatoirement un entier.
    memory = 0

    def __init__(self, x_0, y_0, theta_0, name="n/a", team="n/a"):
        """
        Constructeur: on attribue un robot_id unique, puis on appelle le constructeur parent.
        """
        global nb_robots
        self.robot_id = nb_robots
        nb_robots += 1

        # name = "Robot <id>" juste pour debug/affichage
        super().__init__(x_0, y_0, theta_0, name="Robot " + str(self.robot_id), team="Team")

    def step(self, sensors, sensor_view=None, sensor_robot=None, sensor_team=None):
        """
        Fonction appelée à chaque tick de simulation.
        Elle doit renvoyer: (translation, rotation, reset?)
        reset? = True déclenche un reset du robot par le simulateur.
        """

        # =========================================================
        # SETUP: robustesse (si le simulateur n'a pas fourni sensor_view/team)
        # =========================================================
        if sensor_view is None:
            sensor_view = [0] * 8
        if sensor_team is None:
            sensor_team = [None] * 8

        # clamp: force x à rester dans [lo, hi]
        def clamp(x, lo=-1.0, hi=1.0):
            return lo if x < lo else (hi if x > hi else x)

        def tanh(x):
            return math.tanh(x)

        # =========================================================
        # Ralentisseur global (pour que tes robots ratent moins)
        # - sauf explorer (rid == 3) qui garde sa vitesse normale
        # =========================================================
        def apply_speed(translation, rotation):
            if rid == 3:  # explorer: pas de ralentissement
                return translation, rotation
            # sinon on réduit vitesse et un peu la rotation
            return clamp(translation * SLOW_FACTOR, -1.0, 1.0), clamp(rotation * SLOW_TURN, -1.0, 1.0)

        # rid identifie le "rôle" du robot (0..3)
        rid = self.robot_id % 4

        # facteurs de ralentissement pour tous sauf explorer
        SLOW_FACTOR = 1.0
        SLOW_TURN   = 1.0

        # =========================================================
        # 0) ANTI-COLLISION GLOBALE (commun à tous)
        # ---------------------------------------------------------
        # Si obstacle très proche devant: recule et tourne côté libre
        # =========================================================
        sfl = sensors[sensor_front_left]
        sf  = sensors[sensor_front]
        sfr = sensors[sensor_front_right]
        min_front = min(sfl, sf, sfr)

        if min_front < 0.10:
            # On tourne vers le côté où il y a le plus d'espace
            turn = +1.0 if (sfl > sfr) else -1.0
            return -0.6, turn, False

        # =========================================================
        # 1) ROBOT "GA" (rid == 0)
        # ---------------------------------------------------------
        # Contrôleur perceptron + logique simple de déblocage
        # Utilise self.memory comme "timer de déblocage" signé:
        #   memory > 0  => déblocage en tournant à gauche pendant memory ticks
        #   memory < 0  => déblocage en tournant à droite pendant |memory| ticks
        # =========================================================
        if rid == 0:

            # distances capteurs utiles (avant + côtés + arrière)
            d_fl = sensors[sensor_front_left]
            d_f  = sensors[sensor_front]
            d_fr = sensors[sensor_front_right]
            d_l  = sensors[sensor_left]
            d_r  = sensors[sensor_right]

            d_rl = sensors[sensor_rear_left]
            d_re = sensors[sensor_rear]
            d_rr = sensors[sensor_rear_right]

            min_front = min(d_fl, d_f, d_fr)
            min_rear  = min(d_rl, d_re, d_rr)

            # -----------------------------
            # (1) Mode déblocage actif
            # -----------------------------
            if isinstance(self.memory, int) and self.memory != 0:
                # direction: +1 = gauche, -1 = droite
                direction = +1.0 if self.memory > 0 else -1.0

                # on décrémente le timer sans perdre le signe
                if self.memory > 0:
                    self.memory -= 1
                else:
                    self.memory += 1

                # Si mur derrière: pas possible de reculer, on avance un peu en tournant fort
                if min_rear < 0.10:
                    return 0.25, 1.0 * direction, False

                # sinon: marche arrière + rotation forte -> arc de sortie
                return -0.65, 1.0 * direction, False

            # -----------------------------
            # (2) Déclenchement du déblocage
            # -----------------------------
            # coin = mur devant + mur sur un côté => on est coincé dans un angle
            corner = (min_front < 0.18 and (min(d_l, d_fl) < 0.12 or min(d_r, d_fr) < 0.12))

            if min_front < 0.12 or corner:
                # choisir le côté le plus libre (front-left + left) vs (front-right + right)
                left_free  = 0.6 * d_fl + 0.4 * d_l
                right_free = 0.6 * d_fr + 0.4 * d_r
                direction = +1 if left_free > right_free else -1

                # durée de déblocage (plus long si vraiment collé)
                duration = 18 if min_front < 0.08 else 14
                self.memory = direction * duration

                # effet immédiat (même logique que le mode déblocage)
                if min_rear < 0.10:
                    return 0.25, 1.0 * direction, False
                return -0.65, 1.0 * direction, False
            
            GA_Param = [ 0.992237,  1.086936,  2.191976, -1.764676, -2.191082,  0.416231,  2.168867, -0.378178]

            

            # -----------------------------
            # (3) Contrôle GA normal (perceptron)
            # -----------------------------
            translation_raw = GA_Param[0] + GA_Param[1]*d_fl + GA_Param[2]*d_f + GA_Param[3]*d_fr
            rotation_raw    = GA_Param[4] + GA_Param[5]*d_fl + GA_Param[6]*d_f + GA_Param[7]*d_fr

            translation = math.tanh(translation_raw)
            rotation    = math.tanh(rotation_raw)

            # on ralentit si mur devant (d_f petit)
            translation *= (0.25 + 0.75 * d_f)

            translation = clamp(translation, 0.0, 1.0)
            rotation    = clamp(rotation, -1.0, 1.0)

            # facteur final (tu le gardes)
            translation *= 0.5

            # appliquer ralentissement global (sauf explorer)
            t, rot = apply_speed(translation, rotation)
            return t, rot, False

        # =========================================================
        # 2) ROBOT CHASER (rid == 1)
        # ---------------------------------------------------------
        # Objectif: éviter murs + poursuivre les ennemis (robots pas de la même team)
        # sensor_view[i]:
        #   0 = vide, 1 = mur, 2 = robot
        # sensor_team[i] donne l'équipe du robot vu (si i voit un robot)
        # =========================================================
        if rid == 1:
            # on sépare "distance au mur" et "distance à l'ennemi"
            sensor_to_wall  = [1.0] * 8
            sensor_to_enemy = [1.0] * 8

            for i in range(8):
                if sensor_view[i] == 1:  # mur
                    sensor_to_wall[i] = sensors[i]
                    sensor_to_enemy[i] = 1.0
                elif sensor_view[i] == 2:  # robot
                    sensor_to_wall[i] = 1.0
                    # si c'est un ennemi, on garde sa distance, sinon on ignore
                    if sensor_team[i] != self.team:
                        sensor_to_enemy[i] = sensors[i]
                    else:
                        sensor_to_enemy[i] = 1.0
                else:
                    sensor_to_wall[i] = 1.0
                    sensor_to_enemy[i] = 1.0

            # (A) évitement murs: comparer espace à gauche vs droite
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

            # (B) poursuite ennemis: convertir distance -> "proximité"
            enemy_close = [1.0 - sensor_to_enemy[i] for i in range(8)]

            # direction de rotation associée à chaque capteur (8 directions)
            dir_rot = [0.0, +0.7, +1.0, +0.7, 0.0, -0.7, -1.0, -0.7]

            rot_chase = 0.0
            for i in range(8):
                rot_chase += dir_rot[i] * enemy_close[i]
            rot_chase *= 1.8
            rot_chase = clamp(rot_chase, -1.0, 1.0)

            front_enemy = enemy_close[sensor_front] + 0.5*(enemy_close[sensor_front_left] + enemy_close[sensor_front_right])
            rear_enemy  = enemy_close[sensor_rear]  + 0.5*(enemy_close[sensor_rear_left]  + enemy_close[sensor_rear_right])

            trans_chase = 0.25 + 0.50*front_free + 0.75*front_enemy - 0.35*rear_enemy
            trans_chase = clamp(trans_chase, 0.0, 1.0)

            # force de poursuite = max proximité ennemi
            m = max(enemy_close)
            chase_strength = clamp(2.0*m, 0.0, 1.0)

            # mix évitement/poursuite
            rotation = (1.0 - chase_strength)*rot_avoid + chase_strength*rot_chase
            translation = (1.0 - chase_strength)*trans_avoid + chase_strength*trans_chase

            # si aucun ennemi: légère rotation pour explorer
            if m == 0.0:
                rotation += 0.12

            # petit bruit anti-symétrie près des murs (évite blocages)
            danger = 1.0 - min(sensor_to_wall)
            sym = 1.0 - min(1.0, abs(rotation) / 0.05)
            rotation += 0.03 * danger * sym * (2.0 * random.random() - 1.0)

            # ralentir si mur devant
            translation *= sensors[sensor_front]
            translation *= 0.55

            translation = clamp(translation, 0.0, 1.0)
            rotation = clamp(rotation, -1.0, 1.0)

            t, rot = apply_speed(translation, rotation)
            return t, rot, False

        # =========================================================
        # 3) WALL FOLLOWER (rid == 2)
        # ---------------------------------------------------------
        # Objectif: suivre un mur (gauche ou droite) et gérer:
        # - couloir étroit (mur à gauche ET à droite)
        # - collision frontale (max 2 "tapes", puis mode échappement)
        # Mémoire:
        # - memory >= 0 : nombre de chocs frontaux consécutifs (0..2)
        # - memory < 0  : mode échappement pendant -memory steps
        # =========================================================
        # =========================================================
        # 3) WALL FOLLOWER (rid == 2)  -> VERSION QUI MARCHAIT
        # =========================================================
        if rid == 2:
            # -----------------------------
            # Capteurs utiles
            # -----------------------------
            f  = sensors[sensor_front]
            fl = sensors[sensor_front_left]
            fr = sensors[sensor_front_right]
            l  = sensors[sensor_left]
            r  = sensors[sensor_right]

            # -----------------------------
            # Mémoire: compteur "stuck"
            # On encode dans self.memory:
            #   prev = mémoire du "log translation" (grossier)
            #   cnt  = compteur de ticks sans mouvement
            # -----------------------------
            stuck = int(self.memory) if isinstance(self.memory, int) else 0

            # -----------------------------
            # Seuils simples
            # -----------------------------
            FRONT_DANGER = 0.20      # devant trop proche
            WALL_NEAR    = 0.60      # mur proche sur le côté
            TARGET_SIDE  = 0.22   # distance côté visée quand on suit un mur

            # vitesses
            V_FWD  = 0.70
            V_BACK = -0.30
            K      = 2.0            # gain de correction latérale

            # -----------------------------
            # (A) Anti-blocage ultra simple
            # -> on détecte "pas de mouvement" via log_sum_of_translation
            # -----------------------------
            cur = int(self.log_sum_of_translation * 10) % 1000
            prev = stuck // 100
            cnt  = stuck % 100

            if cur == prev:
                cnt = min(50, cnt + 1)
            else:
                cnt = max(0, cnt - 2)

            # si bloqué longtemps -> petit virage + avance lente
            if cnt >= 25:
                self.memory = (cur * 100) + 0  # reset compteur après "déblocage"
                rot = +0.9 if l > r else -0.9
                return 0.25, rot, False

            # sauver mémoire
            self.memory = (cur * 100) + cnt

            # -----------------------------
            # (1) Mur devant -> recule + tourne côté libre
            # -----------------------------
            if min(f, fl, fr) < FRONT_DANGER:
                turn = +1.0 if l > r else -1.0
                return V_BACK, turn, False

            # -----------------------------
            # (2) Suivi de mur si un côté est proche
            # -----------------------------
            if (l < WALL_NEAR) or (r < WALL_NEAR):
                # choisir le mur le plus proche (le plus "urgent" à suivre)
                follow_right = (r < l)

                if follow_right:
                    # trop près du mur droit => tourner à gauche
                    rotation = clamp(-K * (r - TARGET_SIDE), -0.7, 0.7)
                else:
                    # trop près du mur gauche => tourner à droite
                    rotation = clamp(+K * (l - TARGET_SIDE), -0.7, 0.7)

                translation = clamp(V_FWD * f, 0.20, V_FWD)
                return 0.5*translation, rotation, False

            # -----------------------------
            # (3) Pas de mur proche -> tout droit (important)
            # -----------------------------
            return V_FWD, 0.0, False
        # =========================================================
        # 4) EXPLORER (rid == 3)
        # ---------------------------------------------------------
        # Objectif: explorer l'arène sans rester bloqué.
        # Mémoire encodée en un entier:
        #   mode  = 0/1
        #   timer = compteur
        #   lastT = trace grossière de la translation cumulée pour détecter "pas de mouvement"
        # mode 0 = comportement normal (avance / évite)
        # mode 1 = "déblocage": tourner fort pendant timer steps
        # =========================================================
        if rid == 3:
            mem = self.memory if isinstance(self.memory, int) else 0
            mode  = mem // 1_000_000
            timer = (mem % 1_000_000) // 1000
            lastT = mem % 1000

            curT = int(self.log_sum_of_translation * 100) % 1000
            no_move = (curT == lastT)

            sf = sensors[sensor_front]

            # (A) gestion du mode/timer
            if mode == 0:
                if no_move and min_front < 0.35:
                    timer += 1
                else:
                    timer = max(0, timer - 1)
                if timer >= 12:
                    mode = 1
                    timer = 18 + random.randint(0, 12)
            else:
                timer -= 1
                if timer <= 0:
                    mode = 0
                    timer = 0

            self.memory = int(mode * 1_000_000 + timer * 1000 + curT)

            # (B) évite de foncer dans un allié
            ally_front = False
            if sensor_view[sensor_front] == 2 and sensor_team[sensor_front] == self.team:
                ally_front = True

            # (C) mode déblocage: rotation forte
            if mode == 1:
                rot = 0.9
                trans = 0.25 + 0.35 * sf
                if ally_front:
                    rot *= -1.0
                return clamp(trans, 0.0, 1.0), clamp(rot, -1.0, 1.0), False

            # (D) navigation normale: différence gauche/droite
            left_free  = 0.55*sensors[sensor_front_left] + 0.30*sensors[sensor_left]  + 0.15*sensors[sensor_rear_left]
            right_free = 0.55*sensors[sensor_front_right]+ 0.30*sensors[sensor_right] + 0.15*sensors[sensor_rear_right]

            rotation = tanh(1.7*(left_free - right_free) + 0.08)
            translation = tanh(0.75*sf + 0.25 - 0.65*abs(rotation))

            # si allié devant: grosse correction
            if ally_front:
                rotation += 0.7 if (sensors[sensor_front_left] > sensors[sensor_front_right]) else -0.7

            # bruit quand danger proche et rotation faible
            danger = 1.0 - min(sensors)
            if abs(rotation) < 0.08 and danger > 0.25:
                rotation += (2.0*random.random() - 1.0) * 0.15 * danger

            translation = clamp(translation, 0.0, 1.0) * 0.65
            rotation = clamp(rotation, -1.0, 1.0)

            return translation, rotation, False

        # fallback
        return 0.0, 0.0, False