# optimizer_simple_ga.py
# ------------------------------------------------------------
# Recherche ultra simple de paramètres (style "GA minimal"):
# - on teste une stratégie (8 params)
# - on garde la meilleure
# - on génère une nouvelle stratégie en MUTANT la meilleure (petit bruit)
# -> ça converge vite et ça fournit des paramètres "trouvés à l'avance"
# ------------------------------------------------------------

from robot import *
import math
import random

nb_robots = 0
debug = False

def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

class Robot_player(Robot):
    team_name = "OptimizerSimpleGA"
    robot_id = -1

    # --- réglages (simples) ---
    EVAL_TICKS = 450          # durée d'évaluation d'une stratégie
    MAX_TRIALS = 300          # nombre total de tests
    REPLAY_TICKS = 1200       # durée replay du meilleur
    WARMUP = 10               # ignore les 10 premiers ticks après reset

    # mutation
    MUT_SIGMA = 0.35          # force du bruit gaussien
    MUT_CLAMP = 3.0           # borne sur les paramètres [-3,3]

    def __init__(self, x_0, y_0, theta_0, name="n/a", team="n/a", evaluations=0, it_per_evaluation=0):
        global nb_robots
        self.robot_id = nb_robots
        nb_robots += 1

        self.x0 = x_0
        self.y0 = y_0
        self.theta0 = theta_0

        # Paramètres = [bT, wT_fl, wT_f, wT_fr, bR, wR_fl, wR_f, wR_fr]
        self.current_param = self._random_param()
        self.best_param = self.current_param[:]
        self.best_score = -1e18
        self.best_trial = -1

        self.trial = 0
        self.tick = 0
        self.mode = "search"   # "search" puis "replay"

        # scoring basé sur les logs (différences)
        self.prev_logT = 0.0
        self.prev_logR = 0.0
        self.just_reset = True
        self.score = 0.0

        super().__init__(x_0, y_0, theta_0, name=name, team=team)

        print("Starting OptimizerSimpleGA ...")
        print("Initial param =", self.current_param)

    # --------- utils param ---------
    def _random_param(self):
        # départ pas trop grand pour rester stable
        return [random.uniform(-1.5, 1.5) for _ in range(8)]

    def _mutate_from_best(self):
        # mutation gaussienne autour du best
        p = self.best_param[:]
        for i in range(8):
            p[i] += random.gauss(0.0, self.MUT_SIGMA)
            p[i] = clamp(p[i], -self.MUT_CLAMP, self.MUT_CLAMP)
        return p

    def reset(self):
        super().reset()
        self.tick = 0
        self.score = 0.0
        self.just_reset = True

    # --------- controller ---------
    def _controller(self, p, sensors):
        # on ne regarde que l'avant, simple
        d_fl = sensors[sensor_front_left]
        d_f  = sensors[sensor_front]
        d_fr = sensors[sensor_front_right]

        # Perceptron
        translation_raw = p[0] + p[1]*d_fl + p[2]*d_f + p[3]*d_fr
        rotation_raw    = p[4] + p[5]*d_fl + p[6]*d_f + p[7]*d_fr

        translation = math.tanh(translation_raw)  # [-1,1]
        rotation    = math.tanh(rotation_raw)     # [-1,1]

        # Option: limiter translation à [0,1] pour éviter marche arrière
        translation = clamp(translation, 0.0, 1.0)

        return translation, rotation

    def step(self, sensors, sensor_view=None, sensor_robot=None, sensor_team=None):

        # --- init références logs après reset ---
        if self.just_reset:
            self.prev_logT = self.log_sum_of_translation
            self.prev_logR = self.log_sum_of_rotation
            self.just_reset = False

        # --- scoring à chaque tick ---
        # delta translation / rotation depuis tick précédent
        dT = self.log_sum_of_translation - self.prev_logT
        dR = self.log_sum_of_rotation - self.prev_logR
        self.prev_logT = self.log_sum_of_translation
        self.prev_logR = self.log_sum_of_rotation

        # ignore warmup (stabilise)
        if self.tick >= self.WARMUP:
            # score = avancer en tournant peu (simple et robuste)
            self.score += dT * (1.0 - min(1.0, abs(dR)))

        # --- fin d'une évaluation ---
        if self.mode == "search" and self.tick >= self.EVAL_TICKS:
            self.trial += 1

            # log utile
            dist = math.sqrt((self.x - self.x0)**2 + (self.y - self.y0)**2)
            print(f"trial {self.trial:3d}  score={self.score:8.2f}  dist={dist:6.2f}  param={self.current_param}")

            # update best
            if self.score > self.best_score:
                self.best_score = self.score
                self.best_param = self.current_param[:]
                self.best_trial = self.trial
                print(">>> NEW BEST at trial", self.best_trial, "score=", round(self.best_score, 2))
                print(">>> BEST_PARAM =", [round(x, 4) for x in self.best_param])

            # stop condition
            if self.trial >= self.MAX_TRIALS:
                self.mode = "replay"
                self.current_param = self.best_param[:]
                print("\n=== SEARCH DONE ===")
                print("Best trial =", self.best_trial, "best_score =", round(self.best_score, 2))
                print("BEST_PARAM =", [round(x, 6) for x in self.best_param])
                print("Copy/paste BEST_PARAM into Challenger (rid==0).")
                return 0.0, 0.0, True  # reset pour replay propre

            # sinon -> prochaine stratégie = mutation du best (simple GA)
            self.current_param = self._mutate_from_best()
            return 0.0, 0.0, True  # reset pour évaluer sur base comparable

        # --- replay ---
        if self.mode == "replay" and self.tick >= self.REPLAY_TICKS:
            return 0.0, 0.0, True

        # action normale
        translation, rotation = self._controller(self.current_param, sensors)

        self.tick += 1
        return translation, rotation, False