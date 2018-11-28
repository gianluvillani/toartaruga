import math


def map_speed_to_input(v):
    pwm_velocity = 0
    gamma = 3
    if v < 0:
        pwm_velocity = -(v - 18) * (-1 + math.exp(gamma * v))

    if v > 0:
        pwm_velocity = (v + 13) * (1 - math.exp(-gamma * v))

    if v > 100:
        pwm_velocity = 100

    if v < -100:
        pwm_velocity = -100
    return pwm_velocity
