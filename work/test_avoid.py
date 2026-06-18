import time
import threading
import logging
from typing import Optional

import logger
from t11_argument_parser import parse_args
from t11_robot import Robot
from t11_threads import thread_ultrasonic, thread_line, thread_LED

from t3_servomotors import STEER_HARD_DEG, STEER_SOFT_DEG
from t4_dc_motor import Direction, SPEED_SLOW_PCT, SPEED_TURNING_PCT, SPEED_NORMAL_PCT
from t6_line_tracking import LinePosition
from t11_buzzer_Sirene import POLICE, play 
from test_maneuvre import thread_controller, thread_buzzer

sensor_interval_s = 0.05
scan_pivot_duration_s = 0.25
scan_min_clear_mm = 300.0
avoid_turn_duration_s = 0.6
avoid_bypass_duration_s = 1
avoid_speed_pct = SPEED_SLOW_PCT
obstacle_margin_mm = 250

def thread_avoidance(robot,interval):
    avoid_phase = "idle" 
    avoid_t0 = 0.0
    avoid_side = 1
    dist_left = 0.0
    dist_right = 0.0

    while True : 
        with robot.state.lock : 
            running = robot.state.running
            emergency = robot.state.emergency_stop
        if not running:
            break 
        if emergency and avoid_phase == "idle" : 
            robot.motor.stop()
            robot.head.steer_center()
            with robot.state.lock : 
                robot.state.maneuver = True
            avoid_phase = "scan_left"
            avoid_t0 = time.monotonic()
        if avoid_phase != "idle" : 
            elapsed = time.monotonic() - avoid_t0
            if avoid_phase == "scan_left" : 
                robot.head.steer_left(STEER_HARD_DEG)
                robot.motor.drive(Direction.FORWARD, avoid_speed_pct, fast_accel=True)
                if elapsed >= scan_pivot_duration_s : 
                    with robot.state.lock : 
                        dist_left = robot.state.distance_mm
                    avoid_phase = "scan_left_undo"

