"""Predock navigation: goal sampling and move-to-pose control."""

from __future__ import annotations

import math
import random

from autocharge.common.geometry import wrap_pi
from autocharge.docking.type import PredockGoal, PredockLimits, PredockNoise


class PathFinderController:
    """Continuous controller adapted from move_to_pose."""

    def __init__(self, kp_rho: float, kp_alpha: float, kp_beta: float) -> None:
        self.kp_rho = float(kp_rho)
        self.kp_alpha = float(kp_alpha)
        self.kp_beta = float(kp_beta)

    def calc_control_command(
        self,
        x_diff: float,
        y_diff: float,
        theta: float,
        theta_goal: float,
    ) -> tuple[float, float, float]:
        rho = math.hypot(x_diff, y_diff)
        v = self.kp_rho * rho
        alpha = wrap_pi(math.atan2(y_diff, x_diff) - theta)
        beta = wrap_pi(theta_goal - theta - alpha)
        if alpha > math.pi / 2.0 or alpha < -math.pi / 2.0:
            alpha = wrap_pi(math.atan2(-y_diff, -x_diff) - theta)
            beta = wrap_pi(theta_goal - theta - alpha)
            v = -v
        w = self.kp_alpha * alpha - self.kp_beta * beta
        return (rho, v, w)


class PredockGoalSampler:
    """Builds one predock pose from dock geometry and noise settings."""

    @staticmethod
    def from_dock(
        dock_x: float,
        dock_y: float,
        dock_yaw: float,
        distance_m: float,
    ) -> PredockGoal:
        """Deterministic predock pose in front of the charger (no noise)."""
        c = math.cos(float(dock_yaw))
        s = math.sin(float(dock_yaw))
        x = float(dock_x) + float(distance_m) * c
        y = float(dock_y) + float(distance_m) * s
        yaw = math.atan2(float(dock_y) - y, float(dock_x) - x)
        return PredockGoal(x=float(x), y=float(y), yaw=float(yaw))

    @staticmethod
    def sample(
        dock_x: float,
        dock_y: float,
        dock_yaw: float,
        distance_m: float,
        noise: PredockNoise,
    ) -> PredockGoal:
        c = math.cos(float(dock_yaw))
        s = math.sin(float(dock_yaw))
        x = float(dock_x) + float(distance_m) * c
        y = float(dock_y) + float(distance_m) * s

        if noise.position_std_m > 0.0:
            dx_local = random.gauss(0.0, noise.position_std_m)
            dy_local = random.gauss(0.0, noise.position_std_m)
            x += c * dx_local - s * dy_local
            y += s * dx_local + c * dy_local

        yaw = math.atan2(float(dock_y) - y, float(dock_x) - x)
        if noise.yaw_deg > 0.0:
            yaw = wrap_pi(yaw + math.radians(random.uniform(-noise.yaw_deg, noise.yaw_deg)))
        return PredockGoal(x=float(x), y=float(y), yaw=float(yaw))


class PredockNavigator:
    """Computes predock velocity commands and checks goal reach."""

    def __init__(
        self,
        limits: PredockLimits,
        controller: PathFinderController,
    ) -> None:
        self.limits = limits
        self.controller = controller

    def goal_reached(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        goal: PredockGoal,
    ) -> bool:
        rho = math.hypot(goal.x - robot_x, goal.y - robot_y)
        yaw_err = abs(wrap_pi(goal.yaw - robot_yaw))
        yaw_tol = math.radians(self.limits.goal_yaw_tol_deg)
        if yaw_err > yaw_tol:
            return False
        # Once heading is locked, accept the yaw-finish band so we hand off to
        # IR docking instead of freezing with vx=0 just outside pos_tol.
        pos_band = float(self.limits.goal_pos_tol_m) * max(
            1.0, float(self.limits.yaw_finish_pos_factor)
        )
        return rho <= pos_band

    def compute_cmd(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        goal: PredockGoal,
    ) -> tuple[float, float]:
        x_diff = goal.x - robot_x
        y_diff = goal.y - robot_y
        rho = math.hypot(x_diff, y_diff)
        yaw_err = wrap_pi(goal.yaw - robot_yaw)

        if self.goal_reached(robot_x, robot_y, robot_yaw, goal):
            return (0.0, 0.0)

        limits = self.limits
        pos_tol = limits.goal_pos_tol_m
        yaw_tol = math.radians(limits.goal_yaw_tol_deg)
        if rho <= pos_tol * limits.yaw_finish_pos_factor:
            wz = limits.yaw_finish_kp * yaw_err
            wz = max(-limits.max_angular_speed, min(limits.max_angular_speed, wz))
            # Heading already good but still outside strict pos_tol: creep in
            # so we do not stall forever with vx=0.
            if abs(yaw_err) <= yaw_tol and rho > pos_tol:
                creep = max(limits.near_goal_min_linear_speed, limits.min_linear_speed)
                # Drive toward the goal in the body frame (prefer forward).
                body_x = math.cos(robot_yaw) * x_diff + math.sin(robot_yaw) * y_diff
                vx = creep if body_x >= 0.0 else -creep
                return (float(vx), float(wz))
            return (0.0, float(wz))

        _, vx, wz = self.controller.calc_control_command(
            x_diff,
            y_diff,
            robot_yaw,
            goal.yaw,
        )

        speed_cap = limits.max_linear_speed * min(1.0, rho / max(1e-6, limits.dist_scale_ref_m))
        speed_cap = max(limits.min_linear_speed, speed_cap)
        vx = max(-speed_cap, min(speed_cap, vx))

        if rho <= limits.approach_slow_radius_m:
            t = rho / max(1e-6, limits.approach_slow_radius_m)
            approach_cap = limits.near_goal_min_linear_speed + (
                limits.max_linear_speed - limits.near_goal_min_linear_speed
            ) * t
            vx = max(-approach_cap, min(approach_cap, vx))

        wz = max(-limits.max_angular_speed, min(limits.max_angular_speed, wz))
        return (float(vx), float(wz))
