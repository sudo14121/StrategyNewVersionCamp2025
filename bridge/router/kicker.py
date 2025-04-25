"""
Генерация объектов типа wp.Waypoint для роботов, движущихся с мячом
"""

import math
from time import time
from typing import Optional

from bridge import const
from bridge.auxiliary import aux, fld, rbt
from bridge.router.action import ActionDomain, ActionValues

MAX_ACCELERATION = 500


class KickerAux:
    """Class for smart turns with ball"""

    def __init__(self) -> None:
        self.twist_w: Optional[float] = None
        self.kick_await_timer: Optional[float] = None
        self.last_update: Optional[float] = None

        self.fast_twist_w: Optional[float] = None
        self.fast_twist_vel: Optional[float] = None
        self.dribbling_angle: Optional[float] = None  # update in FastBallGrab
        self.dribbling_start: Optional[aux.Point] = None

        self.robot_voltage: list[int] = [15 for _ in range(const.TEAM_ROBOTS_MAX_COUNT)]

    def reset_kick_consts(self) -> None:
        "Обнулить константы для ударов"
        self.twist_w = None
        self.kick_await_timer = None
        self.last_update = None

        self.fast_twist_w = None
        self.fast_twist_vel = None
        self.dribbling_angle = None
        self.dribbling_start = None

    def twisted(
        self, domain: ActionDomain, kick_point: aux.Point, current_action: ActionValues, safe_twist: bool = False
    ) -> None:
        """
        Прицеливание и удар в точку
        """
        field: fld.Field = domain.field
        kicker: rbt.Robot = domain.robot

        if self.last_update is None or time() - self.last_update > 0.1:
            self.reset_kick_consts()

        x = aux.angle_to_point(kicker.get_pos(), kick_point) - kicker.get_angle()

        # nearest_enemy = fld.find_nearest_robot(kicker.get_pos(), field.enemies)
        # angle_to_enemy = aux.get_angle_between_points(
        #     nearest_enemy.get_pos(), kicker.get_pos(), kick_point
        # )
        # angle_to_kick = aux.wind_down_angle(
        #     aux.angle_to_point(kicker.get_pos(), kick_point) - kicker.get_angle()
        # )
        # if aux.dist(nearest_enemy.get_pos(), kicker.get_pos()) < 500 and abs(
        #     angle_to_enemy
        # ) < abs(angle_to_kick):
        #     if angle_to_enemy < 0 and x < 0:
        #         x += 2 * math.pi
        #     elif angle_to_enemy > 0 and x > 0:
        #         x -= 2 * math.pi
        # else:
        x = aux.wind_down_angle(x)

        if self.twist_w is None or self.last_update is None:
            self.twist_w = math.copysign(0.5, -x)
            self.last_update = time()
            current_action.vel, current_action.angle = spin_with_ball(self.twist_w, safe_twist)
            return

        beta = 6.5
        if safe_twist:
            beta = 4
        a = beta / abs(x) - abs(self.twist_w) / (x**2)
        b = 2 * abs(x) * a - beta  # surrender, even me can't understand it

        if abs(x) < 0.15:
            self.twist_w = x / 4
        else:
            self.twist_w += b * (time() - self.last_update) * aux.sign(x)
        self.last_update = time()

        self.twist_w = aux.minmax(self.twist_w, 7)

        vel, wvel = spin_with_ball(self.twist_w, safe_twist)

        # print("A" * 100)
        # print(self.twist_w, x)
        # field.router_image.print(
        #     aux.Point(-1000, -200),
        #     f"{self.twist_w:.2f}",
        # )
        # field.router_image.print(aux.Point(-1000, 200), f"{x:.2f}")

        field.router_image.draw_line(
            kicker.get_pos(),
            kicker.get_pos() + aux.rotate(aux.RIGHT, kicker.get_angle()) * 5000,
        )
        field.router_image.draw_line(
            kicker.get_pos(),
            kicker.get_pos()
            + aux.rotate(
                aux.RIGHT,
                self.twist_w / 6 + kicker.get_angle(),
            )
            * 500,
            color=(255, 0, 255),
        )
        field.router_image.draw_line(
            kicker.get_pos(),
            kicker.get_pos() + aux.rotate(aux.RIGHT, aux.angle_to_point(kicker.get_pos(), kick_point)) * 500,
            color=(255, 255, 0),
        )

        if (
            self.kick_await_timer is not None
            # and time() - self.kick_await_timer > 0.1
            and abs(x) < const.KICK_ALIGN_ANGLE
        ):
            current_action.auto_kick = 1

        if abs(x) > const.KICK_ALIGN_ANGLE * 2:
            current_action.dribbler_speed = 15  # int(max(7, 15 - 0.5 * abs(self.twist_w))) #NOTE
            self.kick_await_timer = None
        else:
            if self.kick_await_timer is None:
                self.kick_await_timer = time()
            current_action.dribbler_speed = 15

        current_action.vel, current_action.angle = vel, wvel
        return

    def fast_twisted(
        self,
        domain: ActionDomain,
        kick_point: aux.Point,
        kick_speed: float,
        current_action: ActionValues,
    ) -> None:
        """
        Прицеливание и удар в точку
        """
        field: fld.Field = domain.field
        kicker: rbt.Robot = domain.robot

        if self.last_update is None or time() - self.last_update > 0.5:
            self.reset_kick_consts()

        x = aux.wind_down_angle(aux.angle_to_point(kicker.get_pos(), kick_point) - kicker.get_angle())

        if self.dribbling_angle is None:
            self.dribbling_angle = x / (const.FAST_GRAB_MULT - 1)

        if (
            self.fast_twist_w is None
            or self.fast_twist_vel is None
            or self.dribbling_start is None
            or self.last_update is None
        ):
            self.fast_twist_w = -x * 0.0
            self.fast_twist_vel = 0.0
            self.dribbling_start = field.ball.get_pos()
            self.last_update = time()

            vel = aux.rotate(aux.Point(self.fast_twist_vel, 0), kicker.get_angle() + self.dribbling_angle)
            wvel = self.fast_twist_w
            current_action.vel, current_action.angle = vel, wvel
            return

        beta = 4
        a = beta / abs(x) - abs(self.fast_twist_w) / (x**2)
        b = 2 * abs(x) * a - beta  # surrender, even me can't understand it

        if abs(x) < 0.15:
            self.fast_twist_w = math.copysign(0.1, x)
        else:
            self.fast_twist_w += b * (time() - self.last_update) * aux.sign(x)

        self.fast_twist_w = aux.minmax(self.fast_twist_w, 2)

        if abs(self.dribbling_angle) > x / 3:
            self.dribbling_angle = math.copysign(x / 3, self.dribbling_angle)

        if self.fast_twist_vel != kick_speed:
            max_delta_vel = MAX_ACCELERATION * (time() - self.last_update)
            if abs(kick_speed - self.fast_twist_vel) < max_delta_vel:
                self.fast_twist_vel = kick_speed
            else:
                self.fast_twist_vel += math.copysign(max_delta_vel, kick_speed - self.fast_twist_vel)
        self.last_update = time()

        vel = aux.rotate(aux.Point(self.fast_twist_vel, 0), self.dribbling_angle)
        wvel = self.fast_twist_w

        current_action.vel, current_action.angle = vel, wvel
        current_action.dribbler_speed = 15  # int(max(7, 15 - 0.5 * abs(self.fast_twist_w)))  # NOTE
        print(vel, wvel)
        print("\t", self.dribbling_angle, self.fast_twist_vel)


# def shoot_to_goal(field: fld.Field, kicker: rbt.Robot, shoot_point: aux.Point, safe_kick: bool) -> Action:
#     "Удар по воротам"
#     ball = field.ball.get_pos()

# if safe_kick or const.IS_SIMULATOR_USED:  # NOTE
#     pass  # good

# is_ball_in_danger_zone = aux.is_point_inside_poly(
#     field.ball.get_pos(), field.enemy_goal.big_hull
# ) or aux.is_point_inside_poly(field.ball.get_pos(), field.ally_goal.big_hull)

# if is_ball_in_danger_zone:
#     point_ally = aux.nearest_point_on_poly(ball, field.ally_goal.big_hull)
#     point_enemy = aux.nearest_point_on_poly(ball, field.enemy_goal.big_hull)
#     point = point_ally if aux.is_point_inside_poly(field.ball.get_pos(), field.ally_goal.big_hull) else point_enemy

#     angle = aux.angle_to_point(point, ball)
#     field.strategy_image.draw_dot(ball, (255, 127, 127), 30)
#     return wp.Waypoint(shoot_point, angle, wp.WType.S_BALL_TWIST)  # danger zone

# if field.is_ball_in(kicker):
#     angle = aux.angle_to_point(kicker.get_pos(), ball)
#     field.strategy_image.draw_dot(ball, (255, 255, 0), 30)
#     return wp.Waypoint(shoot_point, angle, wp.WType.S_BALL_TWIST)  # twist

# nearest_enemy = fld.find_nearest_robot(ball, field.active_enemies())
# is_enemy_close = aux.dist(ball, nearest_enemy.get_pos()) - 100 < aux.dist(ball, kicker.get_pos())
# if is_enemy_close:
#     angle = aux.angle_to_point(field.ally_goal.center, ball)
#     field.strategy_image.draw_dot(ball, (255, 170, 0), 30)
#     return wp.Waypoint(shoot_point, angle, wp.WType.S_BALL_TWIST)  # goal defense

# is_enemy_near = aux.dist(ball, nearest_enemy.get_pos()) - 1000 < aux.dist(ball, kicker.get_pos())
# if is_enemy_near:  # NOTE
#     angle = aux.angle_to_point(kicker.get_pos(), ball)
#     field.strategy_image.draw_dot(ball, (255, 255, 0), 30)
#     return wp.Waypoint(shoot_point, angle, wp.WType.S_BALL_TWIST)  # twist

# field.strategy_image.draw_dot(ball, (0, 255, 0), 30)
# return Actions.Kick(ball)  # good


def angry_turn(
    field: fld.Field,
    kicker: rbt.Robot,
    kick_point: aux.Point,
    current_action: ActionValues,
    up: bool = False,
) -> None:
    """Turn without stopping"""
    current_action.beep = 1
    flag = (
        aux.is_point_inside_poly(field.ball.get_pos(), field.enemy_goal.big_hull)
        or aux.is_point_inside_poly(field.ball.get_pos(), field.ally_goal.big_hull)
        or aux.dist(
            aux.nearest_point_on_poly(field.ball.get_pos(), field.hull),
            field.ball.get_pos(),
        )
        < 200
    )

    x = aux.wind_down_angle(aux.angle_to_point(kicker.get_pos(), kick_point) - kicker.get_angle())

    if abs(x) < 0.25:
        w = x / 1.5
    else:
        w = math.copysign(5, x)
        if flag:
            w = math.copysign(3, x)

    vel, wvel = spin_with_ball(w, flag)

    if abs(x) < const.KICK_ALIGN_ANGLE * 2.5:
        if up:
            current_action.auto_kick = 2
        else:
            current_action.auto_kick = 1
    else:
        current_action.dribbler_speed = 15
    current_action.vel = vel
    current_action.angle = wvel
    current_action.beep = 1


def spin_with_ball(w: float, flag: bool = False) -> tuple[aux.Point, float]:
    """
    Расчёт скорости робота для поворота с мячом с угловой скоростью w (рад/сек)
    """
    _w = w + 0.0
    if 0.01 < abs(w) < 0.4:
        _w = math.copysign(0.4, w)

    if _w > 0:
        delta_r = aux.Point(50, 25)
    else:
        delta_r = aux.Point(-50, 25)

    if flag:
        delta_r = aux.Point(0, 0)

    vel = delta_r * _w

    return (vel, _w / 5)
