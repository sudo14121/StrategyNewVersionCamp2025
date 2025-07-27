"""High-level strategy code"""

# !v DEBUG ONLY
import math  # type: ignore
from time import time  # type: ignore
from typing import Optional

from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.const import State as GameStates
from bridge.router.base_actions import Action, Actions, KickActions  # type: ignore


class Strategy:
    """Main class of strategy"""

    def __init__(
        self,
    ) -> None:
        self.we_active = False
        self.pointNum = 1
        self.idx = 1
        self.robotin = 1
        self.vector = aux.Point(0, 0)
        self.tochka = aux.Point(0, 0)

    def process(self, field: fld.Field) -> list[Optional[Action]]:
        """Game State Management"""
        if field.game_state not in [GameStates.KICKOFF, GameStates.PENALTY]:
            if field.active_team in [const.Color.ALL, field.ally_color]:
                self.we_active = True
            else:
                self.we_active = False

        actions: list[Optional[Action]] = []
        for _ in range(const.TEAM_ROBOTS_MAX_COUNT):
            actions.append(None)

        if field.ally_color == const.COLOR:
            text = str(field.game_state) + "  we_active:" + str(self.we_active)
            field.strategy_image.print(aux.Point(600, 780), text, need_to_scale=False)
        match field.game_state:
            case GameStates.RUN:
                self.run(field, actions)
            case GameStates.TIMEOUT:
                pass
            case GameStates.HALT:
                return [None] * const.TEAM_ROBOTS_MAX_COUNT
            case GameStates.PREPARE_PENALTY:
                pass
            case GameStates.PENALTY:
                pass
            case GameStates.PREPARE_KICKOFF:
                pass
            case GameStates.KICKOFF:
                pass
            case GameStates.FREE_KICK:
                pass
            case GameStates.STOP:
                # The router will automatically prevent robots from getting too close to the ball
                self.run(field, actions)

        return actions

    def run(self, field: fld.Field, actions: list[Optional[Action]]) -> None:
       
        angel = (field.ball.get_pos() - field.allies[self.idx].get_pos()).arg()

        vecBallRobot2 = -field.allies[self.idx].get_pos() + field.ball.get_pos()

        match self.robotin:
            case 1:
                for i in range(6):
                    if aux.line_circle_intersect(field.ball.get_pos(), field.allies[self.idx].get_pos(), field.enemies[i].get_pos(), 100, "S"):
                        vectRobot = aux.get_angle_between_points(field.enemies[i].get_pos(), field.ball.get_pos(), field.allies[self.idx].get_pos())
                        if vectRobot > 0:
                            print(1)
                            self.vector = aux.rotate(vecBallRobot2, 1/2 * 3.14)
                        else:
                            self.vector = aux.rotate(vecBallRobot2, -1/2 * 3.14)
                        self.robotin = 2
                        self.tochka = (self.vector.unity() * 500) + field.allies[self.idx].get_pos()
            case 2:
                actions[self.idx] = Actions.GoToPoint(self.tochka, angel)
                if aux.dist(field.allies[self.idx].get_pos(), self.tochka) < 50:
                    self.robotin = 1
            case 3:
                actions[self.idx] = Actions.GoToPointIgnore(aux.Point(0, 0), angel)
                if aux.dist(field.allies[self.idx].get_pos(), aux.Point(0, 0)) < 50:
                    self.robotin = 1

        if not aux.is_point_inside_poly(field.allies[self.idx].get_pos(), field.hull):
            self.robotin = 3


