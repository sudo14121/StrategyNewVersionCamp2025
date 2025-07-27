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
        self.idx = 0
        self.robotin = 1
        self.vector = aux.Point(0, 0)
        self.tochka = aux.Point(0, 0)
        self.ballMem = [aux.Point(0, 0) * 5]

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

    def choose_point_to_goal(self, field: fld.Field, actions: list[Action]) -> None:
        angleD = abs(aux.get_angle_between_points(field.enemies[const.ENEMY_GK].get_pos(), field.allies[self.idx].get_pos(), field.enemy_goal.down))
        angleU = abs(aux.get_angle_between_points(field.enemies[const.ENEMY_GK].get_pos(), field.allies[self.idx].get_pos(), field.enemy_goal.up))
        
        if angleD > angleU:
            go = field.enemy_goal.down + (field.enemy_goal.eye_up * 100)
        else:
            go = field.enemy_goal.up + (field.enemy_goal.eye_up * -100)
        actions[self.idx] = Actions.Kick(go)
    
    def goolkeeper(self, field: fld.Field, actions: list[Action]):
        angelB = (field.ball.get_pos() - field.allies[const.GK].get_pos()).arg()
        vecBallRobot2 = aux.get_line_intersection(field.enemies[self.idx].get_pos(), field.ball.get_pos(), (field.ally_goal.down + field.ally_goal.frw_down) / 2, (field.ally_goal.up + field.ally_goal.frw_up) / 2, "RS")
        field.strategy_image.draw_line(field.ball.get_pos(), field.enemies[self.idx].get_pos(), color=(255, 0, 0))
        field.strategy_image.draw_line(field.ally_goal.center_down, field.ally_goal.center_up, color=(0, 0, 255))

        if vecBallRobot2 is None:
            vecBallRobot2 = (field.ally_goal.center + field.ally_goal.frw) / 2 
            
        actions[const.GK] = Actions.GoToPoint(vecBallRobot2, angelB)
        
    def run(self, field: fld.Field, actions: list[Optional[Action]]) -> None:
       
        angel = (field.ball.get_pos() - field.allies[self.idx].get_pos()).arg()
       
        print(field.ally_color)
        if field.ally_color == const.Color.YELLOW:
            self.choose_point_to_goal(field, actions)
        else:
            self.goolkeeper(field, actions)
           
