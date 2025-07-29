"""High-level strategy code"""

# !v DEBUG ONLY
import math  # type: ignore
from time import time  # type: ignore
from typing import Optional

from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.const import State as GameStates
from bridge.router.base_actions import Action, Actions, KickActions  # type: ignore
from bridge.strategy.golkeeper import Goalkeeper
from bridge.strategy.ronaldo import Ronaldo

class Strategy:
    """Main class of strategy"""

    def __init__(
        self,
    ) -> None:
        self.we_active = False
        self.goalkeeper = Goalkeeper()
        self.ronaldo = Ronaldo()
        self.idxR = 0
        self.idxN = 0
        self.side = 1 #-1 if blue

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
                actions[self.idxR] = Actions.Stop()
                actions[self.idxN] = Actions.Stop()
                actions[const.GK] = Actions.Stop()
                return [None] * const.TEAM_ROBOTS_MAX_COUNT
            case GameStates.PREPARE_PENALTY:
                ballangel = (field.ball.get_pos() - field.allies[const.GK].get_pos()).arg()
                if not (self.we_active):
                    actions[self.idxR] = Actions.GoToPoint(aux.Point(field.polarity * -1 * 500, -250), ballangel)
                    actions[self.idxN] = Actions.GoToPoint(aux.Point(field.polarity * -1 * 500, 250), ballangel)
                else:
                    actions[self.idxR] = Actions.GoToPoint(aux.Point(field.polarity * 100, 0), ballangel)
                    actions[self.idxN] = Actions.GoToPoint(aux.Point(field.polarity * 500, 250), ballangel)
                actions[const.GK] = Actions.GoToPoint((field.ally_goal.frw + field.ally_goal.center) / 2, ballangel)
                pass
            case GameStates.PENALTY:
                if self.we_active:
                    self.ronaldo.penalty(field, actions)
                else:
                    self.goalkeeper.rungoal(field, actions)
                pass
            case GameStates.PREPARE_KICKOFF:
                ronaldoxyi = field.ally_goal.frw_up
                actions[self.idxR] = Actions.GoToPoint(aux.Point(field.polarity * 100, 0), ballangel)
                actions[self.idxN] = Actions.GoToPoint(aux.Point(field.polarity * 500, 250), ballangel)
                actions[const.GK] = Actions.GoToPoint((field.ally_goal.frw + field.ally_goal.center) / 2, ballangel)
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
        if field.ally_color == const.Color.YELLOW:
            self.ronaldo.choose_point_to_goal(field, actions)
        else:
            self.goalkeeper.rungoal(field, actions)
           
    
