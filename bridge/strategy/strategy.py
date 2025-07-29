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
        self.idxR = 6
        self.idxN = 1
        self.side = 1 #-1 if blue
        self.ronaldo = Ronaldo(self.idxR)

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
            #print(text)
            
        match field.game_state:
            case GameStates.RUN:
                self.run(field, actions)
            case GameStates.TIMEOUT:
                pass
            case GameStates.HALT:
                actions[self.idxR] = Actions.Stop()
                actions[self.idxN] = Actions.Stop()
                actions[field.gk_id] = Actions.Stop()
                return [None] * const.TEAM_ROBOTS_MAX_COUNT
            case GameStates.PREPARE_PENALTY:
                ballangel = (field.ball.get_pos() - field.allies[field.gk_id].get_pos()).arg()
                if not (self.we_active):
                    actions[self.idxR] = Actions.GoToPoint(aux.Point(field.polarity * -1 * 500, -250), ballangel)
                    actions[self.idxN] = Actions.GoToPoint(aux.Point(field.polarity * -1 * 500, 250), ballangel)
                else:
                    actions[self.idxR] = Actions.GoToPoint(aux.Point(field.polarity * 100, 0), ballangel)
                    actions[self.idxN] = Actions.GoToPoint(aux.Point(field.polarity * 500, 250), ballangel)
                actions[field.gk_id] = Actions.GoToPoint((field.ally_goal.frw + field.ally_goal.center) / 2, ballangel)
                pass
            case GameStates.PENALTY:
                if self.we_active:
                    self.ronaldo.choose_point_to_goal(field, actions)
                else:
                    self.goalkeeper.rungoal(field, actions)
                
            case GameStates.PREPARE_KICKOFF:
                if not self.we_active:
                    ronaldoxy2 = aux.get_line_intersection(field.ally_goal.frw_down, field.ally_goal.center_down, aux.Point(0, 100), aux.Point(0, 0), "LL")
                ronaldoxy = aux.get_line_intersection(field.ally_goal.frw_up, field.ally_goal.center_up, aux.Point(0, 100), aux.Point(0, 0), "LL")
                ballangel1 = (field.ball.get_pos() - field.allies[self.idxR].get_pos()).arg()
                ballangel2 = (field.ball.get_pos() - field.allies[self.idxN].get_pos()).arg()
                ballangel = (field.ball.get_pos() - field.allies[field.gk_id].get_pos()).arg()

                actions[self.idxR] = Actions.GoToPoint((ronaldoxy + field.ally_goal.frw_up) / 2, ballangel1) #type:ignore
                actions[self.idxN] = Actions.GoToPoint((ronaldoxy2 + field.ally_goal.frw_down) / 2, ballangel2) #type:ignore

                actions[field.gk_id] = Actions.GoToPoint((field.ally_goal.frw + field.ally_goal.center) / 2, ballangel)
                pass
            case GameStates.KICKOFF:
                pass
            case GameStates.FREE_KICK:
                pass
            case GameStates.STOP:
                # The router will automatically prevent robots from getting too close to the ball
                self.run(field, actions)
        #print(actions[self.idxR])
        return actions

    def run(self, field: fld.Field, actions: list[Optional[Action]]) -> None:
        if field.ally_color == const.Color.YELLOW:
            self.ronaldo.choose_point_to_goal(field, actions)
        else:
            self.goalkeeper.rungoal(field, actions)
           
    
