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
from bridge.strategy.neymar import Neymar
from bridge.strategy.states import states

class Strategy:
    """Main class of strategy"""

    def __init__(
        self,
    ) -> None:
        self.we_active = False
        self.idxR = 0
        self.idxN = 2
        self.idxE1 = 0
        self.idxE2 = 2
        self.ronaldo = Ronaldo(self.idxR, self.idxN, self.idxE1, self.idxE2)
        self.neymar = Neymar(self.idxR, self.idxN, self.idxE1, self.idxE2)
        self.states = states(self.idxN, self.idxR, self.idxE1, self.idxE2)
        self.goalkeeper = Goalkeeper(self.idxN, self.idxR, self.idxE1, self.idxE2)

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
                self.states.halt(field, actions)
                return [None] * const.TEAM_ROBOTS_MAX_COUNT
            case GameStates.PREPARE_PENALTY:
                self.states.prepare_penalty(field, actions, self.we_active)
            case GameStates.PENALTY:
                self.states.penalty(field, actions, self.we_active)
            case GameStates.PREPARE_KICKOFF:
                self.states.prepare_kikoff(field, actions, self.we_active)
            case GameStates.KICKOFF:
                self.states.kikoff(field, actions, self.we_active)
            case GameStates.FREE_KICK:
                pass
            case GameStates.STOP:
                # The router will automatically prevent robots from getting too close to the ball
                self.run(field, actions)
        #print(actions[self.idxR])
        return actions

    def run(self, field: fld.Field, actions: list[Optional[Action]]) -> None:
        self.ronaldo.opening_to_the_ball(field, actions)
        #self.neymar.run(field, actions)
        #self.goalkeeper.rungoal(field, actions)
           
    
