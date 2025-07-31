from typing import Optional

from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.const import State as GameStates
from bridge.router.base_actions import Action, Actions, KickActions 
from bridge.strategy.golkeeper import Goalkeeper
from bridge.strategy.ronaldo import Ronaldo
from bridge.strategy.neymar import Neymar

class states():
    def __init__(self, idx1N: int, idx1R: int, idxE1: int, idxE2: int, idxG: int, idxEG: int) -> None:
        self.idxN = idx1N
        self.idxR = idx1R
        self.ballMem = [aux.Point(0, 0)] * 5
        self.idxE1 = idxE1
        self.idxE2 = idxE2
        self.goalkeeper = Goalkeeper(self.idxN, self.idxR, self.idxE1, self.idxE2, idxG, idxEG)
        self.ronaldo = Ronaldo(self.idxR, self.idxN, self.idxE1, self.idxE2)
        self.neymar = Neymar(self.idxR, self.idxN, self.idxE1, self.idxE2)


    def halt(self, field: fld.Field, actions: list[Optional[Action]]) -> None:
        actions[self.idxR] = Actions.Stop()
        actions[self.idxN] = Actions.Stop()
        actions[field.gk_id] = Actions.Stop()

    def prepare_penalty(self, field: fld.Field, actions: list[Optional[Action]], we_active: bool) -> None:
        ballangel = (field.ball.get_pos() - field.allies[field.gk_id].get_pos()).arg()
        if not (we_active):
            actions[self.idxR] = Actions.GoToPoint(aux.Point(field.polarity * -1 * 700, -350), ballangel)
            actions[self.idxN] = Actions.GoToPoint(aux.Point(field.polarity * -1 * 700, 350), ballangel)
        else:
            actions[self.idxR] = Actions.GoToPoint(aux.Point(field.polarity * 400, 0), ballangel)
            actions[self.idxN] = Actions.GoToPoint(aux.Point(field.polarity * 900, 250), ballangel)
        actions[field.gk_id] = Actions.GoToPoint((field.ally_goal.frw + field.ally_goal.center) / 2, ballangel)

    def penalty (self, field: fld.Field, actions: list[Optional[Action]], we_active: bool) -> None:
        if we_active:
            self.ronaldo.choose_point_to_goal(field, actions)
        else:
            self.goalkeeper.rungoal(field, actions)
    
    def prepare_kikoff(self, field: fld.Field, actions: list[Optional[Action]], we_active: bool) -> None:
        ballangel1 = (field.ball.get_pos() - field.allies[self.idxR].get_pos()).arg()
        ballangel2 = (field.ball.get_pos() - field.allies[self.idxN].get_pos()).arg()

        if not we_active:
            ronaldoxy = aux.get_line_intersection(field.ally_goal.frw_up, field.ally_goal.center_up, aux.Point(0, 100), aux.Point(0, 0), "LL")
            ronaldoxy2 = aux.get_line_intersection(field.ally_goal.frw_down, field.ally_goal.center_down, aux.Point(0, 100), aux.Point(0, 0), "LL")
            actions[self.idxN] = Actions.GoToPoint((ronaldoxy2 + field.ally_goal.frw_down) / 2, ballangel2) #type:ignore
            actions[self.idxR] = Actions.GoToPoint((ronaldoxy + field.ally_goal.frw_up) / 2, ballangel1) #type:ignore
        else:
            ronaldoxy = aux.point_on_line(field.ball.get_pos(), field.ally_goal.center, 1000)
            ronaldoxy2 = aux.point_on_line(field.ball.get_pos(), field.ally_goal.center, 250)
            actions[self.idxN] = Actions.GoToPoint(ronaldoxy2, ballangel2)
            actions[self.idxR] = Actions.GoToPoint(ronaldoxy, ballangel1)

        ballangel = (field.ball.get_pos() - field.allies[field.gk_id].get_pos()).arg()
        actions[field.gk_id] = Actions.GoToPoint((field.ally_goal.frw + field.ally_goal.center) / 2, ballangel)
    
    def kikoff(self, field: fld.Field, actions: list[Optional[Action]], we_active: bool) -> None:
        ronaldoxy2 = aux.point_on_line(field.ball.get_pos(), field.ally_goal.center, -350)
        print(we_active)
        if we_active:
            if (field.allies[self.idxN].get_pos() + ronaldoxy2).mag() < 20:
                ballangel2 = (field.ball.get_pos() - field.allies[self.idxN].get_pos()).arg()
                actions[self.idxN] = Actions.GoToPoint(ronaldoxy2, ballangel2)
            else:
                actions[self.idxN]  =Actions.Kick(field.allies[self.idxR].get_pos())
        else:
            pass

    def freekick(self, field: fld.Field, actions: list[Optional[Action]], we_active: bool) -> None:
        if we_active:
            if self.ronaldo.ifiam(field, actions):
                self.neymar.opening_to_the_ball(field, actions)
                self.ronaldo.passs(field, actions)
            else:
                self.ronaldo.opening_to_the_ball(field, actions)
                self.neymar.passs(field, actions)
        else:
            actions[self.idxR] = Actions.GoToPoint(aux.point_on_line(field.ball.get_pos(), fld.find_nearest_robot(field.ball.get_pos(), field.enemies).get_pos(), -500), (field.ball.get_pos() - field.allies[self.idxR].get_pos()).arg())
            self.neymar.protect(field, actions)
    
