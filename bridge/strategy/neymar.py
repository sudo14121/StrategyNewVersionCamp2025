from typing import Optional

from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.const import State as GameStates
from bridge.router.base_actions import Action, Actions, KickActions 

class Neymar():
    def __init__(self, idx1R: int, idx1N: int, idxE1: int, idxE2: int) -> None:
        self.idx = idx1N
        self.idxN = idx1R
        self.ballMem = [aux.Point(0, 0)] * 5
        self.idxE1 = idxE1
        self.idxE2 = idxE2
        
    def choose_point_to_goal(self, field: fld.Field, actions: list[Optional[Action]]) -> None:
        angleD = abs(aux.get_angle_between_points(field.enemies[const.ENEMY_GK].get_pos(), field.allies[self.idx].get_pos(), field.enemy_goal.down))
        angleU = abs(aux.get_angle_between_points(field.enemies[const.ENEMY_GK].get_pos(), field.allies[self.idx].get_pos(), field.enemy_goal.up))
        k = 100
        if angleD > angleU:
            go = field.enemy_goal.down + (field.enemy_goal.eye_up * k)
        else:
            go = field.enemy_goal.up - (field.enemy_goal.eye_up * k)
        actions[self.idx] = Actions.Kick(go)
        print(actions[self.idx])

    def ifiam (self, field: fld.Field, actions: list[Optional[Action]]) -> bool:
        if (field.allies[self.idx].get_pos() - field.ball.get_pos()).mag() < (field.allies[self.idxN].get_pos() - field.ball.get_pos()).mag():
            return True
        else:
            return False
        
    def run (self, field: fld.Field, actions: list[Optional[Action]]) -> None:
        if self.ifiam(field, actions):
            self.attacher(field, actions)
        else:
            self.protect(field, actions)

    def attacher(self, field: fld.Field, actions: list[Optional[Action]]) -> None:
        self.choose_point_to_goal(field, actions)

    def protect(self, field: fld.Field, actions: list[Optional[Action]]) -> None:
        enimes_get = field.enemies[self.idxE1].get_pos()
        enimes_get1 = field.enemies[self.idxE2].get_pos()

        go = (enimes_get + enimes_get1) / 2
        ballangel = (field.ball.get_pos() - field.allies[self.idx].get_pos()).arg()
        if self.enimes_half(field, actions) == 1:
            actions[self.idx] = Actions.GoToPoint((field.ally_goal.center + aux.Point(0, 0)) / 5 * 3, ballangel)
        elif self.enimes_half(field, actions) == 0:
            actions[self.idx] = Actions.GoToPoint(aux.point_on_line(field.ally_goal.center, go, (field.ally_goal.center - go).mag() * 3 / 5), ballangel)
        else:
            actions[self.idx] = Actions.GoToPoint(aux.point_on_line(field.ally_goal.center, go, (field.ally_goal.center - go).mag() * 3 / 5), ballangel)
        
    def enimes_half(self, field: fld.Field, actions: list[Optional[Action]]) -> int:
        polog_enimes = 0
        
        if field.enemies[self.idxE1].get_pos().y >= 0:
            polog_enimes += 0
        else:
            polog_enimes += 1

        if field.enemies[self.idxE2].get_pos().y >= 0:
            polog_enimes += 0
        else:
            polog_enimes += 1

        return polog_enimes
        
    