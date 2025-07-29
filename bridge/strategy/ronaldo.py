from typing import Optional

from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.const import State as GameStates
from bridge.router.base_actions import Action, Actions, KickActions 

class Ronaldo():
    def __init__(self) -> None:
        self.idx = 0
        self.ballMem = [aux.Point(0, 0)] * 5
        
    def choose_point_to_goal(self, field: fld.Field, actions: list[Optional[Action]]) -> None:
        angleD = abs(aux.get_angle_between_points(field.enemies[const.ENEMY_GK].get_pos(), field.allies[self.idx].get_pos(), field.enemy_goal.down))
        angleU = abs(aux.get_angle_between_points(field.enemies[const.ENEMY_GK].get_pos(), field.allies[self.idx].get_pos(), field.enemy_goal.up))
        k = 100
        if angleD > angleU:
            go = field.enemy_goal.down + (field.enemy_goal.eye_up * k)
        else:
            go = field.enemy_goal.up - (field.enemy_goal.eye_up * k)
        actions[self.idx] = Actions.Kick(go)

    def penalty(self, field: fld.Field, actions: list[Optional[Action]]) -> None:
        k = 90
        up = field.enemy_goal.up - (field.enemy_goal.eye_up * k)
        down = field.enemy_goal.down + (field.enemy_goal.eye_up * k)

        upGk = aux.dist(field.enemies[const.ENEMY_GK].get_pos(), up)
        downGk = aux.dist(field.enemies[const.ENEMY_GK].get_pos(), down)

        if upGk > downGk:
            go = up
        else:
            go = down

        actions[self.idx] = Actions.Kick(go)
        
    