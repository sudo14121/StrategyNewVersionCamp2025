from typing import Optional

from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.const import State as GameStates
from bridge.router.base_actions import Action, Actions, KickActions 

voltage_kik = 8
voltage_pas = 8


class Goalkeeper():
    def __init__(self, idxN: int, idxR: int, idxE1: int, idxE2: int, idxG: int, idxEG: int) -> None:

        # Индексы моих роботов
        self.gk_idx = idxG
        self.idx1 = idxN   
        self.idx2 =  idxR
    
        # Индексы роботов соперника
        self.gk_idx_enem = idxEG
        self.idx_enem1 = idxE1
        self.idx_enem2 = idxE2

    def rungoal(self, field: fld.Field, actions: list[Optional[Action]]) -> None:
       
        robot_pos_gk = field.allies[self.gk_idx].get_pos()
        robot_pos1 = field.allies[self.idx1].get_pos()
        robot_pos2 = field.allies[self.idx2].get_pos()
    
        robot_pos_gk_enem = field.enemies[self.gk_idx_enem].get_pos()
        robot_pos1_enem = field.enemies[self.idx_enem1].get_pos()
        robot_pos2_enem = field.enemies[self.idx_enem2].get_pos()
    
    
        ball = field.ball.get_pos()


        g_up_xy_goal = field.enemy_goal.up #- field.enemy_goal.eye_up * 50    
        g_down_xy_goal = field.enemy_goal.down #+ field.enemy_goal.eye_up * 50

        up_goal = (g_up_xy_goal - robot_pos_gk_enem).mag()
        down_goal = (robot_pos_gk_enem + g_down_xy_goal).mag()

        if up_goal > down_goal:
            goal_position_gates = g_up_xy_goal
        else:
            goal_position_gates = g_down_xy_goal     

        angle_goal_ball = (goal_position_gates - robot_pos_gk).arg()

    
        if field.ball_start_point is not None:
            goal_position = aux.closest_point_on_line(field.ball_start_point, ball, robot_pos_gk, "R")
        else:
            goal_position = field.ally_goal.center

        position_goal = aux.is_point_inside_poly(goal_position, field.ally_goal.hull)

        if position_goal == False:
            goal_position = field.ally_goal.center

        angle_goalkeeper = (ball - robot_pos_gk).arg()
        
        if (aux.Point(0, 0) - goal_position).mag() > (aux.Point(0, 0) - field.ally_goal.center).mag():
            goal_position = (field.ally_goal.center + field.ally_goal.frw) / 3
        
        actions[self.gk_idx] = Actions.GoToPoint(goal_position, angle_goal_ball)
    
        if aux.is_point_inside_poly(field.ball.get_pos(), field.ally_goal.hull):
            actions[self.gk_idx] = Actions.Kick(goal_position_gates, voltage_kik, is_upper=True)

    
        if field.is_ball_in(field.allies[self.gk_idx]):
            actions[self.gk_idx] = Actions.Kick(goal_position_gates, voltage_kik,is_upper=True)
            
