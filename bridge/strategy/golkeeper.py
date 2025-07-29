from typing import Optional

from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.const import State as GameStates
from bridge.router.base_actions import Action, Actions, KickActions 

voltage_kik = 15
voltage_pas = 4


class Goalkeeper():
    def __init__(self) -> None:

        # Индексы моих роботов
        self.gk_idx = const.GK
        self.idx1 = 0   
        self.idx2 =  2 
    
        # Индексы роботов соперника
        self.gk_idx_enem = 1
        self.idx_enem1 = 0
        self.idx_enem2 = 2

    def rungoal(self, field: fld.Field, actions: list[Optional[Action]]) -> None:
        
        robot_pos_GK = field.allies[self.gk_idx].get_pos()
        robot_pos1 = field.allies[self.idx1].get_pos()
        robot_pos2 = field.allies[self.idx2].get_pos()
    
        robot_pos_GK_enem = field.enemies[self.gk_idx_enem].get_pos()
        robot_pos1_enem = field.enemies[self.idx_enem1].get_pos()
        robot_pos2_enem = field.enemies[self.idx_enem2].get_pos()
    
        ball = field.ball.get_pos()


        g_up_xy_goal = field.enemy_goal.up - field.enemy_goal.eye_up * 65    
        g_down_xy_goal = field.enemy_goal.down + field.enemy_goal.eye_up * 65 

        up_goal = (g_up_xy_goal - robot_pos_GK_enem).mag()
        down_goal = (robot_pos_GK_enem + g_down_xy_goal).mag()

        if up_goal > down_goal:
            goal_position_gates = g_up_xy_goal
        else:
            goal_position_gates = g_down_xy_goal     

        angle_goal_ball = (goal_position_gates - robot_pos_GK).arg()


        if field.ball_start_point is not None:
            goal_position = aux.closest_point_on_line(field.ball_start_point, ball, robot_pos_GK, "R")
        else:
            goal_position = field.ally_goal.center

        position_goal = aux.is_point_inside_poly(goal_position, field.ally_goal.hull)

        if position_goal == False:
            goal_position = field.ally_goal.center

        angle_goalkeeper = (ball - robot_pos_GK).arg()
    
        actions[self.gk_idx] = Actions.GoToPoint(goal_position, angle_goal_ball)

    
        if field.is_ball_stop_near_goal():
            actions[self.gk_idx] = Actions.Kick(goal_position_gates, voltage_kik, is_upper=True)
    
        if field.is_ball_in(field.allies[self.gk_idx]):
            actions[self.gk_idx] = Actions.Kick(goal_position_gates, voltage_kik, is_upper=True)
