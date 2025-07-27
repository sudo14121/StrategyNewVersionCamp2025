from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.const import State as GameStates
from bridge.router.base_actions import Action, Actions, KickActions  # type: ignorec

class Goalkeeper():
    def __init__(self):
        self.idx = const.GK
        self.ballMem = [aux.Point(0, 0)] * 5

    def keep(self, field: fld.Field, actions: list[Action]):
        angelB = (field.ball.get_pos() - field.allies[const.GK].get_pos()).arg()
        if field.is_ball_moves_to_goal():
            vecBallRobot2 = aux.get_line_intersection(self.ballMem[0], self.ballMem[4], (field.ally_goal.down + field.ally_goal.frw_down) / 2, (field.ally_goal.up + field.ally_goal.frw_up) / 2, "RS")
            field.strategy_image.draw_line(self.ballMem[0], self.ballMem[4], color=(255, 0, 0))
            field.strategy_image.draw_line(field.ally_goal.center_down, field.ally_goal.center_up, color=(0, 0, 255))
        else:
            vecBallRobot2 = aux.get_line_intersection(field.enemies[0].get_pos(), field.ball.get_pos(), (field.ally_goal.down + field.ally_goal.frw_down) / 2, (field.ally_goal.up + field.ally_goal.frw_up) / 2, "RS")
        if vecBallRobot2 is None:
            vecBallRobot2 = (field.ally_goal.center + field.ally_goal.frw) / 2 

        actions[const.GK] = Actions.GoToPoint(vecBallRobot2, angelB)
        
    def ball5cadrs(self, field: fld.Field, actions: list[Action]):
        for i in range(4, 0, -1):
            self.ballMem[i] = self.ballMem[i-1]
            print(i)
        self.ballMem[0] = field.ball.get_pos()