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
        self.idx = 1

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

    def run(self, field: fld.Field, actions: list[Optional[Action]]) -> None:
        """
        sigma = aux.get_line_intersection(field.allies[0].get_pos(), field.allies[1].get_pos(), field.enemies[0].get_pos(), field.ball.get_pos(), "LL")
        print(sigma)
        """
        #goalkeeper = aux.closest_point_on_line(field.ball.get_pos(), field.ally_goal.center, field.allies[1].get_pos(), "L")
        """
        angel = (field.ball.get_pos() - field.allies[idx].get_pos()).arg()
        ball = field.ball.get_pos()
        if ball.x > 2000:
            actions[idx] = Actions.GoToPointIgnore(aux.Point(0, 0), angel)
        else:
            actions[idx] = Actions.GoToPointIgnore(ball, angel)
        #print(goalkeeper)
        """
        """angel = (field.allies[0].get_pos() - field.allies[idx].get_pos()).arg()
        go = aux.dist(field.allies[0].get_pos(), field.enemies[0].get_pos())
        go = go / 7 * 2.5
        idxGo = aux.point_on_line(field.allies[0].get_pos(), field.enemies[0].get_pos(), go)
        actions[idx] = Actions.GoToPointIgnore(idxGo, angel)"""
        
        """
        field.strategy_image.draw_line(field.ball.get_pos(), field.allies[idx].get_pos(), color=(0, 255, 255))
        if self.pointNum == 1:
            actions[idx] = Actions.GoToPointIgnore(field.ally_goal.center_down - aux.Point(500, 0), angel)
            if aux.dist(field.allies[idx].get_pos(), field.ally_goal.center_down - aux.Point(500, 0)) < 150:
                self.pointNum = 2
        elif self.pointNum == 2:
            actions[idx] = Actions.GoToPointIgnore(field.ally_goal.center_up - aux.Point(500, 0), angel)
            if aux.dist(field.allies[idx].get_pos(), field.ally_goal.center_up - aux.Point(500, 0)) < 150:
                self.pointNum = 3   
        elif self.pointNum == 3:
            actions[idx] = Actions.GoToPointIgnore(field.enemy_goal.center_down + aux.Point(500, 0), angel)
            if aux.dist(field.allies[idx].get_pos(), field.enemy_goal.center_down + aux.Point(500, 0)) < 150:
                self.pointNum = 4
        elif self.pointNum == 4:
            actions[idx] = Actions.GoToPointIgnore(field.enemy_goal.center_up + aux.Point(500, 0), angel)
            if aux.dist(field.allies[idx].get_pos(), field.enemy_goal.center_up + aux.Point(500, 0)) < 150:
                self.pointNum = 1
        """
        """
        vect1 = field.allies[0].get_pos() - field.enemies[0].get_pos()
        print(vect1)
        vect2 = vect1.unity() * 500
        if self.pointNum == 1:
            actions[idx] = Actions.GoToPointIgnore(aux.rotate(vect2, 2/3 * 3.14) + field.enemies[0].get_pos(), 0)
            if aux.dist(field.allies[idx].get_pos(), aux.rotate(vect2, 2/3 * 3.14) + field.enemies[0].get_pos()) < 150:
                self.pointNum = 2
        elif self.pointNum == 2:
            actions[idx] = Actions.GoToPointIgnore(aux.rotate(vect2, 90 + 2/3 * 3.14) + field.enemies[0].get_pos(), 0)
            if aux.dist(field.allies[idx].get_pos(), aux.rotate(vect2, 90 + 2/3 * 3.14) + field.enemies[0].get_pos()) < 150:
                self.pointNum = 3
        elif self.pointNum == 3:
            actions[idx] = Actions.GoToPointIgnore(aux.rotate(vect2, 2/3 * 3.14 + 180) + field.allies[0].get_pos(), 0)
            if aux.dist(field.allies[idx].get_pos(), aux.rotate(vect2, 2/3 * 3.14 + 180) + field.allies[0].get_pos()) < 150:
                self.pointNum = 4
        elif self.pointNum == 4:
            actions[idx] = Actions.GoToPointIgnore(aux.rotate(vect2, 90 + 2/3 * 3.14 + 270) + field.allies[0].get_pos(), 0)
            if aux.dist(field.allies[idx].get_pos(), aux.rotate(vect2, 90 + 2/3 * 3.14 + 270) + field.allies[0].get_pos()) < 150:
                self.pointNum = 1   
        """
        match self.pointNum:
            case 1:
                x = field.ball.get_pos()
                distNeed = aux.dist(field.ball.get_pos(), field.allies[self.idx].get_pos())
            case 2:
                goalPointMy = field.ally_goal.hull
                goalPoint = field.enemy_goal.hull
                gg = aux.nearest_point_on_poly(field.allies[self.idx].get_pos(), goalPointMy)
                gg2 = aux.nearest_point_on_poly(field.allies[self.idx].get_pos(), goalPoint)
                if aux.dist(field.allies[self.idx].get_pos(), gg) < aux.dist(field.allies[self.idx].get_pos(), gg2):
                    gg2 = gg
                x = gg2
                distNeed = aux.dist(gg2, field.allies[self.idx].get_pos())
        if distNeed < 150:
            self.pointNum += 1
            if self.pointNum >= 3:
                self.pointNum = 1
        angel = (field.ball.get_pos() - field.allies[self.idx].get_pos()).arg()
        #actions[self.idx] = Actions.GoToPointIgnore(x, angel)

        angelBlue0 = field.enemies[0].get_angle()
        angelYel0 = field.allies[0].get_angle()
        angelYel4 = field.allies[4].get_angle()

        vecBT = aux.Point(5000, 0)
        vecYT = aux.Point(5000, 0)
        vecYTT = aux.Point(5000, 0)

        vecB0 = aux.rotate(vecBT, angelBlue0)

        vecY0 = aux.rotate(vecYT, angelYel0)

        vecY4 = aux.rotate(vecYTT, angelYel4)

        pointinter1 = aux.get_line_intersection(vecB0.unity() + field.enemies[0].get_pos(), vecB0.unity()  * 2 + field.enemies[0].get_pos(), vecY0.unity() + field.allies[0].get_pos(), vecY0.unity() * 2 + field.allies[0].get_pos(), "LL")
        pointinter2 = aux.get_line_intersection(vecB0.unity() + field.enemies[0].get_pos(), vecB0.unity()  * 2 + field.enemies[0].get_pos(), vecY4.unity() + field.allies[4].get_pos(), vecY4.unity() * 2 + field.allies[4].get_pos(), "LL")
        pointinter3 = aux.get_line_intersection(vecY4.unity() + field.allies[4].get_pos(), vecY4.unity()  * 2 + field.allies[4].get_pos(), vecY0.unity() + field.allies[0].get_pos(), vecY0.unity() * 2 + field.allies[0].get_pos(), "LL")
        

        if pointinter1 is None or pointinter2 is None or pointinter3 is None:
            print("Нет пересечений")
        else:
            mediana = (pointinter1 + pointinter2 + pointinter3) / 3 
            
        
        vecBallRobot2 = aux.point_on_line(field.ball.get_pos(), field.enemies[0].get_pos(), 500)

        actions[self.idx] = Actions.GoToPointIgnore(vecBallRobot2, angel)

        
