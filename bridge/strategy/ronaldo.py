from typing import Optional

from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.const import State as GameStates
from bridge.router.base_actions import Action, Actions, KickActions 

class ronaldo():
    def __init__(self) -> None:
        self.idx = const.GK
        self.ballMem = [aux.Point(0, 0)] * 5
        