"""
Точка входа в стратегию
"""

from strategy_bridge.processors.box_feedback_collector import BoxFeedbackCollector
from strategy_bridge.runner import Runner

from bridge import const
from bridge.processors.drawing_processor import Drawer
from bridge.processors.field_creator import FieldCreator
from bridge.processors.python_controller import SSLController
from bridge.processors.router_processor import CommandSink

if __name__ == "__main__":
    # config.init_logging("./logs")

    PROCESSORS = [
        FieldCreator(
            debug_mode=True,
            debug_game_state=const.State.RUN,
        ),
        SSLController(
            ally_color=const.COLOR,
        ),
        Drawer(),
        CommandSink(),
    ]

    RUNNER = Runner(processors=PROCESSORS)
    RUNNER.run()
