"""
Модуль-прослойка между стратегией и отправкой пакетов на роботов
"""

import math
import socket
from time import sleep, time
from typing import Optional

import attr
from strategy_bridge.bus import DataBus, DataReader, DataWriter
from strategy_bridge.processors import BaseProcessor

from bridge import const, drawing
from bridge.auxiliary import aux, fld
from bridge.processors.python_controller import RobotCommand
from bridge.router.action import Action, ActionDomain, ActionValues
from bridge.router.base_actions import Actions

UDP_IP = "10.0.120.210"
UDP_PORT = 10000


@attr.s(auto_attribs=True)
class CommandSink(BaseProcessor):
    """
    Прослойка между стратегией и отправкой пакетов на роботов
    """

    processing_pause: Optional[float] = 0.001
    reduce_pause_on_process_time: bool = False

    def initialize(self, data_bus: DataBus) -> None:
        """
        Инициализация
        """
        super().initialize(data_bus)
        self.field_reader = DataReader(data_bus, const.FIELD_TOPIC)
        self.commands_sink_reader = DataReader(data_bus, const.CONTROL_TOPIC)
        self.image_writer = DataWriter(data_bus, const.IMAGE_TOPIC, 20)

        self.tmp_timer = time()

        self.field_b = fld.Field(const.Color.BLUE)
        self.field_y = fld.Field(const.Color.YELLOW)
        self.field: dict[const.Color, fld.Field] = {
            const.Color.BLUE: self.field_b,
            const.Color.YELLOW: self.field_y,
        }

        self.field[const.COLOR].router_image.timer = drawing.FeedbackTimer(time(), 10, 50)

        self.waypoints_b: list[Action] = [Actions.Stop() for _ in range(const.TEAM_ROBOTS_MAX_COUNT)]
        self.waypoints_y: list[Action] = [Actions.Stop() for _ in range(const.TEAM_ROBOTS_MAX_COUNT)]
        self.actions: dict[const.Color, list[Action]] = {
            const.Color.BLUE: self.waypoints_b,
            const.Color.YELLOW: self.waypoints_y,
        }

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def process(self) -> None:
        """
        Метод обратного вызова процесса
        """
        updated = False

        new_field = self.field_reader.read_last()
        if new_field is not None:
            updated_field: fld.Field = new_field.content
            if self.field_b.last_update != updated_field.last_update:
                self.field_b.update_field(updated_field)
                self.field_y.update_field(updated_field)
                updated = True

        cmds = self.commands_sink_reader.read_new()
        for cmd in cmds:
            command: RobotCommand = cmd.content
            if command.color == const.Color.BLUE:
                self.waypoints_b[command.r_id] = command.action
            else:
                self.waypoints_y[command.r_id] = command.action
            updated = True

        if updated:
            self.field[const.COLOR].router_image.timer.start(time())
            for color in [const.Color.BLUE, const.Color.YELLOW]:
                for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                    if self.field[color].allies[i].is_used():
                        self.field[color].allies[i].clear_fields()

                        domain = ActionDomain(
                            field=self.field[color],
                            game_state=self.field[color].game_state,
                            we_active=self.field[color].active_team in [const.Color.ALL, color],
                            robot=self.field[color].allies[i],
                        )
                        values = ActionValues()
                        # print(self.actions[color][i])
                        self.actions[color][i].process(domain, values)
                        action_values_to_rules(values, domain)
                        # print(domain.robot.speed_x)
                        # print(self.field[color].allies[i])

            self.field[const.COLOR].router_image.timer.end(time())
            self.image_writer.write(self.field[const.COLOR].router_image)
            self.image_writer.write(self.field[const.COLOR].path_image)
            self.field_b.clear_images()
            self.field_y.clear_images()

            self.send_rules()

    def finalize(self) -> None:

        for _ in range(50):
            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                packet = create_packet(i, *tuple([0] * 11))
                self.sock.sendto(packet, (UDP_IP, UDP_PORT))
            sleep(0.001)
            continue

        self.sock.close()

    def send_rules(self) -> None:
        """
        Сформировать массив команд для отправки на роботов
        """

        if const.IS_SIMULATOR_USED:
            return
            # b_control_team = self.field_b.allies
            # for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            #     rules.append(0)
            #     rules.append(b_control_team[i].speed_x)
            #     rules.append(b_control_team[i].speed_y)
            #     rules.append(b_control_team[i].speed_r)
            #     rules.append(b_control_team[i].kick_up_)
            #     rules.append(b_control_team[i].kick_forward_)
            #     rules.append(b_control_team[i].auto_kick_)
            #     rules.append(min(float(b_control_team[i].kicker_voltage_ / 1.5), 7))
            #     rules.append(b_control_team[i].dribbler_speed_ > 0)
            #     rules.append(b_control_team[i].dribbler_speed_)
            #     rules.append(b_control_team[i].kicker_voltage_ > 0)
            #     rules.append(b_control_team[i].beep)
            #     rules.append(0)

            # y_control_team = self.field_y.allies
            # for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            #     rules.append(0)
            #     rules.append(y_control_team[i].speed_x)
            #     rules.append(y_control_team[i].speed_y)
            #     rules.append(y_control_team[i].speed_r)
            #     rules.append(y_control_team[i].kick_up_)
            #     rules.append(y_control_team[i].kick_forward_)
            #     rules.append(y_control_team[i].auto_kick_)
            #     rules.append(min(float(y_control_team[i].kicker_voltage_ / 1.5), 7))
            #     rules.append(y_control_team[i].dribbler_speed_ > 0)
            #     rules.append(y_control_team[i].dribbler_speed_)
            #     rules.append(y_control_team[i].kicker_voltage_ > 0)
            #     rules.append(b_control_team[i].beep)
            #     rules.append(0)
        else:

            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                # packet = create_packet(i, *tuple([15] * 11))
                # self.sock.sendto(packet, (UDP_IP, UDP_PORT))
                # print(f"send 15s to {i} robot")
                # continue

                ctrl_idx = const.CONTROL_MAPPING[i]
                if self.field[const.COLOR].allies[ctrl_idx].is_used():
                    control_robot = self.field[const.COLOR].allies[ctrl_idx]
                elif self.field_y.allies[ctrl_idx].is_used():
                    control_robot = self.field_y.allies[ctrl_idx]
                elif self.field_b.allies[ctrl_idx].is_used():
                    control_robot = self.field_b.allies[ctrl_idx]
                else:
                    continue

                if i in const.REVERSED_KICK:
                    control_robot.kick_forward_, control_robot.kick_up_ = (
                        control_robot.kick_up_,
                        control_robot.kick_forward_,
                    )
                    if control_robot.auto_kick_ == 2:
                        control_robot.auto_kick_ = 1
                    elif control_robot.auto_kick_ == 1:
                        control_robot.auto_kick_ = 2

                angle_info = (
                    math.log(18 / math.pi * abs(control_robot.delta_angle) + 1)
                    * aux.sign(control_robot.delta_angle)
                    * (100 / math.log(18 + 1))
                )

                packet = create_packet(
                    i,
                    int(control_robot.speed_x),
                    int(control_robot.speed_y),
                    int(angle_info),
                    control_robot.dribbler_speed_,
                    control_robot.kicker_voltage_,
                    control_robot.kick_up_,
                    control_robot.kick_forward_,
                    control_robot.beep,
                    control_robot.dribbler_speed_ > 0,
                    control_robot.kicker_voltage_ > 0,
                    control_robot.auto_kick_,
                )

                self.sock.sendto(packet, (UDP_IP, UDP_PORT))


def action_values_to_rules(values: ActionValues, domain: ActionDomain) -> None:
    """Turn ActionValues to commands for robots"""
    domain.robot.kick_up_ = values.kick_up
    domain.robot.kick_forward_ = values.kick_forward
    domain.robot.auto_kick_ = values.auto_kick
    domain.robot.kicker_voltage_ = values.kicker_voltage
    domain.robot.dribbler_speed_ = values.dribbler_speed

    if values.beep == 0:
        domain.robot.update_vel_xy(values.vel)
        aerr = aux.wind_down_angle(values.angle - domain.robot.get_angle())
        if const.IS_SIMULATOR_USED:
            ang_vel = domain.robot.angle_reg.process(aerr, -domain.robot.get_anglevel())
            domain.robot.update_vel_w(ang_vel)
        else:
            domain.robot.delta_angle = aerr

        reg_vel = aux.Point(domain.robot.speed_x, -domain.robot.speed_y)
        domain.field.router_image.draw_line(
            domain.robot.get_pos(),
            domain.robot.get_pos() + aux.rotate(reg_vel, domain.robot.get_angle()) * 10,
        )
    else:
        # print("manual speeds: ", values.vel, values.angle)

        domain.robot.speed_x = -1 / domain.robot.k_xx * values.vel.x
        domain.robot.speed_y = 1 / domain.robot.k_yy * values.vel.y
        domain.robot.delta_angle = values.angle


def create_packet(
    bot_number: int,  # unsigned byte (0-255)
    speed_x: int,  # signed byte (-128 to 127)
    speed_y: int,  # signed byte
    speed_w: int,  # signed byte
    dribbler_speed: int,  # unsigned byte
    kicker_voltage: int,  # unsigned byte
    kick_up: int,  # boolean flag (bit 0)
    kick_down: int,  # boolean flag (bit 1)
    beep: int,  # boolean flag (bit 2)
    dribbler_en: int,  # boolean flag (bit 3)
    charge_en: int,  # boolean flag (bit 4)
    autokick: int,  # unsigned byte
) -> bytes:
    # Convert all values to bytes and pack into a list
    bytes_list = [
        0x01,  # Header
        bot_number,
        speed_x.to_bytes(1, "big", signed=True)[0],
        speed_y.to_bytes(1, "big", signed=True)[0],
        speed_w.to_bytes(1, "big", signed=True)[0],
        dribbler_speed,
        kicker_voltage,
        kick_up,
        kick_down,
        beep,
        dribbler_en,
        charge_en,
        autokick,
    ]

    return bytes(bytes_list)
