"""The service clients behind the handheld's commands, and the requests they get."""

from __future__ import annotations

import time

from pashupati_utils.utils.params import set_parameters_request
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from std_srvs.srv import Trigger

from dharma_interfaces.srv import LoadPath, MarkStopPoint, SetMode
from pashupati_bridge.core import commands
from pashupati_bridge.core.routes import route_path

CRUISE = 'speed.cruise'  # the follower's patrol speed


class CommandClients:
    """command name -> (client, request factory(args)); the factory raises ValueError on junk."""

    def __init__(self, node, config, speed_range):
        self.node = node
        self.config = config
        self.speed_range = speed_range  # () -> (low, high) m/s of the patrol speed
        self.table: dict = {}
        for name, service in commands.TRIGGERS.items():
            self.add_trigger(name, service)
        self.table['set_mode'] = (
            node.create_client(SetMode, 'control/set_mode'),
            lambda a: SetMode.Request(mode=commands.mode_of(a), source=config.remote_source),
        )
        self.table['mark_stop'] = (
            node.create_client(MarkStopPoint, 'route/mark_stop_point'),
            lambda a: MarkStopPoint.Request(
                dwell_sec=commands.dwell_seconds(a.get('dwell', 10.0))
            ),
        )
        self.table['load_route'] = (
            node.create_client(LoadPath, 'navigation/load_path'),
            lambda a: LoadPath.Request(path=route_path(config.sites_dir, a.get('file', ''))),
        )
        self.add_params('inspection_set', config.inspection_node, commands.inspection_parameters)
        self.add_params('people_set', config.people_node, commands.people_parameters)
        self.add_params('nav_speed', config.navigation_node, self.speed_parameters)
        self.robot_commands = [name for name in config.robot_commands if name]
        prefix = config.robot_command_prefix.rstrip('/') + '/'
        for name in self.robot_commands:
            self.add_trigger(name, prefix + name)
        follower = config.navigation_node.rstrip('/')
        self.speed_client = node.create_client(GetParameters, f'{follower}/get_parameters')

    def add_trigger(self, name: str, service: str):
        """Add a std_srvs/Trigger command."""
        client = self.node.create_client(Trigger, service)
        self.table[name] = (client, lambda _a: Trigger.Request())

    def add_params(self, name: str, node_name: str, pairs_of):
        """Add a command that sets live parameters of another node."""
        client = self.node.create_client(SetParameters, f'{node_name.rstrip("/")}/set_parameters')
        self.table[name] = (client, lambda a: set_parameters_request(pairs_of(a)))

    def speed_parameters(self, args: dict) -> list:
        """Return the follower's speed.cruise for a nav_speed command."""
        low, high = self.speed_range()
        return [(CRUISE, commands.patrol_speed(args.get('speed'), low, high))]

    def names(self) -> list[str]:
        """Every command the welcome lists."""
        return sorted([*self.table, *commands.LOCAL])


def speed_of(response) -> float | None:
    """Return speed.cruise of a GetParameters answer, None when it is not a double."""
    values = list(getattr(response, 'values', []) or [])
    if len(values) != 1 or values[0].type != ParameterType.PARAMETER_DOUBLE:
        return None
    return float(values[0].double_value)


class PatrolSpeed:
    """The follower's patrol speed for the handheld, asked one request at a time."""

    def __init__(self, client, speed_range):
        self.client = client
        self.speed_range = speed_range
        self.speed = None  # {target, min, max}; None without navigation
        self.asked = 0.0
        self.future = None

    def ask(self):
        """Ask for speed.cruise (no navigation running: no speed)."""
        self.asked = time.monotonic()
        if self.future is not None and not self.future.done():
            return
        if not self.client.service_is_ready():
            self.speed = None
            return
        self.future = self.client.call_async(GetParameters.Request(names=[CRUISE]))
        self.future.add_done_callback(self.on_answer)

    def on_answer(self, future):
        """Keep the follower's answer with the allowed range."""
        try:
            target = speed_of(future.result())
        except Exception:  # the node went away meanwhile: ask again later
            return
        if target is not None:
            low, high = self.speed_range()
            self.speed = {'target': round(target, 2), 'min': low, 'max': round(high, 2)}
