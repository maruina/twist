from __future__ import print_function

import os
import json

from twisted.internet.endpoints import TCP4ServerEndpoint
from twisted.internet.protocol import Protocol, ServerFactory, ProcessProtocol
from twisted.internet import task, reactor
from droneapi.lib import VehicleMode, Location
from pymavlink import mavutil

# Reading configuration
if os.path.exists('twist_env'):
    print("Importing Twist environment from env file...")
    with open('twist_env', 'r') as env_file:
        for line in env_file.readlines():
            if not line.startswith('#') and len(line.strip().split('=')) == 2:
                env_var = line.strip().split('=')
                os.environ[env_var[0]] = env_var[1]
else:
    raise EnvironmentError("Missing env file")

class Config(object):
    telemetry_frequence = float(os.getenv('TELEMETRY_FREQUENCE'))
    telemetry_port = int(os.getenv('TELEMETRY_PORT'))
    commands_port = int(os.getenv('COMMANDS_PORT'))
    media_server_ip = os.getenv('MEDIA_SERVER_IP')
    media_server_port = os.getenv('MEDIA_SERVER_PORT')

config = Config()

class TelemetryProtocol(Protocol):
    def __init__(self, factory_protocol):
        self.factory = factory_protocol
        self.telemetry_loop = None

    def connectionMade(self):
        self.telemetry_loop = task.LoopingCall(self.telemetry)
        self.telemetry_loop.start(config.telemetry_frequence)

    def telemetry(self):
        self.transport.write(json.dumps(self.factory.drone.telemetry))

    def connectionLost(self, reason='Connection Lost'):
        if self.telemetry_loop.running:
            self.telemetry_loop.stop()


class TelemetryFactory(ServerFactory):
    def __init__(self):
        self.drone = None

    def buildProtocol(self, addr):
        return TelemetryProtocol(self)


class CommandsProtocol(Protocol):
    def __init__(self, factory_protocol):
        self.factory = factory_protocol

    def dataReceived(self, data):
        self.factory.drone.route_command(data)


class CommandsFactory(ServerFactory):
    def __init__(self):
        self.drone = None

    def buildProtocol(self, addr):
        return CommandsProtocol(self)


class StreamerProtocol(ProcessProtocol):
    def __init__(self):
        pass

class Drone(object):
    def __init__(self):
        self.api = local_connect()
        self.vehicle = self.api.get_vehicles()[0]
        self.commands = self.vehicle.commands
        self.current_location = None
        self._log("Drone Start")
        self.telemetry = {
            'lat': 0,
            'lon': 0,
            'alt': 0,
            'armed': False,
            'pitch': 0,
            'roll': 0,
            'yaw': 0,
            'mode': "",
            'hdop': 0,
            'fix_type': 0,
            'satellites': 0,
            'ground_speed': 0.0,
            'air_speed': 0.0,
            'battery_voltage': 0.0,
            'battery_current': 0.0,
            'battery_level': 0,
            'heading': 0,
            'throttle': 0,
            'climb_rate': 0.0
        }
        self.stream_process = None
        self.streaming = False

        # Register observers
        self.vehicle.add_attribute_observer('armed', self.armed_callback)
        self.vehicle.add_attribute_observer('location', self.location_callback)
        self.vehicle.add_attribute_observer('mode', self.mode_callback)
        self.vehicle.add_attribute_observer('gps_0', self.gps_callback)
        self.vehicle.add_attribute_observer('attitude', self.attitude_callback)
        self.vehicle.add_attribute_observer('battery', self.battery_callback)
        self.vehicle.add_attribute_observer('hud', self.hud_callback)

    def arm(self):
        self._log('Command ARM received')
        if not self.vehicle.armed:
            if self.vehicle.mode.name != 'guided':
                self.set_mode('GUIDED')
            self.vehicle.armed = True
            self.vehicle.flush()

    def disarm(self):
        self._log('Command DISARM received')
        if self.vehicle.armed:
            self.vehicle.armed = False
            self.vehicle.flush()

    def set_mode(self, mode):
        self._log('Command MODE {} received'.format(mode))
        modes = {'GUIDED', 'LOITER', 'LAND', 'AUTO'}
        if mode not in modes:
            raise ValueError('Mode {} not supported'.format(mode))
        self.vehicle.mode = VehicleMode(mode)
        self.vehicle.flush()

    def takeoff(self, altitude):
        self._log('Command TAKEOFF received')
        if self.vehicle.mode.name != 'guided':
            self.set_mode('GUIDED')
        self.commands.takeoff(float(altitude))
        self.vehicle.flush()

    def land(self):
        self._log('Command LAND received')
        self.set_mode('LAND')

    def go_to_wp(self, lat, lon, alt):
        self._log('Command GOTOWP received')
        if self.vehicle.mode.name != 'guided':
            self.set_mode('GUIDED')
        self.vehicle.commands.goto(Location(float(lat), float(lon), float(alt)))
        self.vehicle.flush()

    def monitor(self):
        self._log('Command MONITOR received')

    def recharge(self):
        self._log('Command RECHARGE received')

    def alarm(self):
        self._log('Command ALARM received')

    def start_stream(self):
        if not self.streaming:
            self._log('Command START_STREAM received')
            streamer_args = "rpicamsrc bitrate=1000000 ! video/x-h264,width=1280,height=720,framerate=15/1,profile=high" \
                            " ! h264parse ! flvmux ! rtmpsink location='rtmp://" + config.media_server_ip + ":" + \
                            config.media_server_port + "/rtmp/live live=1'"
            reactor.spawnProcess(self.streamer, 'gst-launch-1.0', args=['gst-launch-1.0', streamer_args])
            self.streaming = True
        else:
            self._log('Camera is streaming')

    def stop_stream(self):
        if self.streaming:
            self._log('Command STOP_STREAM received')
            self.streamer.transport.signalProcess('KILL')
            self.streaming = False
        else:
            self._log('Camera is stopped')

    def set_mission(self, commands):
        """
        Read an array of command to create a drone Mission
        :param commands:
        :return:
        """
        pass

    def route_command(self, command_message):
        """
        Read the message coming from the CommandsFactory and do the appropriate action
        :param command_message:
        :return:
        """
        commands = {'arm', 'disarm', 'set_mode', 'takeoff', 'land', 'go_to_wp', 'monitor', 'recharge', 'alarm',
                    'start_stream', 'stop_stream'}

        try:
            print('Message: {}'.format(command_message))
            message = json.loads(command_message)
            print("Command {} received".format(message))
        except ValueError as e:
            self._log(e.message)
            return

        if message['command'] not in commands:
            raise NotImplementedError("Command {} not implemented yet".format(message['command']))

        cases = {
            'arm': lambda msg: self.arm(),
            'disarm': lambda msg: self.disarm(),
            'set_mode': lambda msg: self.set_mode(msg['mode']),
            'takeoff': lambda msg: self.takeoff(msg['alt']),
            'land': lambda msg: self.land(),
            'go_to_wp': lambda msg: self.go_to_wp(msg['lat'], msg['lon'], msg['alt']),
            'monitor': lambda msg: self.monitor(),
            'recharge': lambda msg: self.recharge(),
            'alarm': lambda msg: self.alarm(),
            'start_stream': lambda msg: self.start_stream(),
            'stop_stream': lambda msg: self.stop_stream(),
            'set_mission': lambda msg: self.set_mission(msg['commands'])
        }

        cases[message['command']](message)

    @staticmethod
    def _log(message):
        print("[TWIST]: {0}".format(message))

    def location_callback(self, location):
        self.current_location = self.vehicle.location
        self.telemetry['lon'] = self.vehicle.location.lon
        self.telemetry['lat'] = self.vehicle.location.lat
        self.telemetry['alt'] = self.vehicle.location.alt

    def armed_callback(self, armed):
        self.telemetry['armed'] = self.vehicle.armed

    def mode_callback(self, mode):
        self.telemetry['mode'] = self.vehicle.mode.name

    def gps_callback(self, gps):
        self.telemetry['hdop'] = self.vehicle.gps_0.eph
        self.telemetry['fix_type'] = self.vehicle.gps_0.fix_type
        self.telemetry['satellites'] = self.vehicle.gps_0.satellites_visible

    def attitude_callback(self, attitude):
        self.telemetry['pitch'] = self.vehicle.attitude.pitch
        self.telemetry['roll'] = self.vehicle.attitude.roll
        self.telemetry['yaw'] = self.vehicle.attitude.yaw

    def battery_callback(self, battery):
        self.telemetry['battery_voltage'] = self.vehicle.battery.voltage
        self.telemetry['battery_current'] = self.vehicle.battery.current
        self.telemetry['battery_level'] = self.vehicle.battery.level

    def hud_callback(self, heading):
        self.telemetry['heading'] = self.vehicle.hud.heading
        self.telemetry['throttle'] = self.vehicle.hud.throttle
        self.telemetry['climb_rate'] = self.vehicle.hud.climb_rate
        self.telemetry['ground_speed'] = self.vehicle.hud.ground_speed
        self.telemetry['air_speed'] = self.vehicle.hud.air_speed


drone = Drone()
telemetry_factory = TelemetryFactory()
commands_factory = CommandsFactory()
stream_protocol = StreamerProtocol()

telemetry_factory.drone = drone
telemetry_factory.protocol = TelemetryProtocol
commands_factory.drone = drone
commands_factory.protocol = CommandsProtocol
drone.streamer = stream_protocol

telemetry_endpoint = TCP4ServerEndpoint(reactor, config.telemetry_port)
telemetry_endpoint.listen(telemetry_factory)
commands_endpoint = TCP4ServerEndpoint(reactor, config.commands_port)
commands_endpoint.listen(commands_factory)

reactor.run(installSignalHandlers=0)
