from __future__ import print_function

from autobahn.twisted.choosereactor import install_reactor
from twisted.internet.endpoints import TCP4ClientEndpoint
from twisted.internet.protocol import Protocol, ClientFactory


class TelemetryProtocol(Protocol):
    def __init__(self, telemetry_factory):
        self.factory = telemetry_factory

    def dataReceived(self, data):
        print("Ricevuto i dati: {}".format(data))


class TelemetryFactory(ClientFactory):
    def __init__(self):
        pass

    def buildProtocol(self, addr):
        return TelemetryProtocol(self)


reactor = install_reactor()

endpoint = TCP4ClientEndpoint(reactor, '127.0.0.1', 8007)
endpoint.connect(TelemetryFactory())
reactor.run()
