
import inet.applications.udpapp.UdpBasicApp;
import inet.node.inet.Router;
import inet.node.internetcloud.InternetCloud;
import flora.LoRaPhy.LoRaMedium;
import flora.LoraNode.LoRaNode;
import flora.LoraNode.LoRaGW;
import inet.node.inet.StandardHost;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth1G;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.contract.IObjectCache;
@license(LGPL);
//
// TODO documentation
//
network LoRaWANNetwork
{
    parameters:
        int numberOfNodes = default(10);
        int numberOfGateways = default(3);
        int networkSizeX = default(10000);
        int networkSizeY = default(10000);
        @display("bgb=networkSizeX,networkSizeY");

    submodules:
        loRaNodes[numberOfNodes]: LoRaNode {
            @display("p=309,412");
        }
        loRaGW[numberOfGateways]: LoRaGW {
            @display("p=157,238");
        }
        LoRaMedium: LoRaMedium {
            @display("p=9800,26");
        }
    
    connections:
      
}

