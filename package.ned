
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
network LoRaWANNetwork {
    submodules:
       // loraWAN: LoRaWANNetwork;
        node[1]: LoRaNode;
        gateway[1]: LoRaGW;
}