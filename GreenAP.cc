#include <fstream>
#include <string>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/csma-module.h"
#include "ns3/olsr-helper.h"
#include "ns3/internet-module.h"
#include "ns3/netanim-module.h"
#include "ns3/energy-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/simple-device-energy-model.h"
#include "ns3/li-ion-energy-source.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <time.h>

using namespace ns3;
using namespace std;

uint32_t MacTxCount = 0;
uint32_t MacRxCount = 0;
uint32_t MacRxDropCount = 0;
double snr = 30;
    
// Define logging keyword for this file   
NS_LOG_COMPONENT_DEFINE ("GreenAP");

double energy = 0.0;
double totalRemEnergy;
double totalConsumedEnergy;

void 
RemainingEnergy (double oldValue, double remainingEnergy)
{
    ofstream outfile2;
    outfile2.open ("GreenAP_remaining_energy.txt", ios::app);
    // NS_LOG_UNCOND (Simulator::Now().GetSeconds() << "s Current remaining energy = " << remainingEnergy << "J");
    outfile2 << Simulator::Now ().GetSeconds () << "s Current remaining energy = " << remainingEnergy << "J\n";
    outfile2.close ();
    if (remainingEnergy < 0)
    {
       totalRemEnergy = 0;
    }
    else
    {
       totalRemEnergy = remainingEnergy;   
    }   
}

void 
TotalEnergy (double oldValue, double totalEnergy)
{
    totalConsumedEnergy = totalEnergy;
    ofstream outfile3;
    outfile3.open ("GreenAP_total_energy.txt", ios::app);
    // NS_LOG_UNCOND ( Simulator::Now().GetSeconds() <<"s Total energy consumed by radio = " << totalEnergy << "J");
    outfile3 << Simulator::Now ().GetSeconds () << "s Total energy consumed by radio = " << totalEnergy << "J\n";
    outfile3.close ();
    energy = energy + totalEnergy;
}

uint32_t staTable[20][20], result[20];
uint32_t backboneNodes = 6;
uint32_t infraNodes = 8;
uint32_t staNodes = 0;
uint32_t staList[20];
// uint32_t n=0;
uint32_t arr[20][20], p = 0;
uint32_t payloadSize = 1472;
double r_cur = 0, x = 1.0;

void 
FindStaTable (uint32_t i)
{    
    uint32_t k;
    staTable[i][0] = i;
    for (k = 1; k <= staNodes; k++)
    {
        staTable[i][k] = staList[k];            
    }
    staTable[i][k] = 99;	   
}

void 
AdjacentMatrix ()
{
    for (uint32_t i = 0; i < backboneNodes; i++)
    {
        if (i == 0)
	{
	    arr[i][0] = 2;
	    arr[i][1] = 0;
	    arr[i][2] = 1;
	    arr[i][3] = 3;
	    arr[i][4] = 4;
	    arr[i][5] = 99;		
	}
	else if (i == 1)
	{
	    arr[i][0] = 4;
	    arr[i][1] = 2;
	    arr[i][2] = 3;
	    arr[i][3] = 5;
	    arr[i][4] = 99;		
	}
	else if (i == 2)
	{
	    arr[i][0] = 0;
	    arr[i][1] = 1;
	    arr[i][2] = 2;
	    arr[i][3] = 99;
	}
	else if (i == 3)
	{
	    arr[i][0] = 3;
	    arr[i][1] = 2;
	    arr[i][2] = 4;
	    arr[i][3] = 5;
	    arr[i][4] = 99;		
	}
	else if (i == 4)
	{		
	    arr[i][0] = 1;
	    arr[i][1] = 0;
	    arr[i][2] = 2;
	    arr[i][3] = 99;
	}
	else
	{
	    arr[i][0] = 5;
	    arr[i][1] = 3;
	    arr[i][2] = 4;
	    arr[i][3] = 99;		
	}
    }
}	

void 
FindAp ()
{
    uint32_t m,n;
    for (uint32_t k = 0; k < backboneNodes; k++)
    {
         m = arr[k][0];
         for (uint32_t l = 1; arr[k][l] != 99; l++)
         {
	     n = arr[k][l];
	     for (uint32_t i = 1; staTable[m][i] != 99; i++)
             {
	         for (uint32_t j = 1; staTable[n][j] != 99; j++)
	         {
		     if (staTable[m][i] == staTable[n][j])
	             {
		         staTable[n][j] = 100;
		     }
	         }
             }
         }
    }
	
    // Find result matrix
    for (uint32_t i = 0; i < backboneNodes; i++)
    {
        uint32_t flag = 1;
	for (uint32_t j = 1; staTable[i][j] != 99; j++)
	{
	    if (staTable[i][j] != 100)
	    {
	        flag = 0;
	    }		
	}
        if (flag == 1)
	{
	    result[p] = i;
	    p++;
	}
	flag = 1;
    }
}

void 
DisplayResult ()
{
    for(uint32_t i = 0; i < p; i++)
    {
        std::cout<<result[i] << " ";
    }
}

static void
PrintCellInfo (Ptr<LiIonEnergySource> es)
{
    std::cout << "At " << Simulator::Now ().GetSeconds () << " Cell voltage: " << es->GetSupplyVoltage () << " V Remaining Capacity: " << es->GetRemainingEnergy () / (3.6 * 3600) << " Ah" << std::endl;

    if (!Simulator::IsFinished ())
    {
        Simulator::Schedule (Seconds (5), &PrintCellInfo, es);
    }
}

static void 
PhyTrace (Ptr<const Packet> packet, uint16_t channelFreqMhz, uint16_t channelNumber, uint32_t rate, enum WifiPreamble preamble, WifiTxVector txVector, struct mpduInfo info, struct signalNoiseDbm signalnoise)
{
  snr = signalnoise.signal - signalnoise.noise;
  //std::cout<<"SNR:"<<snr;
}

double 
RandRange (double min, double max)
{
  return min + (max - min) * rand () / ((double) RAND_MAX);
}

void
AccessProbability ()
{
    double alpha = 1.0, beta = 0.5;
    double r = alpha / ((1.0 - beta) * totalConsumedEnergy - (beta * snr));
    if (r < r_cur)
    {    
        x = x + r;
    }
    else
    {
        x = x - r;
    }
    if (x > 1.0)
    {
        x = RandRange (0, x);
    }
}

void 
DisplayMatrix ()
{
    //std::cout<<"Station Matrix"<<std::endl;
    for(uint32_t i = 0; i < backboneNodes; i++)
    {
	for(uint32_t j = 0; staTable[i][j] != 99 ; j++)
	{
		std::cout << staTable[i][j] << " ";
	}
	std::cout << std::endl;
    }
}

double rss = -45;    
uint32_t stopTime = 10;
double totaltxpacket = 0, totalrxpacket = 0;
double totalthr = 0.0;
double totaldelay = 0.0, delay_total;
double prevEnergy = 0.0;
// MCS
int mcs = 9; 
// channel bonding    
int cb = 80; 
// Guard interval (800ns)    
int k = 0;       
int totalCount = 2, count2 = 0;
double aggrThroughput = 0.0, aggrplr = 0.0, delaysum = 0.0;
double totalthr2 = 0.0, aggrFairness = 0.0, fairness;
double energy1[100], energy2[100];
// Count number of flows
uint32_t noSta = 0;   


void 
Experiment ()
{
    NodeContainer backbone;
    backbone.Create (backboneNodes);
    
    // Create the backbone wifi net devices and install them into the nodes in     
    WifiHelper wifi;
    wifi.SetStandard (WIFI_PHY_STANDARD_80211ac);
    WifiMacHelper mac;
    // Setting previous energy
    prevEnergy = totalConsumedEnergy;

    std::ostringstream oss;
    oss << "VhtMcs" << mcs;
    // Set data mode and control mode
    wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", StringValue (oss.str ()),
                                            "ControlMode", StringValue (oss.str ()));

    // Set non-unicast mode
    Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue (oss.str ()));
    Ssid ssid = Ssid ("ns3-80211ac");
    mac.SetType ("ns3::AdhocWifiMac");
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
    YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
    phy.Set ("RxGain", DoubleValue (0) ); 
    // Set guard interval
    phy.Set ("ShortGuardEnabled", BooleanValue (k));
    // Set channel bandwidth
    Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue (cb));
    channel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss ("ns3::FixedRssLossModel","Rss", DoubleValue (rss));
    // channel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel", "Exponent", DoubleValue (0.3));
    /// channel.AddPropagationLoss ("ns3::FriisPropagationLossModel");

    phy.SetChannel (channel.Create ());
    NetDeviceContainer backboneDevices = wifi.Install (phy, mac, backbone);
    
    // Add the IPv4 protocol stack to the nodes in our container
    InternetStackHelper internet;
    internet.Install (backbone);    
    Ipv4AddressHelper ipAddrs;
    ipAddrs.SetBase ("192.168.0.0", "255.255.255.0");
    Ipv4InterfaceContainer backboneInterface;
    backboneInterface = ipAddrs.Assign (backboneDevices);    
    // Mobility model
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
    positionAlloc->Add (Vector (0.0, 5.0, 0.0));
    positionAlloc->Add (Vector (3.0, 1.0, 0.0));
    positionAlloc->Add (Vector (6.0, 7.0, 0.0));
    positionAlloc->Add (Vector (10.0, 1.0, 0.0));
    positionAlloc->Add (Vector (14.0, 5.0, 0.0));
    positionAlloc->Add (Vector (20.0, 6.0, 0.0));
    // positionAlloc->Add (Vector (25.0, 2.0, 0.0));
    // positionAlloc->Add (Vector (35.0, 0.0, 0.0));
    mobility.SetPositionAllocator (positionAlloc);
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (backbone);
  
    NetDeviceContainer infraDevices;
    NodeContainer infra;     

    // Configure STAs
    for (uint32_t i = 0; i < backboneNodes; i++)
    {
        NS_LOG_INFO ("Configuring wireless network for backbone node " << i);  
          
        NodeContainer stas;
        stas.Create (infraNodes - 1);
        NodeContainer infra (backbone.Get (i), stas);
            
        WifiHelper wifiInfra;
        wifiInfra.SetStandard (WIFI_PHY_STANDARD_80211ac);
	WifiMacHelper macInfra;
	std::ostringstream oss;
        oss << "VhtMcs" << mcs;
	// set data mode and control mode
        wifiInfra.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", StringValue (oss.str ()),
                                            "ControlMode", StringValue (oss.str ()));

	// Set guard interval
        phy.Set ("ShortGuardEnabled", BooleanValue (k));
        // Set channel bandwidth
        Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue (cb));
       
        phy.SetChannel (channel.Create ());
        // Create unique ssids for these networks
        // Ssid ssid = Ssid ("ns380211a");
        std::string ssidString ("wifi-infra");
        std::stringstream ss;
        ss << i;
        ssidString += ss.str ();
        Ssid ssid = Ssid (ssidString);	 
        // Setup STAs
        macInfra.SetType ("ns3::StaWifiMac",
                           "Ssid", SsidValue (ssid),
                           "ActiveProbing", BooleanValue (false));
        NetDeviceContainer staDevices = wifi.Install (phy, macInfra, stas);
        // Setup AP
        macInfra.SetType ("ns3::ApWifiMac",
                           "Ssid", SsidValue (ssid),
                           "BeaconGeneration", BooleanValue (true),
                           "BeaconInterval", TimeValue (Seconds (2.5)));
        NetDeviceContainer apDevices = wifi.Install (phy, macInfra, backbone.Get (i));
 
        NetDeviceContainer infraDevices (apDevices, staDevices); 
        internet.Install (stas);
        Ipv4InterfaceContainer infraInterface;
        infraInterface = ipAddrs.Assign (infraDevices);

        Ptr<ListPositionAllocator> subnetAlloc = CreateObject<ListPositionAllocator> ();
        for (uint32_t j = 0; j < infra.GetN (); ++j)
        {
            subnetAlloc->Add (Vector (0.0, j + 10, 0.0));
        }
        // Mobility model for STAs
        mobility.PushReferenceMobilityModel (backbone.Get (i));
        mobility.SetPositionAllocator (subnetAlloc);
        mobility.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                                    "Bounds", RectangleValue (Rectangle (-100, 100, -100, 100)),
                                    "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=3.0]"),
                                    "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=0.4]"));
        mobility.Install (stas);
        Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
        if (i == 0)
        {
	    staNodes = 3;
	    staList[1] = 2;
            staList[2] = 6;
            staList[3] = 7;
        }
	else if (i == 1)
	{
	    staNodes = 2;
            staList[1] = 2;
            staList[2] = 6;
	}

	else if (i == 2)
	{
	    staNodes = 4;
            staList[1] = 0;
            staList[2] = 2;
	    staList[3] = 3;
            staList[4] = 5;
	}
 
	else if (i == 3)
	{
	    staNodes = 3;
            staList[1] = 0;
            staList[2] = 3;
	    staList[3] = 4;
	}
	else if (i == 4)
	{
	    staNodes = 4;
            staList[1] = 0;
            staList[2] = 1;
	    staList[3] = 3;
            staList[4] = 4;
	} 
	else 
	{
	    staNodes = 2;
            staList[1] = 1;
            staList[2] = 4;
	}    
        FindStaTable(i);	 
      }

    AdjacentMatrix();
    FindAp();
    // Energy consumption
    BasicEnergySourceHelper basicSourceHelper;
    basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (20.0));
    EnergySourceContainer sources1 = basicSourceHelper.Install (backbone);
    EnergySourceContainer sources2 = basicSourceHelper.Install (infra);
    // sources1 = basicSourceHelper.Install(wifiStaNode);
    WifiRadioEnergyModelHelper radioEnergyHelper;
    radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.0174));
    DeviceEnergyModelContainer deviceModels1 = radioEnergyHelper.Install (backboneDevices, sources1);
    DeviceEnergyModelContainer deviceModels2 = radioEnergyHelper.Install (infraDevices, sources2);
    // deviceModels1 = radioEnergyHelper.Install(staDevice, sources1); 

    // Connect trace sources 
    Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource> (sources1.Get(0));
    Ptr<BasicEnergySource> basicSourcePtr2 = DynamicCast<BasicEnergySource> (sources1.Get(1));
    // Ptr<BasicEnergySource> basicSourcePtr3 = DynamicCast<BasicEnergySource> (sources1.Get(2));
    // Ptr<BasicEnergySource> basicSourcePtr4 = DynamicCast<BasicEnergySource> (sources1.Get(3));
    // Ptr<BasicEnergySource> basicSourcePtr5 = DynamicCast<BasicEnergySource> (sources1.Get(4));
    // Ptr<BasicEnergySource> basicSourcePtr6 = DynamicCast<BasicEnergySource> (sources1.Get(5));
     
    basicSourcePtr -> TraceConnectWithoutContext("RemainingEnergy", MakeCallback (&RemainingEnergy));
    basicSourcePtr2 -> TraceConnectWithoutContext("RemainingEnergy", MakeCallback (&RemainingEnergy));
    // basicSourcePtr3 -> TraceConnectWithoutContext("RemainingEnergy", MakeCallback (&RemainingEnergy));
    // basicSourcePtr4 -> TraceConnectWithoutContext("RemainingEnergy", MakeCallback (&RemainingEnergy));
    // basicSourcePtr5 -> TraceConnectWithoutContext("RemainingEnergy", MakeCallback (&RemainingEnergy));
    // basicSourcePtr6 -> TraceConnectWithoutContext("RemainingEnergy", MakeCallback (&RemainingEnergy));
     
    Ptr<DeviceEnergyModel> basicRadioModelPtr = basicSourcePtr -> FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel").Get (0);
    Ptr<DeviceEnergyModel> basicRadioModelPtr2 = basicSourcePtr2 -> FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel").Get (0);
    NS_ASSERT (basicRadioModelPtr != NULL);
    NS_ASSERT (basicRadioModelPtr2 != NULL);
    basicRadioModelPtr -> TraceConnectWithoutContext("TotalEnergyConsumption", MakeCallback(&TotalEnergy));
    basicRadioModelPtr2 -> TraceConnectWithoutContext("TotalEnergyConsumption", MakeCallback(&TotalEnergy));
    // Calculate SNR
    Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx", MakeCallback (&PhyTrace));

    AccessProbability ();
    // Create Application  
    NS_LOG_INFO ("Create Applications.");   
    uint16_t port = 14;       
    uint32_t firstIndex;  
    port = 9;                                                          
    firstIndex = backboneNodes + 4;
    Ptr<Node> appSource2 = NodeList::GetNode (firstIndex);
    uint32_t lastNodeIndex;
    lastNodeIndex = backboneNodes;
    // uint32_t lastNodeIndex = backboneNodes + backboneNodes*(infraNodes - 1) - 1;
    Ptr<Node> appSink2 = NodeList::GetNode (lastNodeIndex);
 
    Ipv4Address remoteAddr2 = appSink2->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();  
    OnOffHelper onoff2 ("ns3::UdpSocketFactory", 
                        Address (InetSocketAddress (remoteAddr2, port)));
    // onoff.SetAttribute ("PacketSize", UintegerValue (payloadSize));
    onoff2.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
    onoff2.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
    onoff2.SetAttribute ("PacketSize", UintegerValue (payloadSize));
    onoff2.SetAttribute ("DataRate", DataRateValue (1000000000));  
    ApplicationContainer apps2 = onoff2.Install (appSource2);
    apps2.Start (Seconds (1.0));
    apps2.Stop (Seconds (stopTime + 1));  
    // Create a packet sink to receive these packets
    PacketSinkHelper sink2 ("ns3::UdpSocketFactory", 
                            InetSocketAddress (Ipv4Address::GetAny (), port));
    apps2 = sink2.Install (appSink2);
    apps2.Start (Seconds (0.0));
    apps2.Stop (Seconds (stopTime + 1));

    NS_LOG_INFO ("Configure Tracing.");
    AsciiTraceHelper ascii;
    Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream ("GreenAP.tr");
    phy.EnableAsciiAll (stream);   
    internet.EnableAsciiIpv4All (stream);
    // pcap captures on the backbone wifi devices
    phy.EnablePcap ("Mixed-wireless", backboneDevices, false);  
    Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
    AnimationInterface anim("GreenAP.xml");


    // Flow Monitor
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
   
    Ptr<Node> n1 = NodeList::GetNode (lastNodeIndex);
    // Ptr<Node> n2=backbone.Get(3);
    // Ptr<Node> n3=backbone.Get(5);
    Ptr<Ipv4> ipv1 = n1->GetObject<Ipv4> ();
    // Ptr<Ipv4> ipv2=n2->GetObject<Ipv4> ();
    // Ptr<Ipv4> ipv3=n3->GetObject<Ipv4> ();
    Simulator::Schedule (Seconds(10.0), &Ipv4::SetDown, ipv1, 0);
    // Simulator::Schedule(Seconds(20.0), &Ipv4::SetDown, ipv2,0);
    // Simulator::Schedule(Seconds(20.0), &Ipv4::SetDown, ipv3,0);
 
    NS_LOG_INFO ("Run Simulation.");
    Simulator::Stop (Seconds (stopTime + 1));
    Simulator::Run ();

    // Per flow statistics
    double plr, pl, delay_total, totalthr1;
    int lostpackets; 
    monitor->CheckForLostPackets ();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator p = stats.begin (); p != stats.end (); ++p)
    {
        noSta++; 	 
        totaltxpacket = totaltxpacket + p->second.txPackets;
        totalrxpacket = totalrxpacket + p->second.rxPackets;        
        totaldelay += p->second.delaySum.GetSeconds ();                  
        totalthr += p->second.rxPackets * payloadSize * 8.0 / ((stopTime) * 1000000.0);
	totalthr2 += totalthr * totalthr; 	            
    }
    if (totalrxpacket != 0)
    {
	delay_total = ((double) totaldelay / (double) totalrxpacket) * 1000;
    } 
     
    Simulator::Destroy ();
    // Count lost packets
    lostpackets = totaltxpacket - totalrxpacket;             
    if (totaltxpacket != 0)
    {      
        pl = (long double) (lostpackets / totaltxpacket);
   	plr = pl * 100;
    }   

    plr = plr / noSta;
    delay_total = delay_total / noSta;
    totalthr1 = totalthr / noSta; 
    // Calculate aggregate throughput
    aggrThroughput = aggrThroughput + totalthr1;
    // Calculate aggregate PLR
    aggrplr = aggrplr + plr;  
    // Calculate aggregate delay
    delaysum = delaysum + delay_total;
    // Calculate fairness index
    fairness = (totalthr * totalthr) / (noSta * totalthr2);
    aggrFairness = aggrFairness + fairness;
	
    if (totalConsumedEnergy <= prevEnergy)
    {
        if (mcs < 9)
	{
	    mcs = mcs + 1;
	}
	if (cb < 160)
	{
	    cb = cb * 2;
	}
    }
    else
    {
        if (mcs > 0)
	{
	    mcs = mcs - 1;
	}
	if (cb > 20)
	{
	    cb = cb / 2;
	    if (cb < 20)
	    {
	        cb = 20;
	    }
	}
    }
    energy1 [count2] = totalConsumedEnergy;
    energy2 [count2] = totalRemEnergy;
    count2 ++;
}


int 
main (int argc, char *argv[])
{
    double avgThroughput, avgplr, avgdelay, avgFairness;
    int q = 1;
    // Get system time
    time_t ctime=time(0);
    uint32_t t=static_cast<uint32_t>(ctime);
    // Set the seed value
    RngSeedManager::SetSeed(t);
    CommandLine cmd; 
    cmd.AddValue ("stopTime", "Simulation time in seconds", stopTime);
    cmd.AddValue ("backboneNodes", "Backbone nodes", backboneNodes);
    cmd.AddValue ("totalCount", "Counter for algorithm", totalCount);
    cmd.AddValue ("infraNodes", "Number of STA nodes", infraNodes);
    // Execute the algorithm
    while (q <= totalCount)
    {       
        Experiment ();
        q++;
    }
    // Calculate average throughput
    avgThroughput = aggrThroughput / totalCount; 
    // Calculate average PLR
    avgplr = aggrplr / totalCount; 
    // Calculate average delay
    avgdelay = delaysum / totalCount;
    // Calculate fairness index
    avgFairness = aggrFairness / totalCount;
    double consumedEnergySum = 0.0, avgConsumedEnergy;
    double remEnergySum = 0.0, avgRemEnergy;
    for (int p = 0; p < count2; p++)
    {
        consumedEnergySum = consumedEnergySum + energy1 [p];
        remEnergySum = remEnergySum + energy2 [p];
        // std::cout<<"Total consumed energy = "<<energy1[p]<<"  "<<"Remaining energy = "<<energy2[p]<<std::endl;
    }
    // avgConsumedEnergy = consumedEnergySum / (double) totalCount;
    avgRemEnergy = remEnergySum / (double) totalCount;
    avgConsumedEnergy = (20.0 - avgRemEnergy) * noSta;
    std::cout<<avgConsumedEnergy<<"    "<<(avgRemEnergy * noSta)<<std::endl;
    std::cout<<"\t\t"<<avgThroughput<<"\t\t"<<avgplr<<"\t\t"<<avgdelay<<std::endl;
    std::cout<<avgFairness<<std::endl;
    return 0;
}


