/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 *    
 * This source file is modified from wifi-tcp.cc from the examples.
 * 
 * This is the source file to develop, test, and implement Wi-Fi PSM
 * 
 * Check journalPSM.md for details
 * 
 * To run - example:
 *  ./ns3 --run "PSM_MU --simulationTime=5 --enablePSM_flag=true --enableTcpUplink=true --dataPeriod=0.33 --otherStaCount=10 --loopIndex=123321 --dataRatebps_other=31000 --enableMulticast=true --multicastInterval_ms=200" >>trial123.out 
 */


#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/mobility-model.h"
#include "ns3/packet-sink.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/tcp-westwood.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/applications-module.h"
//changes below
#include "ns3/netanim-module.h"
#include "ns3/wifi-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/v4ping-helper.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-mac.h"
#include <iomanip>
#include "ns3/rng-seed-manager.h"

//-*********************************
#define LOGNAME_PREFIX "WiFiPSM_MU"
#define FOLDER_PATH "MU_logs/"

NS_LOG_COMPONENT_DEFINE ("WiFiPSM");

using namespace ns3;

// ---------------------------------------------------------------------------

std::string currentLoopIndex_string = "123213";     // This will be updated with input from cmd line and used when creating log files. 

// ---------------------------------------------------------------------------

Ptr<PacketSink> sink;                         /* Pointer to the packet sink application */
uint64_t lastTotalRx = 0;                     /* The value of the last total received bytes */
uint32_t delACKTimer_ms = 0;                     /* TCP delayed ACK timer in ms   */ 

//Configurable parameters************************
double simulationTime = 20;                        /* Simulation time in seconds. */
bool enablePSM_flag = false;

uint32_t P2PLinkDelay_ms = 10;                  // Set this to be half of the expected RTT

bool enableTcpUplink = true;
uint32_t payloadSize = 1024;                       /* Transport layer payload size in bytes. */
double dataPeriod = 100;                       /* Application data period in seconds. */


// uint32_t dataRatebps_other = 1 * payloadSize*8/dataPeriod;  
  // Multicast traffic parameters
bool enableMulticast =false;
uint32_t multicastPacketSizeBytes = 1024;
uint32_t multicastInterval_ms = 20000;






//-**************************************************************************
// Parse context strings of the form "/NodeList/x/DeviceList/x/..." to extract the NodeId integer
uint32_t
ContextToNodeId (std::string context)
{
  std::string sub = context.substr (10);
  uint32_t pos = sub.find ("/Device");
  return atoi (sub.substr (0, pos).c_str ());
}

//-**************************************************************************
// Send broadcast packet from specific STA MAC

void
SendBroadcast (Ptr<StaWifiMac> broadcastStaMac)
{
  Time multicastPacketPeriod = MilliSeconds (multicastInterval_ms);
  Ptr<Packet> pkt = Create<Packet> (multicastPacketSizeBytes);
  Mac48Address addr1 ("ff:ff:ff:ff:ff:ff");

  broadcastStaMac->Enqueue(pkt,addr1);
  Simulator::Schedule (multicastPacketPeriod, &SendBroadcast, broadcastStaMac);
}



//-*******************************************************************
void
CalculateThroughput ()
{
  // This function has been modified to output the data received every second in Bytes
  Time now = Simulator::Now ();                                         /* Return the simulator's virtual time. */
  double currentDataBytes = (sink->GetTotalRx () - lastTotalRx);
  //double cur = (sink->GetTotalRx () - lastTotalRx) * (double) 8 / 1e5;     /* Convert Application RX Packets to MBits. */
  //std::cout << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;
  std::cout << now.GetSeconds () << "s: Data received in previous second in Bytes\t" << currentDataBytes << " Bytes" << std::endl;
  lastTotalRx = sink->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (1000), &CalculateThroughput);
}



//-**************************************************************************
// PHY state tracing - check log file
template <int node>
void PhyStateTrace (std::string context, Time start, Time duration, WifiPhyState state)
{
  std::stringstream ss;
  ss <<FOLDER_PATH<< "log"<<currentLoopIndex_string<<".statelog";

  static std::fstream f (ss.str ().c_str (), std::ios::out);

  // f << Simulator::Now ().GetSeconds () << "    state=" << state << " start=" << start << " duration=" << duration << std::endl;
  // Do not use spaces. Use '='  and ';'
  f << "state=" <<state<< ";startTime_ns="<<start.GetNanoSeconds()<<";duration_ns=" << duration.GetNanoSeconds()<<";nextStateShouldStartAt_ns="<<(start + duration).GetNanoSeconds()<<";"<< std::endl;
}

//-*************************************************************************

//-**************************************************************************
// PHY Tx state tracing - check log file
template <int node>
void PhyTxStateTrace (std::string context, Time start, Time duration, WifiPhyState state)
{
  std::stringstream ss;
  ss <<FOLDER_PATH<<"log"<<currentLoopIndex_string<<"txnode"<<std::setfill('0') <<std::setw(2)<<node <<".txlog";

  static std::fstream f (ss.str ().c_str (), std::ios::out);

  // f << Simulator::Now ().GetSeconds () << "    state=" << state << " start=" << start << " duration=" << duration << std::endl;
  // Do not use spaces. Use '='  and ';'
  if (state == WifiPhyState::TX)
  {
    f << "node="<<node <<";startTime_ns="<<start.GetNanoSeconds()<<";duration_ns=" << duration.GetNanoSeconds()<< std::endl;
  }
  
}

//-*************************************************************************
//-**************************************************************************
// PHY Tx state tracing - check log file
void PhyTxStateTraceAll (std::string context, Time start, Time duration, WifiPhyState state)
{
  std::stringstream ss;
  ss <<FOLDER_PATH<<"log"<<currentLoopIndex_string<<".txlog";

  static std::fstream f (ss.str ().c_str (), std::ios::out);

  // f << Simulator::Now ().GetSeconds () << "    state=" << state << " start=" << start << " duration=" << duration << std::endl;
  // Do not use spaces. Use '='  and ';'
  if (state == WifiPhyState::TX)
  {
    f<<"startTime_ns="<<start.GetNanoSeconds()<<";duration_ns=" << duration.GetNanoSeconds()<< std::endl;
  }
  
}

//-*************************************************************************

//-*************************************************************************
void
AckedMpduTrace (std::string context, Ptr<const WifiMacQueueItem> qItem)
{
  std::stringstream ss;
  ss <<FOLDER_PATH<<"log"<<currentLoopIndex_string<<".ackedMpdu";

  static std::fstream f (ss.str ().c_str (), std::ios::out);
  std::stringstream tempQueueStream, packetStream;
  qItem->Print(tempQueueStream);
  qItem->GetPacket()->Print(packetStream);
  
  // std::cout<<"time="<<Simulator::Now().GetNanoSeconds()<<";NodeId:=" << ContextToNodeId (context)<<";queueItem:"<<tempQueueStream.str()<<";packet:"<<packetStream.str()<< std::endl;
  f<<"time="<<Simulator::Now().GetNanoSeconds()<<";NodeId=" << ContextToNodeId (context)<<";queueItem="<<tempQueueStream.str()<<";packet:"<<packetStream.str()<< std::endl;
  
  

}
//-*************************************************************************
//-*************************************************************************
void
MacTxTrace (std::string context, Ptr<const Packet> p)
{
  std::stringstream ss;
  ss <<FOLDER_PATH<<"log"<<currentLoopIndex_string<<".macTx";

  static std::fstream f (ss.str ().c_str (), std::ios::out);
  std::stringstream tempStream;
  
  p->Print(tempStream) ;
  
  f<<"time="<<Simulator::Now().GetNanoSeconds()<<";NodeId=" << ContextToNodeId (context)<<";packetId="<<p<<";packet:"<<tempStream.str()<< std::endl;
  // std::cout<<"time="<<Simulator::Now().GetNanoSeconds()<<";NodeId:=" << ContextToNodeId (context)<<"packetId:"<<p<<";packet:"<<tempStream.str()<< std::endl;
  

}
//-*************************************************************************

//-**************************************************************************
// PHY state tracing - print as logs to output

void PhyStateTracePrint (std::string context, Time start, Time duration, WifiPhyState state)
{
  NS_LOG(LOG_INFO, Simulator::Now ().GetSeconds () << "    state=" << state << " start=" << start << " duration=" << duration << std::endl);
}

//-*************************************************************************


//-*******************************************************************
void
putToSleep (Ptr<WifiPhy> phy)
{
  // This function puts the specific PHY to sleep
  phy->SetSleepMode();
}
//-*******************************************************************



//-*******************************************************************
void
wakeFromSleep (Ptr<WifiPhy> phy)
{
  // This function wakes the specific PHY from sleep
  phy->ResumeFromSleep();
}
//-*******************************************************************


//-*******************************************************************
void
changeStaPSM (Ptr<StaWifiMac> staMac, bool PSMenable)
{
  // This function puts the specific PHY to sleep
  staMac->SetPowerSaveMode(PSMenable);
}
//-*******************************************************************

//-*******************************************************************
void
printStaPSM (Ptr<StaWifiMac> staMac)
{
  // This function puts the specific PHY to sleep
  NS_LOG_INFO("STA MAC "<<staMac<<" is in PS = "<<staMac->GetPowerSaveMode());
}
//-*******************************************************************


//-*******************************************************************
void
changePosition (Ptr <Node> staWifiNode, double StaDistance)
{
    /* Mobility model */
  MobilityHelper mobility2;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (StaDistance, 0.0, 0.0));

  mobility2.SetPositionAllocator (positionAlloc);
  mobility2.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility2.Install (staWifiNode);
  
  NS_LOG_INFO("STA position changed now to be " << StaDistance << " meter(s) from the AP.");
}
//-*******************************************************************




int
main (int argc, char *argv[])
{

  

  std::string tcpVariant = "TcpNewReno";             /* TCP variant type. */
  std::string phyRate = "HtMcs7";                    /* Physical layer bitrate. */
  std::string phyRate_AP_Data = "HtMcs7";
  // std::string phyRate_AP_Data = "HtMcs5";
  std::string phyRate_STA_control = "HtMcs3";  
  std::string phyRate_AP_control = "HtMcs3";  

  bool pcapTracing = false;                          /* PCAP Tracing is enabled or not. */


  uint32_t staMaxMissedBeacon = 10;                 // Set the max missed beacons for STA before attempt for reassociation
  uint8_t apDtimPeriod = 3;                         // Set the AP DTIM period here. Note: AP is node 0. WiFi interface is at 1.
  // Time AdvanceWakeupPS = MicroSeconds (150);     // Used in all simulations
  // Time AdvanceWakeupPS = MicroSeconds (400);
  Time AdvanceWakeupPS = MicroSeconds (0);
  // Time AdvanceWakeupPS = MicroSeconds (200);
  uint32_t loopIndex = 0;

  // Setting up other STA uplink UDP traffic

  uint32_t otherStaCount = 10;  
  uint32_t dataRatebps_other = 1000000;  
  // std::string dataRate_other = std::to_string(dataRatebps_other) + std::string("bps");
  uint32_t udpUplinkPacketSizeBytes = 1024;
  uint32_t udpUplinkPacketSizeBits = udpUplinkPacketSizeBytes * 8;
  uint32_t maxUdpPacketCount = 4294967295;      // max for this attribute 
  
  
  // Random seed
  uint32_t randSeed = 1;     




  /* Command line argument parser setup. */ 
  CommandLine cmd (__FILE__);
  cmd.AddValue ("simulationTime", "Simulation duration in seconds", simulationTime);
  cmd.AddValue ("enableTcpUplink", "Enable/Disable uplink TCP from PS STA. Set to true/false", enableTcpUplink);
  cmd.AddValue ("enablePSM_flag", "Enable/Disable PSM for the PS STA. Set to true/false", enablePSM_flag);
  cmd.AddValue ("dataPeriod", "Uplink TCP data rate from PS STA. Double value. Used only if enableTcpUplink is true.", dataPeriod);
  cmd.AddValue ("otherStaCount", "Number of other non-PS STAs. Integer between 0 and 100", otherStaCount);
  cmd.AddValue ("loopIndex", "The index of current interation. Integer between 0 and 999999", loopIndex);
  cmd.AddValue ("dataRatebps_other", "Uplink datarate of other STAs in bps. >0.", dataRatebps_other);
  cmd.AddValue ("enableMulticast", "true/false to enable/disable multicast DL traffic", enableMulticast);
  cmd.AddValue ("multicastInterval_ms", "Multicast packet interval in ms - integer value - use >= 100.", multicastInterval_ms);
  cmd.AddValue ("randSeed", "Random seed to initialize position and app start times - unit32", randSeed);
  cmd.Parse (argc, argv);

  // Calculate tcp UL data rate
  uint32_t dataRatebps = 1 *  payloadSize*8/dataPeriod;  
  std::string dataRate = std::to_string(dataRatebps) + std::string("bps");
  // Creating index string to add to log file names
  
  std::stringstream indexStringTemp;
  indexStringTemp << std::setfill('0') << std::setw(6) << loopIndex;
  currentLoopIndex_string = indexStringTemp.str();

  std::cout<<"\nLoop index is "<<currentLoopIndex_string;
  std::cout<<"\nsimulationTime is "<<simulationTime;
  std::cout<<"\nmulticastInterval_ms is "<<multicastInterval_ms;
  std::cout<<"\nenableTcpUplink is "<<enableTcpUplink;
  std::cout<<"\nenableMulticast is "<<enableMulticast;
  std::cout<<"\nTCP dataPeriod (seconds) is "<<dataPeriod;
  std::cout<<"\nUDP dataRatebps_other bps is "<<dataRatebps_other;
  // Other UDP traffic
  
  Time interPacketIntervalUdp = Seconds (double (udpUplinkPacketSizeBits)/double(dataRatebps_other));
  std::cout<<"\nConfigured Udp uplink interPacketInterval: "<<interPacketIntervalUdp.GetMilliSeconds()<<" ms";


  // Random var setup
  RngSeedManager::SetSeed(randSeed);
  

  // App start time setup
  Ptr<UniformRandomVariable> ClientAppRand = CreateObject<UniformRandomVariable> ();
  ClientAppRand->SetAttribute ("Min", DoubleValue (0.0));
  ClientAppRand->SetAttribute ("Max", DoubleValue (1.0));

  // Room dimension in meters - creating uniform random number generator
  double minX = -25.0;
  double maxX = 25.0;
  double minY = -25.0;
  double maxY = 25.0;

  // Other STA positions
  double currentX, currentY;  
  Ptr<UniformRandomVariable> xCoordinateRand = CreateObject<UniformRandomVariable> ();
  Ptr<UniformRandomVariable> yCoordinateRand = CreateObject<UniformRandomVariable> ();

  xCoordinateRand->SetAttribute ("Min", DoubleValue (minX));
  xCoordinateRand->SetAttribute ("Max", DoubleValue (maxX));
  yCoordinateRand->SetAttribute ("Min", DoubleValue (minY));
  yCoordinateRand->SetAttribute ("Max", DoubleValue (maxY));


  // LogComponentEnable ("Config", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("WifiHelper", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("StaWifiMac", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("WifiMacQueue", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("WifiRemoteStationManager", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_DEBUG));     // To monitor state changes at AP
  // LogComponentEnable ("WifiPhy", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("ApWifiMac", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("FrameExchangeManager", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("PhyEntity", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("WifiPhyStateHelper", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  

  //tcpVariant = std::string ("ns3::") + tcpVariant;
  tcpVariant = std::string ("ns3::TcpNewReno");
  Config::SetDefault ("ns3::TcpL4Protocol::SocketType", TypeIdValue (TypeId::LookupByName (tcpVariant)));


  /* Configure TCP Options */
  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (payloadSize));
  Config::SetDefault ("ns3::TcpSocket::DelAckTimeout", TimeValue (MilliSeconds (delACKTimer_ms)));



  WifiMacHelper wifiMac;
  WifiHelper wifiHelper, wifiHelper_AP;
  //wifiHelper.SetStandard (WIFI_PHY_STANDARD_80211n_5GHZ);
  wifiHelper.SetStandard (WIFI_STANDARD_80211n_2_4GHZ);
  wifiHelper_AP.SetStandard (WIFI_STANDARD_80211n_2_4GHZ);

  /* Set up Legacy Channel */
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  //wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel", "Frequency", DoubleValue (5e9));
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel", "Frequency", DoubleValue (2.4e9));

  /* Setup Physical Layer */
  YansWifiPhyHelper wifiPhy;
  wifiPhy.SetChannel (wifiChannel.Create ());
  wifiPhy.SetErrorRateModel ("ns3::YansErrorRateModel");


  wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode", StringValue (phyRate),
                                      "ControlMode", StringValue (phyRate_STA_control),
                                      "NonUnicastMode", StringValue (phyRate));

  wifiHelper_AP.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
					                              "DataMode", StringValue (phyRate_AP_Data),
					                              "ControlMode", StringValue (phyRate_AP_control),
                                        "NonUnicastMode", StringValue (phyRate_AP_Data));
  // wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
  //                                     "DataMode", StringValue (phyRate),
  //                                     "ControlMode", StringValue (phyRate_STA_control),
  //                                     "NonUnicastMode", StringValue ("DsssRate1Mbps"));

  // wifiHelper_AP.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
	// 				                              "DataMode", StringValue (phyRate_AP_Data),
	// 				                              "ControlMode", StringValue ("DsssRate1Mbps"),
  //                                       "NonUnicastMode", StringValue ("DsssRate1Mbps"));

  NodeContainer wifiNodes;
  wifiNodes.Create (2);     // First node = STA, second = PSM STA
  Ptr<Node> apWifiNode = wifiNodes.Get (0);
  Ptr<Node> staWifiNode = wifiNodes.Get (1);

  NodeContainer otherStaNodes;
  if (otherStaCount > 0)
  {
    otherStaNodes.Create (otherStaCount);
  }
  

  NodeContainer serverNodes;                // This will be connected to AP by P2P links
  serverNodes.Create (1);     
  Ptr<Node> TCPServerNode = serverNodes.Get (0);

 


  // Setup P2P nodes

  NodeContainer P2PNodes;
  P2PNodes.Add(apWifiNode);
  P2PNodes.Add(TCPServerNode);

  // Printing Node IDs
  // forPrinting 
  std::cout<<"\nNode IDs:\n";
  std::cout<<"\tP2P node0: "<<unsigned(P2PNodes.Get(0)->GetId())<<std::endl;
  std::cout<<"\tP2P node1 (TCP server): "<<unsigned(P2PNodes.Get(1)->GetId())<<std::endl;
  std::cout<<"\tAP: "<<unsigned(apWifiNode->GetId()) <<std::endl;
  std::cout<<"\tPSM STA is: "<<unsigned(staWifiNode->GetId()) <<std::endl;
  for (uint32_t ii = 0; ii < otherStaCount; ii++)
  {
    std::cout<<"\tOther STA"<<ii<<": "<<otherStaNodes.Get(ii)->GetId()<<std::endl;
  }


  PointToPointHelper pointToPoint;
  std::stringstream delayString;
  delayString<<P2PLinkDelay_ms <<"ms";
  pointToPoint.SetDeviceAttribute ("DataRate", StringValue ("1000Mbps"));
  pointToPoint.SetChannelAttribute ("Delay", StringValue (delayString.str()));

  NetDeviceContainer P2Pdevices;
  P2Pdevices = pointToPoint.Install (P2PNodes);
  /* Configure AP */
  Ssid ssid = Ssid ("network");
  wifiMac.SetType ("ns3::ApWifiMac",
                   "Ssid", SsidValue (ssid));

  NetDeviceContainer apWiFiDevice;
  apWiFiDevice = wifiHelper_AP.Install (wifiPhy, wifiMac, apWifiNode);

  // ---------------- To change DTIM and disable beacon jitter of AP
  // Config path is "/NodeList/0/DeviceList/1/$ns3::WifiNetDevice/Mac/$ns3::ApWifiMac/DtimPeriod"
  Config::Set ("/NodeList/0/DeviceList/1/$ns3::WifiNetDevice/Mac/$ns3::ApWifiMac/DtimPeriod", UintegerValue(apDtimPeriod));

  // ---------------- To change max queue size and delay of buffer for PS STAs at AP - does not work - note - shyam
  Config::Set ("/NodeList/0/DeviceList/1/$ns3::WifiNetDevice/Mac/$ns3::ApWifiMac/PsUnicastBufferSize", QueueSizeValue(QueueSize ("678p")));
  Config::Set ("/NodeList/0/DeviceList/1/$ns3::WifiNetDevice/Mac/$ns3::ApWifiMac/PsUnicastBufferDropDelay", TimeValue (MilliSeconds (1123)));



  // Adding basic modes to AP - use this to change beacon data rates

 // Ptr<WifiRemoteStationManager> apStationManager = DynamicCast<WifiNetDevice>(apWiFiDevice.Get (0))->GetRemoteStationManager ();
  //apStationManager->AddBasicMode (WifiMode ("ErpOfdmRate6Mbps"));
  //DynamicCast<WifiNetDevice>(apWiFiDevice.Get (0))->GetRemoteStationManager ()->AddBasicMode (WifiMode ("ErpOfdmRate6Mbps"));

  /* Configure STA */
  // ssid = Ssid ("network1");     //Configuring with wrong SSID
  wifiMac.SetType ("ns3::StaWifiMac",
                   "Ssid", SsidValue (ssid));

  NetDeviceContainer staWiFiDevice;
  staWiFiDevice = wifiHelper.Install (wifiPhy, wifiMac, staWifiNode);

  NetDeviceContainer otherStaWiFiDevices;
  if (otherStaCount > 0)
  {
    otherStaWiFiDevices = wifiHelper.Install (wifiPhy, wifiMac, otherStaNodes);
  }
  
  // RngSeedManager::SetSeed (1);
  // RngSeedManager::SetRun (1);
  // int64_t streamNumber = 1;
  // streamNumber += wifiHelper.AssignStreams (apWiFiDevice, streamNumber);
  // streamNumber += wifiHelper.AssignStreams (staWiFiDevice, streamNumber);
  // streamNumber += wifiHelper.AssignStreams (otherStaWiFiDevices, streamNumber);



  // Accessing the WifiPhy object and enabling sleep
  // std::cout<<"Number of devices:"<<staWiFiDevice.GetN()<<std::endl;

  // std::cout<<"STA Device:"<<staWiFiDevice.Get(0)<<std::endl;   // This returns a pointer to the NetDevice and not WifiNetDevice - does not work for all functions 

  Ptr<WifiNetDevice> device = staWiFiDevice.Get(0)->GetObject<WifiNetDevice> ();    //This returns the pointer to the object - works for all functions from WifiNetDevice
  
  Ptr<WifiMac> staMacTemp = device->GetMac ();
  Ptr<StaWifiMac> staMac = DynamicCast<StaWifiMac> (staMacTemp);



// Enable / disable PSM and sleep state using MAC attribute change - through a function - not directly changing the MAC attribute 
  if (enablePSM_flag)
  {
    // Simulator::Schedule (Seconds (0.7), &changeStaPSM, staMac, true);
    // Simulator::Schedule (Seconds (1.7), &changeStaPSM, staMac, true);
    Simulator::Schedule (Seconds (6.8), &changeStaPSM, staMac, true);
  }
  
  // Simulator::Schedule (Seconds (2.1), &printStaPSM, staMac);
  // Simulator::Schedule (Seconds (2.1), &changeStaPSM, staMac, true);
  // Simulator::Schedule (Seconds (2.6), &printStaPSM, staMac);
  // Simulator::Schedule (Seconds (4.5), &changeStaPSM, staMac, false);
  // Simulator::Schedule (Seconds (3.1), &printStaPSM, staMac);
  // Simulator::Schedule (Seconds (6.0), &changeStaPSM, staMac, true);


  // IFS durations
  Ptr<WifiPhy> phy = device->GetPhy ();
  Time sifs = phy->GetSifs();    
  Time pifs = phy->GetPifs();    
  Time slot = phy->GetSlot();    
  Time difs = 2 * slot + sifs;     
  std::cout<<"\nSlot and IFS durations:\n";
  std::cout<<"\nslot (us) = "<<slot.GetMicroSeconds()<<"\nPIFS (us) = "<<pifs.GetMicroSeconds();
  std::cout<<"\nSIFS (us) = "<<sifs.GetMicroSeconds()<<"\nDIFS (us) = "<<difs.GetMicroSeconds()<<"\n\n";
  
  // phy->GetPifs();
  
  // ------------ Sleep and wake up - manually on PHY
  // Simulator::Schedule (Seconds (2.0), &putToSleep, phy);
  // Simulator::Schedule (Seconds (3.0), &wakeFromSleep, phy);

  // ---------------- To test STA behavior when beacons are missed, position is changed
  // Simulator::Schedule (Seconds (5.0), &changePosition, staWifiNode, double (1000.0));
  // Simulator::Schedule (Seconds (7.1), &changePosition, staWifiNode, double (10.0));
  


// Changing attributes for STA
Config::Set ("/NodeList/1/DeviceList/0/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::StaWifiMac/MaxMissedBeacons", UintegerValue(staMaxMissedBeacon));
Config::Set ("/NodeList/1/DeviceList/0/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::StaWifiMac/AdvanceWakeupPS", TimeValue(AdvanceWakeupPS));

  
// To enable short GI for all nodes--------------------------
// Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HtConfiguration/ShortGuardIntervalSupported", BooleanValue (true));
//---------------------------------


  /* Mobility model */
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (10.0, 0.0, 0.0));
  std::cout<<"Positions:\n";
  for (uint32_t ii = 0; ii <otherStaCount ; ii++)
  {
    currentX = xCoordinateRand->GetValue ();
    currentY = yCoordinateRand->GetValue ();
    std::cout<<"\totherSTA "<<ii<<" : ["<< currentX<<", "<<currentY <<", 0.0 ];\n";
    positionAlloc->Add (Vector (currentX, currentY, 0.0));
  }
  
  
  positionAlloc->Add (Vector (-100.0, 0.0, 0.0));

  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (apWifiNode);
  mobility.Install (staWifiNode);
  if (otherStaCount > 0)
  {
    mobility.Install (otherStaNodes);
  }
  
  mobility.Install(TCPServerNode);


  
  /* Internet stack */
  InternetStackHelper stack;
  stack.Install (wifiNodes);
  stack.Install (serverNodes);
  if (otherStaCount > 0)
  {
    stack.Install (otherStaNodes);
  }
  

  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.255.255.0");
  Ipv4InterfaceContainer apInterface;
  apInterface = address.Assign (apWiFiDevice);
  Ipv4InterfaceContainer staInterface;
  staInterface = address.Assign (staWiFiDevice);

  Ipv4InterfaceContainer otherStaInterfaces;
  if (otherStaCount > 0)
  {
    otherStaInterfaces = address.Assign (otherStaWiFiDevices);
  }
  
  

  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer P2PInterfaces;
  P2PInterfaces = address.Assign (P2Pdevices);

  // // Printing MAC Addresses to console
  // std::cout<<"MAC Addresses:\n";
  // std::cout<<"\tP2P device 0: "<<P2Pdevices.Get(0)->GetAddress()<<std::endl;
  // std::cout<<"\tP2P device 1: "<<P2Pdevices.Get(1)->GetAddress()<<std::endl;
  // std::cout<<"\tAP: "<<apWiFiDevice.Get(0)->GetAddress()<<std::endl;
  // std::cout<<"\tPSM STA: "<<staWiFiDevice.Get(0)->GetAddress()<<std::endl;
  // for (uint32_t ii = 0; ii < otherStaCount; ii++)
  // {
  //   std::cout<<"\tOther STA"<<ii<<": "<<otherStaWiFiDevices.Get(ii)->GetAddress()<<std::endl;
  // }

  // // Printing IP Addresses to console
  // std::cout<<"IP Addresses:\n";
  // std::cout<<"\tP2P device 0: "<<P2PInterfaces.GetAddress(0)<<std::endl;
  // std::cout<<"\tP2P device 1: "<<P2PInterfaces.GetAddress(1)<<std::endl;
  // std::cout<<"\tAP: "<<apInterface.GetAddress(0)<<std::endl;
  // std::cout<<"\tPSM STA: "<<staInterface.GetAddress(0)<<std::endl;
  // for (uint32_t ii = 0; ii < otherStaCount; ii++)
  // {
  //   std::cout<<"\tOther STA"<<ii<<": "<<otherStaInterfaces.GetAddress(ii)<<std::endl;
  // }

  /* Populate routing table */
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();



  /* Install TCP Receiver on the P2P TCP server */
  PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), 9));
  ApplicationContainer sinkApp = sinkHelper.Install (TCPServerNode);
  sink = StaticCast<PacketSink> (sinkApp.Get (0));
  sinkApp.Start (Seconds (0.0));

  /* Install TCP/UDP Transmitter on the station */ //Uncomment to introduce TCP traffic
  if (enableTcpUplink)
  {
    OnOffHelper server ("ns3::TcpSocketFactory", (InetSocketAddress (P2PInterfaces.GetAddress (1), 9)));
    server.SetAttribute ("PacketSize", UintegerValue (payloadSize));
    server.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
    server.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
    server.SetAttribute ("DataRate", DataRateValue (DataRate (dataRate)));
    std::cout<<"\nPSM STA Data rate is "<<dataRate<<std::endl;
    ApplicationContainer serverApp = server.Install (staWifiNode);

    // serverApp.Start (Seconds (7.04));   //Uncomment to introduce TCP traffic
    serverApp.Start (Seconds (7.0 + ClientAppRand->GetValue()));   //Uncomment to introduce TCP traffic
    // serverApp.Start (Seconds (0.85));   //Uncomment to introduce TCP traffic
  }


  // Install UDP uplink Transmitter other WiFi stations - they communicate with AP only.

  ApplicationContainer otherUdpClientApps;
  ApplicationContainer UdpServerAppsAP;
  ApplicationContainer otherServerApps;
  for (uint32_t ii = 0; ii < otherStaCount; ii++)
  {

    UdpServerHelper server ((100 + ii));
    ApplicationContainer tempUdpServer = server.Install (apWifiNode);
    tempUdpServer.Start(Seconds(0.1));
    UdpServerAppsAP.Add(tempUdpServer);
    

    UdpClientHelper tempClient (apInterface.GetAddress (0), (100 + ii));
    tempClient.SetAttribute ("MaxPackets", UintegerValue (maxUdpPacketCount));
    tempClient.SetAttribute ("Interval", TimeValue (interPacketIntervalUdp));
    tempClient.SetAttribute ("PacketSize", UintegerValue (udpUplinkPacketSizeBytes));
    ApplicationContainer tempUdpClient = tempClient.Install (otherStaNodes.Get(ii));
    // Time tempStartTime = Seconds (8 + double(2 * ii)/100);
    Time tempStartTime = Seconds (8 + ClientAppRand->GetValue());
    tempUdpClient.Start (tempStartTime);
    
    
    std::cout<<"Client in otherSta "<<ii<<" starts at "<<tempStartTime.GetSeconds()<<" seconds\n";
    otherUdpClientApps.Add(tempUdpClient);
    
  

  }
  

  // Scheduling broadcast packets from first of other STA
  if ((otherStaCount > 0) && enableMulticast)
  {
    
    Ptr<WifiNetDevice> otherDevice = otherStaWiFiDevices.Get(0)->GetObject<WifiNetDevice> ();    //This returns the pointer to the object - works for all functions from WifiNetDevice
    Ptr<WifiMac> staMacOtherTemp = otherDevice->GetMac ();
    Ptr<StaWifiMac> staMacOther = DynamicCast<StaWifiMac> (staMacOtherTemp);
    Simulator::Schedule (Seconds (0.5), &SendBroadcast, staMacOther);
    
  }
  
  




  Simulator::Schedule (Seconds (1.0), &CalculateThroughput);

  /* Enable Traces */
  if (pcapTracing)
    {
      std::stringstream ss1, ss2, ss4;
      wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
      ss1<<FOLDER_PATH<< LOGNAME_PREFIX <<"_AP";
      wifiPhy.EnablePcap (ss1.str(), apWiFiDevice);
      ss2<<FOLDER_PATH<< LOGNAME_PREFIX <<"_STA";
      wifiPhy.EnablePcap (ss2.str(), staWiFiDevice);
      // ss3<<FOLDER_PATH<< LOGNAME_PREFIX <<"_otherSTA";
      // wifiPhy.EnablePcap (ss3.str(), otherStaWiFiDevices);
      ss4<<FOLDER_PATH<< LOGNAME_PREFIX <<"_P2P";
      pointToPoint.EnablePcapAll (ss4.str());
    }


  // For state tracing - to log file
  // Config::Connect ("/NodeList/1/DeviceList/*/Phy/State/State", MakeCallback (&PhyStateTrace<1>));
  // Config::Connect ("/NodeList/0/DeviceList/1/Phy/State/State", MakeCallback (&PhyStateTrace<0>));
  // Config::Connect ("/NodeList/3/DeviceList/0/Phy/State/State", MakeCallback (&PhyStateTrace<3>));

  // Mac queue tracing for PS STA
  // Config::Connect ("/NodeList/1/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx", MakeCallback (&MacTxTrace));
  // Config::Connect ("/NodeList/1/DeviceList/*/$ns3::WifiNetDevice/Mac/AckedMpdu", MakeCallback (&AckedMpduTrace));

  // For PHY Tx state tracing - used for Channel Busy Time calculation only
  Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/State", MakeCallback (&PhyTxStateTraceAll));
  // Config::Connect ("/NodeList/0/DeviceList/1/Phy/State/State", MakeCallback (&PhyTxStateTrace<0>));
  // Config::Connect ("/NodeList/1/DeviceList/*/Phy/State/State", MakeCallback (&PhyTxStateTrace<1>));

  // for (uint32_t ii = 0 ; ii < otherStaCount ; ii++)
  // {
  //   const int nodeId = 2 + ii;
  //   Config::Connect ("/NodeList/3/DeviceList/0/Phy/State/State", MakeCallback (&PhyTxStateTrace<3>));
  // }
  



  // For state tracing - to console
  // Config::Connect ("/NodeList/1/DeviceList/0/$ns3::WifiNetDevice/Phy/State/State", MakeCallback (&PhyStateTracePrint));


  /* Start Simulation */
  Simulator::Stop (Seconds (simulationTime + 1));

  //To enable ASCII traces
  // AsciiTraceHelper ascii;
  // std::stringstream ss;
  // ss <<FOLDER_PATH<< "log"<<currentLoopIndex_string<<".asciitr" ;
  // wifiPhy.EnableAsciiAll (ascii.CreateFileStream (ss.str()));
  //Changes below - Note: If position is changed here, it changes regardless of the code above.
  // AnimationInterface anim("energyModelSim.xml");
  // anim.SetConstantPosition(networkNodes.Get(0), 0.0, 0.0);
  // anim.SetConstantPosition(networkNodes.Get(1), 10.0, 0.0);
  // anim.SetConstantPosition(networkNodes.Get(2), -100.0, 0.0);
  // anim.EnablePacketMetadata(true);
  //******************************************


  Simulator::Run ();

  double averageThroughput = ((sink->GetTotalRx () * 8) / (1e6 * simulationTime));

  Simulator::Destroy ();
  std::cout << "\nAverage throughput: " << averageThroughput << " Mbit/s" << std::endl;
  return 0;
}
