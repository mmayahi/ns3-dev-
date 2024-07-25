
/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 *    
 * This source file is modified from wifi-tcp.cc from the examples.
 * 
 * This is the source file to develop, test, and implement Wi-Fi PSM
 * 
 * Check journalPSM.md for details
 * 
 * To see available arguments on this example:
 * % sudo ./waf --run "scratch/Scratch-twt-psm-udp-tcp.cc --help" --gdb
 *
 * To run - example:
 *  ./ns3 --run "PSM_MU --simulationTime=5 --enablePSM_flag=true --enableUdpUplink=true --dataPeriod=0.33 --StaCount=10 --loopIndex=123321 --dataRatebps_other=31000 --enableDownlink=true --multicastInterval_ms=200 --PSM_activation_time=6.8" >>trial123.out 
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
//#include "ns3/netanim-module.h"
#include "ns3/wifi-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/v4ping-helper.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-mac.h"
#include <iomanip>
#include "ns3/rng-seed-manager.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/wifi-radio-energy-model-helper.h"
#include "ns3/energy-module.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/show-progress.h"
#include "ns3/pointer.h"
#include<cmath>

//-*********************************
//#define LOGNAME_PREFIX "WiFiPSM_MU"
#define FOLDER_PATH "MU_logs/"

NS_LOG_COMPONENT_DEFINE ("WiFiPSM");

using namespace ns3;

// ---------------------------------------------------------------------------

std::string LoopIndex = "123213";     // This will be updated with input from cmd line and used when creating log files. 

// ---------------------------------------------------------------------------

ApplicationContainer sinkApps;                         /* packet sink application */
uint64_t lastTotalRx = 0;                     /* The value of the last total received bytes */
uint32_t delACKTimer_ms = 0;                     /* TCP delayed ACK timer in ms   */ 

//Configurable parameters************************
double simulationTime = 30;                        /* Simulation time in seconds. */

uint32_t payloadSize = 1024;                       /* Transport layer payload size in bytes. */
//double dataPeriod = 0.001;                       /* Application data period in seconds. */
uint32_t flowMonStartTime_s = 0.0;   // Start time for flow monitor in seconds
uint32_t ampduLimitBytes = 20000;       //Maximum length in bytes of an A-MPDU for AC_BE access class 
//50,000 at 143 Mbps and 5/6 coding takes 3.4 ms. refer to "BE_MaxAmpduSize"
bool useExtendedBlockAck {false};
uint32_t bsrLife_ms = 10;                //Lifetime of Buffer Status Reports received from stations.
// in ms - BSR life time. refer to "BsrLifetime"
int gi {800}; // Guard interval in nanoseconds (can be 800, 1600 or 3200 ns)
uint8_t staCountModulusForTwt = 1;      // Modulus for STA count for TWT SP start - TWT SP start Offset is added only after each 'staCountModulusForTwt' STAs
uint8_t maxOffsetModulusMultiplier = 2;      // Offsets added = providedOffset % (maxOffsetModulusMultiplier * BI)
double nextStaTwtSpOffsetDivider = 16;   // K, where nextStaTwtSpOffset = BI / K; K is double


//uint32_t dataRatebps_other = 1 * payloadSize*8/dataPeriod;  
bool enable_throughput_trace = false;
bool enableEnergyTrace = true;  
bool enablePSM_flag = false;
//uint32_t PSM_activation_time = 3.0;     // to reproduce the bug please run with " % ./waf --run "scratch/Scratch1-twt-psm-udp-tcp --randSeed=20 --link=2 --power=1 --traffic=3 --udp=0 --StaCount=2" comman
uint32_t PSM_activation_time = 8.5;
uint32_t link = 1; //communication link = 1: uplink, 2: downlink, 3: douplex 
bool enablePhyStateTrace = true ;
bool enableFlowMon = true;          // Enable flow monitor if true
bool enableDownlink =false;
bool enableUplink =false;
bool enableTwt = false;
uint32_t traffic{1}; // traffic generator {1: periodic, 2: poisson, 3: full buffer}
bool forcePeriodicTraffic = true;     //if true, poisson is converted to predictable periodic traffic
bool poissonTraffic = false; //if true, predictable periodic is converted to possion traffic
Time beaconInterval = MicroSeconds(102400);  // 102.4 ms as beacon interval
uint32_t packetsPerSecond = 10;        // UL Packets per second per station
Time firstTwtSpOffsetFromBeacon = MilliSeconds (2);    // Offset from beacon for first TWT SP
Time firstTwtSpStart = (73 * beaconInterval);
double twtWakeIntervalMultiplier = 0.25;    // K, where twtWakeInterval = BI * K; K is double
double twtNominalWakeDurationDivider = 16;    // K, where twtNomimalWakeDuration = BI / K; K is double
bool twtTriggerBased = false; // Set it to false for contention-based TWT

uint32_t StaCount = 1;  
uint32_t uplinkpoissonDataRate = 30e3; // more than 111 Mbps uplink 
uint32_t downlinkpoissonDataRate = 20e3; // more than 78 Mbps downlink 

bool udp = true; //transport protocol: true for udp and false for tcp
uint32_t staMaxMissedBeacon = 1000;                 // Set the max missed beacons for STA before attempt for reassociation

Time AdvanceWakeupPS = MicroSeconds (10);

//uint16_t power{2};             //power save mechanism {1: power save mode, 2: target wake time, 3: active mode}

bool pcapTracing = true;                          /* PCAP Tracing is enabled or not. */
//-**************************************************************************

// output file for sta throughput 
  std::stringstream thrpt;
// output file for packet loss 
  std::stringstream pcktloss;
// output file for delay 
  std::stringstream dlay;
// output file for energy 
  std::stringstream enrgy;
// output file for overhead energy 
  std::stringstream ovrhead_nrg;
//output file for nodes energy trends
  std::stringstream energy_trend;

  
static std::map<u_int32_t , double> allTxtime; //all transsmission time
static std::map<u_int32_t , double> allRxtime; //all reception time
static std::map<u_int32_t , double> allIdletime; //all idle time
static std::map<u_int32_t , double> allCCA_BUSYtime; //all cca busy time
static std::map<u_int32_t , double> allSleeptime; //all cca busy time
static std::map<u_int32_t , double> Txsum; //data transsmission time
static std::map<u_int32_t , double> Rxsum; //data reception time
static std::map<u_int32_t , double> TxsigSum; // signaling transmission time
static std::map<u_int32_t , double> RxsigSum; // signaling reception time
static std::map<u_int32_t , double> StaDis; // STAS's distance from the AP
static std::map<double , double> StaEnr; // STAS's Energy consumption trend
static std::map<double , double> ApEnr; // Ap's Energy consumption trend


// Parse context strings of the form "/NodeList/x/DeviceList/x/..." to extract the NodeId integer

///*************************************************************************//////
uint32_t
ContextToNodeId (std::string context)
{
  std::string sub = context.substr (10);
  uint32_t pos = sub.find ("/Device");
  return atoi (sub.substr (0, pos).c_str ());
}

static void PingRtt (std::string context, Time rtt)
{
  std::cout << "Time: " <<Simulator::Now().GetSeconds() << context << rtt.GetSeconds () << std::endl;
}

void 
initiateTwtAtAp (Ptr<WifiNetDevice> apWifiDevice, Mac48Address staMacAddress, Time twtWakeInterval, Time nominalWakeDuration, Time nextTwt)

{
  Ptr<ApWifiMac> apMac = DynamicCast<ApWifiMac> (apWifiDevice->GetMac ());
  apMac->SetTwtSchedule (0, staMacAddress, false, true, true, twtTriggerBased, true, 0, twtWakeInterval, nominalWakeDuration, nextTwt);
  return;
}

void PhyStateTrace (std::string context, Time start, Time duration, WifiPhyState state)
{
switch(state)
  {
    case CCA_BUSY:
      //state_current = 50;
      allCCA_BUSYtime[ContextToNodeId (context)]+= duration.GetSeconds();
      break;
    case IDLE:
      //state_current = 50;
      allIdletime[ContextToNodeId (context)]+= duration.GetSeconds();
      break;
    case TX:
      //state_current = 232;
      allTxtime[ContextToNodeId (context)]+= duration.GetSeconds();
      //NS_LOG_UNCOND(state <<"   "<< duration.GetSeconds());
      break;
    case RX:
      //state_current = 66;
      allRxtime[ContextToNodeId (context)]+= duration.GetSeconds();
      break;
    default:
     // state_current = 0.12; // sleeping
    allSleeptime[ContextToNodeId (context)]+= duration.GetSeconds();
  }
}


void callbackfunctions( WifiHelper wifiHelper)
{
 //wifiHelper.EnableLogComponents();
 LogComponentEnable ("ApWifiMac", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
 LogComponentEnable ("StaWifiMac", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
/* LogComponentEnable ("TcpHeader", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
 LogComponentEnable ("TcpSocketBase", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
 LogComponentEnable ("TcpSocket", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
 LogComponentEnable ("TcpL4Protocol", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
 LogComponentEnable("TcpCongestionOps", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE |LOG_LEVEL_ALL));
 LogComponentEnable("Socket", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE |LOG_LEVEL_ALL));
*/
}

//-**************************************************************************
// Send broadcast packet from specific STA MAC
void
PhyTxPsduBegin(std::string context, WifiConstPsduMap psdu, WifiTxVector txVector, double txPowerW, Time txDuration) //ppdu duration 
{
  Ptr<Packet> p = psdu.begin ()->second->GetPacket()->Copy ();
  if (txVector.IsAggregation ())
  {
    AmpduSubframeHeader subHdr;
    uint32_t extractedLength;
    p->RemoveHeader (subHdr);
    extractedLength = subHdr.GetLength ();
    p = p->CreateFragment (0, static_cast<uint32_t> (extractedLength));
  }
  WifiMacHeader macHdr;
  p->PeekHeader(macHdr);
  if(macHdr.IsData())
  {
    Txsum[ContextToNodeId (context)]+= txDuration.GetSeconds();
  }
  else
  {
    TxsigSum [ContextToNodeId (context)] += txDuration.GetSeconds();
  }

}
void
PhyRxPayloadBegin(std::string context, Ptr<const WifiPpdu> ppdu, WifiTxVector txVector, Time rxDuration)
{
  Ptr<Packet> p = ppdu->GetPsdu()->GetPacket()->Copy ();
  if (txVector.IsAggregation ())
  {
    AmpduSubframeHeader subHdr;
    uint32_t extractedLength;
    p->RemoveHeader (subHdr);
    extractedLength = subHdr.GetLength ();
    p = p->CreateFragment (0, static_cast<uint32_t> (extractedLength));
  }

  WifiMacHeader macHdr;
  p->PeekHeader(macHdr);
  if(macHdr.IsData())
  {
  
    Rxsum[ContextToNodeId(context)]+= rxDuration.GetSeconds();
  }
  else
  {
    RxsigSum[ContextToNodeId(context)]+= rxDuration.GetSeconds();
  }

}

//-*******************************************************************
void
CalculateThroughput ()
{
  // This function has been modified to output the data received every second in Bytes
  Time now = Simulator::Now ();
  double currentDataBytes = ((StaticCast<PacketSink>(sinkApps.Get(0)))->GetTotalRx () - lastTotalRx);
  std::cout << now.GetSeconds () << "s: Data received in previous second in Bytes\t" << currentDataBytes << " Bytes" << std::endl;
  lastTotalRx = (StaticCast<PacketSink>(sinkApps.Get(0)))->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (1000), &CalculateThroughput);
}


void
changeStaPSM (Ptr<StaWifiMac> staMac, bool PSMenable)
{
  // This function puts the specific PHY to sleep
  staMac->SetPowerSaveMode(PSMenable);
}
//-*******************************************************************

void
printStaPSM (Ptr<StaWifiMac> staMac)
{
  // This function puts the specific PHY to sleep
  NS_LOG_INFO("STA MAC "<<staMac<<" is in PS = "<<staMac->GetPowerSaveMode());
}
//-*******************************************************************


void
RemainingStaEnergy(double oldValue, double remainingEnergy)
{
  NS_LOG_UNCOND(Simulator::Now().GetSeconds()
  << "s Current remaining energy in STA's Source = " << remainingEnergy << "J");
}


void
TotalStaEnergy(std::string context, double oldValue, double totalValue)
{
  std::cout << "Time: " <<Simulator::Now().GetSeconds() << "s Total energy consumed by STA's radio = "
  << context << " - Total energy consumption: " << totalValue << "J" << std::endl;
}

// Function to print energy consumption every second
void PrintEnergyConsumption(NodeContainer nodes)
{
    double sumEnergy=0;
    double ApEnergy=0;
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        Ptr<Node> node = nodes.Get(i);
        Ptr<NetDevice> device = node->GetDevice(0);
        Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice>(device);
        if (wifiDevice) {
            Ptr<WifiPhy> wifiPhy = wifiDevice->GetPhy();
            if (wifiPhy) {
                Ptr<WifiRadioEnergyModel> energyModel = wifiPhy->GetWifiRadioEnergyModel();                 
                if (energyModel) {
                    double Energy_cons = energyModel->GetTotalEnergyConsumption();
                    //std::cout << "Time: "<< Simulator::Now().GetSeconds()<< ", Nodeid: "<< i << ", average energy consumption: " << Energy_cons << " Joules" << std::endl; 
                    if (i == 0){
                      ApEnergy = Energy_cons;
                    }
                    else{
                      sumEnergy += Energy_cons;
                    }
                }
            }
        }
    }
    sumEnergy = sumEnergy/nodes.GetN();
    //std::cout << "Time: "<< Simulator::Now().GetSeconds()<< ", average energy consumption: " << sumEnergy << " Joules" << std::endl; 
    StaEnr[Simulator::Now().GetSeconds()]= sumEnergy;
    ApEnr[Simulator::Now().GetSeconds()]= ApEnergy;
    // Schedule the next print event
    Simulator::Schedule(Seconds(1.0), &PrintEnergyConsumption, nodes);
}


//-*******************************************************************

int
main (int argc, char *argv[])
{
  
  thrpt <<"thrpt.log";
  std::fstream TH (thrpt.str ().c_str (), std::ios::app);

  ovrhead_nrg <<"ovrhead-nrg.log";
  std::fstream ovr_NRG (ovrhead_nrg.str ().c_str (), std::ios::app);

  energy_trend <<"energy-trend.log";
  std::fstream NRG_trend (energy_trend.str ().c_str (), std::ios::app);

  enrgy <<"nrg.log";
  std::fstream NRG (enrgy.str ().c_str (), std::ios::app);

  dlay <<"dlay.log";
  std::fstream DLY (dlay.str ().c_str (), std::ios::app);
 
  pcktloss <<"pcktloss.log";
  std::fstream PcktLoss (pcktloss.str ().c_str (), std::ios::app);


  /* Command line argument parser setup. */ 
  CommandLine cmd (__FILE__);
  cmd.AddValue ("link", "Communication link = 1 for uplink, 2 for downlink, 3 for duplex", link);
//  cmd.AddValue("power", "power save mechanism (1 for PSM, 2 for twt and 3 for active mode).", power);
  cmd.AddValue ("traffic", "traffic generator. 1: periodic traffic, 2:poisson traffic, 3: full buffer", traffic);
  cmd.AddValue ("udp", "UDP if set to 1, TCP otherwise", udp);
  cmd.AddValue ("packetsPerSecond", "UL/DL Packets per second per STA/AP", packetsPerSecond);
  cmd.AddValue ("StaCount", "Number of other STAs. Integer between 0 and 100", StaCount);
  cmd.AddValue ("simulationTime", "Simulation duration in seconds", simulationTime);

  cmd.Parse (argc, argv);
  // Parameter verification
  //NS_ABORT_MSG_IF(power < 1 || power > 3,"Invalid power (must be 1, 2 or 3)");
  
 /* if (power == 1)
  {
    enablePSM_flag = true;
    enableTwt = false;    
  }

  else if (power == 2)
  {
    enablePSM_flag = false;
    enableTwt = true;    
  }
  else
  {
    enablePSM_flag = false;
    enableTwt = false;    
  }
*/
  NS_ABORT_MSG_IF(link < 1 || link > 3, "Invalid link (must be 1, 2 or 3)");
  
  if (link == 1)
  {
    enableUplink = true;
    enableDownlink = false;    
  }

  else if (link == 2)
  {
    enableUplink = false;
    enableDownlink = true;
  }
  else {
    enableUplink = true;
    enableDownlink = true; 
  }

  NS_ABORT_MSG_IF(traffic < 1 || traffic > 3, "Invalid traffic (must be 1, 2 or 3)");
  
  if (traffic == 1)
  {
    forcePeriodicTraffic = true;
    poissonTraffic = false;  
  }
  else if (traffic == 2)
  {
    forcePeriodicTraffic = false;
    poissonTraffic = true;
  }
  else
  {
    forcePeriodicTraffic = false;
    poissonTraffic = false;
    uplinkpoissonDataRate = 200e3; // more than 111 Mbps uplink 
    downlinkpoissonDataRate = 200e3; // more than 78 Mbps downlink 
  }


  std::stringstream indexStringTemp;
  indexStringTemp << std::setfill('0') << std::setw(6) << LoopIndex;
  LoopIndex = indexStringTemp.str();


  double avr_dl_pkt_los=0;
  double avr_ul_pkt_los=0;

  double avr_ul_dly=0;
  double avr_dl_dly=0;

  double average_sta_energy = 0;
  double ap_energy = 0;

  double ap_ovrhd_energy=0;
  double sta_ovrhd_energy = 0;

  double average_sta_throughput = 0;
  double average_sta_tx_signaling = 0;
  double average_sta_rx_signaling = 0;


  std::string uplinkstr = std::to_string(uplinkpoissonDataRate/StaCount)+"kb/s";

  std::string downlinkstr = std::to_string(downlinkpoissonDataRate/StaCount)+"kb/s";




  // Logging if necessary
  
  //LogComponentEnable ("IdealWifiManager", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  //
  // LogComponentEnable ("WifiHelper", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  //LogComponentEnable ("StaWifiMac", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("ApWifiMac", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("RegularWifiMac", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("WifiMacQueue", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("WifiMacQueueItem", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("WifiRemoteStationManager", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  /* LogComponentEnable ("WifiPhy", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  LogComponentEnable ("PhyEntity", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  LogComponentEnable ("DsssPhy", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  LogComponentEnable ("OfdmPhy", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  LogComponentEnable ("HePhy", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  LogComponentEnable ("FrameExchangeManager", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL | LOG_DEBUG ));
  LogComponentEnable ("QosFrameExchangeManager", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  */// LogComponentEnable ("VhtFrameExchangeManager", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("HtFrameExchangeManager", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  //LogComponentEnable ("HeFrameExchangeManager", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("PhyEntity", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("WifiPhyStateHelper", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  //LogComponentEnable ("TwtRrMultiUserScheduler", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("MultiUserScheduler", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("WifiDefaultAckManager", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("WifiAckManager", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("ChannelAccessManager", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  //LogComponentEnable ("QosTxop", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("Txop", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  /*LogComponentEnable ("BlockAckManager", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  LogComponentEnable ("BlockAckAgreement", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  LogComponentEnable ("OriginatorBlockAckAgreement", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  */// LogComponentEnable ("BlockAckWindow", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("MacTxMiddle", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("WifiPhyStateHelper", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("WifiPhyStateHelper", LogLevel (LOG_LEVEL_INFO));
  // LogComponentEnable ("BasicEnergySource", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("PacketSink", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("VoiPApplication", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("SeqTsHeader", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("UdpServer", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("UdpClient", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("UdpSocket", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("TcpHeader", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("TcpSocketBase", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  //LogComponentEnable ("TcpSocket", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));
  // LogComponentEnable ("OnOffApplication", LogLevel(LOG_PREFIX_ALL | LOG_LEVEL_INFO));
  // LogComponentEnable ("WifiTxParameters", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL));

  //*******************************************************
  //Time interPacketIntervalUdp = Seconds (double (udpUplinkPacketSizeBits)/double(dataRatebps_other));
  //  std::cout<<"\nConfigured Udp uplink interPacketInterval: "<<interPacketIntervalUdp.GetMilliSeconds()<<" ms";

  
  // house dimension in meters - creating uniform random number generator house of 400 sqm free space
  double minX = -20.0;
  double maxX = 20.0;
  double minY = -20.0;
  double maxY = 20.0;

  // Other STA positions
  double currentX, currentY;  
  Ptr<UniformRandomVariable> xCoordinateRand = CreateObject<UniformRandomVariable> ();
  Ptr<UniformRandomVariable> yCoordinateRand = CreateObject<UniformRandomVariable> ();

  xCoordinateRand->SetAttribute ("Min", DoubleValue (minX));
  xCoordinateRand->SetAttribute ("Max", DoubleValue (maxX));
  yCoordinateRand->SetAttribute ("Min", DoubleValue (minY));
  yCoordinateRand->SetAttribute ("Max", DoubleValue (maxY));

  
  NodeContainer ApNodes;
  ApNodes.Create (1);    
  Ptr<Node> apWifiNode = ApNodes.Get (0);
  
  NodeContainer StaNodes;
  StaNodes.Create (StaCount);

  NodeContainer AllNodes;
  AllNodes.Add(ApNodes);
  AllNodes.Add(StaNodes);

  WifiMacHelper wifiMac_AP;
  WifiMacHelper wifiMac__STA;

  NetDeviceContainer staWiFiDevices;
  NetDeviceContainer apWiFiDevices;

  if (!udp)
  {

    Config::SetDefault ("ns3::TcpL4Protocol::SocketType", TypeIdValue (TypeId::LookupByName ("ns3::TcpNewReno")));
    //Configure TCP Options 
    Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (payloadSize));
    Config::SetDefault ("ns3::TcpSocket::DelAckTimeout", TimeValue (MilliSeconds (delACKTimer_ms)));
    Config::SetDefault ("ns3::TcpSocket::ConnTimeout", TimeValue (MilliSeconds(200)));    //"TCP retransmission timeout when opening connection (seconds) - default 3 seconds"
    Config::SetDefault ("ns3::TcpSocket::DataRetries", UintegerValue (20));
  }

  WifiHelper wifiHelper;

  Ssid ssid;
  ssid = Ssid ("ns3-80211ax");
  Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel> ();
  SpectrumWifiPhyHelper wifiPhy_twt; //phisical layer for twt
  /* Set up Legacy Channel */
  //YansWifiChannelHelper psmwifiChannel;
  /* Setup Physical Layer for no-twt */
  //YansWifiPhyHelper wifiPhy_psm;
  wifiHelper.SetRemoteStationManager ("ns3::IdealWifiManager");
  
  wifiHelper.SetStandard (WIFI_STANDARD_80211ax_2_4GHZ);

  Config::SetDefault ("ns3::LogDistancePropagationLossModel::ReferenceLoss", DoubleValue (40));
  Config::SetDefault ("ns3::LogDistancePropagationLossModel::Exponent", DoubleValue (2));
  //set up twt channel configuration
//  if (enableTwt)
//  {
    NS_LOG_UNCOND("set up twt channel configuration");
    Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue (65535));
    Config::SetDefault ("ns3::WifiMacQueue::MaxSize", QueueSizeValue(QueueSize ("10000p")));
    Config::SetDefault ("ns3::WifiMacQueue::MaxDelay", TimeValue (MilliSeconds (60000)));
    Config::SetDefault("ns3::TableBasedErrorRateModel::FallbackErrorRateModel", PointerValue(CreateObject<NistErrorRateModel>()));


    wifiMac_AP.SetMultiUserScheduler ("ns3::TwtRrMultiUserScheduler", "EnableUlOfdma", BooleanValue (true), "EnableBsrp", BooleanValue (false), "NStations", UintegerValue (1)); //maxMuSta = 1 or StaCount
    wifiMac_AP.SetAckManager("ns3::WifiDefaultAckManager", "DlMuAckSequenceType", EnumValue (WifiAcknowledgment::DL_MU_AGGREGATE_TF));
    wifiMac_AP.SetType ("ns3::ApWifiMac","EnableBeaconJitter", BooleanValue (true), "BE_BlockAckThreshold", UintegerValue (1), "BE_MaxAmpduSize", UintegerValue (ampduLimitBytes), "BsrLifetime", TimeValue (MilliSeconds (bsrLife_ms)), "Ssid", SsidValue (ssid));    
    wifiPhy_twt.SetChannel (spectrumChannel);
    wifiPhy_twt.Set ("ChannelSettings", StringValue ("{0, 20, BAND_2_4GHZ, 0}"));
    wifiMac__STA.SetType ("ns3::StaWifiMac", "BE_BlockAckThreshold", UintegerValue (1), "Ssid", SsidValue (ssid)); // If AMPDU is used, Block Acks will always be used regardless of this value
    
    staWiFiDevices = wifiHelper.Install (wifiPhy_twt, wifiMac__STA, StaNodes);
    apWiFiDevices = wifiHelper.Install (wifiPhy_twt, wifiMac_AP, apWifiNode);
    
    // Set guard interval and MPDU buffer size
    Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval", TimeValue (NanoSeconds (gi)));
    Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/MpduBufferSize", UintegerValue (useExtendedBlockAck ? 256 : 64));
  //}
 
  //set up non-twt channel configuration
 /* else
  {
    NS_LOG_UNCOND ("set up non-twt channel configuration");
    //wifiHelper.SetStandard (WIFI_STANDARD_80211n_2_4GHZ);
    Config::SetDefault ("ns3::WifiMacQueue::MaxSize", QueueSizeValue(QueueSize ("10000p")));
    Config::SetDefault ("ns3::WifiMacQueue::MaxDelay", TimeValue (MilliSeconds (60000)));
    psmwifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    psmwifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel", "Frequency", DoubleValue (2.4e9));
    wifiPhy_psm.Set ("ChannelSettings", StringValue ("{0, 20, BAND_2_4GHZ, 0}"));
    wifiPhy_psm.SetErrorRateModel ("ns3::YansErrorRateModel");
    wifiMac_AP.SetAckManager("ns3::WifiDefaultAckManager", "DlMuAckSequenceType", EnumValue (WifiAcknowledgment::DL_MU_AGGREGATE_TF));
    wifiPhy_psm.SetChannel (psmwifiChannel.Create ());

    wifiMac_AP.SetType ("ns3::ApWifiMac",
                      "EnableBeaconJitter", BooleanValue (false),
                      "BE_MaxAmpduSize", UintegerValue (ampduLimitBytes),
                      "BsrLifetime", TimeValue (MilliSeconds (bsrLife_ms)),
                      "BE_BlockAckThreshold", UintegerValue (1),
                      "EnableUApsd", BooleanValue(false),                    
                      "Ssid", SsidValue (ssid));
    wifiMac__STA.SetType ("ns3::StaWifiMac",
                      "BE_BlockAckThreshold", UintegerValue (1),
                      "Ssid", SsidValue (ssid));
    staWiFiDevice = wifiHelper.Install (wifiPhy_psm, wifiMac__STA, StaNodes);
    apWiFiDevice = wifiHelper.Install (wifiPhy_psm, wifiMac_AP, apWifiNode);

    Config::Set ("/NodeList/0/DeviceList/1/$ns3::WifiNetDevice/Mac/$ns3::ApWifiMac/DtimPeriod", UintegerValue(3));
    Config::Set ("/NodeList/0/DeviceList/1/$ns3::WifiNetDevice/Mac/$ns3::ApWifiMac/PsUnicastBufferSize", QueueSizeValue(QueueSize ("678p")));
    Config::Set ("/NodeList/0/DeviceList/1/$ns3::WifiNetDevice/Mac/$ns3::ApWifiMac/PsUnicastBufferDropDelay", TimeValue (MilliSeconds (1123)));
*/
  //  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval", TimeValue (NanoSeconds (gi)));
  //  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/MpduBufferSize", UintegerValue (useExtendedBlockAck ? 256 : 64));
 // }
 
  // Set max missed beacons at STAs to avoid dis-association
  for (u_int32_t ii = 0; ii < StaCount; ii++)
  {
    
    std::stringstream nodeIndexStringTemp, maxBcnStr, advWakeStr;
    nodeIndexStringTemp << StaNodes.Get(ii)->GetId();
    maxBcnStr << "/NodeList/" << nodeIndexStringTemp.str() << "/DeviceList/0/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::StaWifiMac/MaxMissedBeacons";
    advWakeStr << "/NodeList/" << nodeIndexStringTemp.str() << "/DeviceList/0/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::StaWifiMac/AdvanceWakeupPS";

    Config::Set (maxBcnStr.str(), UintegerValue(staMaxMissedBeacon));
    Config::Set (advWakeStr.str(), TimeValue(AdvanceWakeupPS));
  }

  // Enable / disable PSM and sleep state using MAC attribute change - through a function - not directly changing the MAC attribute 
  //if (enablePSM_flag)
  //{

    for (u_int32_t ii = 1; ii < StaCount ; ii+=3)
    {
      Ptr<WifiNetDevice> device = staWiFiDevices.Get(ii)->GetObject<WifiNetDevice> ();    //This returns the pointer to the object - works for all functions from WifiNetDevice
      Ptr<WifiMac> staMacTemp = device->GetMac ();
      Ptr<StaWifiMac> staMac = DynamicCast<StaWifiMac> (staMacTemp);
      NS_LOG_UNCOND("staMac->GetAddress() " << staMac->GetAddress());

      Simulator::Schedule (Seconds (PSM_activation_time)+ MilliSeconds(10*ii), &changeStaPSM, staMac, true);
    }
//  }
  
 // Setting up TWT
  Ptr<WifiNetDevice> apWifiDevice = apWiFiDevices.Get(0)->GetObject<WifiNetDevice> ();    //This returns the pointer to the object - works for all functions from WifiNetDevice

 // if (enableTwt)
 // {
    for (u_int32_t ii = 0; ii < StaCount ; ii+=3)
    {
      // Setting up TWT for Sta Mac
      Ptr<WifiNetDevice> device = staWiFiDevices.Get(ii)->GetObject<WifiNetDevice> ();    //This returns the pointer to the object - works for all functions from WifiNetDevice
      Ptr<WifiMac> staMacTemp = device->GetMac ();
      Ptr<StaWifiMac> staMac = DynamicCast<StaWifiMac> (staMacTemp);
      
      Time delta = firstTwtSpOffsetFromBeacon + int(ii/staCountModulusForTwt)*beaconInterval/nextStaTwtSpOffsetDivider;
      delta = MicroSeconds( delta.GetMicroSeconds()% (maxOffsetModulusMultiplier*beaconInterval.GetMicroSeconds()) );   
      // To ensure that offsets are not too large. That may cause frame drops before the first SP begins. Above line ensures that the first scheduled SP is no further than k*BI from the next beacon in the future
            
      Time scheduleTwtAgreement = (firstTwtSpStart + MilliSeconds(20) + ii*MicroSeconds(100));
      Time twtWakeInterval = twtWakeIntervalMultiplier*beaconInterval;
      Time twtNominalWakeDuration = beaconInterval/twtNominalWakeDurationDivider;

      // TWT at AP MAC
      std::cout<<"\nTWT agreement for STA:"<<staMac->GetAddress()<<"\nAction frame acheduled at t = "
      <<scheduleTwtAgreement.GetSeconds() << "s;\nTWT SP starts at "<<delta.GetMicroSeconds()/1000.0<<
      " ms after next beacon;\nTWT Wake Interval = "<< twtWakeInterval.GetMicroSeconds()/1000.0<<
      " ms;\nTWT Nominal Wake Duration = "<< twtNominalWakeDuration.GetMicroSeconds()/1000.0<<" ms;\n\n\n";

      Simulator::Schedule (scheduleTwtAgreement, &initiateTwtAtAp, apWifiDevice, staMac->GetAddress(), twtWakeInterval, twtNominalWakeDuration, delta);
    } 
 // }
  

  /* Mobility model */
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  for (uint32_t ii = 0; ii <StaCount ; ii++)
  {
    currentX = xCoordinateRand->GetValue ();
    currentY = yCoordinateRand->GetValue ();
    positionAlloc->Add (Vector (currentX, currentY, 0.0));
    StaDis[ii]= sqrt(pow(currentX, 2) + pow(currentY, 2));
  }


  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (apWifiNode);
  mobility.Install (StaNodes);
  

  /** Energy Model **/
  /***************************************************************************/
  /* energy source STA (iPhone X)*/
  BasicEnergySourceHelper STASourceHelper;
  // configure energy source
  STASourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(3000)); 
  STASourceHelper.Set("BasicEnergySupplyVoltageV", DoubleValue(4.9));
  STASourceHelper.Set("PeriodicEnergyUpdateInterval", TimeValue(Seconds(1.0)));
  // install source
  EnergySourceContainer STAsources = STASourceHelper.Install(StaNodes);
  /* device energy model */
  WifiRadioEnergyModelHelper STAradioEnergyHelper;
  // configure radio energy model
  STAradioEnergyHelper.Set("TxCurrentA", DoubleValue(0.801));
  STAradioEnergyHelper.Set("IdleCurrentA", DoubleValue(0.239));
  STAradioEnergyHelper.Set("CcaBusyCurrentA", DoubleValue(0.239));
  STAradioEnergyHelper.Set("RxCurrentA", DoubleValue(0.515));
  STAradioEnergyHelper.Set("SwitchingCurrentA", DoubleValue(0.515));
  STAradioEnergyHelper.Set("SleepCurrentA", DoubleValue(0.16));
  // install device model
  DeviceEnergyModelContainer STAdeviceModels = STAradioEnergyHelper.Install(staWiFiDevices, STAsources);
   

  /* energy source AP (LB6)*/
  BasicEnergySourceHelper APSourceHelper;
  // configure energy source
  APSourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(3000)); 
  APSourceHelper.Set("BasicEnergySupplyVoltageV", DoubleValue(12.2));
  APSourceHelper.Set("PeriodicEnergyUpdateInterval", TimeValue(Seconds(1.0)));
  // install source
  EnergySourceContainer APsources = APSourceHelper.Install(apWifiNode);
  /* device energy model */
  WifiRadioEnergyModelHelper APradioEnergyHelper;
  // configure radio energy model
  APradioEnergyHelper.Set("TxCurrentA", DoubleValue(0.668));
  APradioEnergyHelper.Set("IdleCurrentA", DoubleValue(0.546));
  APradioEnergyHelper.Set("CcaBusyCurrentA", DoubleValue(0.546));
  APradioEnergyHelper.Set("RxCurrentA", DoubleValue(0.568));
  APradioEnergyHelper.Set("SwitchingCurrentA", DoubleValue(0.568));
  APradioEnergyHelper.Set("SleepCurrentA", DoubleValue(0.423));
  // install device model
  DeviceEnergyModelContainer APdeviceModels = APradioEnergyHelper.Install(apWiFiDevices, APsources);

  
  /* Internet stack */
  InternetStackHelper stack;
  stack.Install (apWifiNode);
  //stack.Install (serverNodes);
  stack.Install (StaNodes);

  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.255.255.0");
  Ipv4InterfaceContainer apInterface;
  apInterface = address.Assign (apWiFiDevices);
  Ipv4InterfaceContainer staInterface;
  staInterface = address.Assign (staWiFiDevices);


  /* Populate routing table */
  //Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  // create a map of IP addresses to MAC addresses
  std::map<Ipv4Address, Mac48Address> ipToMac;
  for (uint32_t i = 0; i < StaNodes.GetN(); i++)
  {
    Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice> (StaNodes.Get (i)->GetDevice (0)); //assuming only one device
    Ptr<WifiMac> wifi_mac = wifi_dev->GetMac ();
    Ptr<StaWifiMac> sta_mac = DynamicCast<StaWifiMac> (wifi_mac);
    ipToMac[staInterface.GetAddress (i)] = sta_mac->GetAddress ();
  }
  
  // Pretty print ipToMac
  std::cout << "IP to MAC mapping:\n";
  for (auto it = ipToMac.begin (); it != ipToMac.end (); it++)
  {
    std::cout << it->first << " => " << it->second << '\n';
  }

  Ptr<UniformRandomVariable> randTime = CreateObject<UniformRandomVariable> ();
  randTime->SetAttribute ("Min", DoubleValue ((0.0)));
  randTime->SetAttribute ("Max", DoubleValue ((1.0)));

  //Poisson Traffic
  // On time = payload size in bytes * 8/ data rate = 1434*8/100Mbps = 0.00011472 seconds
  double onTime = (1.0*payloadSize * 8.0/(1.0*uplinkpoissonDataRate)+randTime->GetValue()/200000);
  NS_LOG_UNCOND ("ON time: " << onTime);


  // Off time nean = (Beacon Interval /nPacketsPerBI) - OnTime
  // double offTimeMean = (beaconInterval.GetMicroSeconds()/(packetCountPerBeaconPeriod*1.0e6)) - onTime; 
  double offTimeMean = abs(((1.0)/(packetsPerSecond*1.0)) - onTime) ; 
  NS_LOG_UNCOND ("Off time mean: " << offTimeMean);
  //std::cout <<"OFF time: " << offTimeMean <<std::endl;


  std::ostringstream onTimeStr1, offTimeStr1;
  onTimeStr1 << onTime;
  offTimeStr1 << offTimeMean;
  // std::string str = strs.str();


  std::string onTimeString;
  std::string offTimeString;
  if (forcePeriodicTraffic)
  {
    onTimeString = "ns3::ConstantRandomVariable[Constant="+ onTimeStr1.str() +"]";  
    offTimeString = "ns3::ConstantRandomVariable[Constant="+ offTimeStr1.str() +"]";   // Use this to force periodic predictable traffic
  }
  else if(poissonTraffic)
  {
    onTimeString = "ns3::ConstantRandomVariable[Constant="+ onTimeStr1.str() +"]";
    offTimeString = "ns3::ExponentialRandomVariable[Mean="+ offTimeStr1.str() +"]";    // Use this to force Poisson traffic
  }
  else
  {
    onTimeString = "ns3::ConstantRandomVariable[Constant=1.0]";
    offTimeString = "ns3::ConstantRandomVariable[Constant=0.0]";   // Use this to force full buffer traffic
  }


  if (enableUplink)
  {
    NS_LOG_UNCOND ("UP LINK Installed");
    uint16_t port = 50000;
    //ping the server(10.1.1.1) from all STAs
    V4PingHelper ping = V4PingHelper (apInterface.GetAddress (0));
    if (udp)
    {
      //UDP flow
      /* Install UDP Receiver on the P2P UDP server */
      PacketSinkHelper sinkHelper ("ns3::UdpSocketFactory", InetSocketAddress ( apInterface.GetAddress (0), port));
      ApplicationContainer tempsinkApp;
      tempsinkApp = sinkHelper.Install (apWifiNode);
      sinkApps.Start (Seconds (0.0));
      sinkApps.Add(tempsinkApp);
      OnOffHelper onoff ("ns3::UdpSocketFactory", (InetSocketAddress ( apInterface.GetAddress (0), port)));
      onoff.SetAttribute ("OnTime",  StringValue (onTimeString));
      onoff.SetAttribute ("OffTime", StringValue (offTimeString));
      onoff.SetAttribute ("DataRate", DataRateValue (DataRate (uplinkstr)));
      onoff.SetAttribute ("PacketSize", UintegerValue (payloadSize));
            
      
      for (uint32_t appcount =0 ; appcount < StaCount ; appcount++)
      {
        ApplicationContainer clientApp = onoff.Install (StaNodes.Get(appcount));
        clientApp.Start (Seconds(2 + randTime->GetValue()));
        clientApp.Stop (Seconds (simulationTime + 2));
        ApplicationContainer pinger = ping.Install(StaNodes.Get(appcount));
        pinger.Start (Seconds (1.0 ) + MilliSeconds(10*appcount));
        pinger.Stop (Seconds (2.0));
      }
    }

    else
    {
      //TCP flow
      /* Install UDP Receiver on the P2P UDP server */
      PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory",InetSocketAddress (apInterface.GetAddress(0), port));
      ApplicationContainer tempsinkApp;
      tempsinkApp = sinkHelper.Install (apWifiNode);
      tempsinkApp.Start (Seconds (0.0));
      sinkApps.Add(tempsinkApp);
      OnOffHelper onoff ("ns3::TcpSocketFactory", InetSocketAddress (apInterface.GetAddress(0), port));
      onoff.SetAttribute ("OnTime",  StringValue (onTimeString));
      onoff.SetAttribute ("OffTime", StringValue (offTimeString));
      onoff.SetAttribute ("DataRate", DataRateValue (DataRate (uplinkstr)));
      onoff.SetAttribute ("PacketSize", UintegerValue (payloadSize));
      for (uint32_t appcount =0 ; appcount < StaCount ; appcount++)
      {
        ApplicationContainer clientApp = onoff.Install (StaNodes.Get(appcount));
        clientApp.Start (Seconds(2 + randTime->GetValue()));
        clientApp.Stop (Seconds (simulationTime + 2));
        ApplicationContainer pinger = ping.Install(StaNodes.Get(appcount));
        pinger.Start (Seconds (1.0) + MilliSeconds(10*appcount));
        pinger.Stop (Seconds (2.0));
      }
    }          
    Config::Connect ("/NodeList/*/ApplicationList/*/$ns3::V4Ping/Rtt", MakeCallback (&PingRtt));
  }
  
  Simulator::Schedule (Seconds (0.1), &Ipv4GlobalRoutingHelper::PopulateRoutingTables);

  // Scheduling downlink packets from MainUDPServerNode to each of STAs
  if (enableDownlink) 
  {          
    NS_LOG_UNCOND ("Down LINK Installed");
    //UDP flow
    uint16_t port = 10;
    if (udp)
    {
      //Servers and sinks at STAs
      for(uint32_t in = 0; in < StaCount; in++)
      {          
        PacketSinkHelper sinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (staInterface.GetAddress(in), port));
        ApplicationContainer tempsinkApp;
        tempsinkApp = sinkHelper.Install (StaNodes.Get(in));
        tempsinkApp.Start (Seconds (0.0));
        sinkApps.Add(tempsinkApp);

        //Client at remote node
        OnOffHelper onoff ("ns3::UdpSocketFactory", InetSocketAddress (staInterface.GetAddress(in), port));
        onoff.SetAttribute ("OnTime",  StringValue (onTimeString));
        onoff.SetAttribute ("OffTime", StringValue (offTimeString));
        onoff.SetAttribute ("DataRate", DataRateValue (DataRate (downlinkstr)));
        onoff.SetAttribute ("PacketSize", UintegerValue (payloadSize));
        ApplicationContainer clientApp = onoff.Install (apWifiNode);
        clientApp.Start (Seconds(2 + randTime->GetValue()));
        clientApp.Stop (Seconds (simulationTime + 2));

        //ping the servers(10.0.0.*) from the remote server
        V4PingHelper ping = V4PingHelper (staInterface.GetAddress (in));
        ApplicationContainer pinger = ping.Install(apWifiNode);
        pinger.Start (Seconds (1.0) + MilliSeconds(10*in));
        pinger.Stop (Seconds (2.0));
      }  
    }
    else
    {
      for(uint32_t in = 0; in < StaCount; in++)
      {          
        PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory", InetSocketAddress (staInterface.GetAddress(in), port));
        ApplicationContainer tempsinkApp;
        tempsinkApp = sinkHelper.Install (StaNodes.Get(in));
        tempsinkApp.Start (Seconds (0.0));
        sinkApps.Add(tempsinkApp);
        
        OnOffHelper onoff ("ns3::TcpSocketFactory", InetSocketAddress (staInterface.GetAddress(in), port));
        onoff.SetAttribute ("OnTime",  StringValue (onTimeString));
        onoff.SetAttribute ("OffTime", StringValue (offTimeString));
        onoff.SetAttribute ("DataRate", DataRateValue (DataRate (downlinkstr)));
        onoff.SetAttribute ("PacketSize", UintegerValue (payloadSize));
        ApplicationContainer clientApp = onoff.Install (apWifiNode);
        clientApp.Start (Seconds(2 +  randTime->GetValue()));
        clientApp.Stop (Seconds (simulationTime + 2));

        //ping the servers(10.0.0.*) from the remote server
        V4PingHelper ping = V4PingHelper (staInterface.GetAddress (in));
        ApplicationContainer pinger = ping.Install(apWifiNode);
        pinger.Start (Seconds (1.0) + MilliSeconds(10*in));
        pinger.Stop (Seconds (2.0));
      }
      Config::Connect ("/NodeList/*/ApplicationList/*/$ns3::V4Ping/Rtt", MakeCallback (&PingRtt));
    }
  }
  
  if (enable_throughput_trace)
  {
    Simulator::Schedule (Seconds (1.0), &CalculateThroughput );
  }
  /* Enable Traces */
  if (pcapTracing)
  {
    std::stringstream ss1, ss2, ss4;
    wifiPhy_twt.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
    ss1<< "TWT_AP";
    wifiPhy_twt.EnablePcap (ss1.str(), apWiFiDevices);
    ss2<< "TWT_STA";
    wifiPhy_twt.EnablePcap (ss2.str(), staWiFiDevices);
  }

  if(enablePhyStateTrace)
  {
    //phy state
    std::stringstream nodeIndexStringTemp, phyStateStr, PhyTxPsduBeginStr, phyRxBeginStr;

    phyStateStr << "/NodeList/*/DeviceList/*/Phy/State/State";
    phyRxBeginStr << "/NodeList/*/DeviceList/*/Phy/PhyRxPayloadBeginWithPacket";
    PhyTxPsduBeginStr << "/NodeList/*/DeviceList/*/Phy/PhyTxPsduBegin";

    Config::Connect (phyStateStr.str(), MakeCallback (&PhyStateTrace));
    Config::Connect (PhyTxPsduBeginStr.str(), MakeCallback (&PhyTxPsduBegin));
    Config::Connect (phyRxBeginStr.str(), MakeCallback (&PhyRxPayloadBegin));
  }


  if (enableEnergyTrace)
  {
    /** connect trace sources **/
    /***************************************************************************/
    //Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/WifiRadioEnergyModel/TotalEnergyConsumption", MakeCallback (&TotalStaEnergy));
    
    Simulator::Schedule(Seconds(1.0), &PrintEnergyConsumption, AllNodes);
    /***************************************************************************/
  }

  //Simulator::Schedule(Seconds(5.0), &callbackfunctions, wifiHelper);
  // If flowmon is needed
  // FlowMonitor setup
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor;
  if (enableFlowMon)
  {
    flowmon.SetMonitorAttribute("StartTime", TimeValue(Seconds (flowMonStartTime_s)));
    monitor = flowmon.InstallAll();
  }

  NS_LOG_INFO("== Run simulation ==");
  /* Start Simulation */
  Simulator::Stop (Seconds (simulationTime + 2));

  ShowProgress spinner(Seconds(10));
  spinner.SetVerbose(false);

  Simulator::Run ();


  // create a map from STA mac address to the pair
  std::map<Mac48Address, std::pair<double, double>> staMacToTotalBitsUplinkDownlink;
  std::map<Mac48Address, std::pair<double, double>> staMacToUplinkDownlinkThroughput_kbps;
  std::map<Mac48Address, std::pair<double, double>> staMacToUplinkDownlinkLatency_usPerPkt;
  // Initialize for all existing STAs with <0,0>
  for (uint32_t i = 0; i < StaNodes.GetN(); i++)
  {
    Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice> (StaNodes.Get (i)->GetDevice (0)); //assuming only one device
    Ptr<WifiMac> wifi_mac = wifi_dev->GetMac ();
    Ptr<StaWifiMac> sta_mac = DynamicCast<StaWifiMac> (wifi_mac);
    staMacToTotalBitsUplinkDownlink[sta_mac->GetAddress ()] = std::make_pair (0, 0);
    staMacToUplinkDownlinkThroughput_kbps[sta_mac->GetAddress ()] = std::make_pair (0, 0);
    staMacToUplinkDownlinkLatency_usPerPkt[sta_mac->GetAddress ()] = std::make_pair (0, 0);
  }
  // Log file for flow stats - goodput and latency
  double *avgDelay_us;
  Histogram *delayHist;
  double sum_throughput_at_sta = 0;
  double sum_throughput_at_ap = 0;

  if (enableFlowMon)
  {
    std::cout<<"\n-----------------\n";
    std::cout<<"\nFlow Level Stats:\n";
    std::cout<<"-----------------\n\n";

    std::cout<<"STA distance from AP:\n\n";

    for (const auto & [key, value] : StaDis)
        std::cout <<"["<<key<<"] , " << value << "\n";
    std::cout<<"-----------------\n\n";

    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
    // find size of stats
    size_t flowCount = stats.size();  // number of flows
    avgDelay_us = new double [flowCount];
    delayHist = new Histogram [flowCount];
    double sum_uplink_delay = 0;
    double sum_downlink_delay = 0;
    double sum_uplink_packet_lost = 0;
    double sum_downlink_packet_lost = 0;
    double sum_downlink_tx_packet = 0;
    double sum_uplink_tx_packet = 0;
    int counter = 0;
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
      
      double totalBitsRx = i->second.rxBytes * 8.0;
      double totalBitsTx = i->second.txBytes * 8.0;
      double throughputKbps =  i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds())/1000;
      double avgDelayMicroSPerPkt = i->second.delaySum.GetMicroSeconds()/((i->second.rxPackets)+1) ;
      //std::cout << "avgDelayMicroSPerPkt: " <<  avgDelayMicroSPerPkt << std::endl;
      double lostPackets = 0 ;
      lostPackets = (i->second.txBytes - i->second.rxBytes) / payloadSize;;
      double txPackets = (i->second.txPackets);
      avgDelay_us[counter] = avgDelayMicroSPerPkt;
      delayHist[counter] = i->second.delayHistogram;
      if (t.destinationAddress ==  apInterface.GetAddress (0) && t.destinationPort == 50000)
      {
        std::cout <<std::setw(30)<<std::left <<"Flow ID" <<":\t"<<i->first<< " -> UpLink Data stream\n";
        sum_uplink_delay += avgDelayMicroSPerPkt ;          
        sum_uplink_packet_lost += lostPackets;
        sum_uplink_tx_packet += txPackets;
        sum_throughput_at_ap += throughputKbps;
      }
      else if (t.sourceAddress ==  apInterface.GetAddress (0) && t.sourcePort == 50000)
      {
        std::cout <<std::setw(30)<<std::left <<"Flow ID" <<":\t"<<i->first<< " -> UpLink Control stream\n";
      }
      else
      {
        for (uint32_t nsta =0 ; nsta < StaNodes.GetN() ; nsta ++)
        {
          if (t.destinationAddress ==  staInterface.GetAddress (nsta) && t.destinationPort == 10)
          {
            std::cout <<std::setw(30)<<std::left <<"Flow ID" <<":\t"<<i->first<< " -> DownLink Data stream\n";
            sum_downlink_delay += avgDelayMicroSPerPkt;
            sum_downlink_tx_packet += txPackets;
            sum_downlink_packet_lost += lostPackets;
            sum_throughput_at_sta += throughputKbps;
          }
          else if (t.sourceAddress ==  staInterface.GetAddress (nsta) && t.sourcePort == 10)
          {
            std::cout <<std::setw(30)<<std::left <<"Flow ID" <<":\t"<<i->first<< " -> DownLink Control stream\n";
          }
        }
      }
      std::cout<< std::setw(30)<<std::left << "Source IP and Port" <<":\t"<<t.sourceAddress<<" , "<<t.sourcePort<<"\n";
      // if sourceAddress is found in ipToMac map, then print the corresponding MAC address
      if (ipToMac.find(t.sourceAddress) != ipToMac.end())
      {
        std::cout<< std::setw(30)<<std::left << "Source STA MAC" <<":\t"<<ipToMac[t.sourceAddress]<<"\n";
        // Add totalBitsRx to Uplink of this STA in staMacToTotalBitsUplinkDownlink
        staMacToTotalBitsUplinkDownlink[ipToMac[t.sourceAddress]].first += totalBitsRx;
        staMacToUplinkDownlinkThroughput_kbps[ipToMac[t.sourceAddress]].first = throughputKbps;
        staMacToUplinkDownlinkLatency_usPerPkt[ipToMac[t.sourceAddress]].first = avgDelayMicroSPerPkt;
      }
      std::cout<< std::setw(30)<<std::left << "Destination IP and Port" <<":\t"<<t.destinationAddress<<" , "<<t.destinationPort<<"\n";
      if (ipToMac.find(t.destinationAddress) != ipToMac.end())
      {
        std::cout<< std::setw(30)<<std::left << "Destination STA MAC" <<":\t"<<ipToMac[t.destinationAddress]<<"\n";
        // Add totalBitsRx to Uplink of this STA in staMacToTotalBitsUplinkDownlink
        staMacToTotalBitsUplinkDownlink[ipToMac[t.destinationAddress]].second += totalBitsRx;
        staMacToUplinkDownlinkThroughput_kbps[ipToMac[t.destinationAddress]].second = throughputKbps;
        staMacToUplinkDownlinkLatency_usPerPkt[ipToMac[t.destinationAddress]].second = avgDelayMicroSPerPkt;
      }
      std::cout<< std::setw(30)<<std::left << "Throughput (kbps)" <<":\t"<< throughputKbps<<"\n";
      std::cout<< std::setw(30)<<std::left << "Total bits received" <<":\t"<<totalBitsRx<<"\n";
      std::cout<< std::setw(30)<<std::left << "Total bits sent" <<":\t"<<totalBitsTx<<"\n";
      //lostPackets = totalBitsRx/totalBitsRx;
      std::cout<< std::setw(30)<<std::left << "Avg. Delay ( us/pkt)" <<":\t"<< avgDelayMicroSPerPkt << " us/pkt\n";
      std::cout<< std::setw(30)<<std::left << "Lost Packets" <<":\t"<< lostPackets << " pkts\n";
      
      // std::cout<<"\n\n";
      std::cout<<"-----------------\n\n";
    }
    // flowmon ------------------------------------------------- 

    avr_ul_pkt_los= (sum_uplink_packet_lost / sum_uplink_tx_packet)*100 ; // packet loss in percent: (lost packets / tx packet) *100
    avr_dl_pkt_los= (sum_downlink_packet_lost / sum_downlink_tx_packet)*100 ;
    
    avr_ul_dly = (sum_uplink_delay  / (StaCount * 1000 ));
    avr_dl_dly = (sum_downlink_delay  / (StaCount * 1000 ));
  }
  for (DeviceEnergyModelContainer::Iterator iter = STAdeviceModels.Begin(); iter != STAdeviceModels.End(); iter++)
  {
    average_sta_energy += (*iter)->GetTotalEnergyConsumption();
    //NS_ASSERT(energyConsumed <= 0.1);
    NS_LOG_UNCOND("End of simulation ("<< Simulator::Now().GetSeconds()
    << "s) Total energy consumed by STA radio = " << (*iter)->GetTotalEnergyConsumption() << "J");
  }
  for (DeviceEnergyModelContainer::Iterator iter = APdeviceModels.Begin(); iter != APdeviceModels.End(); iter++)
  {
    ap_energy = (*iter)->GetTotalEnergyConsumption();
    NS_LOG_UNCOND("End of simulation ("
                  << Simulator::Now().GetSeconds()
                  << "s) Total energy consumed by AP radio = " << ap_energy << "J");
  }
  Simulator::Destroy ();


  for (uint32_t out = 1 ; out <= StaCount+1 ; out ++)
  {
    //node 0 is the remote server , node 1 is the AP so the STAs start from node id 2
    average_sta_tx_signaling += (TxsigSum[out]);
    average_sta_rx_signaling += (RxsigSum[out]);
  }
  average_sta_tx_signaling = average_sta_tx_signaling/StaCount;
  average_sta_rx_signaling = average_sta_rx_signaling/StaCount;
 
  ap_ovrhd_energy = ((TxsigSum [0]) * 0.668 * 12.2) + ((RxsigSum[0]) * 0.568 *12.2);
  //average overhead energy at STAs = overhead tx time * power at tx mode + overhead rx time * power at rx mode
  sta_ovrhd_energy = (average_sta_tx_signaling * 0.801 * 4.9) + (average_sta_rx_signaling * 0.515 * 4.9);
  average_sta_throughput = (sum_throughput_at_sta) / (StaCount);
  double ap_throughput= sum_throughput_at_ap / StaCount;


  TH << link << ", " << 4 << ", " << traffic << ", " << udp << ", " << packetsPerSecond << ", " << StaCount; for (const auto & [key, value] : StaDis){TH<<", " << value ;} TH <<", " << average_sta_throughput/1000<< ", " << ap_throughput/1000<< std::endl;
  PcktLoss << link << ", " << 4 << ", " << traffic << ", " << udp << ", " << packetsPerSecond << ", " << StaCount; for (const auto & [key, value] : StaDis){PcktLoss<<", " << value;} PcktLoss << ", " << avr_ul_pkt_los<< ", " << avr_dl_pkt_los<< std::endl;
  DLY << link << ", " << 4 << ", " << traffic << ", " << udp << ", " << packetsPerSecond << ", " << StaCount ; for (const auto & [key, value] : StaDis){ DLY <<", " << value;} DLY << ", " << avr_ul_dly<< ", " << avr_dl_dly<< std::endl;
  average_sta_energy = average_sta_energy/StaCount;
  NRG << link << ", " << 4 << ", " << traffic << ", " << udp << ", " <<  packetsPerSecond << ", " << StaCount ; for (const auto & [key, value] : StaDis){NRG <<", " << value;} NRG  << ", " << average_sta_energy<< ", " << ap_energy<< std::endl;
  ovr_NRG << link << ", " << 4 << ", " << traffic << ", " << udp << ", " << packetsPerSecond << ", " << StaCount ; for (const auto & [key, value] : StaDis){ovr_NRG <<", " << value;} ovr_NRG << ", " << sta_ovrhd_energy<< ", " << ap_ovrhd_energy<< std::endl;
  NRG_trend << link << ", " << 4 << ", " << traffic << ", " << udp << ", " << packetsPerSecond << ", " << StaCount ; for (const auto & [key, value] : StaDis){NRG_trend <<", " << value;} for (const auto & [key, value] : StaEnr){NRG_trend <<", " <<key <<", " << value;} for (const auto & [key, value] : ApEnr){NRG_trend <<", " <<key <<", " << value;} NRG_trend << std::endl;

  allTxtime.clear(); //all transsmission time
  allRxtime.clear(); //all reception time
  allIdletime.clear(); //all idle time
  allCCA_BUSYtime.clear(); //all cca busy time
  allSleeptime.clear(); //all cca busy time
  Txsum.clear(); //data transsmission time
  Rxsum.clear(); //data reception time
  TxsigSum.clear(); //signaling transsmission time
  RxsigSum.clear(); //signaling reception time
  StaDis.clear(); // Node positions
  ApEnr.clear(); // Ap Energy
  StaEnr.clear(); // Sta Energy
}
