#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/netsimulyzer-module.h"
#include "ns3/netsimulyzer-3D-models.h"

#include "ns3/config-store-module.h"
#include "ns3/energy-module.h"

#include "ns3/aodv-module.h"
#include "ns3/aodv-helper.h"
#include "ns3/netanim-module.h"

#include "ns3/gnuplot.h"

#include <iostream>
#include <math.h>
#include <cmath>
#include <list> 
#include <string.h>  

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("first-ns3");

/* *************** Set the global variables *************** */
uint32_t packetSize = 512; // bytes //the packet Size
uint32_t headerSize = 25; //bytes // the header Size
int numPackets = 5000; //the number of packets to generate
//double interval = 1; // seconds //The interval between two packetS
double simulationTime = 360.0; // 1000 seconds, Simualation time

double initEnergy = 200000; //(200000) 3600 * 80;//initial drone energy in Joules (1 watt hour (W) = 3,600.00 joules (J)) 80wh = 80*3600=288000j

const double maxRange = 250; //350.0; //meters, the transmission range of nodes
const double maxCRange = 100; //50.0; //meters, the coverage range of nodes

double area = 5000;//area size
double areaX = 5000;//area size
double areaY = 50;//area size
//double areaZ = 100;//area size

double p = sqrt((maxRange*maxRange)/2);
int Nb_SubArea = round (areaX * areaY)/(4 * p * p);

double BsX = areaX/Nb_SubArea; //-20.0; //500;
double BsY = areaY/Nb_SubArea; //750.0; //500;
Vector sinkPosition = Vector (BsX, BsY, 0.0);

const double idlePx = 0.0001;//Js, Energy consumption on idle
const double idlePxCH = 0.001;//Js, Energy consumption on idle for CH ???????????????
const double idleFly = 10;//269.48;//27.35; //38.4 W or Js, Energy consumption on flying mode
//const double idleFly = idlePxCH * 2;
//const double idleHover = 400.88;//38.35; //27.35 W or Js, Energy consumption on hovering mode

uint32_t nWifi = 40;  // 40 nodeS
vector<double> aete;

int nn = nWifi; 
double interval = (nWifi / (nWifi*0.1))*2; //The interval between two rounds

NodeContainer Nodes;
NetDeviceContainer Devices;
TypeId tid; 
list<int> chList;

const int xmin = 4;
const int xmax = 8;

double timing = 0;

long double ra_adv = ((headerSize + 4)*8)/(1e6); // Time needed to transfer the advertisment by a ch =(nbrbytes*8)/radio_speed;
double ra_adv_total = ra_adv*((nWifi*0.1)*4+1); // RA Time (s) for nodes' join reqs
double ra_join = 0.01*nWifi; // Buffer time for join

// ---- NetSimulyzer ----
auto orchestrator = CreateObject<netsimulyzer::Orchestrator> ("first-ns3.json");

bool m_show_plots = false;//show plot or not
string file_EXT=".txt";

ofstream output;

string Our_Proposition_energy = "Our_Proposition_Energy";
string Our_Proposition_alive = "Our_Proposition_Alive";

string simple_area_division_energy = "SAD_Energy";
string simple_area_division_alive = "SAD_Alive";

string CBLADSR_energy = "CBLADSR_Energy";
string CBLADSR_alive = "CBLADSR_Alive";

int ENERGY_CONSUMPTION = 1;
int CH_LIFE_TIME = 2;
uint32_t Plot_max_X = 0 ;
double Plot_max_Y = 0;

double chch = 0;

bool show_log = false;
bool delevery_log = false;

bool select1 = false;
bool select2 = false;
bool select3 = false;
bool select4 = false;
bool delevery = false;

bool calcAvrage = false;

/* ********** Types of packets *********** */
#define WORK   1
#define SELECTION   2
#define ADVERTISE   3
#define INFO   4
#define DELEVERY   5

/* ******************************************************************************************* */
                 /****************** functions declarations  ****************/
/* ******************************************************************************************* */
/****** setup and configuration functions ************/
void NodesCreation();// creat the nodes
void WifiConfig(bool verbose);// configure the communication model
void EnergyInit();// configure the energy model
void ProtocolInit();// configure the protocol model
void MobilityInit();// configure the mobility model
void ConsumeIdle();// idle consumption function
void ConsumeIdle2();// mobility./flying consumption function

void testCommunicate ();

void BroadcastingADVPacket (int CHID); // Broadcasting ADV packet methods

/***** info functions ****************/
double RandDouble(double nMin, double nMax);//generate random numbers
int RandInt(int nMin, int nMax);


double PercentOf(double percent, double of);//get the percentage percent of a value of

/*********** statistics functions **************/
double ComputedEnergyConsumption (); //used to calculate the total energy consumed
double checkAlive ();
void saveResult (double result);
void saveResultAlive (double result);
void ShowPlot ();
void ShowPlotAlive ();

/*************************************************************************************** The main function *******************************************************************************************/

/* In this scenario the network is devided into n clusters 
* every cluster head is selected at the initialization
* and then in the maintnance phase cluster heads are selected using
* the fitness function.
*/

int main (int argc, char *argv[])
{
  srand((unsigned) time(0));
  std::string phyMode ("DsssRate1Mbps");
  //LogComponentEnable("AodvRoutingProtocol", LOG_LEVEL_ALL);
  bool verbose = false;
  
  bool tracing = true;

  CommandLine cmd;
  cmd.AddValue ("nWifi", "Number of wifi devices", nWifi);
  cmd.AddValue ("initEnergy", "initial drone energy", initEnergy);
  cmd.AddValue ("verbose", "Tell echo applications to log if true", verbose);
  cmd.AddValue ("show", "show plot", m_show_plots);
  cmd.AddValue ("tracing", "Enable pcap tracing", tracing);
  cmd.AddValue ("calcAvrage","calculate the average", calcAvrage);

  cmd.Parse (argc,argv);

  remove("first-ns3_Output.txt");
  output.open ("first-ns3_Output.txt", ios_base::app | ios_base::out);

  /* *************** Buildings creation *************** */
  //BuildingCreation();
  
  /* *************** Nodes creation *************** */
  NodesCreation();

  /* *************** The Wifi channel configuration *************** */
  WifiConfig(verbose);

  /* *************** Energy Model initialization *************** */
  EnergyInit();

  /* ************** Internet Protocol Initialization ************* */
  ProtocolInit();


  /* *************** mobility initialization *************** */
  MobilityInit ();
  
  /* ************** Clusters formation ************* */
  testCommunicate ();

  /* ***************** Idle consumption scheduling *************** */
  Simulator:: Schedule (Seconds(0.1), &ConsumeIdle);
  Simulator:: Schedule (Seconds(0.1), &ConsumeIdle2);

  /* *************** NetAnim config *************** */
  AnimationInterface anim ("first-ns3.xml");
  
  for (size_t j=0; j < nWifi+1; j++)
  {
      //Ptr<BlackHoleCircleMobilityModelel> mobility = Nodes.Get(j)->GetObject<BlackHoleCircleMobilityModel> ();
      double x =  Nodes.Get(j)->GetObject<GaussMarkovMobilityModel> ()->GetPosition ().x;
      double y =  Nodes.Get(j)->GetObject<GaussMarkovMobilityModel> ()->GetPosition ().y;
      double z =  Nodes.Get(j)->GetObject<GaussMarkovMobilityModel> ()->GetPosition ().z;
      Ptr<Node> node = Nodes.Get(j);
      anim.UpdateNodeColor (node, 60+j, 70+j, 80+j);
      anim.SetConstantPosition (Nodes.Get(j), x, y, z);
  }

  Simulator::Stop (Seconds (simulationTime));

  cout <<"Starting simulation \n";
  Simulator::Run ();
  Simulator::Destroy ();
  cout << "Simulation finished !!!!! \n";
  
  
  /************************ Energy Consumption Plot ***********************/  
  saveResult(ComputedEnergyConsumption());
  if(m_show_plots){
    ShowPlot();
  }
  
  /*********************** Checkalive **********************************/
  saveResultAlive(checkAlive());
  if(m_show_plots){
    ShowPlotAlive();
  }

    return 0;
}

/*********** functions code **************/

/*********** percentage calculation ***********************/
double PercentOf(double percent,double of){
  return percent * of / 100;
}

/************** true random *****************/
double RandDouble(double nMin, double nMax)
{
  double randNum = nMin + ((double)rand() / (((double) RAND_MAX)+1) * (nMax-nMin));
  return randNum;
}

/************** int Random ******************/
int RandInt(int nMin, int nMax)
{
  int randNum = RandDouble(nMin, nMax);
  return (int) randNum;
}

/* *************** Building creation *************** */
void BuildingCreation(){
  cout << "Buildings creation \n";

  BuildingContainer buildings;

  Ptr<Building> simpleBuilding = CreateObject<Building> ();
  simpleBuilding->SetBoundaries ({50.0, 30.0, 0.0, 50.0, 0.0, 20.0});
  buildings.Add (simpleBuilding);

  Ptr<Building> simpleBuilding2 = CreateObject<Building> ();
  simpleBuilding2->SetBoundaries ({950.0, 930.0, 750.0, 730.0, 0.0, 20.0});
  buildings.Add (simpleBuilding2);

  Ptr<Building> twoFloorBuilding = CreateObject<Building> ();
  twoFloorBuilding->SetBoundaries ({150.0, 130.0, 150.0, 130.0, 0.0, 40.0});
  twoFloorBuilding->SetNFloors (2);
  buildings.Add (twoFloorBuilding);

  Ptr<Building> threeFloorBuilding = CreateObject<Building> ();
  threeFloorBuilding->SetBoundaries ({250.0, 230.0, 250.0, 230.0, 0.0, 60.0});
  threeFloorBuilding->SetNFloors (3);
  buildings.Add (threeFloorBuilding);

  Ptr<Building> threeFloorBuilding2 = CreateObject<Building> ();
  threeFloorBuilding2->SetBoundaries ({450.0, 430.0, 650.0, 630.0, 0.0, 60.0});
  threeFloorBuilding2->SetNFloors (3);
  buildings.Add (threeFloorBuilding2);

  // Use helper to configure buildings and export them
  netsimulyzer::BuildingConfigurationHelper buildingHelper{orchestrator};

  buildingHelper.Install(buildings);

}

/* *************** Nodes creation *************** */
void NodesCreation(){
  cout << "Nodes creation \n";
  Nodes.Create (nWifi+1);
}

/* *************** The Wifi channel configuration *************** */
void WifiConfig(bool verbose){
  cout << "Wifi channel configuration \n";

  WifiHelper wifi;
  if (verbose)
  {
	  wifi.EnableLogComponents ();  // Turn on all Wifi logging
  }

  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue ("OfdmRate54Mbps"));

  YansWifiPhyHelper phy ;//= YansWifiPhyHelper::Default (); //For wireleSS communication, uSe the YanSWifiPhyHelper
  phy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO); // ns-3 supports RadioTap and Prism tracing extensions for 802.11
  Config::SetDefault( "ns3::RangePropagationLossModel::MaxRange", DoubleValue (maxRange) );
  YansWifiChannelHelper channel = YansWifiChannelHelper ();  
  channel.AddPropagationLoss ("ns3::RangePropagationLossModel");
  channel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  phy.SetChannel (channel.Create ());

  Ssid ssid = Ssid ("ns-3-ssid");
  WifiMacHelper mac;
  mac.SetType ("ns3::AdhocWifiMac");
  
  Devices = wifi.Install (phy, mac, Nodes);

  /* *************** Pcap tracing *************** */
  /* To enbable packet capture on each network device */
  // phy.EnablePcap ("first-ns3", Devices); 

  /* *************** Ascii tracing *************** */
  AsciiTraceHelper ascii;
  phy.EnableAsciiAll(ascii.CreateFileStream("first-ns3.tr"));
}

/* *************** Energy Model initialization *************** */
void EnergyInit(){
  for (uint32_t i = 0; i < nWifi; i++)
  {
    Nodes.Get (i)->SetInitialEnergy(initEnergy);
    Nodes.Get (i)->SetIdlePx(idlePx);
  }
  Nodes.Get(nWifi)->SetInitialEnergy(initEnergy*100);
  Nodes.Get(nWifi)->SetIdlePx(0);
}

/* *************** protocol stack *************** */
void ProtocolInit(){
  cout << "Internet protocol stack installation \n";
  //AodvHelper aodv;// you can configure AODV attributes here using aodv.Set(name, value)
  InternetStackHelper stack;
  //stack.SetRoutingHelper (aodv); // has effect on the next Install ()
  stack.Install (Nodes);

  Ipv4AddressHelper address;
  address.SetBase ("192.168.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interface = address.Assign (Devices);
  tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
}

/* *************** Mobility initialization ****************** */
void MobilityInit ()
{
  cout << "Mobility helper configuration \n";
  for (int i = 0; i<(int)nWifi; i++)
  {
      Ptr<Node> node = Nodes.Get (i);
      MobilityHelper mobility;
      string x = "ns3::UniformRandomVariable[Min=" + to_string(0) + "|Max=" + to_string(areaX) + "]";
      string y = "ns3::UniformRandomVariable[Min=" + to_string(0) + "|Max=" + to_string(areaY) + "]";
      string z = "ns3::UniformRandomVariable[Min=" + to_string(70) + "|Max=" + to_string(areaZ) + "]";

      mobility.SetPositionAllocator ("ns3::RandomBoxPositionAllocator",
        "X", StringValue (x),
        "Y", StringValue (y),
        "Z", StringValue (z));

      mobility.SetMobilityModel ("ns3::GaussMarkovMobilityModel",
        "Bounds", BoxValue (Box (0, areaX, 0, areaY, 70, areaZ)),
        "TimeStep", TimeValue (Seconds (0.5)),
        "Alpha", DoubleValue (0.5),
        "MeanVelocity", StringValue ("ns3::UniformRandomVariable[Min=80|Max=120]"),
        "MeanDirection", StringValue ("ns3::UniformRandomVariable[Min=0|Max=6.283185307]"),
        "MeanPitch", StringValue ("ns3::UniformRandomVariable[Min=0.05|Max=0.05]"),
        "NormalVelocity", StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=0.0|Bound=0.0]"),
        "NormalDirection", StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=0.2|Bound=0.4]"),
        "NormalPitch", StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=0.02|Bound=0.04]"));

      mobility.Install (node);
   }
     
  //sink location
  Ptr<Node> sink = Nodes.Get (nWifi);
  
  if (!(sink-> GetGrp ()).empty ())
  {
      sink->GetGrp ().clear();
  }
  
  MobilityHelper mobility_sink;

  string x = "ns3::UniformRandomVariable[Min=" + to_string(sinkPosition.x) + "|Max=" + to_string(sinkPosition.x) + "]";
  string y = "ns3::UniformRandomVariable[Min=" + to_string(sinkPosition.y) + "|Max=" + to_string(sinkPosition.y) + "]";
  string z = "ns3::UniformRandomVariable[Min=" + to_string(0) + "|Max=" + to_string(0) + "]";

  mobility_sink.SetPositionAllocator ("ns3::RandomBoxPositionAllocator",
    "X", StringValue (x),
    "Y", StringValue (y),
    "Z", StringValue (z));


  mobility_sink.SetMobilityModel ("ns3::GaussMarkovMobilityModel",
    "Bounds", BoxValue (Box (sinkPosition.x, sinkPosition.x, sinkPosition.y, sinkPosition.y, 0, 0)),
    "TimeStep", TimeValue (Seconds (0.5)),
    "Alpha", DoubleValue (0.5),
    "MeanVelocity", StringValue ("ns3::UniformRandomVariable[Min=0|Max=0]"),
    "MeanDirection", StringValue ("ns3::UniformRandomVariable[Min=0|Max=0]"),
    "MeanPitch", StringValue ("ns3::UniformRandomVariable[Min=0|Max=0]"),
    "NormalVelocity", StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=0.0|Bound=0.0]"),
    "NormalDirection", StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=0.0|Bound=0.0]"),
    "NormalPitch", StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=0.0|Bound=0.0]"));

  mobility_sink.Install (sink);
  
  // Use helper to define model for visualizing nodes and aggregate to Node object
  netsimulyzer::NodeConfigurationHelper nodeHelper(orchestrator);
  string dronemodel = netsimulyzer::models::QUADCOPTER_UAV;
  nodeHelper.Set ("Model", StringValue(dronemodel));
  nodeHelper.Install (Nodes);

  auto decoration1 = CreateObject<netsimulyzer::Decoration>(orchestrator);
  decoration1->SetAttribute ("Model", StringValue("HouseAndRoad.obj"));
  decoration1->SetPosition ({0,0,0});
  decoration1->SetAttribute ("Scale" , DoubleValue (2));
  // decoration->SetAttribute ("Height", OptionalValue<double> (100));
}

/***************** idle consumption call *******************************/
void ConsumeIdle(){
  for (size_t i = 0; i < nWifi; i++)
  {
    Nodes.Get(i)->NotifyIdle();
  }
  Simulator:: Schedule (Seconds(0.1), &ConsumeIdle);
}

void ConsumeIdle2(){
  for (size_t i = 0; i < nWifi; i++)
  {
       /*double thrustf = pow (0.5*9.80665,1.5);
       double f = pow (2*1.2*(0.2*0.4), 0.5);
       Nodes.Get (i)->NotifyIdle2 (thrustf/f);*/
       Nodes.Get (i)->NotifyIdle2 (idleFly);
  }
  Simulator:: Schedule (Seconds(0.1), &ConsumeIdle2);
  //Simulator:: Schedule (Seconds(interval), &ConsumeIdle2);
}

/**************** total energy consumed ********************/
double ComputedEnergyConsumption(){
  double energy_consumed = 0;
    for (uint32_t i = 0; i <nWifi; i++){
       //cout<<"Node :: "<<i<<" Total energy :: "<<Nodes.Get(i)->GetTotalEnergy()<<endl;
       energy_consumed += Nodes.Get(i)->GetTotalEnergy();
    }

    return energy_consumed;
}


/* #########################################################################################################*/
 /* ******************** ::::::::::::::::::  Communication test  ::::::::::::::::::: ********************** */
/* ########################################################################################################*/
void testCommunicate ()
{

  /*************** :::::::::: Broadcasting the ADV packet ::::::::::: ******************/
  cout<<"Broadcasting the ADV packet "<<endl;
  for (int i=0 ; i<(int)nWifi ;i++)
  {
      Ptr<Node> ch = Nodes.Get (i);
      double rand = RandDouble (0, ra_adv);
      double now = (Simulator::Now ()).GetSeconds ();  
  
      Simulator::Schedule(Seconds(now + 1 + rand), &BroadcastingADVPacket, i);
  }
}

/*###################################################################################################*/
/* ***************** :::::::::::::: Sending the ADV packet methods:::::::::::::: ****************** */
/*##################################################################################################*/
void ReceiveADVPacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address senderAd;
  Ptr<Node> receiver = socket->GetNode ();

  while (packet = socket->RecvFrom (senderAd))
  { 
        if (receiver->GetIsCH () == 0)
        {
           receiver->SetIsCH(false);
           receiver->SetIdlePx(idlePx);
           receiver->NotifyRx(packetSize*8);
           uint8_t *buffer_temp=new uint8_t[2];
           packet->CopyData(buffer_temp,1);
           int ch = int(*buffer_temp);
           output << "Node " << receiver->GetId() << " received ADV packet from node "<<ch<<" at "<<(Simulator::Now ()).GetSeconds ()<<"\n";   
        }
  }
}

static void GenerateADVTraffic (Ptr<Socket> socket, int msg)
{
  uint8_t CHID = socket->GetNode ()->GetId ();
  uint8_t buffer[1] = {CHID};
  Ptr<Packet> pkt1 = Create<Packet> (reinterpret_cast<const uint8_t*> (buffer), 1);
  socket->GetNode ()->NotifyTx (maxRange,packetSize*8);
  socket->Send (pkt1);
  output << "Node " << (int)CHID << " broadcasting ADV packet at: "<< (Simulator::Now ()).GetSeconds () << " \n";
}

void BroadcastingADVPacket (int CHID)
{
   Ptr<Node> CH = Nodes.Get(CHID);
   uint16_t port = 50000;
   // 1. Define a broadcast address:
   static const InetSocketAddress Broadcast = InetSocketAddress(Ipv4Address("255.255.255.255"), port);

   //2. On each node to receive :
   for (uint32_t j=0; j<nWifi; j++)
   {
        Ptr<Socket> recvSink = Socket::CreateSocket (Nodes.Get (j), tid); //The Destination (receiver) ;
        InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), port);
        recvSink->Bind (local);
        recvSink->SetRecvCallback (MakeCallback (&ReceiveADVPacket));//on receive callback
    }
    
    //3. source node:
    Ptr<Socket> source = Socket::CreateSocket (CH, tid); //The sender (source) ;
    source->SetAllowBroadcast (true);
    source->Connect (Broadcast);

    int msg = CHID;

    Simulator::ScheduleWithContext (source->GetNode ()->GetId (), Seconds(0), &GenerateADVTraffic, source, msg);
}
/* ##################################################################################################*/



/****************************************************************************************************/
/*********************************** ::::::::::: Results :::::::::::: *******************************/
/****************************************************************************************************/

/*##########*/
double ReadResultFile(string fileName){
  double average = 0;  
  double result = 0;
  double counter = 0;

  string line;
  ifstream file;
  file.open (fileName);
  if (!file.is_open()) 
    return 0;

  string word;

  while(!file.eof())
  {
      getline(file, line);
      if(line.empty())
          continue;

      istringstream iss(line);
      string word1; 
      string word2; 

      iss >> word1;
      iss >> word2;

      result = stod(word2);

      if(result == -1)
        result = 240;

      average+=result;
      counter++;     
  }

  if(!m_show_plots)
    remove(fileName.c_str());       

  return (average/counter);
}
/*#############*/

/********************** Read File  ****************************/
Gnuplot2dDataset readFile(Gnuplot2dDataset dataset,string fileName)
{   
    double x;
    double y;

    string line;
    ifstream file;
    file.open (fileName);
    if (!file.is_open()) return dataset;

    string word;
  
    while(!file.eof())
    {
        getline(file, line);
        if(line.empty())
            continue;
        istringstream iss(line);
        string word1; 
        string word2; 

        iss >> word1;
        iss >> word2;

        cout<<" x : " << word1 << " y : "<<word2 <<endl; 

        x = stod(word1);
        y = stod(word2);
        dataset.Add (x, y);

        if(Plot_max_X < x)
          Plot_max_X = x;

        if(Plot_max_Y < y)
          Plot_max_Y = y;            
    }
  return dataset;
}

/***********************  Export plot of EnergyConsumption  ***************************************/
void ShowPlot ()
{
  string fileNameWithNoExtension = Our_Proposition_energy; // "Our_Proposition_Energy";

  string graphicsFileName        = fileNameWithNoExtension + ".png";
  string plotFileName            = fileNameWithNoExtension + ".plt";
  //string plotTitle               = "Energy consumption comparaison";
  string dataTitle               = "SAD-CA";
  string dataTitle2               = "CBLADSR";
  string dataTitle3               = "Our Proposition";
 
  // Instantiate the plot and set its title.
  Gnuplot plot (graphicsFileName);
  //plot.SetTitle (plotTitle);

  // Make the graphics file, which the plot file will create when it
  // is used with Gnuplot, be a PNG file.
  plot.SetTerminal ("png");

  // Set the labels for each axis.
  plot.SetLegend ("Nodes", "Energy Consumption(J)");

 
  // Instantiate the dataset, set its title, and make the points be
  // plotted along with connecting lines.
  Gnuplot2dDataset dataset;
  Gnuplot2dDataset dataset2;
  Gnuplot2dDataset dataset3;
  dataset.SetTitle (dataTitle);
  dataset2.SetTitle(dataTitle2);
  dataset3.SetTitle(dataTitle3);
  dataset.SetStyle (Gnuplot2dDataset::LINES_POINTS);
  dataset2.SetStyle(Gnuplot2dDataset::LINES_POINTS);
  dataset3.SetStyle(Gnuplot2dDataset::LINES_POINTS);


  dataset = readFile(dataset,simple_area_division_energy+file_EXT);
  dataset2 = readFile(dataset2,CBLADSR_energy+file_EXT);
  dataset3 = readFile(dataset3,Our_Proposition_energy+file_EXT);

 // Set the range for the x & y axis.
  plot.AppendExtra ("set xrange [0:"+to_string(Plot_max_X+Plot_max_X*0.1)+"]");
  plot.AppendExtra ("set yrange [0:"+to_string(Plot_max_Y+Plot_max_Y*0.1)+"]");

  // Add the dataset to the plot.  
  plot.AddDataset (dataset);
  plot.AddDataset(dataset2);
  plot.AddDataset(dataset3);

  // Open the plot file.
  std::ofstream plotFile (plotFileName.c_str());

  // Write the plot file.
  plot.GenerateOutput (plotFile);

  // Close the plot file.
  plotFile.close ();

  string plotGenerationCommand = "gnuplot "+plotFileName;
  system(plotGenerationCommand.c_str());

  string ShowResult = "eog "+graphicsFileName;
  cout<<" File Name : " <<ShowResult <<endl;
  system(ShowResult.c_str());
  Plot_max_X = 0;
  Plot_max_Y = 0;
}

/************************ save results of simulation in a file for energy ***********************/
void saveResult(double result){
  ofstream myfile;
  
  myfile.open (Our_Proposition_energy+"_temp"+file_EXT, ios_base::app | ios_base::out);
  myfile << nWifi<< "\t"<< result<< endl;
  myfile.close();

  if(calcAvrage){
    myfile.open (Our_Proposition_energy+file_EXT, ios_base::app | ios_base::out);
    myfile << nWifi<< "\t"<< ReadResultFile(Our_Proposition_energy+"_temp"+file_EXT)<< endl;
    myfile.close();
  }
}

          /* ####################################################*/
/* ###################  check alive ################## */
double checkAlive ()
{
   int nbdied = 0;
   for (int i = 0; i < (int)nWifi ; i++)
   {
      Ptr<Node> node = Nodes.Get (i);
      double remEner = node->GetRemainingEnergy ();
      //double t = node->GetTotalEnergy ();
      //if (t > initEnergy )
      if (remEner <= 0) //{
         nbdied ++;//cout<<"hereeeeeeeeeeeeee \n"; }
    }
    double alive = 100- ( (nbdied / (int)nWifi)*100);
    return alive;
}

/***********************  Export plot of check alive  ***************************************/
void ShowPlotAlive(){
  string fileNameWithNoExtension = Our_Proposition_alive; // ="Our_Proposition_Alive";

  string graphicsFileName        = fileNameWithNoExtension + ".png";
  string plotFileName            = fileNameWithNoExtension + ".plt";
  //string plotTitle               = "Average end-to-end delay";
  string dataTitle               = "SAD-CA";
  string dataTitle2              = "CBLADSR";
  string dataTitle3              = "Our Proposition";
 
  // Instantiate the plot and set its title.
  Gnuplot plot (graphicsFileName);
  //plot.SetTitle (plotTitle);

  // Make the graphics file, which the plot file will create when it
  // is used with Gnuplot, be a PNG file.
  plot.SetTerminal ("png");

  // Set the labels for each axis.
  plot.SetLegend ("Nodes", "Average of living nodes (%)");

  // Instantiate the dataset, set its title, and make the points be
  // plotted along with connecting lines.
  Gnuplot2dDataset dataset;
  Gnuplot2dDataset dataset2;
  Gnuplot2dDataset dataset3;
  dataset.SetTitle (dataTitle);
  dataset2.SetTitle(dataTitle2);
  dataset3.SetTitle(dataTitle3);
  dataset.SetStyle (Gnuplot2dDataset::LINES_POINTS);
  dataset2.SetStyle(Gnuplot2dDataset::LINES_POINTS);
  dataset3.SetStyle(Gnuplot2dDataset::LINES_POINTS);

  dataset = readFile(dataset,simple_area_division_alive+file_EXT);
  dataset2 = readFile(dataset2,CBLADSR_alive+file_EXT);
  dataset3 = readFile(dataset3,Our_Proposition_alive+file_EXT);

 // Set the range for the x & y axis.
  plot.AppendExtra ("set xrange [0:"+to_string(Plot_max_X+Plot_max_X*0.1)+"]");
  plot.AppendExtra ("set yrange [0:"+to_string(Plot_max_Y+Plot_max_Y*0.1)+"]");

  // Add the dataset to the plot.  
  plot.AddDataset (dataset);
  plot.AddDataset(dataset2);
  plot.AddDataset(dataset3);

  // Open the plot file.
  std::ofstream plotFile (plotFileName.c_str());

  // Write the plot file.
  plot.GenerateOutput (plotFile);

  // Close the plot file.
  plotFile.close ();

  string plotGenerationCommand = "gnuplot "+plotFileName;
  system(plotGenerationCommand.c_str());

  string ShowResult = "eog "+graphicsFileName;
  cout<<" File Name : " <<ShowResult <<endl;
  system(ShowResult.c_str());
  Plot_max_X = 0;
  Plot_max_Y = 0;
}

/************************ save results of simulation in a file for checkalive ***********************/
void saveResultAlive(double result){
  ofstream myfile;
  myfile.open (Our_Proposition_alive+"_temp"+file_EXT, ios_base::app | ios_base::out);
  myfile << nWifi<< "\t"<< result<< endl;
  myfile.close();

  if(calcAvrage){
    myfile.open (Our_Proposition_alive+file_EXT, ios_base::app | ios_base::out);
    myfile << nWifi<< "\t"<< ReadResultFile(Our_Proposition_alive+"_temp"+file_EXT)<< endl;
    myfile.close();
  }
}
