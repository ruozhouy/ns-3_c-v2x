/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
  This software was developed at the National Institute of Standards and
  Technology by employees of the Federal Government in the course of
  their official duties. Pursuant to titleElement 17 Section 105 of the United
  States Code this software is not subject to copyright protection and
  is in the public domain.
  NIST assumes no responsibility whatsoever for its use by other parties,
  and makes no guarantees, expressed or implied, about its quality,
  reliability, or any other characteristic.

  We would appreciate acknowledgement if the software is used.

  NIST ALLOWS FREE USE OF THIS SOFTWARE IN ITS "AS IS" CONDITION AND
  DISCLAIM ANY LIABILITY OF ANY KIND FOR ANY DAMAGES WHATSOEVER RESULTING
  FROM THE USE OF THIS SOFTWARE.

 * Modified by: Fabian Eckermann <fabian.eckermann@udo.edu> (CNI)
 *              Moritz Kahlert <moritz.kahlert@udo.edu> (CNI)
 */

#include "ns3/lte-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/lte-v2x-helper.h"
#include "ns3/config-store.h"
#include "ns3/lte-hex-grid-enb-topology-helper.h"
#include <ns3/buildings-helper.h>
#include <ns3/cni-urbanmicrocell-propagation-loss-model.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/spectrum-analyzer-helper.h>
#include <ns3/multi-model-spectrum-channel.h>
#include "ns3/ns2-mobility-helper.h"
#include <cfloat>
#include <sstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("v2x_communication_mode_4");

// Output 
std::string simtime = "log_simtime_v2x.csv"; 
std::string rx_data = "log_rx_data_v2x.csv";
std::string tx_data = "log_tx_data_v2x.csv";
std::string connections = "log_connections_v2x.csv";
std::string positions = "log_positions_v2x.csv"; 

Ptr<OutputStreamWrapper> log_connections;
Ptr<OutputStreamWrapper> log_simtime;
Ptr<OutputStreamWrapper> log_positions;
Ptr<OutputStreamWrapper> log_rx_data;
Ptr<OutputStreamWrapper> log_tx_data;

// Global variables
uint32_t ctr_totRx = 0; 	// Counter for total received packets
uint32_t ctr_totTx = 0; 	// Counter for total transmitted packets
uint32_t numVeh;
uint16_t lenCam;  
uint16_t pRsvp;
double baseline= 150.0;     // Baseline distance in meter (150m for urban, 320m for freeway)

uint32_t nPkt = 1;    // ryu5: # packets to send per burst
uint32_t numCam = 1;  // ryu5: # CAM messages per RRI
bool     idealScheduled = false;    // ryu5: do we use ideal scheduling?
bool     pickFromMore = false;      // ryu5: do we pick from more than 20% resources?

// Responders users 
NodeContainer ueVeh;

/*void SidelinkV2xMonitoringTrace (Ptr<OutputStreamWrapper> stream)
{
	*stream->GetStream () << Simulator::Now ().GetSeconds () << std::endl;
}

void SidelinkV2xAnnouncementPhyTrace (Ptr<OutputStreamWrapper> stream)
{
	*stream->GetStream () << Simulator::Now ().GetSeconds () << std::endl;
}

void SidelinkV2xAnnouncementMacTrace (Ptr<OutputStreamWrapper> stream)
{
	*stream->GetStream () << Simulator::Now ().GetSeconds () << std::endl;
}*/

void 
PrintStatus (uint32_t s_period, Ptr<OutputStreamWrapper> log_simtime)
{
    if (ctr_totRx > ctr_totTx)
    {
        ctr_totRx = ctr_totTx; 
    }

    // ryu5: compute some values
    double t = Simulator::Now ().GetSeconds (); // ryu5
	*log_simtime->GetStream() << t << ";" << ctr_totRx << ";" << ctr_totTx << ";" << (double) ctr_totRx / ctr_totTx << std::endl; 
    double thruputPerVeh = (double)lenCam * 8 * numCam * ((double) ctr_totRx / (ctr_totTx + LteUeMac::skippedTx*(numVeh-1))) * 1000 / pRsvp / 1000;  // Take into account skipped Txs

    uint32_t collisionEvent = 0, collisionCount = 0;
    for (std::map<uint32_t, uint32_t>::iterator stmit = LteUePhy::subframeTxMap.begin(); stmit != LteUePhy::subframeTxMap.end(); stmit ++) {
        if (stmit->second > 1) {
            collisionEvent += 1;
            collisionCount += stmit->second;
        }
    }
    double collisionRatio = 0;
    if (ctr_totTx / (numVeh - 1) > 0) {
        collisionRatio = 1.0 * collisionCount / ( ctr_totTx / (numVeh - 1) );
        //collisionRatio = 1.0 * collisionEvent / ( ctr_totTx / (numVeh - 1) );
    }

    std::cout << "t=" << t 
                << "\t Rx/Tx="<< ctr_totRx << "/" << ctr_totTx 
                << "\t PRR=" << (double) ctr_totRx / ctr_totTx 
                << "\t Veh. thruput. = " << thruputPerVeh << " kbps" // lenCam is per pRsvp ms, this is translating to Kbps.
                << "\t All thruput. = " << thruputPerVeh * numVeh / 1000 << " mbps" 
                << " or " << (double)ctr_totRx / (numVeh-1) * lenCam * 8 / (Simulator::Now().GetSeconds() - 3) / 1000000 << " mbps" //< This should be the most accurate measure.
                << "\t Skipped Tx = " << LteUeMac::skippedTx
                << "\t Collision event = " << collisionEvent
                << "\t Collision count = " << collisionCount
                << "\t Collision ratio = " << collisionRatio
                << std::endl;
    if (t == 3) {
        ctr_totRx = ctr_totTx = 0;
        LteUeMac::skippedTx = 0; // also reset this...
        LteUePhy::subframeTxMap.clear(); // also reset this...
        NS_LOG_INFO("Reset Tx/Rx counter after t=3 for accurate throughput computation.");
    }
    // std::cout << "  -> LteUeMac::nPktsTxed = " << LteUeMac::nPktsTxed 
    //         << " | " << "LteUePhy::nPktsTxed = " << LteUePhy::nPktsTxed 
    //         << " | " << "MultiModelSpectrumChannel::nPktsTxed = " << MultiModelSpectrumChannel::nPktsTxed 
    //         << " | " << "LteSpectrumPhy::nPktsRxed = " << LteSpectrumPhy::nPktsRxed 
    //         << " | " << "LteSpectrumPhy::nPktsCrpt = " << LteSpectrumPhy::nPktsCrpt 
    //         << std::endl; //< All MAC pushed are transmitted by PHY; but some are not received...
    Simulator::Schedule(Seconds(s_period), &PrintStatus, s_period,log_simtime);
}

void
SidelinkV2xAnnouncementMacTrace(Ptr<Socket> socket)
{
    Ptr <Node> node = socket->GetNode(); 
    int id = node->GetId();
    uint32_t simTime = Simulator::Now().GetMilliSeconds(); 
    Ptr<MobilityModel> posMobility = node->GetObject<MobilityModel>();
    Vector posTx = posMobility->GetPosition();

    // check for each UE distance to transmitter
    for (uint8_t i=0; i<ueVeh.GetN();i++)
    {
        Ptr<MobilityModel> mob = ueVeh.Get(i)->GetObject<MobilityModel>(); 
        Vector posRx = mob->GetPosition();
        
        double distance = sqrt(pow((posTx.x - posRx.x),2.0)+pow((posTx.y - posRx.y),2.0));
        if  (distance > 0 && distance <= baseline)
        {
            // ctr_totTx++;
            ctr_totTx += nPkt;
        }
    }
    // Generate CAM 
    std::ostringstream msgCam;
    msgCam << id-1 << ";" << simTime << ";" << (int) posTx.x << ";" << (int) posTx.y << '\0'; 
    Ptr<Packet> packet = Create<Packet>((uint8_t*)msgCam.str().c_str(),lenCam);
    // socket->Send(packet);
    for (uint32_t i=0; i<nPkt; i++) {
        socket->Send(packet);
    }
    *log_tx_data->GetStream() << ctr_totTx << ";" << simTime << ";"  << id-1 << ";" << (int) posTx.x << ";" << (int) posTx.y << std::endl;
    //NS_LOG_INFO( "t=" <<  Simulator::Now().GetSeconds() << "\t Sending by " << id );
}

static void
ReceivePacket(Ptr<Socket> socket)
{   
    Ptr<Node> node = socket->GetNode();
    Ptr<MobilityModel> posMobility = node->GetObject<MobilityModel>();
    Vector posRx = posMobility->GetPosition();
    Ptr<Packet> packet = socket->Recv (); 
    uint8_t *buffer = new uint8_t[packet->GetSize()];
    packet->CopyData(buffer,packet->GetSize());
    std::string s = std::string((char*)buffer);  

    size_t pos = 0; 
    std::string copy = s; 
    std::string token;
    int posTx_x;
    int posTx_y;  
    for (int i = 0; i < 3; i++)
    {
        if (copy.find(";") != std::string::npos)
        {
            pos = copy.find(";");
            token = copy.substr(0,pos);
            if(i == 2)
            {
                posTx_x = atoi(token.c_str());
            }
            copy.erase (0,pos+1);
        }  
    }  
    posTx_y = atoi(copy.c_str()); 

    double distance = sqrt(pow((posTx_x - posRx.x),2.0)+pow((posTx_y - posRx.y),2.0));
    if (distance <= baseline)
    {         
        int id = node->GetId();
        int simTime = Simulator::Now().GetMilliSeconds();
        ctr_totRx++; 
        *log_rx_data->GetStream() << ctr_totRx << ";" << simTime << ";"  << id-1 << ";" << s << std::endl; 
    }
}

int 
main (int argc, char *argv[])
{
    LogComponentEnable ("v2x_communication_mode_4", LOG_INFO);

    // Initialize some values
    // NOTE: commandline parser is currently (05.04.2019) not working for uint8_t (Bug 2916)

    uint16_t simTime = 100;                 // Simulation time in seconds
    numVeh = 100;                           // Number of vehicles
    lenCam = 190;                           // Length of CAM message in bytes [50-300 Bytes]
    double ueTxPower = 23.0;                // Transmission power in dBm
    double probResourceKeep = 0.8;          // Probability to select the previous resource again [0.0-0.8]
    uint32_t mcs = 20;                      // Modulation and Coding Scheme
    bool harqEnabled = false;               // Retransmission enabled 
    bool adjacencyPscchPssch = true;        // Subchannelization scheme
    bool partialSensing = false;            // Partial sensing enabled (actual only partialSensing is false supported)
    uint16_t sizeSubchannel = 10;           // Number of RBs per subchannel // adjacency: [5, 6, 10, 20, 25, 50, 75, 100]; non-adjacency: [4, 5, 6, 8, 9, 10, 12, 16, 18, 20, 30, 48, 72, 96]
    uint16_t numSubchannel = 3;             // Number of subchannels per subframe // [1, 3, 5, 8, 10, 15, 20]
    uint16_t startRbSubchannel = 0;         // Index of first RB corresponding to subchannelization
    pRsvp = 100;				    // Resource reservation interval 
    uint16_t t1 = 4;                        // T1 value of selection window
    uint16_t t2 = 100;                      // T2 value of selection window
    uint16_t slBandwidth;                   // Sidelink bandwidth
    std::string tracefile;                  // Name of the tracefile 
    // ryu5: # pkts per burst []
    nPkt = 1;
    // ryu5: # CAM messages per RRI
    numCam = 1;
    // ryu5: do we use ideal scheduling?
    idealScheduled = false;

    // Command line arguments
    CommandLine cmd;
    cmd.AddValue ("time", "Simulation Time", simTime);
    cmd.AddValue ("numVeh", "Number of Vehicles", numVeh);
    cmd.AddValue ("adjacencyPscchPssch", "Scheme for subchannelization", adjacencyPscchPssch); 
    cmd.AddValue ("sizeSubchannel", "Number of RBs per Subchannel", sizeSubchannel);
    cmd.AddValue ("numSubchannel", "Number of Subchannels", numSubchannel);
    cmd.AddValue ("startRbSubchannel", "Index of first subchannel index", startRbSubchannel); 
    cmd.AddValue ("T1", "T1 Value of Selection Window", t1);
    cmd.AddValue ("T2", "T2 Value of Selection Window", t2);
    //cmd.AddValue ("harqEnabled", "HARQ Retransmission Enabled", harqEnabled);
    //cmd.AddValue ("partialSensingEnabled", "Partial Sensing Enabled", partialSensing);
    cmd.AddValue ("lenCam", "Packetsize in Bytes", lenCam);
    cmd.AddValue ("mcs", "Modulation and Coding Scheme", mcs);
    cmd.AddValue ("pRsvp", "Resource Reservation Interval", pRsvp); 
    cmd.AddValue ("probResourceKeep", "Probability for selecting previous resource again", probResourceKeep); 
    cmd.AddValue ("log_simtime", "name of the simtime logfile", simtime);
    cmd.AddValue ("log_rx_data", "name of the rx data logfile", rx_data);
    cmd.AddValue ("log_tx_data", "name of the tx data logfile", tx_data);
    cmd.AddValue ("tracefile", "Path of ns-3 tracefile", tracefile); 
    cmd.AddValue ("baseline", "Distance in which messages are transmitted and must be received", baseline);
    // ryu5: nPkt
    cmd.AddValue ("nPkt", "Number of packets to send per burst", nPkt);
    // ryu5: numCam
    // -> This is the correct mode of transmitting more packets. 
    // -> To do a reasonable simulation, we should set nPkt above as 1, adjust
    // -> the lenCam above to maximum value of 400 bytes (or 405), and then 
    // -> adjust this numCam parameters for the actual data size in one RRI.
    // ** Maximum lenCam depends on sizeSubchannel.
    // ** When sizeSubchannel = 25, max lenCam = 1207.
    // ** When sizeSubchannel =  5, max lenCam =  129
    cmd.AddValue ("numCam", "Number of CAM messages per RRI", numCam);
    // ryu5: idealScheduled?
    cmd.AddValue ("idealScheduled", "Do we use ideal scheduling? (Default = false)", idealScheduled);
    // ryu5: pick numCam from 20%, or (20+numCam)%? => Depending on implementation of Sensing-based SPS
    cmd.AddValue ("pickFromMore", "Do we modify SPS to pick numCam from more resources (Default = false)", pickFromMore);
    cmd.Parse (argc, argv);

    // ryu5: output important values
    NS_LOG_INFO ("- idealScheduled " << idealScheduled);
    NS_LOG_INFO ("- numVeh " << numVeh);
    NS_LOG_INFO ("- numCam " << numCam);
    NS_LOG_INFO ("- sizeSubchannel " << sizeSubchannel);
    NS_LOG_INFO ("- numSubchannel " << numSubchannel);

    // ryu5: obtaining maximum tbSize => This is accurate, verifying with the above values (25 -> 1207 B, 10 -> 405 B, 5 -> 129 B)
    uint32_t subchLen = 1; // LteUeMac; length of a subchannel in ms / subframe?
    uint32_t nprb = subchLen*sizeSubchannel - 2 * adjacencyPscchPssch; // LteUeMac::DoSubframeIndication()
    int itbs = LteAmc::McsToItbsUl[mcs]; // LteAmc::GetUlTbSizeFromMcs()
    int maxTbSize = LteAmc::TransportBlockSizeTable[nprb - 1][itbs]; // LteAmc::GetUlTbSizeFromMcs()
    maxTbSize /= 8;     // bits to bytes
    maxTbSize -= 32;    // upper layer overheads
    NS_LOG_INFO ("* Maximum tbSize = " << maxTbSize);

    // Regulate lenCam to maximize throughput
    if (lenCam < maxTbSize) {
        NS_LOG_INFO ("* Current lenCam=" << lenCam << ": TOO SMALL for throughput maximization. Adjusted to maxTbSize=" << maxTbSize);
        lenCam = maxTbSize;
    } else if (lenCam > maxTbSize) {
        NS_LOG_INFO ("* Current lenCam=" << lenCam << ": TOO LARGE for throughput maximization. Adjusted to maxTbSize=" << maxTbSize);
        lenCam = maxTbSize;
    }

    // Regulate numCam to ensure reasonable ideal scheduling
    if (idealScheduled && numCam > pRsvp * numSubchannel / numVeh) {
        // Should actually be pRsvp / numVeh, since idealScheduled should not work with numSubchannel > 1 (will have the issue of 0 SINR for multiple slots within the same subframe)
        NS_LOG_INFO ("* Current numCam=" << numCam << ": TOO LARGE for ideal scheduling. Adjusted to pRsvp*numSubchannel/numVeh=" << pRsvp * numSubchannel / numVeh);
        numCam = pRsvp * numSubchannel / numVeh;
    }

    // ryu5: if idealScheduled, calculate scheduling right here!
    if (idealScheduled) {
        //uint32_t numSlotsToAlloc = ((t2-t1<pRsvp) ? (t2-t1) : pRsvp ) * (uint32_t)numSubchannel;    // => In total this number of subchannels within a selection window to allocate...
        uint32_t numSlotsToAlloc = pRsvp * (uint32_t)numSubchannel;    // => In total this number of subchannels within a selection window to allocate...
        uint32_t numSlotsPerVeh = numSlotsToAlloc / numVeh;      // => Each vehicle should have this many to go. Some may be wasted if cannot fully divide.
        //std::cout << t2-t1 << " " << pRsvp << " " << numSubchannel << " " << numSlotsPerVeh << " " << numSlotsToAlloc << std::endl;
        NS_LOG_INFO ("Calculating ideal schedule: numSlotsToAlloc = " << numSlotsToAlloc << ", numSlotsPerVeh = " << numSlotsPerVeh);
        NS_ASSERT_MSG( numCam <= numSlotsPerVeh, "[X] Ideal schedule: numCam = " << numCam << " is greater than numSlotsPerVeh = " << numSlotsPerVeh );
        NS_ASSERT_MSG( numCam * numVeh <= pRsvp * numSubchannel, "[X] Ideal schedule: numCam * numVeh = " << numCam * numVeh << " is greater than pRsvp * numSubchannel = " << pRsvp * numSubchannel );

        // We actually need some spacing to minimize cross-subframe co-channel interference
        uint32_t subframeSpacing = (pRsvp * numSubchannel) / (numCam * numVeh) - 1;
        NS_LOG_INFO ("Calculating ideal schedule: subframeSpacing = " << subframeSpacing);

        // Compute scheduling map for idealScheduled
        std::vector< std::vector< uint32_t > > idealScheduleResourceMap ( numVeh );
        uint32_t idxRbAll = 0; // we use this to iterate over Rbs
        for (uint32_t idxRb = 0; idxRb < numCam; idxRb++) {
            for (uint32_t idxVeh = 0; idxVeh < numVeh; idxVeh++) {
                //-> No spacing scheduling
                idealScheduleResourceMap[idxVeh].push_back( (idxVeh * numCam + idxRb) * (1 + subframeSpacing) ); //< All RBs of one vehicle scheduled together -- due to cross-subframe co-channel interference, PRR is extremely low (0)
                //idealScheduleResourceMap[idxVeh].push_back( idxVeh + idxRb * numVeh ); //< Much better PRR, but still only ~ 80% maximum
                //idealScheduleResourceMap[idxVeh].push_back( (idxRb * numVeh + idxVeh) * (1 + subframeSpacing) ); //< Much better PRR, but still only ~ 80% maximum

                //-> Scheduling with spacing between subframes for transmission
                // idealScheduleResourceMap[idxVeh].push_back( idxRbAll );
                // idxRbAll ++;
                // if (subframeSpacing > 0 && idxRbAll % numSubchannel == 0) {
                //     idxRbAll += numSubchannel * subframeSpacing; // skip the number of subframes defined by subframeSpacing
                // }
            }
        }

        // Print resource map
        std::cout << "Output idealScheduled resource map:" << std::endl;
        for (uint32_t idxVeh = 0; idxVeh < numVeh; idxVeh++) {
            std::cout << "  Vehicle " << idxVeh << ": ";
            for (uint32_t idxRb = 0; idxRb < numCam; idxRb++) {
                std::cout << idealScheduleResourceMap[idxVeh][idxRb] << " ";
            }
            std::cout << std::endl;
        }

        // Assign those to LteUeMac
        Config::SetDefault ("ns3::LteUeMac::IdealScheduled", BooleanValue(idealScheduled));
        LteUeMac::idealScheduleResourceMap = idealScheduleResourceMap;
    }

    Config::SetDefault ("ns3::LteUeMac::PickFromMore", BooleanValue(pickFromMore));
    // !ryu5

    AsciiTraceHelper ascii;
    log_simtime = ascii.CreateFileStream(simtime);
    log_rx_data = ascii.CreateFileStream(rx_data);
    log_tx_data = ascii.CreateFileStream(tx_data);
    log_connections = ascii.CreateFileStream(connections);
    log_positions = ascii.CreateFileStream(positions); 

    NS_LOG_INFO ("Starting network configuration..."); 

    // Set the UEs power in dBm
    Config::SetDefault ("ns3::LteUePhy::TxPower", DoubleValue (ueTxPower));
    Config::SetDefault ("ns3::LteUePhy::RsrpUeMeasThreshold", DoubleValue (-10.0));
    // Enable V2X communication on PHY layer
    Config::SetDefault ("ns3::LteUePhy::EnableV2x", BooleanValue (true));

    // Set power
    Config::SetDefault ("ns3::LteUePowerControl::Pcmax", DoubleValue (ueTxPower));
    Config::SetDefault ("ns3::LteUePowerControl::PsschTxPower", DoubleValue (ueTxPower));
    Config::SetDefault ("ns3::LteUePowerControl::PscchTxPower", DoubleValue (ueTxPower));

    if (adjacencyPscchPssch) 
    {
        slBandwidth = sizeSubchannel * numSubchannel;
    }
    else 
    {
        slBandwidth = (sizeSubchannel+2) * numSubchannel; 
    }

    // Configure for UE selected
    Config::SetDefault ("ns3::LteUeMac::UlBandwidth", UintegerValue(slBandwidth));
    Config::SetDefault ("ns3::LteUeMac::EnableV2xHarq", BooleanValue(harqEnabled));
    Config::SetDefault ("ns3::LteUeMac::EnableAdjacencyPscchPssch", BooleanValue(adjacencyPscchPssch));
    Config::SetDefault ("ns3::LteUeMac::EnablePartialSensing", BooleanValue(partialSensing));
    Config::SetDefault ("ns3::LteUeMac::SlGrantMcs", UintegerValue(mcs));
    Config::SetDefault ("ns3::LteUeMac::SlSubchannelSize", UintegerValue (sizeSubchannel));
    Config::SetDefault ("ns3::LteUeMac::SlSubchannelNum", UintegerValue (numSubchannel));
    Config::SetDefault ("ns3::LteUeMac::SlStartRbSubchannel", UintegerValue (startRbSubchannel));
    Config::SetDefault ("ns3::LteUeMac::SlPrsvp", UintegerValue(pRsvp));
    Config::SetDefault ("ns3::LteUeMac::SlProbResourceKeep", DoubleValue(probResourceKeep));
    Config::SetDefault ("ns3::LteUeMac::SelectionWindowT1", UintegerValue(t1));
    Config::SetDefault ("ns3::LteUeMac::SelectionWindowT2", UintegerValue(t2));
    //Config::SetDefault ("ns3::LteUeMac::EnableExcludeSubframe", BooleanValue(excludeSubframe)); 
    // ryu5
    Config::SetDefault ("ns3::LteUeMac::NumCam", UintegerValue(numCam));

    ConfigStore inputConfig; 
    inputConfig.ConfigureDefaults(); 

    // Create node container to hold all UEs 
    NodeContainer ueAllNodes; 

    NS_LOG_INFO ("Installing Mobility Model...");

    if (tracefile.empty())
    {
        // Create nodes
        ueVeh.Create (numVeh);
        ueAllNodes.Add (ueVeh);

        // Install constant random positions 
        MobilityHelper mobVeh;
        mobVeh.SetMobilityModel("ns3::ConstantPositionMobilityModel"); 
        Ptr<ListPositionAllocator> staticVeh[ueVeh.GetN()];
        for (uint16_t i=0; i<ueVeh.GetN();i++)
        {
            staticVeh[i] = CreateObject<ListPositionAllocator>();
            Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable> ();
            int x = rand->GetValue (0,100);
            int y = rand->GetValue (0,100);
            double z = 1.5;
            staticVeh[i]->Add(Vector(x,y,z)); 
            mobVeh.SetPositionAllocator(staticVeh[i]);
            mobVeh.Install(ueVeh.Get(i));
        }
    }
    else
    {
        // Create nodes
        ueVeh.Create (numVeh);
        ueAllNodes.Add (ueVeh);

        Ns2MobilityHelper ns2 = Ns2MobilityHelper(tracefile);
        ns2.Install();
    }


    NS_LOG_INFO ("Creating helpers...");
    // EPC
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    Ptr<Node> pgw = epcHelper->GetPgwNode();

    // LTE Helper
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    lteHelper->SetEpcHelper(epcHelper);
    lteHelper->DisableNewEnbPhy(); // Disable eNBs for out-of-coverage modelling
    
    // V2X 
    Ptr<LteV2xHelper> lteV2xHelper = CreateObject<LteV2xHelper> ();
    lteV2xHelper->SetLteHelper (lteHelper); 

    // Configure eNBs' antenna parameters before deploying them.
    lteHelper->SetEnbAntennaModelType ("ns3::NistParabolic3dAntennaModel");

    // Set pathloss model
    // FIXME: InstallEnbDevice overrides PathlossModel Frequency with values from Earfcn
    // 
    lteHelper->SetAttribute ("UseSameUlDlPropagationCondition", BooleanValue(true));
    Config::SetDefault ("ns3::LteEnbNetDevice::UlEarfcn", StringValue ("54990"));
    //Config::SetDefault ("ns3::CniUrbanmicrocellPropagationLossModel::Frequency", DoubleValue(5800e6));
    lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::CniUrbanmicrocellPropagationLossModel"));

    
    // Create eNB Container
    NodeContainer eNodeB;
    eNodeB.Create(1); 

    // Topology eNodeB
    Ptr<ListPositionAllocator> pos_eNB = CreateObject<ListPositionAllocator>(); 
    pos_eNB->Add(Vector(5,-10,30));

    //  Install mobility eNodeB
    MobilityHelper mob_eNB;
    mob_eNB.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob_eNB.SetPositionAllocator(pos_eNB);
    mob_eNB.Install(eNodeB);

    // Install Service
    NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice(eNodeB);

    // Required to use NIST 3GPP model
    BuildingsHelper::Install (eNodeB);
    BuildingsHelper::Install (ueAllNodes);
    BuildingsHelper::MakeMobilityModelConsistent (); 

    // Install LTE devices to all UEs 
    NS_LOG_INFO ("Installing UE's network devices...");
    lteHelper->SetAttribute("UseSidelink", BooleanValue (true));
    NetDeviceContainer ueRespondersDevs = lteHelper->InstallUeDevice (ueVeh);
    NetDeviceContainer ueDevs;
    ueDevs.Add (ueRespondersDevs); 

    // Install the IP stack on the UEs
    NS_LOG_INFO ("Installing IP stack..."); 
    InternetStackHelper internet;
    internet.Install (ueAllNodes); 

    // Assign IP adress to UEs
    NS_LOG_INFO ("Allocating IP addresses and setting up network route...");
    Ipv4InterfaceContainer ueIpIface; 
    ueIpIface = epcHelper->AssignUeIpv4Address (ueDevs);
    Ipv4StaticRoutingHelper Ipv4RoutingHelper;

    for(uint32_t u = 0; u < ueAllNodes.GetN(); ++u)
        {
            Ptr<Node> ueNode = ueAllNodes.Get(u);
            // Set the default gateway for the UE
            Ptr<Ipv4StaticRouting> ueStaticRouting = Ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
            ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress(), 1);
        }

    NS_LOG_INFO("Attaching UE's to LTE network...");
    //Attach each UE to the best available eNB
    lteHelper->Attach(ueDevs); 

    NS_LOG_INFO ("Creating sidelink groups...");
    std::vector<NetDeviceContainer> txGroups;
    txGroups = lteV2xHelper->AssociateForV2xBroadcast(ueRespondersDevs, numVeh); 

    lteV2xHelper->PrintGroups(txGroups); 
    // compute average number of receivers associated per transmitter and vice versa
    double totalRxs = 0;
    std::map<uint32_t, uint32_t> txPerUeMap;
    std::map<uint32_t, uint32_t> groupsPerUe;

    std::vector<NetDeviceContainer>::iterator gIt;
    for(gIt=txGroups.begin(); gIt != txGroups.end(); gIt++)
        {
            uint32_t numDevs = gIt->GetN();

            totalRxs += numDevs-1;
            uint32_t nId;

            for(uint32_t i=1; i< numDevs; i++)
                {
                    nId = gIt->Get(i)->GetNode()->GetId();
                    txPerUeMap[nId]++;
                }
        }

    double totalTxPerUe = 0; 
    std::map<uint32_t, uint32_t>::iterator mIt;
    for(mIt=txPerUeMap.begin(); mIt != txPerUeMap.end(); mIt++)
        {
            totalTxPerUe += mIt->second;
            groupsPerUe [mIt->second]++;
        }

    // lteV2xHelper->PrintGroups (txGroups, log_connections);

    NS_LOG_INFO ("Installing applications...");
    
    // Application Setup for Responders
    std::vector<uint32_t> groupL2Addresses; 
    uint32_t groupL2Address = 0x00; 
    Ipv4AddressGenerator::Init(Ipv4Address ("225.0.0.0"), Ipv4Mask("255.0.0.0"));
    Ipv4Address clientRespondersAddress = Ipv4AddressGenerator::NextAddress (Ipv4Mask ("255.0.0.0"));

    uint16_t application_port = 8000; // Application port to TX/RX
    NetDeviceContainer activeTxUes;

    // Set Sidelink V2X Traces
    /*AsciiTraceHelper ascii;
    Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream ("sidelinkV2x_out_monitoring.tr");
    *stream->GetStream () << "Time" << std::endl;
    
    AsciiTraceHelper ascii1;
    Ptr<OutputStreamWrapper> stream1 = ascii1.CreateFileStream ("sidelinkV2x_out_announcement_phy.tr");
    *stream1->GetStream () << "Time" << std::endl;
    
    AsciiTraceHelper ascii2;
    Ptr<OutputStreamWrapper> stream2 = ascii1.CreateFileStream ("sidelinkV2x_out_announcement_mac.tr");
    *stream2->GetStream () << "Time" << std::endl;

    std::ostringstream oss;
    oss.str("");*/

    for(gIt=txGroups.begin(); gIt != txGroups.end(); gIt++)
        {
            // Create Sidelink bearers
            // Use Tx for the group transmitter and Rx for all the receivers
            // Split Tx/Rx

            NetDeviceContainer txUe ((*gIt).Get(0));
            activeTxUes.Add(txUe);
            NetDeviceContainer rxUes = lteV2xHelper->RemoveNetDevice ((*gIt), txUe.Get (0));
            Ptr<LteSlTft> tft = Create<LteSlTft> (LteSlTft::TRANSMIT, clientRespondersAddress, groupL2Address);
            lteV2xHelper->ActivateSidelinkBearer (Seconds(0.0), txUe, tft);
            tft = Create<LteSlTft> (LteSlTft::RECEIVE, clientRespondersAddress, groupL2Address);
            lteV2xHelper->ActivateSidelinkBearer (Seconds(0.0), rxUes, tft);

            //std::cout << "Created group L2Address=" << groupL2Address << " IPAddress=";
            //clientRespondersAddress.Print(std::cout);
            //std::cout << std::endl;

            //Individual Socket Traffic Broadcast everyone
            Ptr<Socket> host = Socket::CreateSocket(txUe.Get(0)->GetNode(),TypeId::LookupByName ("ns3::UdpSocketFactory"));
            host->Bind();
            host->Connect(InetSocketAddress(clientRespondersAddress,application_port));
            host->SetAllowBroadcast(true);
            host->ShutdownRecv();

            //Ptr<LteUeRrc> ueRrc = DynamicCast<LteUeRrc>( txUe.Get (0)->GetObject<LteUeNetDevice> ()->GetRrc () );    
            //ueRrc->TraceConnectWithoutContext ("SidelinkV2xMonitoring", MakeBoundCallback (&SidelinkV2xMonitoringTrace, stream));
            //oss << txUe.Get(0) ->GetObject<LteUeNetDevice>()->GetImsi(); 
            //Ptr<LteUePhy> uePhy = DynamicCast<LteUePhy>( txUe.Get (0)->GetObject<LteUeNetDevice> ()->GetPhy () );
            //uePhy->TraceConnect ("SidelinkV2xAnnouncement", oss.str() ,MakeBoundCallback (&SidelinkV2xAnnouncementPhyTrace, stream1));
            //uePhy->TraceConnectWithoutContext ("SidelinkV2xAnnouncement", MakeBoundCallback (&SidelinkV2xAnnouncementPhyTrace, host));
            Ptr<LteUeMac> ueMac = DynamicCast<LteUeMac>( txUe.Get (0)->GetObject<LteUeNetDevice> ()->GetMac () );
            ueMac->TraceConnectWithoutContext ("SidelinkV2xAnnouncement", MakeBoundCallback (&SidelinkV2xAnnouncementMacTrace, host));
            //ueMac->TraceConnect ("SidelinkV2xAnnouncement", oss.str() ,MakeBoundCallback (&SidelinkV2xAnnouncementMacTrace, stream2));

            Ptr<Socket> sink = Socket::CreateSocket(txUe.Get(0)->GetNode(),TypeId::LookupByName ("ns3::UdpSocketFactory"));
            sink->Bind(InetSocketAddress (Ipv4Address::GetAny (), application_port));
            sink->SetRecvCallback (MakeCallback (&ReceivePacket));

            //store and increment addresses
            groupL2Addresses.push_back (groupL2Address);
            groupL2Address++;
            clientRespondersAddress = Ipv4AddressGenerator::NextAddress (Ipv4Mask ("255.0.0.0"));
        }

        NS_LOG_INFO ("Creating Sidelink Configuration...");
        Ptr<LteUeRrcSl> ueSidelinkConfiguration = CreateObject<LteUeRrcSl>();
        ueSidelinkConfiguration->SetSlEnabled(true);
        ueSidelinkConfiguration->SetV2xEnabled(true);

        LteRrcSap::SlV2xPreconfiguration preconfiguration;
        preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommPreconfigGeneral.carrierFreq = 54890;
        preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommPreconfigGeneral.slBandwidth = slBandwidth;
        
        preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.nbPools = 1;
        preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.nbPools = 1;

        SlV2xPreconfigPoolFactory pFactory;
        pFactory.SetHaveUeSelectedResourceConfig (true);
        pFactory.SetSlSubframe (std::bitset<20> (0xFFFFF));
        pFactory.SetAdjacencyPscchPssch (adjacencyPscchPssch);
        pFactory.SetSizeSubchannel (sizeSubchannel);
        pFactory.SetNumSubchannel (numSubchannel);
        pFactory.SetStartRbSubchannel (startRbSubchannel);
        pFactory.SetStartRbPscchPool (0);
        pFactory.SetDataTxP0 (-4);
        pFactory.SetDataTxAlpha (0.9);

        preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0] = pFactory.CreatePool ();
        preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0] = pFactory.CreatePool ();
        ueSidelinkConfiguration->SetSlV2xPreconfiguration (preconfiguration); 

        // Print Configuration
        *log_rx_data->GetStream() << "RxPackets;RxTime;RxId;TxId;TxTime;xPos;yPos" << std::endl;
        *log_tx_data->GetStream() << "TxPackets;TxTime;TxId;xPos;yPos" << std::endl;

        NS_LOG_INFO ("Installing Sidelink Configuration...");
        lteHelper->InstallSidelinkV2xConfiguration (ueRespondersDevs, ueSidelinkConfiguration);

        NS_LOG_INFO ("Enabling LTE traces...");
        lteHelper->EnableTraces();

        *log_simtime->GetStream() << "Simtime;TotalRx;TotalTx;PRR" << std::endl; 
        Simulator::Schedule(Seconds(1), &PrintStatus, 1, log_simtime);

        NS_LOG_INFO ("Starting Simulation...");
        Simulator::Stop(MilliSeconds(simTime*1000+40));
        Simulator::Run();
        Simulator::Destroy();

        NS_LOG_INFO("Simulation done.");
        return 0;  
}   