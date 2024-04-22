#ifndef SIMULATION_H
#define SIMULATION_H

// Inclure les bibliothèques NS-3 nécessaires
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy-standard.h"
#include "ns3/adhoc-wifi-mac.h"
#include "ns3/yans-wifi-phy.h"

using namespace ns3;

// Définition de la structure de cluster
struct Cluster {
    std::vector<Ptr<Node>> nodes;
    double mobilityScore;
};

// Déclaration des fonctions de simulation
void FormClusters(std::vector<Ptr<Node>>& nodes, double clusterRadius, std::vector<Cluster>& clusters);
double CalculateMobilityScore(Cluster& cluster);
void PrintClusters(std::vector<Cluster>& clusters);

// Déclaration de la fonction principale de simulation
int RunSimulation(int argc, char* argv[]);

#endif // SIMULATION_H
