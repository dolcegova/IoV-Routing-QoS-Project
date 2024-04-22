
/*
 * Auteur : Oussama SENOUCI 
   Nom du projet  Enhancing Routing and Quality of Service Using AI-driven Technique
for Internet of Vehicles Contexts
 */

/*
	C'est le point de départ de la simulation et des expériences.
	La fonction principale analysera les entrées et les paramètres de configuration.
	Crée une autoroute et configure les paramètres de l'autoroute. Ensuite, lie les événements
	(callbacks) au contrôleur créé et aux gestionnaires conçus. Définit l'heure de début et de fin de l'autoroute,
	et finalement exécute la simulation qui consiste essentiellement à exécuter une autoroute avec un contrôleur.
	Vous pouvez ajouter vos fonctions au contrôleur pour créer divers scénarios.
*/



// Importer les bibliothèques NS-3
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

// Déclaration de fonctions
void FormClusters(std::vector<Ptr<Node>>& nodes, double clusterRadius, std::vector<Cluster>& clusters);
double CalculateMobilityScore(Cluster& cluster);
void PrintClusters(std::vector<Cluster>& clusters);

int main(int argc, char* argv[]) {
    // Paramètres de configuration
    uint32_t numNodes = 60; // Nombre de véhicules à simuler
    double simulationTime = 360.0; // Temps de simulation en secondes
    double clusterRadius = 50.0; // Rayon de clustering en mètres
    double minVelocity = 10.0; // Vitesse minimale en m/s
    double maxVelocity = 35.0; // Vitesse maximale en m/s
    
    // Créer un ensemble de nœuds
    NodeContainer nodes;
    nodes.Create(numNodes);

    // Configurer la mobilité des nœuds
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
        "Speed", StringValue("ns3::UniformRandomVariable[Min=10|Max=35]"),
        "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0]"),
        "PositionAllocator", StringValue("ns3::RandomRectanglePositionAllocator"));
    mobility.Install(nodes);

    // Configurer le Wi-Fi
    WifiHelper wifi;
    wifi.SetStandard(WIFI_PHY_STANDARD_80211p);
    
    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");

    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiPhy.SetChannel(wifiChannel.Create());
    
    // Créer les périphériques Wi-Fi pour les nœuds
    NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, nodes);

    // Configurer la pile Internet
    InternetStackHelper internet;
    internet.Install(nodes);

    // Exécuter la simulation
    Simulator::Stop(Seconds(simulationTime));
    Simulator::Run();

    // Créer les clusters
    std::vector<Cluster> clusters;
    FormClusters(nodes, clusterRadius, clusters);

    // Calculer et afficher les scores de mobilité
    for (Cluster& cluster : clusters) {
        cluster.mobilityScore = CalculateMobilityScore(cluster);
    }

    // Afficher les clusters
    PrintClusters(clusters);

    Simulator::Destroy();
    return 0;
}

// Fonction pour former les clusters
void FormClusters(std::vector<Ptr<Node>>& nodes, double clusterRadius, std::vector<Cluster>& clusters) {
    // Utilisez un vecteur pour garder la trace des nœuds déjà clusterisés
    std::vector<bool> isClustered(nodes.size(), false);

    // Parcourir les nœuds et former des clusters
    for (size_t i = 0; i < nodes.size(); i++) {
        // Si le nœud est déjà clusterisé, passer au nœud suivant
        if (isClustered[i]) {
            continue;
        }

        // Créer un nouveau cluster pour chaque nœud non clusterisé
        Cluster cluster;
        cluster.nodes.push_back(nodes[i]);
        isClustered[i] = true;

        // Vérifier les voisins dans le rayon de clustering
        for (size_t j = i + 1; j < nodes.size(); j++) {
            // Calculer la distance entre les nœuds
            Ptr<Node> neighbor = nodes[j];
            double distance = nodes[i]->GetObject<MobilityModel>()->GetDistanceFrom(neighbor->GetObject<MobilityModel>());

            // Si le voisin est dans le rayon de clustering, ajoutez-le au cluster
            if (distance <= clusterRadius) {
                cluster.nodes.push_back(neighbor);
                isClustered[j] = true; // Marquer le voisin comme clusterisé
            }
        }

        // Calculer le score de mobilité du cluster
        cluster.mobilityScore = CalculateMobilityScore(cluster);

        // Ajouter le cluster formé à la liste des clusters
        clusters.push_back(cluster);
    }
}

// Fonction pour calculer le score de mobilité d'un cluster
double CalculateMobilityScore(Cluster& cluster) {
    double totalVelocity = 0.0;
    double totalDistance = 0.0;

    // Calculer la somme des vitesses et des distances entre les nœuds du cluster
    for (size_t i = 0; i < cluster.nodes.size(); i++) {
        Ptr<Node> node = cluster.nodes[i];
        Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
        totalVelocity += mobility->GetVelocity().GetLength();

        for (size_t j = i + 1; j < cluster.nodes.size(); j++) {
            Ptr<Node> neighbor = cluster.nodes[j];
            double distance = mobility->GetDistanceFrom(neighbor->GetObject<MobilityModel>());
            totalDistance += distance;
        }
    }

    // Calculer les moyennes
    double averageVelocity = totalVelocity / cluster.nodes.size();
    double averageDistance = totalDistance / (cluster.nodes.size() * (cluster.nodes.size() - 1) / 2);

    // Calculer le score de mobilité
    double mobilityScore = averageVelocity / averageDistance;

    return mobilityScore;
}

// Fonction pour afficher les clusters
void PrintClusters(std::vector<Cluster>& clusters) {
    std::cout << "Clusters formed:" << std::endl;
    for (size_t i = 0; i < clusters.size(); i++) {
        std::cout << "Cluster " << i + 1 << ":";
        for (Ptr<Node> node : clusters[i].nodes) {
            std::cout << " Node " << node->GetId();
        }
        std::cout << " Mobility Score: " << clusters[i].mobilityScore << std::endl;
    }
}
