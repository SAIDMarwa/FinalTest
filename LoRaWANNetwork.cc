#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <cstdlib> // Pour rand()
#include <ctime>   // Pour srand()
#include <numeric> // Pour std::accumulate
#include <sstream>
#include <string>
#include <limits>
#include <omnetpp.h>
#include <chrono>

#define NOISE_POWER -100.0  // Puissance du bruit en dBm
#define PACKET_LOSS_THRESHOLD -10.0  // Seuil de perte de paquets en dB
int clusterCount = 20;  // Nombre de clusters ou de passerelles à placer
const int terrainWidth = 200000;  // Largeur du terrain en mètres
const int terrainHeight = 200000;  // Hauteur du terrain en mètres
const int N = 100;  // Nombre total de nœuds
const double D_min = 2000;  // Distance minimale entre deux nœuds en mètres
const double E_total = 1000.0;  // Énergie totale admissible (par exemple en mAh)
const double S_terrain = terrainWidth * terrainHeight;  // Surface totale du terrain (en m²)
const int capacity = 10;  // Capacité d'une passerelle en nombre de nœuds gérés
const double energy = 15.0;  // Consommation d'énergie d'une passerelle (en watts)
const int rayon = 200000;  // Rayon de couverture des gateways (en mètres)


// Définition des variables de décision
std::vector<int> X(clusterCount, 0);  // X_r : 1 si une passerelle est placée à l'emplacement r, sinon 0
std::vector<std::vector<int>> Y(clusterCount, std::vector<int>(N, 0));  // Y_rn : 1 si la passerelle r sert le nœud n, sinon 0
std::vector<int> Z(N, 0);  // Z_n : 1 si le nœud est activé, sinon 0
std::vector<int> C_r(clusterCount, 20);  // Exemple : chaque passerelle a une capacité de 20 nœuds
std::vector<double> E_r(clusterCount, 15.0);  // Consommation d'énergie des passerelles (en watts)
std::vector<double> S_n(N, 60.0);  // Chaque nœud couvre 60.0 m²

// Structure pour représenter un nœud
struct Node {
    double x, y; // Coordonnées du nœud
    bool isActive; // Variable de décision Z_n
    double rayonN;
};

// Structure pour représenter une passerelle (Gateway)
struct Gateway {
    double x, y; // Coordonnées de la passerelle
    double capacity; // Capacité de la passerelle C_r
    double energy;   // Énergie consommée E_r
    double rayon;  //rayon de couverture R_c
    std::vector<int> assignedNodes; // Nœuds assignés à cette passerelle
};

// Structure pour représenter un obstacle
struct Obstacle {
    double x, y, height; // Coordonnées et hauteur de l'obstacle
};

// Calcul de distance entre deux points
double calculateDistance(const Node& a, const Node& b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

double calculateDistance1(double x1, double y1, double x2, double y2) {
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}


// Fonction f_N : Minimiser le nombre de nœuds activés
double f_N(const std::vector<int>& Z) {
    return std::accumulate(Z.begin(), Z.end(), 0);
}

// Fonction f_C : Maximiser la somme des distances entre les passerelles et les nœuds
double f_C(const std::vector<std::vector<int>>& Y, const std::vector<std::vector<double>>& D_rn) {
    double sum = 0.0;
    for (int r = 0; r < clusterCount; ++r) { // Utiliser clusterCount au lieu de R
        for (int n = 0; n < N; ++n) {
            sum += Y[r][n] * D_rn[r][n];
        }
    }
    return sum;
}

// Fonction f_E : Minimiser l'énergie totale consommée par les passerelles
double f_E(const std::vector<int>& X, const std::vector<double>& E_r) {
    double sum = 0.0;
    for (int r = 0; r < clusterCount; ++r) {
        sum += X[r] * E_r[r];
    }
    return sum;
}

// Fonction globale f : Combinaison des fonctions f_N, f_C, et f_E
double f(double alpha, double beta, double gamma, const std::vector<int>& Z, const std::vector<std::vector<int>>& Y, const std::vector<std::vector<double>>& D_rn, const std::vector<int>& X, const std::vector<double>& E_r) {
    return alpha * f_N(Z) + beta * f_C(Y, D_rn) + gamma * f_E(X, E_r);
}

// Contrainte 1 : Chaque nœud doit être servi par au moins une passerelle
bool constraint1(const std::vector<std::vector<int>>& Y) {
    for (int n = 0; n < N; ++n) {
        int sum = 0;
        for (int r = 0; r < clusterCount; ++r) {
            sum += Y[r][n];
        }
        if (sum < 1) return false;
    }
    return true;
}

// Contrainte 2 : Une passerelle ne peut servir un nœud que si elle est activée
bool constraint2(const std::vector<std::vector<int>>& Y, const std::vector<int>& X) {
    for (int r = 0; r < clusterCount; ++r) {
        for (int n = 0; n < N; ++n) {
            if (Y[r][n] > X[r]) return false;
        }
    }
    return true;
}

// Contrainte 3 : La capacité de chaque passerelle ne doit pas être dépassée
bool constraint3(const std::vector<std::vector<int>>& Y, const std::vector<int>& C_r) {
    for (int r = 0; r < clusterCount; ++r) {
        int sum = 0;
        for (int n = 0; n < N; ++n) {
            sum += Y[r][n];
        }
        if (sum > C_r[r]) return false;
    }
    return true;
}

// Contrainte 4 : L'énergie totale consommée ne doit pas dépasser E_total
bool constraint4(const std::vector<int>& X, const std::vector<double>& E_r) {
    double sum = 0.0;
    for (int r = 0; r < clusterCount; ++r) {
        sum += X[r] * E_r[r];
    }
    return sum <= E_total;
}

// Contrainte 5 : La surface couverte par les nœuds doit être au moins égale à S_terrain
bool constraint5(const std::vector<int>& Z, const std::vector<double>& S_n) {
    double sum = 0.0;
    for (int n = 0; n < N; ++n) {
        sum += Z[n] * S_n[n];
    }
    return sum >= S_terrain;
}

// Contrainte 6 : La distance minimale entre deux nœuds doit être respectée
bool constraint6(const std::vector<Node>& nodes, double D_min) {
    for (int m = 0; m < N; ++m) {
        for (int n = m + 1; n < N; ++n) {
            double distance = calculateDistance(nodes[m], nodes[n]);
            if (distance < D_min) return false;
        }
    }
    return true;
}

// Fonction pour générer des nœuds aléatoires
std::vector<Node> generateNodes(int nodeCount, double width, double height) {
    std::vector<Node> nodes;
    srand(time(0));
    for (int i = 0; i < nodeCount; ++i) {
        nodes.push_back({static_cast<double>(rand() % int(width)), static_cast<double>(rand() % int(height))});
    }
    return nodes;
}

// Fonction pour écrire les nœuds dans un fichier
void saveNodesToFile(const std::vector<Node>& nodes, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << "\n";
        return;
    }

    for (size_t i = 0; i < nodes.size(); ++i) {
        file << "**.loRaNodes[" << i << "].**.initialX = " << nodes[i].x << "m" << std::endl;
        file << "**.loRaNodes[" << i << "].**.initialY = " << nodes[i].y << "m" << std::endl;
       //file <<"["<< nodes[i].x <<","<< nodes[i].y<<"]," << std::endl;
    }

    file.close();
    std::cout << "Nodes saved to " << filename << "\n";
}

// Fonction pour écrire les gateways dans un fichier
void saveGatewaysToFile(const std::vector<Gateway>& centroids, const std::string& filename1) {
    std::ofstream file(filename1);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << filename1 << "\n";
        return;
    }

    for (size_t i = 0; i < centroids.size(); ++i) {
        file << "**.loRaGW[" << i << "].**.initialX = " << centroids[i].x << "m" << std::endl;
        file << "**.loRaGW[" << i << "].**.initialY = " << centroids[i].y << "m" << std::endl;
       // file <<"["<< centroids[i].x <<","<< centroids[i].y<<"]," << std::endl;
    }

    file.close();
    std::cout << "Gateways saved to " << filename1 << "\n";
}
// Fonction pour initialiser les centroïdes avec KMeans++
std::vector<Gateway> kMeansPlusPlus(const std::vector<Node>& nodes, int clusterCount) {
    std::vector<Gateway> centroids;
    srand(time(0));

    // Choisir le premier centroïde aléatoirement
    centroids.push_back({nodes[rand() % nodes.size()].x, nodes[rand() % nodes.size()].y, capacity, energy, rayon});

    // Choisir les centroïdes suivants
    for (int i = 1; i < clusterCount; ++i) {
        std::vector<double> distances(nodes.size(), std::numeric_limits<double>::max());

        // Calculer la distance de chaque nœud au centroïde le plus proche
        for (size_t j = 0; j < nodes.size(); ++j) {
            for (const auto& centroid : centroids) {
                double dist = calculateDistance(nodes[j], {centroid.x, centroid.y});
                if (dist < distances[j]) {
                    distances[j] = dist;
                }
            }
        }

        // Choisir le prochain centroïde avec une probabilité proportionnelle à la distance au carré
        double totalDistance = std::accumulate(distances.begin(), distances.end(), 0.0);
        double randomValue = static_cast<double>(rand()) / RAND_MAX * totalDistance;
        double cumulativeDistance = 0.0;
        for (size_t j = 0; j < nodes.size(); ++j) {
                    cumulativeDistance += distances[j];
                    if (cumulativeDistance >= randomValue) {
                              centroids.push_back({nodes[j].x, nodes[j].y, capacity, energy, rayon});
                              break;
                          }
                      }
                  }

                  return centroids;
                  }

// Fonction pour calculer WCSS (Within-Cluster Sum of Squares)
double calculateWCSS(const std::vector<Node>& nodes, const std::vector<Gateway>& centroids, const std::vector<int>& assignments) {
    double wcss = 0.0;
    for (size_t i = 0; i < nodes.size(); ++i) {
        int cluster = assignments[i];
        double dist = calculateDistance(nodes[i], {centroids[cluster].x, centroids[cluster].y});
        wcss += dist * dist;
    }
    return wcss;
}
// Fonction KMeans pour trouver les centroïdes des clusters
std::pair<std::vector<Gateway>, std::vector<int>> kMeans(const std::vector<Node>& nodes, int clusterCount, int iterations = 100) {
    // Initialisation des centroïdes avec KMeans++
    std::vector<Gateway> centroids = kMeansPlusPlus(nodes, clusterCount);
    std::vector<int> assignments(nodes.size(), -1); // Initialiser à -1 pour détecter les erreurs

    for (int iter = 0; iter < iterations; ++iter) {
        // Attribution des nœuds aux centroïdes
        for (size_t i = 0; i < nodes.size(); ++i) {
            double minDist = std::numeric_limits<double>::max();
            int bestCluster = -1;

            for (int j = 0; j < clusterCount; ++j) {
                double dist = calculateDistance(nodes[i], {centroids[j].x, centroids[j].y});
                if (dist < minDist) {
                    minDist = dist;
                    bestCluster = j;
                }
            }

            if (bestCluster == -1) {
                std::cerr << "Erreur : Aucun cluster trouvé pour le nœud " << i << "\n";
            } else {
                assignments[i] = bestCluster;
            }
        }

        // Mise à jour des centroïdes
        std::vector<Gateway> newCentroids(clusterCount, {0, 0, capacity, energy, rayon});
        std::vector<int> counts(clusterCount, 0);
        for (size_t i = 0; i < nodes.size(); ++i) {
            int cluster = assignments[i];
            if (cluster < 0 || cluster >= clusterCount) {
                std::cerr << "Erreur : Cluster invalide pour le nœud " << i << "\n";
                continue;
            }
            newCentroids[cluster].x += nodes[i].x;
            newCentroids[cluster].y += nodes[i].y;
            counts[cluster]++;
        }
        for (int j = 0; j < clusterCount; ++j) {
            if (counts[j] > 0) {
                newCentroids[j].x /= counts[j];
                newCentroids[j].y /= counts[j];
            }
        }
        centroids = newCentroids;
    }

    return {centroids, assignments};
}
// Fonction pour exécuter K-means et calculer WCSS pour différentes valeurs de k
std::vector<double> elbowMethod(const std::vector<Node>& nodes, int maxClusters) {
    std::vector<double> wcssValues;
    for (int k = 1; k <= maxClusters; ++k) {
        // Exécuter K-means pour k clusters
        auto [centroids, assignments] = kMeans(nodes, k);

        // Calculer WCSS
        double wcss = calculateWCSS(nodes, centroids, assignments);
        wcssValues.push_back(wcss);

        std::cout << "k = " << k << ", WCSS = " << wcss << "\n";
    }
    return wcssValues;
}
// Fonction pour trouver le nombre optimal de clusters
int findOptimalClusters(const std::vector<double>& wcssValues) {
    if (wcssValues.size() < 2) {
        std::cerr << "Erreur : Le vecteur WCSS doit contenir au moins 2 valeurs.\n";
        return -1;
    }

    // Calculer les différences de WCSS (pentes)
    std::vector<double> slopes;
    for (size_t i = 1; i < wcssValues.size(); ++i) {
        slopes.push_back(wcssValues[i - 1] - wcssValues[i]);
    }

    // Calculer les différences de pentes (dérivée seconde)
    std::vector<double> slopeDifferences;
    for (size_t i = 1; i < slopes.size(); ++i) {
        slopeDifferences.push_back(std::abs(slopes[i - 1] - slopes[i]));
    }

    // Trouver l'indice du coude (point où la dérivée seconde est maximale)
    auto maxDiffIt = std::max_element(slopeDifferences.begin(), slopeDifferences.end());
    size_t elbowIndex = std::distance(slopeDifferences.begin(), maxDiffIt) + 1; // +1 car on commence à k=2

    // Retourner le nombre optimal de clusters
    return static_cast<int>(elbowIndex + 1); // +1 car les clusters commencent à k=1
}
// Fonction pour tracer la courbe du coude (à implémenter selon votre environnement)
void plotElbowCurve(const std::vector<double>& wcssValues) {
    // Vous pouvez exporter les données vers un fichier et utiliser un outil externe (Python, Excel, etc.) pour tracer la courbe.
    std::ofstream file("wcss_values.txt");
    if (file.is_open()) {
        for (size_t i = 0; i < wcssValues.size(); ++i) {
            file << i + 1 << " " << wcssValues[i] << "\n";
        }
        file.close();
        std::cout << "WCSS values saved to wcss_values.txt\n";
    } else {
        std::cerr << "Error: Unable to open file wcss_values.txt\n";
    }
}

// Vérifie si une passerelle est bloquée par un obstacle
bool isBlockedByObstacle(const Gateway& gateway, const std::vector<Obstacle>& obstacles, double thresholdDistance) {
    for (const auto& obstacle : obstacles) {
        if (calculateDistance1(gateway.x, gateway.y, obstacle.x, obstacle.y) < thresholdDistance) {
            return true; // La passerelle est bloquée par cet obstacle
        }
    }
    return false;
}

// Repositionne une passerelle si elle est bloquée
void repositionGateway(Gateway& gateway, const std::vector<Obstacle>& obstacles, double thresholdDistance) {
    while (isBlockedByObstacle(gateway, obstacles, thresholdDistance)) {
        gateway.x = static_cast<double>(rand() % 100); // Nouvelle position aléatoire (exemple)
        gateway.y = static_cast<double>(rand() % 100);
    }
}

// Fonction pour regrouper les nœuds par cluster
std::vector<std::vector<Node>> groupNodesByCluster(const std::vector<Node>& nodes, const std::vector<int>& assignments, int clusterCount) {
    std::vector<std::vector<Node>> clusters(clusterCount);
    for (size_t i = 0; i < nodes.size(); ++i) {
        if (assignments[i] < 0 || assignments[i] >= clusterCount) {
            std::cerr << "Erreur : Valeur invalide dans assignments : " << assignments[i] << "\n";
            continue; // Ignorer cette valeur ou gérer l'erreur
        }
        clusters[assignments[i]].push_back(nodes[i]);
    }
    return clusters;
}

// Fonction principale pour repositionner les passerelles si elles sont bloquées
void repositionGatewaysIfBlocked(std::vector<Gateway>& gateways, const std::vector<Obstacle>& obstacles, double thresholdDistance) {
    // Initialisation du générateur de nombres aléatoires
    std::srand(static_cast<unsigned int>(std::time(0)));

    // Afficher les positions initiales des passerelles
    std::cout << "Positions initiales des passerelles :\n";
    for (const auto& gateway : gateways) {
        std::cout << "Gateway: (" << gateway.x << ", " << gateway.y << ")\n";
    }

    // Parcourir toutes les passerelles
    for (auto& gateway : gateways) {
        // Vérifier si la passerelle est bloquée par un obstacle
        if (isBlockedByObstacle(gateway, obstacles, thresholdDistance)) {
            // Repositionner la passerelle
            repositionGateway(gateway, obstacles, thresholdDistance);
        }
    }

    // Afficher les nouvelles positions des passerelles
    std::cout << "Nouvelles positions des passerelles :\n";
    for (const auto& gateway : gateways) {
        std::cout << "Gateway: (" << gateway.x << ", " << gateway.y << ")\n";
    }
}

// Fonction pour sauvegarder les clusters dans un fichier
void saveClustersToFile(const std::vector<std::vector<Node>>& clusters, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << "\n";
        return;
    }

    for (size_t i = 0; i < clusters.size(); ++i) {
        file << "Cluster " << i + 1 << ":\n";
        for (const auto& node : clusters[i]) {
            file << "  Node: (" << node.x << ", " << node.y << ")\n";
        }
        file << "\n";
    }

    file.close();
    std::cout << "Clusters saved to " << filename << "\n";
}

// Fonction pour afficher les clusters
void printClusters(const std::vector<std::vector<Node>>& clusters) {
    for (size_t i = 0; i < clusters.size(); ++i) {
        std::cout << "Cluster " << i + 1 << ":\n";
        for (const auto& node : clusters[i]) {
            std::cout << "  Node: (" << node.x << ", " << node.y << ")\n";
        }
        std::cout << std::endl;
    }
}
// Fonction K-means modifiée pour prendre en compte la capacité des passerelles
std::pair<std::vector<Gateway>, std::vector<int>> constrainedKMeans(const std::vector<Node>& nodes, int clusterCount, int gatewayCapacity, int maxIterations = 100) {
    std::vector<Gateway> gateways(clusterCount);
    std::vector<int> assignments(nodes.size(), -1);

    // Initialisation des centroïdes avec K-means++
    srand(static_cast<unsigned int>(time(0)));
    gateways[0].x = nodes[rand() % nodes.size()].x;
    gateways[0].y = nodes[rand() % nodes.size()].y;
    gateways[0].capacity = gatewayCapacity;

    for (int i = 1; i < clusterCount; ++i) {
        std::vector<double> distances(nodes.size(), std::numeric_limits<double>::max());
        for (size_t j = 0; j < nodes.size(); ++j) {
            for (int k = 0; k < i; ++k) {
                double dist = calculateDistance(nodes[j], {gateways[k].x, gateways[k].y});
                if (dist < distances[j]) {
                    distances[j] = dist;
                }
            }
        }
        double totalDistance = std::accumulate(distances.begin(), distances.end(), 0.0);
        double randomValue = static_cast<double>(rand()) / RAND_MAX * totalDistance;
        double cumulativeDistance = 0.0;
        for (size_t j = 0; j < nodes.size(); ++j) {
            cumulativeDistance += distances[j];
            if (cumulativeDistance >= randomValue) {
                gateways[i].x = nodes[j].x;
                gateways[i].y = nodes[j].y;
                gateways[i].capacity = gatewayCapacity;
                break;
            }
        }
    }

    // Algorithme K-means
    for (int iter = 0; iter < maxIterations; ++iter) {
        // Assignation des nœuds aux clusters
        for (size_t i = 0; i < nodes.size(); ++i) {
            double minDist = std::numeric_limits<double>::max();
            int bestCluster = -1;

            // Trouver le cluster le plus proche avec de la capacité disponible
            for (int j = 0; j < clusterCount; ++j) {
                if (gateways[j].assignedNodes.size() < gateways[j].capacity) {
                    double dist = calculateDistance(nodes[i], {gateways[j].x, gateways[j].y});
                    if (dist < minDist) {
                        minDist = dist;
                        bestCluster = j;
                    }
                }
            }

            // Si aucun cluster n'a de capacité disponible, assigner au cluster le plus proche
            if (bestCluster == -1) {
                for (int j = 0; j < clusterCount; ++j) {
                    double dist = calculateDistance(nodes[i], {gateways[j].x, gateways[j].y});
                    if (dist < minDist) {
                        minDist = dist;
                        bestCluster = j;
                    }
                }
            }

            // Assigner le nœud au cluster sélectionné
            assignments[i] = bestCluster;
            gateways[bestCluster].assignedNodes.push_back(i);
        }

        // Mise à jour des centroïdes
        for (int j = 0; j < clusterCount; ++j) {
            double sumX = 0.0, sumY = 0.0;
            int count = gateways[j].assignedNodes.size();
            for (int nodeIndex : gateways[j].assignedNodes) {
                sumX += nodes[nodeIndex].x;
                sumY += nodes[nodeIndex].y;
            }
            if (count > 0) {
                gateways[j].x = sumX / count;
                gateways[j].y = sumY / count;
            }
        }

        // Réinitialiser les nœuds assignés pour la prochaine itération
        for (auto& gateway : gateways) {
            gateway.assignedNodes.clear();
        }
    }

    return {gateways, assignments};
}

// Modèle de path loss logarithmique pour calculer le RSSI (en dBm)
double calculateRSSI(const Node& node, const Gateway& gateway) {
    double distance = calculateDistance1(node.x, node.y, gateway.x, gateway.y);
    double tx_power = 14.0;  // Puissance d'émission en dBm
    double path_loss_exponent = 2.9;  // Typique pour LoRa en environnement urbain
    double reference_distance = 1.0;  // Référence en mètres
    double path_loss_db = 0.0;

    if (distance < 1.0) distance = 1.0;  // Éviter log(0)

    path_loss_db = 10 * path_loss_exponent * log10(distance / reference_distance);
    return tx_power - path_loss_db;
}

// Calcul du SNR (Signal-to-Noise Ratio) en dB
double calculateSNR(double rssi) {
    return rssi - NOISE_POWER;
}

// Vérifier si un paquet est perdu en fonction du SNR
bool isPacketLost(double snr) {
    return snr < PACKET_LOSS_THRESHOLD;
}

// Calculer le pourcentage de paquets perdus dans le réseau
double calculatePacketLossRate(const std::vector<Node>& nodes, const std::vector<Gateway>& gateways) {
    int total_packets = nodes.size();
    int lost_packets = 0;

    for (const auto& node : nodes) {
        bool received = false;
        for (const auto& gateway : gateways) {
            double rssi = calculateRSSI(node, gateway);
            double snr = calculateSNR(rssi);

            if (!isPacketLost(snr)) {
                received = true;
                break;
            }
        }

        if (!received) {
            lost_packets++;
        }
    }

    return (static_cast<double>(lost_packets) / total_packets) * 100;
}


// Fonction pour calculer le GCR
double calculateGCR(const std::vector<Node>& nodes, const std::vector<Gateway>& gateways, double rssiThreshold, const std::vector<int>& assignments) {
    int connectedNodes = 0;
    for (size_t i = 0; i < nodes.size(); ++i) { // Utilisez un itérateur pour accéder aux éléments
        int cluster = assignments[i];
        double rssi = calculateRSSI(nodes[i], gateways[cluster]);
        if (rssi > rssiThreshold) {
            connectedNodes++;
        }
    }
    return (static_cast<double>(connectedNodes) / nodes.size()) * 100;
}

// Fonction pour calculer la distance moyenne
double calculateAverageDistance(const std::vector<Node>& nodes, const std::vector<Gateway>& gateways, const std::vector<int>& assignments) {
    double totalDistance = 0.0;
    for (size_t i = 0; i < nodes.size(); ++i) {
        int cluster = assignments[i];
        totalDistance += calculateDistance(nodes[i], {gateways[cluster].x, gateways[cluster].y});
    }
    return totalDistance / nodes.size();
}

int main() {
    std::string filename = "nodes.txt";
    std::string filename1 = "gateways.txt";
    std::vector<Obstacle> obstacles;
    double thresholdDistance = 5.0;

    std::ifstream inputFile("obstacles.txt");
    if (!inputFile) {
        std::cerr << "Erreur : Impossible de lire le fichier obstacles.txt" << std::endl;
        return 1;
    }

    // Génération des nœuds
    auto nodes = generateNodes(N, terrainWidth, terrainHeight);

    // Enregistrement des nœuds dans un fichier
    saveNodesToFile(nodes, filename);

    // Appliquer la méthode du coude pour déterminer le nombre optimal de clusters
    int maxClusters = 20;
    std::vector<double> wcssValues = elbowMethod(nodes, maxClusters);

    // Trouver le nombre optimal de clusters
     //int optimalClusters = findOptimalClusters(wcssValues);
    int optimalClusters =10;
    if (optimalClusters < 1 || optimalClusters > nodes.size()) {
        std::cerr << "Erreur : Nombre de clusters invalide : " << optimalClusters << "\n";
        return 1; // Arrêter le programme ou gérer l'erreur
    }
    // KMeans clustering avec contrainte de capacité
    auto [centroids, assignments] = constrainedKMeans(nodes, optimalClusters, capacity);

    // Les centroïdes sont les gateways
    std::vector<Gateway> gateways = centroids;
    std::cout << "Nombre de passerelles (centroïdes) : " << centroids.size() << "\n";

    // Groupement des nœuds par cluster
    auto clusters = groupNodesByCluster(nodes, assignments, optimalClusters);

    // Repositionner les passerelles si elles sont bloquées
    repositionGatewaysIfBlocked(centroids, obstacles, thresholdDistance);

    // Affichage des centroïdes
    std::cout << "Positions optimales des passerelles (centroïdes) :\n";
    for (const auto& centroid : centroids) {
        std::cout << "x: " << centroid.x << ", y: " << centroid.y << "\n";
    }

    // Enregistrement des passerelles dans un fichier
    saveGatewaysToFile(centroids, filename1);

    // Affichage et sauvegarde des clusters
    printClusters(clusters);
    saveClustersToFile(clusters, "clusters.txt");

    // Vérification des contraintes
    bool allConstraintsSatisfied = true;

    // Vérifier chaque contrainte individuellement
    if (!constraint1(Y)) {
        std::cout << "Contrainte 1 non satisfaite : Chaque nœud doit être servi par au moins une passerelle.\n";
        allConstraintsSatisfied = false;
    }

    if (!constraint2(Y, X)) {
        std::cout << "Contrainte 2 non satisfaite : Une passerelle ne peut servir un nœud que si elle est activée.\n";
        allConstraintsSatisfied = false;
    }

    if (!constraint3(Y, C_r)) {
        std::cout << "Contrainte 3 non satisfaite : La capacité de chaque passerelle ne doit pas être dépassée.\n";
        allConstraintsSatisfied = false;
    }

    if (!constraint4(X, E_r)) {
        std::cout << "Contrainte 4 non satisfaite : L'énergie totale consommée ne doit pas dépasser E_total.\n";
        allConstraintsSatisfied = false;
    }

    if (!constraint5(Z, S_n)) {
        std::cout << "Contrainte 5 non satisfaite : La surface couverte par les nœuds doit être au moins égale à S_terrain.\n";
        allConstraintsSatisfied = false;
    }

    if (!constraint6(nodes, D_min)) {
        std::cout << "Contrainte 6 non satisfaite : La distance minimale entre deux nœuds doit être respectée.\n";
        allConstraintsSatisfied = false;
    }

    // Afficher un message global
    if (allConstraintsSatisfied) {
        std::cout << "Toutes les contraintes sont satisfaites.\n";
    } else {
        std::cout << "Certaines contraintes ne sont pas satisfaites.\n";
    }
    // Calcul du taux de perte de paquets
      double packet_loss_rate = calculatePacketLossRate(nodes, centroids);

      for (const auto& node : nodes) {
          for (const auto& gateway : gateways) {
              double rssi = calculateRSSI(node, gateway);
              double snr = calculateSNR(rssi);
              bool lost = isPacketLost(snr);

              std::cout << "Node: (" << node.x << ", " << node.y << ") -> Gateway: (" << gateway.x << ", " << gateway.y << ")\n";
              std::cout << "  Distance: " << calculateDistance1(node.x, node.y, gateway.x, gateway.y) << " m\n";
              std::cout << "  RSSI: " << rssi << " dBm\n";
              std::cout << "  SNR: " << snr << " dB\n";
              std::cout << "  Packet Lost: " << (lost ? "Yes" : "No") << "\n";
          }
      }
      // Mesurer le temps d'exécution
        auto start = std::chrono::high_resolution_clock::now();

        // ... (exécution de l'algorithme) ...

        auto end = std::chrono::high_resolution_clock::now();
        double elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();

        // Calcul des métriques
        double gcr = calculateGCR(nodes, gateways, -90.0,assignments); // Exemple de seuil RSSI
        double avgDistance = calculateAverageDistance(nodes, gateways, assignments);

        // Affichage des résultats
        std::cout << "GCR: " << gcr << "%" << std::endl;
        std::cout << "Distance moyenne: " << avgDistance << " m" << std::endl;
        std::cout << "WCSS: " << wcssValues.back() << std::endl; // WCSS déjà calculé
        std::cout << "Temps d'exécution: " << elapsed_seconds << " s" << std::endl;
    return 0;
}
