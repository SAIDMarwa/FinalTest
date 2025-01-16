#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <fstream>

const int terrainWidth = 1000;  // Largeur du terrain (en mètres)
const int terrainHeight = 1000; // Hauteur du terrain (en mètres)
const int numNodes = 100; // Nombre de nœuds
const double rayonN = 50.0; // Rayon de couverture des capteurs (en mètres)
const double minDistance = 10.0; // Distance minimale entre deux nœuds (en mètres)
const int J = 10000; // Nombre de points à couvrir sur le terrain
const int numGateways = 10;  // Nombre de passerelles
const int capacity = 20;  // Capacité d'une passerelle (en nombre de nœuds gérés)
const double energy = 10.0; // Consommation d'énergie d'une passerelle (en watts)
const double E_total = 1000.0; // Consommation d'énergie totale admissible (en watts)
const double terrainArea = terrainWidth * terrainHeight;  // Surface totale du terrain (en m²)
double D_nn[numNodes][numNodes];
#define MAX_ITERATIONS 500
const int rayon =150; // rayon de couverture des gateways (en mètres)
const double TOLERANCE = 1e-6;
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
};

// Structure pour représenter un obstacle
struct Obstacle {
    double x, y, height; // Coordonnées et hauteur de l'obstacle
};

// Fonction pour calculer la distance entre un nœud et une passerelle
double distance(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

// Fonction pour vérifier si une passerelle est bloquée par un obstacle
bool isGatewayBlockedByObstacle(const Gateway& gateway, const std::vector<Obstacle>& obstacles) {
    for (const auto& obstacle : obstacles) {
        // Vérification de la proximité d'un obstacle à la passerelle
        double dist = sqrt(pow(gateway.x - obstacle.x, 2) + pow(gateway.y - obstacle.y, 2));
        if (dist < rayon && obstacle.height > 0) { // Si l'obstacle est assez proche et a une hauteur
            return true; // L'obstacle bloque la passerelle
        }
    }
    return false; // Pas de blocage
}

// Fonction pour charger les obstacles depuis un fichier texte
void loadObstaclesFromFile(const char* filename, std::vector<Obstacle>& obstacles) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Erreur d'ouverture du fichier des obstacles." << std::endl;
        return;
    }

    double x, y, height;
    while (file >> x >> y >> height) {
        obstacles.push_back({x, y, height});
    }

    file.close();
}

// Fonction pour sauvegarder les obstacles dans un fichier texte
void saveObstaclesToFile(const std::vector<Obstacle>& obstacles, const char* filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Erreur d'ouverture du fichier pour sauvegarder les obstacles." << std::endl;
        return;
    }

    for (const auto& obstacle : obstacles) {
        file << obstacle.x << " " << obstacle.y << " " << obstacle.height << std::endl;
    }

    file.close();
}

// Fonction pour assigner les noeuds aux passerelles les plus proches
void assignNodesToGateways(const std::vector<Node> &nodes, const std::vector<Gateway> &gateways, int assignments[]) {
    for (int i = 0; i < nodes.size(); ++i) {
        double minDistance = distance(nodes[i].x, nodes[i].y, gateways[0].x, gateways[0].y);
        int clusterIndex = 0;
        for (int j = 1; j < gateways.size(); ++j) {
            double dist = distance(nodes[i].x, nodes[i].y, gateways[j].x, gateways[j].y);
            if (dist < minDistance) {
                minDistance = dist;
                clusterIndex = j;
            }
        }
        assignments[i] = clusterIndex;
    }
}

// Fonction pour mettre à jour les positions des passerelles
void updateGateways(const std::vector<Node>& nodes, std::vector<Gateway>& gateways, int assignments[], int numNodes, int numGateways, const std::vector<Obstacle>& obstacles) {
    for (int i = 0; i < numGateways; i++) {
        double sumX = 0, sumY = 0;
        int count = 0;

        // Calculer le centre de chaque cluster
        for (int j = 0; j < numNodes; j++) {
            if (assignments[j] == i) {
                sumX += nodes[j].x;
                sumY += nodes[j].y;
                count++;
            }
        }

        if (count > 0) {
            gateways[i].x = sumX / count;
            gateways[i].y = sumY / count;
        } else {
            // Si aucun nœud n'est assigné à cette passerelle, réinitialiser à une position aléatoire
            gateways[i].x = (rand() % 100);  // Valeur aléatoire, ajuster la plage selon votre environnement
            gateways[i].y = (rand() % 100);
        }

        // Vérifier si la passerelle est bloquée par un obstacle
        if (isGatewayBlockedByObstacle(gateways[i], obstacles)) {
            std::cout << "La passerelle " << i << " est bloquée par un obstacle. Nouvelle position requise." << std::endl;
            gateways[i].x = rand() % terrainWidth;  // Nouvelle position aléatoire
            gateways[i].y = rand() % terrainHeight;
        }
        std::cerr << "Gateway " << i << gateways[i].x << ", " << gateways[i].y << ")" << std::endl;
    }
    std::cerr << "TESTGA" << std::endl;
}


// Fonction K-means
void kMeans(const std::vector<Node> &nodes, std::vector<Gateway> &gateways, const std::vector<Obstacle> &obstacles) {
    if (nodes.empty() || gateways.empty()) {
        std::cerr << "Erreur : Nombre de noeuds ou de passerelles invalide." << std::endl;
        return;
    }
    std::vector<int> assignments(nodes.size(), -1);
    int iterations = 0;

    std::cerr << "Début de kMeans" << std::endl;
    while (iterations < MAX_ITERATIONS) {
        assignNodesToGateways(nodes, gateways, assignments.data());
        std::vector<Gateway> oldGateways = gateways;

        // Mettre à jour les positions des passerelles
        for (int i = 0; i < gateways.size(); ++i) {
            double sumX = 0, sumY = 0;
            int count = 0;
            for (int j = 0; j < nodes.size(); ++j) {
                if (assignments[j] == i) {
                    sumX += nodes[j].x;
                    sumY += nodes[j].y;
                    ++count;
                }
            }
            if (count > 0) {
                gateways[i].x = sumX / count;
                gateways[i].y = sumY / count;
            } else {
                gateways[i].x = rand() % terrainWidth;
                gateways[i].y = rand() % terrainHeight;
            }
        }

        // Vérifier les déplacements des passerelles
        bool assignmentsChanged = false;
        for (int i = 0; i < gateways.size(); ++i) {
            double deltaX = std::abs(gateways[i].x - oldGateways[i].x);
            double deltaY = std::abs(gateways[i].y - oldGateways[i].y);
            if (deltaX > TOLERANCE || deltaY > TOLERANCE) {
                assignmentsChanged = true;
                std::cerr << "Passerelle " << i << " déplacée à (" << gateways[i].x << ", " << gateways[i].y << ")" << std::endl;
            }
        }

        if (!assignmentsChanged) break;
        ++iterations;
    }

    std::cerr << "Fin de kMeans après " << iterations << " itérations" << std::endl;
}

// Fonction pour calculer l'inertie
double calculateInertia(const std::vector<Node>& nodes, const std::vector<Gateway>& gateways, int assignments[], int numNodes, int numGateways) {
    double inertia = 0.0;
    for (int i = 0; i < numNodes; i++) {
        double dist = pow(nodes[i].x - gateways[assignments[i]].x, 2) +
                      pow(nodes[i].y - gateways[assignments[i]].y, 2);
        inertia += dist;
    }
    return inertia;
}

// Fonction pour trouver le nombre optimal de clusters
int findOptimalClusters(const std::vector<Node>& nodes, int numNodes,const std::vector<Obstacle>& obstacles) {
    const int maxClusters = 10;  // Tester jusqu'à 10 clusters
    double inertias[maxClusters];
    int optimalK = 1;

    std::cerr << "ht" << std::endl;
    for (int k = 1; k <= maxClusters; k++) {
        std::vector<Gateway> gateways(k);
        int assignments[numNodes];
        std::cerr << "ht1" << std::endl;
        // Initialiser des passerelles aléatoires
        for (int i = 0; i < k; i++) {
            gateways[i].x = nodes[i % numNodes].x;
            gateways[i].y = nodes[i % numNodes].y;
            std::cerr <<  gateways[i].x << std::endl;
        }

        // Appliquer K-Means
        kMeans(nodes, gateways, obstacles);


        std::cerr << "ht2" << std::endl;
        // Calculer l'inertie
        assignNodesToGateways(nodes, gateways, assignments);
        inertias[k - 1] = calculateInertia(nodes, gateways, assignments, numNodes, k);
    }

    // Trouver le coude
    for (int i = 1; i < maxClusters - 1; i++) {
        double decrease1 = inertias[i - 1] - inertias[i];
        double decrease2 = inertias[i] - inertias[i + 1];
        if (decrease1 > decrease2) {
            optimalK = i;
            break;
        }
    }

    return optimalK;

}



// Fonction pour vérifier la distance minimale entre les nœuds
bool checkMinDistance(const std::vector<Node>& nodes, double D_min) {
    for (size_t i = 0; i < nodes.size(); ++i) {
        for (size_t j = i + 1; j < nodes.size(); ++j) {
            double distance = std::sqrt(std::pow(nodes[i].x - nodes[j].x, 2) + std::pow(nodes[i].y - nodes[j].y, 2));
            if (distance < D_min) {
                return false;  // Si la distance entre deux nœuds est inférieure à D_min, retourner false
            }
        }
    }
    return true;
}



// Calculer la distance entre deux points
double calculateDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

// Fonction objective : f_N
int calculateObjective_fN(const std::vector<Node>& nodes) {
    int activeNodes = 0;
    for (const auto& node : nodes) {
        if (node.isActive) {
            activeNodes++;
        }
    }
    return activeNodes;
}

// Fonction objective : f_C
double calculateObjective_fC(const std::vector<Node>& nodes, const std::vector<Gateway>& gateways) {
    double totalDistance = 0.0;
    for (const auto& node : nodes) {
        if (!node.isActive) continue;
        double minDistance = 1e9; // Distance minimale pour un nœud à une passerelle
        for (const auto& gateway : gateways) {
            double distance = calculateDistance(node.x, node.y, gateway.x, gateway.y);
            minDistance = std::min(minDistance, distance);
        }
        totalDistance += minDistance;
    }
    return totalDistance;
}

// Fonction objective : f_E
double calculateObjective_fE(const std::vector<Gateway>& gateways) {
    double totalEnergy = 0.0;
    for (const auto& gateway : gateways) {
        totalEnergy += gateway.energy;
    }
    return totalEnergy;
}

// Vérification des contraintes
bool verifyConstraints(const std::vector<Node>& nodes, const std::vector<Gateway>& gateways, double minDistance, double terrainArea) {
    // Vérification de la distance minimale entre nœuds
    for (size_t i = 0; i < nodes.size(); ++i) {
        for (size_t j = i + 1; j < nodes.size(); ++j) {
            if (calculateDistance(nodes[i].x, nodes[i].y, nodes[j].x, nodes[j].y) < minDistance) {
                std::cerr << "Distance minimale" <<minDistance<< std::endl;
                std::cerr << "Distance" <<calculateDistance(nodes[i].x, nodes[i].y, nodes[j].x, nodes[j].y) << std::endl;
                std::cerr << "Contrainte non respectée : distance minimale entre les nœuds." << std::endl;
                return false;
            }
        }
        // Calcul des distances entre les nœuds matrice des distances
        /*  for (size_t i = 0; i < nodes.size(); ++i) {
              for (size_t j = 0; j < nodes.size(); ++j) {
                  D_nn[i][j] = sqrt(pow(nodes[i].x - nodes[j].x, 2) + pow(nodes[i].y - nodes[j].y, 2));
                  if(D_nn[i][j]<minDistance){

                  }
              }
          }
        // Affichage de la matrice de distances
           std::cout << "Matrice des distances :" << std::endl;
           for (size_t i = 0; i < nodes.size(); ++i) {
               for (size_t j = 0; j < nodes.size(); ++j) {
                   std::cout << D_nn[i][j] << " ";
               }
               std::cout << std::endl;
           }*/
    }

    // Vérification de la couverture du terrain
    double coveredArea = 0.0;
    for (const auto& node : nodes) {
        if (node.isActive) {
            coveredArea += M_PI * std::pow(rayonN, 2);
        }
    }
    std::cerr << coveredArea << std::endl;
    if (coveredArea < terrainArea) {
        std::cerr << "Contrainte non respectée : couverture du terrain insuffisante." << std::endl;
        return false;
    }

    // Vérification de la capacité des passerelles
    for (const auto& gateway : gateways) {
        int connectedNodes = 0;
        for (const auto& node : nodes) {
            double distance = calculateDistance(node.x, node.y, gateway.x, gateway.y);
            if (distance <= gateway.rayon) { // Exemple : rayon de couverture d'une passerelle
                connectedNodes++;
            }
        }
        if (connectedNodes > gateway.capacity) {
            std::cerr << "Contrainte non respectée : capacité de la passerelle dépassée." << std::endl;
            return false;
        }
    }

    return true;
}

void writeNodePositionsToFile(const std::vector<Node>& nodes, const std::string& filename) {
    std::ofstream file(filename);  // Ouvre le fichier en mode écriture
    if (!file) {
        std::cerr << "Erreur d'ouverture du fichier !" << std::endl;
        return;
    }

    // Écrire les positions des nœuds dans le fichier avec le format spécifié
    for (size_t i = 0; i < nodes.size(); ++i) {
        file << "**.loRaNodes[" << i << "].**.initialX = " << nodes[i].x << "m" << std::endl;
        file << "**.loRaNodes[" << i << "].**.initialY = " << nodes[i].y << "m" << std::endl;
    }

    file.close();  // Fermer le fichier
}

void writeGatewayPositionsToFile(const std::vector<Gateway>& gateways, const std::string& filename) {
    std::ofstream file(filename);  // Ouvre le fichier en mode écriture
    if (!file) {
        std::cerr << "Erreur d'ouverture du fichier !" << std::endl;
        return;
    }

    // Écrire les positions des nœuds dans le fichier avec le format spécifié
    for (size_t i = 0; i < gateways.size(); ++i) {
        file << "**.loRaGW[" << i << "].**.initialX = " << gateways[i].x << "m" << std::endl;
        file << "**.loRaGW[" << i << "].**.initialY = " << gateways[i].y << "m" << std::endl;
    }

    file.close();  // Fermer le fichier
}
int main() {

    std::srand(std::time(0)); // Initialisation du générateur aléatoire
    // Charger les obstacles depuis un fichier
    std::vector<Obstacle> obstacles;
        loadObstaclesFromFile("obstacles.txt", obstacles);
    // Génération des nœuds
    std::vector<Node> nodes(numNodes);
    std::vector<Gateway> gateways(numGateways);
    int assignments[numNodes];
    // Générer les nœuds aléatoirement
        for (int i = 0; i < numNodes; i++) {
            nodes[i].x = rand() % terrainWidth;
            nodes[i].y = rand() % terrainHeight;
            nodes[i].isActive = true;
            nodes[i].rayonN = rayonN;
        }
       std::cout << "Nombre de nœuds générés : " << nodes.size() << std::endl;
       writeNodePositionsToFile(nodes, "nodes.txt");
       std::cerr << "test" << std::endl;

    // Trouver le nombre optimal de clusters (passerelles)
    int optimalClusters = findOptimalClusters(nodes, numNodes, obstacles);
    std::cout << "Nombre optimal de passerelles : " << optimalClusters << std::endl;

         // Génération des passerelles
         for (int i = 0; i < optimalClusters; i++) {
                 gateways[i].x = rand() % terrainWidth;
                 gateways[i].y = rand() % terrainHeight;
                 gateways[i].capacity = capacity;
                 gateways[i].energy = energy;
                 gateways[i].rayon = rayon;
             }

         // Enregistrer les passerelles dans un fichier
             writeGatewayPositionsToFile(gateways, "gateways.txt");
    // Vérification des contraintes
    /*if (!verifyConstraints(nodes, gateways, minDistance, terrainArea)) {
        std::cerr << "Échec des contraintes. Veuillez ajuster les paramètres." << std::endl;
        return 1;
    }
*/
    // Calcul des fonctions objectives
    int fN = calculateObjective_fN(nodes);
    double fC = calculateObjective_fC(nodes, gateways);
    double fE = calculateObjective_fE(gateways);

    // Affichage des résultats
    std::cout << "Fonctions objectives :" << std::endl;
    std::cout << "f_N (Nombre de nœuds actifs) = " << fN << std::endl;
    std::cout << "f_C (Distance cumulée) = " << fC << std::endl;
    std::cout << "f_E (Énergie consommée) = " << fE << std::endl;





      return 0;
}
