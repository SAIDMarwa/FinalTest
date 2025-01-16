#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

typedef struct {
    double x;
    double y;
} Position;

int isTooClose(Position *positions, int count, double x, double y, double minDistance) {
    for (int i = 0; i < count; i++) {
        double distance = sqrt(pow(x - positions[i].x, 2) + pow(y - positions[i].y, 2));
        if (distance < minDistance) {
            return 1; // Trop proche
        }
    }
    return 0; // Distance acceptable
}

void generatePositions(int numNodes, double minX, double maxX, double minY, double maxY, double minDistance, Position *positions) {
    int count = 0;
    while (count < numNodes) {
        double x = minX + (rand() / (double)RAND_MAX) * (maxX - minX);
        double y = minY + (rand() / (double)RAND_MAX) * (maxY - minY);

        if (!isTooClose(positions, count, x, y, minDistance)) {
            positions[count].x = x;
            positions[count].y = y;
            count++;
        }
    }
}

int main() {
    int numNodes = 10;                // Nombre de nœuds
    double areaMinX = 0;              // Limite minimale de X
    double areaMaxX = 10000;           // Limite maximale de X
    double areaMinY = 0;              // Limite minimale de Y
    double areaMaxY = 10000;           // Limite maximale de Y
    double minDistance = 50;          // Distance minimale entre les nœuds

    Position positions[numNodes];
    srand(time(NULL));  // Initialisation de la graine pour les nombres aléatoires

    generatePositions(numNodes, areaMinX, areaMaxX, areaMinY, areaMaxY, minDistance, positions);

    // Enregistrement dans le fichier
    FILE *file = fopen("node_positions.txt", "w");
    if (file == NULL) {
        printf("Erreur lors de l'ouverture du fichier.\n");
        return 1;
    }

    for (int i = 0; i < numNodes; i++) {
        fprintf(file, "**.loRaNodes[%d].initialX = %.2fm\n", i, positions[i].x);
        fprintf(file, "**.loRaNodes[%d].initialY = %.2fm\n", i, positions[i].y);
    }

    fclose(file);
    printf("Positions générées et enregistrées dans node_positions.txt\n");

    return 0;
}
