// implémentation A* pour simulation avec MATLAB
// Compilation: g++ -Astar_Algorithm.cpp -o Astar_Algorithm
// Compilation mex: mex myAStar.cpp

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <queue>
#include <unordered_set>
#include <string>
#include <algorithm>

#include "mex.h"
#include <vector>

struct Node_s
{
    std::array<double, 3> state;
    double g;
    double h;
    Node_s *parent;
    double poids() const { return g + h; }
};

// renvoie true si a est "moins prioritaire" que b
struct CompareNode
{
    bool operator()(const Node_s *a, const Node_s *b) const
    {
        return a->poids() > b->poids(); // donc le plus petit poids passe devant
    }
};

// angle compris entre [-pi; pi]
double
wrapToPi(double theta)
{
    if (theta < -M_PI)
    {
        double tmp = theta;
        while (tmp < -M_PI)
        {
            tmp += 2 * M_PI;
        }
        return tmp;
    }
    else if (theta > M_PI)
    {
        double tmp = theta;
        while (tmp > M_PI)
        {
            tmp -= 2 * M_PI;
        }
        return tmp;
    }
    else
    {
        return theta;
    }
}

// A faire : définir types des états
double heuristic(const std::array<double, 3> &state, const std::array<double, 3> &goal)
{
    double dx = state[0] - goal[0];
    double dy = state[1] - goal[1];
    double dtheta = wrapToPi(state[2] - goal[2]);

    return std::hypot(dx, dy) + abs(dtheta) * 0.05;
}

std::vector<std::array<double, 3>> getNeighbors(const std::array<double, 3> &state, mwSize rows, mwSize cols)
{
    int cellPerMeter = 10;
    double d = 1.0 / cellPerMeter;
    double dtheta = M_PI / 2;

    double x = state[0];
    double y = state[1];
    double theta = state[2];
    double c = d * cos(state[2]);
    double s = d * sin(state[2]);

    std::vector<std::array<double, 3>> neighbors = {
        {x + c, y + s, theta},                    // tout droit
        {x + s, y - c, wrapToPi(theta + dtheta)}, // tourner à gauche avant déplacement
        {x - s, y + c, wrapToPi(theta - dtheta)}  // tourner à droite avant déplacement
    };

    // Filtrage des voisins en dehors de la carte
    std::vector<std::array<double, 3>> validNeighbors;
    for (const auto &n : neighbors)
    {
        int i = static_cast<int>(n[0] * cellPerMeter);
        int j = static_cast<int>(n[1] * cellPerMeter);

        if (i >= 0 && i < static_cast<int>(rows) && j >= 0 && j < static_cast<int>(cols))
        {
            validNeighbors.push_back(n);
        }
    }

    mexPrintf("Liste voisins\n");
    int i = 1;
    for (const std::array<double, 3> &n : validNeighbors)
    {
        mexPrintf("\tvoisin %d : %f, %f, %f\n", i, n[0] * 10, n[1] * 10, n[2]);
        i++;
    }
    mexPrintf("Fin liste voisins\n");

    return validNeighbors;
}

std::string stateTokey(const std::array<double, 3> &state)
{
    int x = static_cast<int>(round(state[0] * 100));
    int y = static_cast<int>(round(state[1] * 100));
    int theta = static_cast<int>(round(wrapToPi(state[2]) * 100));

    return std::to_string(x) + "_" + std::to_string(y) + "_" + std::to_string(theta);
}

bool goalReach(std::array<double, 3> state, std::array<double, 3> goal)
{
    double dx = state[0] - goal[0];
    double dy = state[1] - goal[1];
    double dtheta = wrapToPi(state[2] - goal[2]);

    return (std::hypot(dx, dy) < 0.05 && std::abs(dtheta) < 0.1);
}

// Astar_Algorithm(start, goal, map)
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nrhs != 3)
    {
        mexErrMsgTxt("Usage: path = myAStar(start, goal, map)");
    }
    // ne peuvent pas correspondre au coordonnées dans le tableau
    // a faire : adapter en fonction du nombre de cellule par metre
    // a faire : assert dans la grille
    double *startTmp = mxGetPr(prhs[0]); // start[0]=x, start[1]=y, start[2]=theta
    double *goalTmp = mxGetPr(prhs[1]);

    std::array<double, 3> goalState;
    std::array<double, 3> startState;
    for (int i = 0; i < 3; i++)
    {
        goalState[i] = goalTmp[i];
        startState[i] = startTmp[i];
    }

    mexPrintf("%f %f %f\n", startState[0] * 10, startState[1] * 10, startState[2]);
    mexPrintf("%f %f %f\n", goalState[0] * 10, goalState[1] * 10, goalState[2]);

    mxArray *map = (mxArray *)prhs[2];
    double *mapData = mxGetPr(map);
    mwSize rows = mxGetM(map);
    mwSize cols = mxGetN(map);
    // /!\ stockage matrice en colonne majeure : mapData[i + j * rows] avec i le num de la ligne et j la colonne

    mexPrintf("test : %d\t", static_cast<int>(1.0));
    mexPrintf("test : %d\n", static_cast<int>(0.0));
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            mexPrintf("| %d ", static_cast<int>(round(mapData[i + j * rows])));
        }
        mexPrintf("|\n");
    }
    // mexPrintf("rows : %zu", rows);
    // mexPrintf("cols : %zu", cols);

    // validationState

    std::vector<Node_s *> allNodes;
    std::vector<std::array<double, 3>> exploredStates; // open and close state
    std::priority_queue<Node_s *, std::vector<Node_s *>, CompareNode> openList;
    std::unordered_set<std::string> closeList;

    Node_s *startNode = new Node_s;
    for (int i = 0; i < 3; i++)
    {
        startNode->state[i] = startState[i];
    }
    // startNode.state = startState;
    startNode->g = 0;
    startNode->h = heuristic(startState, goalState);
    startNode->parent = NULL;

    openList.push(startNode);
    allNodes.push_back(startNode);

    Node_s *current;

    mexPrintf("Exploration des noeuds\n");

    while (!openList.empty())
    {
        current = openList.top();
        openList.pop();

        exploredStates.push_back(current->state);

        mexPrintf("%f %f %f ouvert\n", current->state[0] * 10, current->state[1] * 10, current->state[2]);

        if (goalReach(current->state, goalState))
        {
            // a faire : j'aime pas, enlever break mieux gérer la condition d'arret du while
            mexPrintf("Objectif atteint\n");
            break;
        }
        // condition d'arret ? -> victoire
        // une liste de triplet [x, y, θ]
        std::vector<std::array<double, 3>> neighbors = getNeighbors(current->state, rows, cols);

        for (const std::array<double, 3> &neighbor : neighbors)
        {
            mexPrintf("neighbor actuel : %f, %f, %f\n", neighbor[0] * 10, neighbor[1] * 10, neighbor[2] * 10);
            // a faire : adapter en fonction du nombre de cellule par metre
            int i = static_cast<int>(neighbor[0] * 10);
            int j = static_cast<int>(neighbor[1] * 10);

            if (i < 0 || i >= rows || j < 0 || j >= cols)
            {
                continue;
            }
            if (static_cast<int>(round(mapData[i + j * rows])) == 1) // /!\ stockage matrice en colonne majeure : mapData[i + j * rows] avec i le num de la ligne et j la colonne
            {                                                        // si c'est un mur, ignorer
                mexPrintf("Mur\n");
                continue;
            }
            std::string test = stateTokey(neighbor);
            if (closeList.count(test) == 1)
            { // si l'état est fermé
                mexPrintf("Fermé\n");
                continue;
            }

            // pas d'amélioration si l'état est ouvert mais pas encore fermé ?

            Node_s *newNode = new Node_s;
            newNode->state = neighbor;
            double dist = std::hypot(neighbor[0] - current->state[0], neighbor[1] - current->state[1]);
            newNode->g = current->g + dist + 0.05 * std::abs(neighbor[2] - current->state[2]);
            newNode->h = heuristic(neighbor, goalState);
            newNode->parent = current;

            mexPrintf("Nouveau noeuds : \n \t State : %f, %f, %f\n\tHeuristique : %f\n\tCout : %f\n\tPoids : %f\n", newNode->state[0] * 10, newNode->state[1] * 10, newNode->state[2], newNode->h, newNode->g, newNode->poids());

            openList.push(newNode);
            allNodes.push_back(newNode);
        }
        closeList.insert(stateTokey(current->state));
        mexPrintf("Fermeture noeud\n");
        // delete current;
        // a faire : on ne peut pas delete ici car on utilise current ensuite pour retrouver le chemin
    }

    mexPrintf("Fin d'exploration\n");
    std::vector<std::array<double, 3>> path;
    while (current != NULL)
    {
        path.push_back(current->state);
        mexPrintf("boucle 2 : %f, %f, %f\n", current->state[0], current->state[1], current->state[2]);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());

    // std::vector<std::array<double, 3>> path = {{startState[0], startState[1], startState[2]}, {goalState[0], goalState[1], goalState[2]}};
    // std::vector<std::array<double, 3>> tmpReversed = {{goalState[0], goalState[1], goalState[2]}};
    // while (current->parent != NULL)
    // {
    //     current = current->parent;
    //     mexPrintf("boucle 2 : %f, %f, %f\n", current->state[0], current->state[1], current->state[2]);
    //     tmpReversed.push_back(current->state);
    // }
    // mexPrintf("Path\n");
    // while (!tmpReversed.empty())
    // {
    //     std::array<double, 3> tmp = tmpReversed.back();
    //     mexPrintf("%f, %f, %f\n", tmp[0], tmp[1], tmp[2]);
    //     path.push_back(tmp);
    //     tmpReversed.pop_back();
    // }

    for (Node_s *node : allNodes)
    {
        delete node;
    }

    // Convert result to MATLAB output
    mwSize dims[2] = {path.size(), 2};
    plhs[0] = mxCreateDoubleMatrix(dims[0], dims[1], mxREAL);
    double *out = mxGetPr(plhs[0]);

    for (size_t i = 0; i < path.size(); ++i)
    {
        out[i] = path[i][0];           // x
        out[i + dims[0]] = path[i][1]; // y (column-major)
    }

    if (nlhs > 1)
    {
        mwSize dimsExplored[2] = {exploredStates.size(), 2};
        plhs[1] = mxCreateDoubleMatrix(dimsExplored[0], dimsExplored[1], mxREAL);
        double *outExplored = mxGetPr(plhs[1]);

        for (size_t i = 0; i < exploredStates.size(); ++i)
        {
            outExplored[i] = exploredStates[i][0];                   // x
            outExplored[i + dimsExplored[0]] = exploredStates[i][1]; // y
        }
    }
}