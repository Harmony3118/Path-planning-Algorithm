// implémentation A* pour simulation avec MATLAB
// Compilation: g++ -Astar_Algorithm.cpp -o Astar_Algorithm
// Compilation mex: mex myAStar.cpp

// il faut tester la compilation automatique

#include <math.h>
#include "mex.h"

#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <vector>

#include <string>
#include <algorithm>

struct State
{
    double x, y, theta;

    bool operator==(const State &other) const
    {
        return abs(x - other.x) < 1e-3 &&
               abs(y - other.y) < 1e-3 &&
               abs(theta - other.theta) < 1e-2;
    }
};

namespace std
{
    template <>
    struct hash<State>
    {
        size_t operator()(const State &s) const
        {
            return hash<double>()(round(s.x * 100)) ^
                   hash<double>()(round(s.y * 100)) ^
                   hash<double>()(round(s.theta * 100));
        }
    };
}

struct Node
{
    State state;
    double g;
    double h;
    double poids() const { return g + h; }

    // renvoie true si a est "moins prioritaire" que b
    bool operator>(const Node &other) const
    {
        return poids() > other.poids();
    }
};

struct AStartResult
{
    std::vector<State> path;
    std::vector<State> explored;
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

double heuristic(const State &state, const State &goal)
{
    double dx = state.x - goal.x;
    double dy = state.y - goal.y;
    double dtheta = wrapToPi(state.theta - goal.theta);

    return std::hypot(dx, dy) + abs(dtheta) * 0.05;
}

std::vector<State> getNeighbors(const State &state, double *map, mwSize rows, mwSize cols)
{
    int cellPerMeter = 10;
    double d = 1.0 / cellPerMeter;
    double dtheta = M_PI / 2;

    double x = state.x;
    double y = state.y;
    double theta = state.theta;
    double c = d * cos(state.theta);
    double s = d * sin(state.theta);

    std::vector<State> neighbors = {
        {x + c, y + s, theta},                    // tout droit
        {x - s, y + c, wrapToPi(theta + dtheta)}, // tourner à gauche avant déplacement
        {x + s, y - c, wrapToPi(theta - dtheta)}  // tourner à droite avant déplacement
    };

    // Filtrage des voisins en dehors de la carte
    std::vector<State> validNeighbors;
    for (const auto &n : neighbors)
    {
        int i = static_cast<int>(n.x * cellPerMeter);
        int j = rows - 1 - static_cast<int>(n.y * cellPerMeter); // c++ lign-major while matlab column-major

        bool inLimits = i >= 0 && i < static_cast<int>(cols) && j >= 0 && j < static_cast<int>(rows);
        bool isWall = static_cast<int>(round(map[j + i * rows])) == 1;

        if (inLimits && !isWall)
        {
            validNeighbors.push_back(n);
        }
    }

    return validNeighbors;
}

bool goalReach(const State &state, const State &goal)
{
    return heuristic(state, goal) < 0.1;
}

AStartResult AStar(const State &start, const State &goal, double *map, mwSize rows, mwSize cols)
{
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openList;
    std::unordered_map<State, double> gCost;   // dictionnaire key = state, data = cost
    std::unordered_map<State, State> cameFrom; // dictionnaire key = state, data = parent
    std::vector<State> exploredStates;

    openList.push({start, 0.0, heuristic(start, goal)});
    gCost[start] = 0.0;

    while (!openList.empty())
    {
        Node current = openList.top();
        openList.pop();

        exploredStates.push_back(current.state);

        if (goalReach(current.state, goal))
        {
            std::vector<State> path;
            State s = current.state;
            while (cameFrom.count(s))
            {
                path.push_back(s);
                s = cameFrom[s];
            }
            path.push_back(start);
            reverse(path.begin(), path.end());

            return {path, exploredStates};
        }

        std::vector<State> neighbors = getNeighbors(current.state, map, rows, cols);
        for (const State &neighbor : neighbors)
        {
            double dtheta = 0.05 * std::abs(wrapToPi(current.state.theta - neighbor.theta));
            double tryNewG = gCost[current.state] + 0.1 + dtheta;

            if (!gCost.count(neighbor) || tryNewG < gCost[neighbor])
            {
                gCost[neighbor] = tryNewG;
                cameFrom[neighbor] = current.state;
                double h = heuristic(neighbor, goal);
                openList.push({neighbor, tryNewG, h});
            }
        }
    }
    std::vector<State> path;
    path.push_back(start);
    return {path, exploredStates};
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    // a faire : adapter en fonction du nombre de cellule par metre
    if (nrhs != 3)
    {
        mexErrMsgTxt("Usage: path = Astar_Algorithm_v2(start, goal, map)");
    }
    double *startTmp = mxGetPr(prhs[0]);
    double *goalTmp = mxGetPr(prhs[1]);

    State start = {startTmp[0], startTmp[1], startTmp[2]};
    State goal = {goalTmp[0], goalTmp[1], goalTmp[2]};

    mxArray *map = (mxArray *)prhs[2];
    double *mapData = mxGetPr(map);
    mwSize rows = mxGetM(map);
    mwSize cols = mxGetN(map);

    // for (int j = 0; j < rows; j++) // ligne en haut dans le code c++ mais en bas dans matlab
    // {
    //     for (int i = 0; i < cols; i++)
    //     {
    //         mexPrintf("| %d ", static_cast<int>(round(mapData[j + i * rows])));
    //     }
    //     mexPrintf("|\n");
    // }

    auto [path, explored] = AStar(start, goal, mapData, rows, cols);

    mwSize dims[3] = {path.size(), 3};
    plhs[0] = mxCreateDoubleMatrix(path.size(), 3, mxREAL);
    // plhs[0] = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS, mxREAL);
    double *out = mxGetPr(plhs[0]);

    for (size_t i = 0; i < path.size(); i++)
    {
        out[i] = path[i].x;
        out[i + dims[0]] = path[i].y;         // column-major
        out[i + 2 * dims[0]] = path[i].theta; // column-major
    }

    if (nlhs > 1)
    {
        mwSize dimsExplored[3] = {explored.size(), 3};
        plhs[1] = mxCreateDoubleMatrix(explored.size(), 3, mxREAL);
        // plhs[1] = mxCreateDoubleMatrix(dimsExplored[0], dimsExplored[1], dimsExplored[2], mxREAL);
        // plhs[1] = mxCreateNumericArray(3, dimsExplored, mxDOUBLE_CLASS, mxREAL);
        double *outExplored = mxGetPr(plhs[1]);

        for (size_t i = 0; i < explored.size(); ++i)
        {
            outExplored[i] = explored[i].x;                           // x
            outExplored[i + dimsExplored[0]] = explored[i].y;         // y
            outExplored[i + 2 * dimsExplored[0]] = explored[i].theta; // theta
        }
    }
}