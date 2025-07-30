#include <math.h>
#include "mex.h"

#include <iostream>
#include <random>
#include <vector>
#include <unordered_map>
#include <array>
#include <algorithm>

// a faire : définir struct pour la map histoire de ne pas avoir à passer en argument ligne, colonne et carte à chaque fois

struct State
{
    int x, y;
    double theta;

    bool operator==(const State &other) const
    {
        return x == other.x && y == other.y && std::abs(theta - other.theta) < 1e-6;
    }
};

// To use State as a key in unordered_map (dictionnary)
namespace std
{
    template <>
    struct hash<State>
    {
        size_t operator()(const State &s) const
        {
            return hash<int>()(s.x) ^
                   (hash<int>()(s.y) << 1) ^
                   (hash<int>()(static_cast<int>(s.theta * 100)) << 2);
        }
    };
}

double distance(const State &s1, const State &s2)
{
    int dx = s1.x - s2.x;
    int dy = s1.y - s2.y;
    return std::hypot(dx, dy);
}

// Binary search tree definition for 2 dimensions used to find the nearest node in the RRT tree
// Node, insert, destroy functions
struct ABR2dimNode
{
    State state;
    ABR2dimNode *left = nullptr;
    ABR2dimNode *right = nullptr;

    int depth;

    ABR2dimNode(const State &s, int d) : state(s), depth(d) {}
};

class ABR2dimTree
{
public:
    ABR2dimNode *root;

    ABR2dimTree(const State &start)
    {
        root = new ABR2dimNode(start, 0);
    }

    ~ABR2dimTree()
    {
        destroyTree(root);
    }

    bool insertState(State &state)
    {
        return addNode(state, root, 0);
    }

private:
    // Recursive function to add a node to the tree
    // Returns true if the node was added, false if it already exists
    bool addNode(const State &state, ABR2dimNode *&root, int depth = 0)
    {
        if (root == nullptr)
        {
            root = new ABR2dimNode(state, depth);
            return true;
        }

        if (state == root->state)
        {
            return false;
        }

        int var = depth % 2; // 0 for x, 1 for y
        if ((var == 0 && state.x < root->state.x) ||
            (var == 1 && state.y < root->state.y))
        {
            return addNode(state, root->left, depth + 1);
        }
        else
        {
            return addNode(state, root->right, depth + 1);
        }
    }

    void destroyTree(ABR2dimNode *&root)
    {
        if (root != nullptr)
        {
            destroyTree(root->left);
            destroyTree(root->right);
            delete root;
            root = nullptr;
        }
    }
};

template <typename T>
T getCoordonnate(T low, T high)
{
    static std::random_device random_device;
    static std::mt19937 engine{random_device()};

    std::uniform_int_distribution<int> dist(low, high);
    // dist est une distribution uniforme
    return dist(engine);
};

struct RRTResult
{
    std::vector<State> path;
    std::unordered_map<State, State> RRTTreeParent;
};

State pickRandomPoint(const int &rows, const int &cols, const double *&map)
{
    State newState;
    bool isWall = true;
    while (isWall)
    {
        newState.x = getCoordonnate(0, cols);

        newState.y = getCoordonnate(0, rows);
        newState.theta = getCoordonnate(-M_PI, M_PI);
        isWall = static_cast<int>(round(map[newState.y + newState.x * rows])) == 1;
    }
    return newState;
}

State bestState(const State &u1, const State &u2, const State &actualState)
{
    if (distance(u1, actualState) < distance(u2, actualState))
    {
        return u1;
    }
    else
    {
        return u2;
    }
}

State getNearestState(const State &randState, ABR2dimNode *&T, int depth = 0)
{
    if (T == nullptr)
    {
        return {3000, 3000, 3000}; // return problème
        // a faire : gérer le retour null
    }

    if ((depth == 0 && randState.x < T->state.x) ||
        (depth == 1 && randState.y < T->state.y))
    {
        State u1 = getNearestState(randState, T->left, (depth + 1) % 2);
        State u2 = bestState(u1, T->state, randState);
        double dist = distance(u2, randState);
        if ((depth == 0 && abs(randState.x - T->state.x) <= dist) ||
            (depth == 1 && abs(randState.y - T->state.y) <= dist))
        {
            State u3 = getNearestState(randState, T->right, (depth + 1) % 2);
            State u4 = bestState(u2, u3, randState);
            return u4;
        }
        else
        {
            return u2;
        }
    }
    else
    {
        State u1 = getNearestState(randState, T->right, (depth + 1) % 2);
        State u2 = bestState(u1, T->state, randState);
        double dist = distance(u2, randState);
        if ((depth == 0 && abs(randState.x - T->state.x) <= dist) ||
            (depth == 1 && abs(randState.y - T->state.y) <= dist))
        {
            State u3 = getNearestState(randState, T->left, (depth + 1) % 2);
            State u4 = bestState(u2, u3, randState);
            return u4;
        }
        else
        {
            return u2;
        }
    }
}

State steer(const State &randState, const State &nearestState, double stepSize)
{
    double dist = distance(nearestState, randState);
    if (dist <= stepSize)
    {
        return randState;
    }
    else
    {
        double dx = (nearestState.x - randState.x);
        double dy = (nearestState.y - randState.y);
        double ratio = (dist - stepSize) / dist;

        State newState;
        newState.x = static_cast<int>(randState.x - dx * ratio);
        newState.y = static_cast<int>(randState.y - dy * ratio);
        newState.theta = randState.theta;

        return newState;
    }
}

bool isCollisionFree(const State &a, const State &b, const double *&map, const int &rows, const int &cols)
{
    int steps = static_cast<int>(std::hypot(b.x - a.x, b.y - a.y));
    for (int i = 0; i <= steps; ++i)
    {
        double t = steps == 0 ? 0.0 : static_cast<double>(i) / steps;
        int x = static_cast<int>(std::round(a.x + t * (b.x - a.x)));
        int y_tmp = static_cast<int>(std::round(a.y + t * (b.y - a.y)));
        int y = rows - 1 - y_tmp;

        if (x < 0 || x >= cols || y < 0 || y >= rows)
            return false;
        if (map[y + x * rows] > 0.5)
            return false;
    }
    return true;
}

RRTResult RRT(const State &start, const State &goal, const double *&map, int rows, int cols, double stepSize)
{
    RRTResult result;
    State randState, nearestState, newState;

    // init ABR-2 T
    ABR2dimTree T(start);
    // a faire : supprimer T à la fin de l'algo

    // a faire : iterations en argument par l'utilisateur
    int iterations = 10000;
    for (int i = 0; i < iterations; i++)
    {
        randState = pickRandomPoint(rows, cols, map);
        nearestState = getNearestState(randState, T.root);
        newState = steer(randState, nearestState, stepSize);

        if (isCollisionFree(nearestState, newState, map, rows, cols))
        {
            bool isNotDoublon = T.insertState(newState);
            if (isNotDoublon)
            {
                result.RRTTreeParent[newState] = nearestState;
            }
        }

        if (goal == newState)
        {
            // retourne path + explored states
            State current = newState;

            while (result.RRTTreeParent.count(current))
            {
                result.path.push_back(current);
                current = result.RRTTreeParent[current];
            }
            result.path.push_back(start);
            reverse(result.path.begin(), result.path.end());
            // T.~ABR2dimTree(); // destroy the tree => no need, it will be destroyed automatically
            return result;
        }
    }
    return result;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nrhs != 4)
    {
        mexErrMsgTxt("Usage: [path,explored] = RRT_Algorithm(start, goal, map, cellPerMeter)");
    }

    double *startTmp = mxGetPr(prhs[0]);
    double *goalTmp = mxGetPr(prhs[1]);

    int cellPerMeter = static_cast<int>(mxGetPr(prhs[3])[0]);

    State start = {static_cast<int>(startTmp[0] * cellPerMeter), static_cast<int>(startTmp[1] * cellPerMeter), startTmp[2]};
    State goal = {static_cast<int>(goalTmp[0] * cellPerMeter), static_cast<int>(goalTmp[1] * cellPerMeter), goalTmp[2]};

    mxArray *map = (mxArray *)prhs[2];
    const double *mapData = mxGetPr(map);
    int rows = static_cast<int>(mxGetM(map));
    int cols = static_cast<int>(mxGetN(map));

    auto [path, explored] = RRT(start, goal, mapData, rows, cols, 10.0);

    mwSize dims[3] = {path.size(), 3};
    plhs[0] = mxCreateDoubleMatrix(path.size(), 3, mxREAL);
    double *out = mxGetPr(plhs[0]);

    for (size_t i = 0; i < path.size(); i++)
    {
        out[i] = static_cast<double>(path[i].x) / cellPerMeter;
        out[i + dims[0]] = static_cast<double>(path[i].y) / cellPerMeter; // column-major
        out[i + 2 * dims[0]] = path[i].theta;                             // column-major
    }

    if (nlhs > 1)
    {
        mwSize dimsExplored[3] = {explored.size(), 6};
        plhs[1] = mxCreateDoubleMatrix(explored.size(), 6, mxREAL);
        double *outExplored = mxGetPr(plhs[1]);

        size_t i = 0;
        for (const auto &s : explored)
        {
            outExplored[i] = static_cast<double>(s.first.x) / cellPerMeter;                        // x child
            outExplored[i + dimsExplored[0]] = static_cast<double>(s.first.y) / cellPerMeter;      // y child
            outExplored[i + 2 * dimsExplored[0]] = s.first.theta;                                  // theta child
            outExplored[i + 3 * dimsExplored[0]] = static_cast<double>(s.second.x) / cellPerMeter; // x parent
            outExplored[i + 4 * dimsExplored[0]] = static_cast<double>(s.second.y) / cellPerMeter; // y parent
            outExplored[i + 5 * dimsExplored[0]] = s.second.theta;                                 // theta parent
            i++;
        }
    }
}