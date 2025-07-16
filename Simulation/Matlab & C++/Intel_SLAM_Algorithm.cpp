// g++ -std=c++17 -I$MATLAB_ROOT/extern/include -L$MATLAB_ROOT/bin/glnxa64 -lmat -lmx read_slam.cpp -o read_slam
// replace $MATLAB_ROOT by matlab install path

#include "mex.h"
#include "mat.h"
#include "matrix.h"

#include <iostream>
#include <vector>
#include <array>
#include <string>
#include <algorithm>

#include <Eigen/Dense>

struct Scan
{
    std::vector<double> ranges;
    std::vector<double> angles;
    std::vector<std::vector<double>> cartesian;
    int count;
    std::array<double, 3> pose; // [x, y, theta]
};

struct Node
{
    int id;
    Eigen::Vector3d pose; // [x, y, θ]
};

struct Edge
{
    int from;
    int to;
    Eigen::Vector3d relativePose; // dx, dy, dtheta
    Eigen::Matrix3d omega;
};

double calculateMaxRange(std::vector<Scan> scans)
{
    double max = 0;
    for (const auto &s : scans)
    {
        auto maxIt = std::max_element(s.ranges.begin(), s.ranges.end());

        if (maxIt != s.ranges.end() && *maxIt > max)
        {
            max = *maxIt;
        }
        // mexPrintf("max actuel : %f\n", max);
    }
    return max;
}

double normalizeAngle(double angle)
{
    double tmp = fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
    if (tmp > M_PI)
    {
        tmp -= 2.0 * M_PI;
    }
    return tmp;
}

std::vector<Scan> buildScansStructure(char *filename)
{
    // Open MAT file with reading
    MATFile *pmat = matOpen(filename, "r");
    if (!pmat)
    {
        mexErrMsgTxt("Cannot open MAT file.");
    }

    // Get variable
    mxArray *mx_scans = matGetVariable(pmat, "scanStruct");
    if (!mx_scans)
    {
        matClose(pmat);
        mexErrMsgTxt("Variable 'scanStruct' not found.");
    }

    size_t n_scans = mxGetNumberOfElements(mx_scans);
    mexPrintf("Loaded %zu scans from %s\n", n_scans, filename);

    std::vector<Scan> scans(n_scans);

    for (size_t i = 0; i < n_scans; i++)
    {
        mxArray *mx_ranges = mxGetField(mx_scans, i, "ranges");
        mxArray *mx_angles = mxGetField(mx_scans, i, "angles");
        mxArray *mx_cartesian = mxGetField(mx_scans, i, "cartesian");
        mxArray *mx_count = mxGetField(mx_scans, i, "count");
        mxArray *mx_pose = mxGetField(mx_scans, i, "pose");

        if (!mx_ranges || !mx_angles || !mx_cartesian || !mx_count || !mx_pose)
        {
            mexPrintf("Missing field at scan %zu\n", i);
            continue;
        }

        Scan s;

        double *pr = mxGetPr(mx_ranges);
        size_t nr = mxGetNumberOfElements(mx_ranges);
        s.ranges.assign(pr, pr + nr);

        double *pa = mxGetPr(mx_angles);
        size_t na = mxGetNumberOfElements(mx_angles);
        s.angles.assign(pa, pa + na);

        double *pc = mxGetPr(mx_cartesian);
        size_t rows = mxGetM(mx_cartesian);
        size_t cols = mxGetN(mx_cartesian);
        s.cartesian.resize(rows, std::vector<double>(cols));
        for (size_t r = 0; r < rows; r++)
        {
            for (size_t c = 0; c < cols; c++)
            {
                s.cartesian[r][c] = pc[r + c * rows];
            }
        }

        s.count = static_cast<int>(mxGetScalar(mx_count));

        double *pp = mxGetPr(mx_pose);
        size_t np = mxGetNumberOfElements(mx_pose);
        std::copy(pp, pp + np, s.pose.begin());

        scans[i] = s;
    }

    matClose(pmat);
    return scans;
}

void printfVectorScan(std::vector<Scan> scans)
{
    for (int i = 0; i < 1; i++)
    {
        Scan s = scans[i];

        mexPrintf("Ranges\n");
        for (int j = 0; j < 10; j++)
        {
            mexPrintf("\t%f\n", s.ranges[j]);
        }
        mexPrintf("\t...\n");

        mexPrintf("Angles\n");
        for (int j = 0; j < 10; j++)
        {
            mexPrintf("\t%f\n", s.angles[j]);
        }
        mexPrintf("\t...\n");

        mexPrintf("Cartesian\n");
        for (int j = 0; j < 10; j++)
        {
            mexPrintf("\t%f, %f\n", s.cartesian[j][0], s.cartesian[j][1]);
        }
        mexPrintf("\t...\n");

        mexPrintf("Count\n\t%d\n", s.count);

        mexPrintf("Pose\n");
        for (int j = 0; j < 3; j++)
        {
            mexPrintf("\t%f\n", s.pose[j]);
        }
    }
}

Eigen::Vector3d computeRelativeTransform(const Eigen::Vector3d &Ti, const Eigen::Vector3d &Tj)
{
    double dx = Tj.x() - Ti.x();
    double dy = Tj.y() - Ti.y();
    double dtheta = Tj.z() - Ti.z();

    double cos_theta = cos(-Ti.z());
    double sin_theta = sin(-Ti.z());

    double x_rel = cos_theta * dx - sin_theta * dy;
    double y_rel = sin_theta * dx + cos_theta * dy;

    return Eigen::Vector3d(x_rel, y_rel, normalizeAngle(dtheta));
}

void createNodeAndEdge(const std::vector<Scan> &scans, std::vector<Node> &nodes, std::vector<Edge> &edges)
{
    for (size_t i = 0; i < scans.size(); i++)
    {
        Node node;
        node.id = i;
        node.pose = Eigen::Vector3d(scans[i].pose[0], scans[i].pose[1], scans[i].pose[2]);
        nodes.push_back(node);

        if (i > 0)
        {
            const auto &T_i = nodes[i - 1].pose;
            const auto &T_j = nodes[i].pose;

            Eigen::Vector3d T_ij = computeRelativeTransform(T_i, T_j);

            Edge edge;
            edge.from = i - 1;
            edge.to = i;
            edge.relativePose = T_ij;
            // a faire : améliorer avec les mesures d'odometrie (tv, rv, accel)
            // et des timestamps
            Eigen::Matrix3d covariance;
            covariance << 0.01, 0, 0,
                0, 0.01, 0,
                0, 0, 0.05;
            edge.omega = covariance.inverse(); // matrice incertitude = Inverse de la covariance
            edges.push_back(edge);
        }
    }
}
// Complexity : n
// Fonction called for each scan : worst case = O(n²)
bool isLoopClosure(std::vector<Scan> scans, Scan currentScan)
{
    for (auto s : scans)
    {
        if (s.pose == currentScan.pose)
        {
            return true;
        }
    }
    return false;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{ // nrhs :nb d'args, prhs: liste args, nlhs: nb sortis, plhs: liste sorties
    if (nrhs != 1 || !mxIsChar(prhs[0]))
    {
        mexErrMsgTxt("Usage: SLAM_Algorithm('offlineSlamData.mat')");
    }

    char *filename = mxArrayToString(prhs[0]);
    if (!filename)
    {
        mexErrMsgTxt("Could not read filename input.");
    }

    mexPrintf("Construction scan\n");
    std::vector<Scan> scans = buildScansStructure(filename);

    mxFree(filename);

    printfVectorScan(scans);

    double maxRangeAll = calculateMaxRange(scans);
    double maxLidarRange = maxRangeAll * 0.90;
    double mapResolution = 20; // cell per meter

    // find loop closure
    // Pose node and constraint edges creation
    std::vector<Node> nodes;
    std::vector<Edge> edges;

    mexPrintf("Construction noeuds et aretes");
    createNodeAndEdge(scans, nodes, edges);

    for (int i = 0; i < 3; i++)
    {
        mexPrintf("node:\n\tid: %d\n\tpose: %f %f %f\n", nodes[i].id, nodes[i].pose.x(), nodes[i].pose.y(), nodes[i].pose.z());
        mexPrintf("Edge:\n\tfrom: %d\n\tyo: %d\n", edges[i].from, edges[i].to);
    }

    // algo
}