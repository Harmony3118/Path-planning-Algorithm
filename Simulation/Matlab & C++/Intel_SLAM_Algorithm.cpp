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

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam2d/types_slam2d.h>

// libg2o_core, libg2o_types_slam2d, libg2o_solver_dense

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

void graphOptimisation(std::vector<Node> &nodes, const std::vector<Edge> &edges)
{
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(true);

    // Create a linear solver
    auto linearSolver = std::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>>();
    auto solver_ptr = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));
    optimizer.setAlgorithm(solver_ptr);

    mexPrintf("\tLinear solver created\n");

    // Add nodes
    for (const auto &node : nodes)
    {
        auto *v = new g2o::VertexSE2();
        v->setId(node.id);
        v->setEstimate(g2o::SE2(node.pose.x(), node.pose.y(), node.pose.z()));
        if (node.id == 0)
        {
            v->setFixed(true); // Fix the first node
        }
        optimizer.addVertex(v);
    }

    mexPrintf("\tNodes added\n");

    // Add edges
    for (const auto &edge : edges)
    {
        auto *e = new g2o::EdgeSE2();
        e->setVertex(0, optimizer.vertex(edge.from));
        e->setVertex(1, optimizer.vertex(edge.to));
        e->setMeasurement(g2o::SE2(edge.relativePose[0], edge.relativePose[1], edge.relativePose[2]));
        e->setInformation(edge.omega); // Set the information
        optimizer.addEdge(e);
    }

    mexPrintf("\tEdges added\n");

    // Debug: Print initial node poses
    for (size_t i = 0; i < nodes.size(); i++)
    {
        auto *v = dynamic_cast<g2o::VertexSE2 *>(optimizer.vertex(i));
        g2o::SE2 estimate = v->estimate();
        mexPrintf("\tInitial node %zu pose: (%f, %f, %f)\n", i, estimate.translation()[0], estimate.translation()[1], estimate.rotation().angle());
    }

    // Optimisation
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    mexPrintf("\tOptimization done\n");

    // // Debug: Print optimized node poses
    // for (size_t i = 0; i < nodes.size(); i++)
    // {
    //     auto *v = dynamic_cast<g2o::VertexSE2 *>(optimizer.vertex(i));
    //     g2o::SE2 estimate = v->estimate();
    //     mexPrintf("\tOptimized node %zu pose: (%f, %f, %f)\n", i, estimate.translation()[0], estimate.translation()[1], estimate.rotation().angle());
    // }

    // Update node poses
    for (size_t i = 0; i < nodes.size(); i++)
    {
        auto *v = dynamic_cast<g2o::VertexSE2 *>(optimizer.vertex(i));
        g2o::SE2 estimate = v->estimate();
        nodes[i].pose = Eigen::Vector3d(estimate.translation()[0], estimate.translation()[1], estimate.rotation().angle());
    }

    mexPrintf("\tNode poses updated\n");

    optimizer.clear();
    optimizer.setAlgorithm(nullptr);
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

    std::vector<Scan> scans = buildScansStructure(filename);
    mexPrintf("Construction scan done\n");

    mxFree(filename);

    // printfVectorScan(scans);

    double maxRangeAll = calculateMaxRange(scans);
    double maxLidarRange = maxRangeAll * 0.90;
    double mapResolution = 20; // cell per meter

    // find loop closure
    // Pose node and constraint edges creation
    std::vector<Node> nodes;
    std::vector<Edge> edges;

    createNodeAndEdge(scans, nodes, edges);
    mexPrintf("Construction noeuds et aretes done\n\n");

    // Retour 1 vers matlab
    plhs[0] = mxCreateDoubleMatrix(nodes.size(), 3, mxREAL);
    double *poseData = mxGetPr(plhs[0]);

    for (size_t i = 0; i < nodes.size(); i++)
    {
        poseData[i] = nodes[i].pose[0];                    // x
        poseData[i + nodes.size()] = nodes[i].pose[1];     // y
        poseData[i + 2 * nodes.size()] = nodes[i].pose[2]; // theta
    }

    graphOptimisation(nodes, edges);
    mexPrintf("Optimisation done\n");

    // Retour 2 vers matlab
    plhs[1] = mxCreateDoubleMatrix(nodes.size(), 3, mxREAL);
    double *poseData2 = mxGetPr(plhs[1]);

    for (size_t i = 0; i < nodes.size(); i++)
    {
        poseData2[i] = nodes[i].pose[0];                    // x
        poseData2[i + nodes.size()] = nodes[i].pose[1];     // y
        poseData2[i + 2 * nodes.size()] = nodes[i].pose[2]; // theta
    }
}