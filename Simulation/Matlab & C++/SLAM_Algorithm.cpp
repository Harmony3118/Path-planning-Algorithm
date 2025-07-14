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

struct Scan
{
    std::vector<double> ranges;
    std::vector<double> angles;
    std::vector<std::vector<double>> cartesian;
    int count;
    std::array<double, 3> pose; // [x, y, theta]
};

struct PoseGraph
{
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

    // Open MAT file
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
    mxFree(filename);

    for (int i = 0; i < 3; i++)
    {
        Scan s = scans[i];
        
        for (int j = 0; j < 10; j++)
        {
            mexPrintf("Ranges\n");
            mexPrintf("\t%f\n", s.ranges[j]);
            mexPrintf("\t\t%.15f\n", s.ranges[j]);
            mexPrintf("\t...\n");

            mexPrintf("Angles\n");
            mexPrintf("\t%f\n", s.angles[j]);
            mexPrintf("\t...\n");

            mexPrintf("Cartesian\n");
            mexPrintf("\t%f, %f\n", s.cartesian[j][0], s.cartesian[j][1]);
            mexPrintf("\t...\n");

            mexPrintf("Count\n\t%d\n", s.count);

            mexPrintf("Pose\n");
            mexPrintf("\t%f\n", s.pose[j]);
        }
    }

    double maxRangeAll = calculateMaxRange(scans);
    double maxLidarRange = maxRangeAll * 0.90;
    double mapResolution = 20; // cell per meter

    // find loop closure

    //    slamAlgo();

    // algo
}