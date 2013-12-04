/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cmath>
#include <cstdlib>
#include <sbpl/planners/planner.h>
#include <sbpl/utils/utils.h>
#include <sbpl/utils/key.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/sbpl_bfs_2d.h>

using namespace std;

#if MEM_CHECK == 1
void DisableMemCheck()
{
    // Get the current state of the flag
    // and store it in a temporary variable
    int tmpFlag = _CrtSetDbgFlag( _CRTDBG_REPORT_FLAG );

    // Turn On (OR) - All freed memory is re-initialized with xDD
    tmpFlag |= _CRTDBG_DELAY_FREE_MEM_DF;

    // Turn Off (AND) - memory checking is disabled for future allocations
    tmpFlag &= ~_CRTDBG_ALLOC_MEM_DF;

    // Set the new state for the flag
    _CrtSetDbgFlag( tmpFlag );
}

void EnableMemCheck()
{
    // Get the current state of the flag
    // and store it in a temporary variable
    int tmpFlag = _CrtSetDbgFlag( _CRTDBG_REPORT_FLAG );

    // Turn On (OR) - All freed memory is re-initialized with xDD
    tmpFlag |= _CRTDBG_DELAY_FREE_MEM_DF;

    // Turn On (OR) - memory checking is enabled for future allocations
    tmpFlag |= _CRTDBG_ALLOC_MEM_DF;

    // Set the new state for the flag
    _CrtSetDbgFlag( tmpFlag );
}
#endif

void checkmdpstate(CMDPSTATE* state)
{
#if DEBUG == 0
    SBPL_ERROR("ERROR: checkMDPstate is too expensive for not in DEBUG mode\n");
    throw new SBPL_Exception();
#endif

    for (int aind = 0; aind < (int)state->Actions.size(); aind++) {
        for (int aind1 = 0; aind1 < (int)state->Actions.size(); aind1++) {
            if (state->Actions[aind1]->ActionID == state->Actions[aind]->ActionID && aind1 != aind) {
                SBPL_ERROR("ERROR in CheckMDP: multiple actions with the same ID exist\n");
                throw new SBPL_Exception();
            }
        }
        for (int sind = 0; sind < (int)state->Actions[aind]->SuccsID.size(); sind++) {
            for (int sind1 = 0; sind1 < (int)state->Actions[aind]->SuccsID.size(); sind1++) {
                if (state->Actions[aind]->SuccsID[sind] == state->Actions[aind]->SuccsID[sind1] && sind != sind1) {
                    SBPL_ERROR("ERROR in CheckMDP: multiple outcomes with the same ID exist\n");
                    throw new SBPL_Exception();
                }
            }
        }
    }
}

void CheckMDP(CMDP* mdp)
{
    for (int i = 0; i < (int)mdp->StateArray.size(); i++) {
        checkmdpstate(mdp->StateArray[i]);
    }
}

void PrintMatrix(int** matrix, int rows, int cols, FILE* fOut)
{
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            SBPL_FPRINTF(fOut, "%d ", matrix[r][c]);
        }
        SBPL_FPRINTF(fOut, "\n");
    }
}

//return true if there exists a path from sourcestate to targetstate and false otherwise
bool PathExists(CMDP* pMarkovChain, CMDPSTATE* sourcestate, CMDPSTATE* targetstate)
{
    CMDPSTATE* state;
    vector<CMDPSTATE*> WorkList;
    int i;
    bool *bProcessed = new bool[pMarkovChain->StateArray.size()];
    bool bFound = false;
    
    for (i = 0; i < (int)pMarkovChain->StateArray.size(); i++) 
    {
        bProcessed[i] = false;
    }


    //insert the source state
    WorkList.push_back(sourcestate);
    while ((int)WorkList.size() > 0) {
        //get the state and its info
        state = WorkList[WorkList.size() - 1];
        WorkList.pop_back();

        //Markov Chain should just contain a single policy
        if ((int)state->Actions.size() > 1) {
            SBPL_ERROR("ERROR in PathExists: Markov Chain is a general MDP\n");
            throw new SBPL_Exception();
        }

        if (state == targetstate) {
            //path found
            bFound = true;
            break;
        }

        //otherwise just insert policy successors into the worklist unless it is a goal state
        for (int sind = 0; (int)state->Actions.size() != 0 && sind < (int)state->Actions[0]->SuccsID.size(); sind++) {
            //get a successor
            for (i = 0; i < (int)pMarkovChain->StateArray.size(); i++) {
                if (pMarkovChain->StateArray[i]->StateID == state->Actions[0]->SuccsID[sind]) break;
            }
            if (i == (int)pMarkovChain->StateArray.size()) {
                SBPL_ERROR("ERROR in PathExists: successor is not found\n");
                throw new SBPL_Exception();
            }
            CMDPSTATE* SuccState = pMarkovChain->StateArray[i];

            //insert at the end of list if not there or processed already
            if (!bProcessed[i]) {
                bProcessed[i] = true;
                WorkList.push_back(SuccState);
            }
        } //for successors
    }//while WorkList is non empty

    delete[] bProcessed;

    return bFound;
}

int ComputeNumofStochasticActions(CMDP* pMDP)
{
    int i;
    int nNumofStochActions = 0;
    SBPL_PRINTF("ComputeNumofStochasticActions...\n");

    for (i = 0; i < (int)pMDP->StateArray.size(); i++) {
        for (int aind = 0; aind < (int)pMDP->StateArray[i]->Actions.size(); aind++) {
            if ((int)pMDP->StateArray[i]->Actions[aind]->SuccsID.size() > 1) nNumofStochActions++;
        }
    }
    SBPL_PRINTF("done\n");

    return nNumofStochActions;
}

void EvaluatePolicy(CMDP* PolicyMDP, int StartStateID, int GoalStateID, double* PolValue, bool *bFullPolicy,
                    double* Pcgoal, int *nMerges, bool* bCycles)
{
    int i, j, startind = -1;
    double delta = INFINITECOST;
    double mindelta = 0.1;

    *Pcgoal = 0;
    *nMerges = 0;

    SBPL_PRINTF("Evaluating policy...\n");

    //create and initialize values
    double* vals = new double[PolicyMDP->StateArray.size()];
    double* Pcvals = new double[PolicyMDP->StateArray.size()];
    for (i = 0; i < (int)PolicyMDP->StateArray.size(); i++) {
        vals[i] = 0;
        Pcvals[i] = 0;

        //remember the start index
        if (PolicyMDP->StateArray[i]->StateID == StartStateID) {
            startind = i;
            Pcvals[i] = 1;
        }
    }

    //initially assume full policy
    *bFullPolicy = true;
    bool bFirstIter = true;
    while (delta > mindelta) {
        delta = 0;
        for (i = 0; i < (int)PolicyMDP->StateArray.size(); i++) {
            //get the state
            CMDPSTATE* state = PolicyMDP->StateArray[i];

            //do the backup for values
            if (state->StateID == GoalStateID) {
                vals[i] = 0;
            }
            else if ((int)state->Actions.size() == 0) {
                *bFullPolicy = false;
                vals[i] = UNKNOWN_COST;
                *PolValue = vals[startind];
                return;
            }
            else {
                //normal backup
                CMDPACTION* action = state->Actions[0];

                //do backup
                double Q = 0;
                for (int oind = 0; oind < (int)action->SuccsID.size(); oind++) {
                    //get the state
                    for (j = 0; j < (int)PolicyMDP->StateArray.size(); j++) {
                        if (PolicyMDP->StateArray[j]->StateID == action->SuccsID[oind]) break;
                    }
                    if (j == (int)PolicyMDP->StateArray.size()) {
                        SBPL_ERROR("ERROR in EvaluatePolicy: incorrect successor %d\n", action->SuccsID[oind]);
                        throw new SBPL_Exception();
                    }
                    Q += action->SuccsProb[oind] * (vals[j] + action->Costs[oind]);
                }

                if (vals[i] > Q) {
                    SBPL_ERROR("ERROR in EvaluatePolicy: val is decreasing\n");
                    throw new SBPL_Exception();
                }

                //update delta
                if (delta < Q - vals[i]) delta = Q - vals[i];

                //set the value
                vals[i] = Q;
            }

            //iterate through all the predecessors and compute Pc
            double Pc = 0;
            //go over all predecessor states
            int nMerge = 0;
            for (j = 0; j < (int)PolicyMDP->StateArray.size(); j++) {
                for (int oind = 0; (int)PolicyMDP->StateArray[j]->Actions.size() > 0 &&
                                   oind < (int)PolicyMDP->StateArray[j]->Actions[0]->SuccsID.size(); oind++)
                {
                    if (PolicyMDP->StateArray[j]->Actions[0]->SuccsID[oind] == state->StateID) {
                        //process the predecessor
                        double PredPc = Pcvals[j];
                        double OutProb = PolicyMDP->StateArray[j]->Actions[0]->SuccsProb[oind];

                        //accumulate into Pc
                        Pc = Pc + OutProb * PredPc;
                        nMerge++;

                        //check for cycles
                        if (bFirstIter && !(*bCycles)) {
                            if (PathExists(PolicyMDP, state, PolicyMDP->StateArray[j])) *bCycles = true;
                        }
                    }
                }
            }
            if (bFirstIter && state->StateID != GoalStateID && nMerge > 0) *nMerges += (nMerge - 1);

            //assign Pc
            if (state->StateID != StartStateID) Pcvals[i] = Pc;

            if (state->StateID == GoalStateID) *Pcgoal = Pcvals[i];
        } //over  states
        bFirstIter = false;
    } //until delta small

    *PolValue = vals[startind];

    SBPL_PRINTF("done\n");
}

void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params)
{
    params->UsingYIndex = 0;

    if (fabs((double)(p2y - p1y) / (double)(p2x - p1x)) > 1) (params->UsingYIndex)++;

    if (params->UsingYIndex) {
        params->Y1 = p1x;
        params->X1 = p1y;
        params->Y2 = p2x;
        params->X2 = p2y;
    }
    else {
        params->X1 = p1x;
        params->Y1 = p1y;
        params->X2 = p2x;
        params->Y2 = p2y;
    }

    if ((p2x - p1x) * (p2y - p1y) < 0) {
        params->Flipped = 1;
        params->Y1 = -params->Y1;
        params->Y2 = -params->Y2;
    }
    else
        params->Flipped = 0;

    if (params->X2 > params->X1)
        params->Increment = 1;
    else
        params->Increment = -1;

    params->DeltaX = params->X2 - params->X1;
    params->DeltaY = params->Y2 - params->Y1;

    params->IncrE = 2 * params->DeltaY * params->Increment;
    params->IncrNE = 2 * (params->DeltaY - params->DeltaX) * params->Increment;
    params->DTerm = (2 * params->DeltaY - params->DeltaX) * params->Increment;

    params->XIndex = params->X1;
    params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y)
{
    if (params->UsingYIndex) {
        *y = params->XIndex;
        *x = params->YIndex;
        if (params->Flipped) *x = -*x;
    }
    else {
        *x = params->XIndex;
        *y = params->YIndex;
        if (params->Flipped) *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params)
{
    if (params->XIndex == params->X2) {
        return 0;
    }
    params->XIndex += params->Increment;
    if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
        params->DTerm += params->IncrE;
    else {
        params->DTerm += params->IncrNE;
        params->YIndex += params->Increment;
    }
    return 1;
}

//converts discretized version of angle into continuous (radians)
//maps 0->0, 1->delta, 2->2*delta, ...
double DiscTheta2Cont(int nTheta, int NUMOFANGLEVALS)
{
    double thetaBinSize = 2.0 * PI_CONST / NUMOFANGLEVALS;
    return nTheta * thetaBinSize;
}

//converts continuous (radians) version of angle into discrete
//maps 0->0, [delta/2, 3/2*delta)->1, [3/2*delta, 5/2*delta)->2,...
int ContTheta2Disc(double fTheta, int NUMOFANGLEVALS)
{
    double thetaBinSize = 2.0 * PI_CONST / NUMOFANGLEVALS;
    return (int)(normalizeAngle(fTheta + thetaBinSize / 2.0) / (2.0 * PI_CONST) * (NUMOFANGLEVALS));
}

//input angle should be in radians
//counterclockwise is positive
//output is an angle in the range of from 0 to 2*PI
double normalizeAngle(double angle)
{
    double retangle = angle;

    //get to the range from -2PI, 2PI
    if (fabs(retangle) > 2 * PI_CONST) retangle = retangle - ((int)(retangle / (2 * PI_CONST))) * 2 * PI_CONST;

    //get to the range 0, 2PI
    if (retangle < 0) retangle += 2 * PI_CONST;

    if (retangle < 0 || retangle > 2 * PI_CONST) {
        SBPL_ERROR("ERROR: after normalization of angle=%f we get angle=%f\n", angle, retangle);
    }

    return retangle;
}

/*
 * point - the point to test
 *
 * Function derived from http://ozviz.wasp.uwa.edu.au/~pbourke/geometry/insidepoly/
 */
bool IsInsideFootprint(sbpl_2Dpt_t pt, vector<sbpl_2Dpt_t>* bounding_polygon)
{
    int counter = 0;
    int i;
    double xinters;
    sbpl_2Dpt_t p1;
    sbpl_2Dpt_t p2;
    int N = bounding_polygon->size();

    p1 = bounding_polygon->at(0);
    for (i = 1; i <= N; i++) {
        p2 = bounding_polygon->at(i % N);
        if (pt.y > __min(p1.y, p2.y)) {
            if (pt.y <= __max(p1.y, p2.y)) {
                if (pt.x <= __max(p1.x, p2.x)) {
                    if (p1.y != p2.y) {
                        xinters = (pt.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
                        if (p1.x == p2.x || pt.x <= xinters) counter++;
                    }
                }
            }
        }
        p1 = p2;
    }

    if (counter % 2 == 0)
        return false;
    else
        return true;
#if DEBUG
    //SBPL_PRINTF("Returning from inside footprint: %d\n", c);
#endif
    //  return c;
}

double computeMinUnsignedAngleDiff(double angle1, double angle2)
{
    //get the angles into 0-2*PI range
    angle1 = normalizeAngle(angle1);
    angle2 = normalizeAngle(angle2);

    double anglediff = fabs(angle1 - angle2);

    //see if we can take a shorter route
    if (anglediff > PI_CONST) {
        anglediff = fabs(anglediff - 2 * PI_CONST);
    }

    return anglediff;
}

//computes 8-connected distances to obstacles and non-free areas in two linear
//passes and returns them in disttoObs_incells 
//and disttoNonfree_incells arrays. The distances are in terms of the number of cells but are floats. These distances
//can then be converted into the actual distances using the actual discretization values
//areas outside of the map are considered to be obstacles
void computeDistancestoNonfreeAreas(unsigned char** Grid2D, int width_x, int height_y, unsigned char obsthresh,
                                    float** disttoObs_incells, float** disttoNonfree_incells)
{
    int x, y, nbrx, nbry;
    float mindisttoObs, mindisttoNonfree;
    float maxDist = (float)(__min(width_x, height_y));
    float disttoObs, disttoNonfree;
    int dir;
    const int NUMOF2DQUASIDIRS = 4;

    // for quasi-Euclidean distance transform
    // going left-to-right, top-to-bottom
    int dxdownlefttoright_[NUMOF2DQUASIDIRS];
    int dydownlefttoright_[NUMOF2DQUASIDIRS];
    int dxdownrighttoleft_[NUMOF2DQUASIDIRS];
    int dydownrighttoleft_[NUMOF2DQUASIDIRS];

    // going right-to-left, bottom-to-top
    int dxuprighttoleft_[NUMOF2DQUASIDIRS];
    int dyuprighttoleft_[NUMOF2DQUASIDIRS];
    int dxuplefttoright_[NUMOF2DQUASIDIRS];
    int dyuplefttoright_[NUMOF2DQUASIDIRS];

    // distances to the above nbrs
    float distdownlefttoright_[NUMOF2DQUASIDIRS];
    float distdownrighttoleft_[NUMOF2DQUASIDIRS];
    float distuprighttoleft_[NUMOF2DQUASIDIRS];
    float distuplefttoright_[NUMOF2DQUASIDIRS];

    // and for distance transform:
    // increasing x (outer)
    // increasing y (inner)
    //  [2]
    //  [1][s]
    //  [0][3]
    dxdownlefttoright_[0] = -1;
    dydownlefttoright_[0] = -1;
    dxdownlefttoright_[1] = -1;
    dydownlefttoright_[1] = 0;
    dxdownlefttoright_[2] = -1;
    dydownlefttoright_[2] = 1;
    dxdownlefttoright_[3] = 0;
    dydownlefttoright_[3] = -1;

    // increasing x (outer)
    // decreasing y (inner)
    //  [2][3]
    //  [1][s]
    //  [0] 
    dxdownrighttoleft_[0] = -1;
    dydownrighttoleft_[0] = -1;
    dxdownrighttoleft_[1] = -1;
    dydownrighttoleft_[1] = 0;
    dxdownrighttoleft_[2] = -1;
    dydownrighttoleft_[2] = 1;
    dxdownrighttoleft_[3] = 0;
    dydownrighttoleft_[3] = 1;

    // decreasing x (outer)
    // decreasing y (inner)
    //  [3][2]
    //  [s][1]
    //     [0] 
    dxuprighttoleft_[0] = 1;
    dyuprighttoleft_[0] = -1;
    dxuprighttoleft_[1] = 1;
    dyuprighttoleft_[1] = 0;
    dxuprighttoleft_[2] = 1;
    dyuprighttoleft_[2] = 1;
    dxuprighttoleft_[3] = 0;
    dyuprighttoleft_[3] = 1;

    // decreasing x (outer)
    // increasing y (inner)
    //     [2]
    //  [s][1]
    //  [3][0] 
    dxuplefttoright_[0] = 1;
    dyuplefttoright_[0] = -1;
    dxuplefttoright_[1] = 1;
    dyuplefttoright_[1] = 0;
    dxuplefttoright_[2] = 1;
    dyuplefttoright_[2] = 1;
    dxuplefttoright_[3] = 0;
    dyuplefttoright_[3] = -1;

    // insert the corresponding distances
    distdownlefttoright_[0] = (float)1.414;
    distdownlefttoright_[1] = (float)1.0;
    distdownlefttoright_[2] = (float)1.414;
    distdownlefttoright_[3] = (float)1.0;

    distdownrighttoleft_[0] = (float)1.414;
    distdownrighttoleft_[1] = (float)1.0;
    distdownrighttoleft_[2] = (float)1.414;
    distdownrighttoleft_[3] = (float)1.0;

    distuprighttoleft_[0] = (float)1.414;
    distuprighttoleft_[1] = (float)1.0;
    distuprighttoleft_[2] = (float)1.414;
    distuprighttoleft_[3] = (float)1.0;

    distuplefttoright_[0] = (float)1.414;
    distuplefttoright_[1] = (float)1.0;
    distuplefttoright_[2] = (float)1.414;
    distuplefttoright_[3] = (float)1.0;

    // step through the map from top to bottom,
    // alternating left-to-right then right-to-left
    // This order maintains the invariant that the min distance for each
    // cell to all previously-visited obstacles is accurate
    for (x = 0; x < width_x; x++) {
        // move from left to right
        if (x % 2 == 0) {

            for (y = 0; y < height_y; y++) {

                mindisttoObs = maxDist; // initialize to max distance
                mindisttoNonfree = maxDist;

                // if cell is an obstacle, set disttoObs to 0 and continue
                if (Grid2D[x][y] >= obsthresh) {
                    disttoObs_incells[x][y] = 0;
                    disttoNonfree_incells[x][y] = 0;
                    continue;
                }

                if (Grid2D[x][y] > 0) {
                    mindisttoNonfree = 0;
                }

                //iterate over predecessors
                for (dir = 0; dir < NUMOF2DQUASIDIRS; dir++) {
                    nbrx = x + dxdownlefttoright_[dir];
                    nbry = y + dydownlefttoright_[dir];

                    //make sure it is inside the map and has no obstacle
                    // compute min cost to an obstacle for each cell, using
                    // *just* the cells already computed this pass for checking distance
                    if (nbrx < 0 || nbrx >= width_x || nbry < 0 || nbry >= height_y) {
                        disttoObs = distdownlefttoright_[dir];
                        disttoNonfree = disttoObs;
                    }
                    else {
                        disttoObs = distdownlefttoright_[dir] + disttoObs_incells[nbrx][nbry];
                        disttoNonfree = distdownlefttoright_[dir] + disttoNonfree_incells[nbrx][nbry];
                    }

                    if (disttoObs < mindisttoObs) mindisttoObs = disttoObs;
                    if (disttoNonfree < mindisttoNonfree) mindisttoNonfree = disttoNonfree;
                }//over preds

                disttoObs_incells[x][y] = mindisttoObs;
                disttoNonfree_incells[x][y] = mindisttoNonfree;
            }

        }
        else {

            // move from right to left
            for (y = height_y - 1; y >= 0; y--) {

                mindisttoObs = maxDist; // initialize to max distance
                mindisttoNonfree = maxDist;

                // if cell is an obstacle, set disttoObs to 0 and continue
                if (Grid2D[x][y] >= obsthresh) {
                    disttoObs_incells[x][y] = 0;
                    disttoNonfree_incells[x][y] = 0;
                    continue;
                }

                if (Grid2D[x][y] > 0) {
                    mindisttoNonfree = 0;
                }

                //iterate over predecessors
                for (dir = 0; dir < NUMOF2DQUASIDIRS; dir++) {
                    nbrx = x + dxdownrighttoleft_[dir];
                    nbry = y + dydownrighttoleft_[dir];

                    //make sure it is inside the map and has no obstacle
                    // compute min cost to an obstacle for each cell, using
                    // *just* the cells already computed this pass for checking distance
                    if (nbrx < 0 || nbrx >= width_x || nbry < 0 || nbry >= height_y) {
                        disttoObs = distdownrighttoleft_[dir];
                        disttoNonfree = disttoObs;
                    }
                    else {
                        disttoObs = distdownrighttoleft_[dir] + disttoObs_incells[nbrx][nbry];
                        disttoNonfree = distdownrighttoleft_[dir] + disttoNonfree_incells[nbrx][nbry];
                    }

                    if (disttoObs < mindisttoObs) mindisttoObs = disttoObs;
                    if (disttoNonfree < mindisttoNonfree) mindisttoNonfree = disttoNonfree;
                }

                disttoObs_incells[x][y] = mindisttoObs;
                disttoNonfree_incells[x][y] = mindisttoNonfree;
            }
            //SBPL_PRINTF("x=%d\n", x);
        }
    }

    // step through the map from bottom to top
    for (x = width_x - 1; x >= 0; x--) {
        // move from right to left
        if (x % 2 == 0) {
            for (y = height_y - 1; y >= 0; y--) {
                // initialize to current distance
                mindisttoObs = disttoObs_incells[x][y];
                mindisttoNonfree = disttoNonfree_incells[x][y];

                //iterate over predecessors
                for (dir = 0; dir < NUMOF2DQUASIDIRS; dir++) {
                    nbrx = x + dxuprighttoleft_[dir];
                    nbry = y + dyuprighttoleft_[dir];

                    //make sure it is inside the map and has no obstacle
                    // compute min cost to an obstacle for each cell, using
                    // *just* the cells already computed this pass for checking distance
                    if (nbrx < 0 || nbrx >= width_x || nbry < 0 || nbry >= height_y) {
                        disttoObs = distuprighttoleft_[dir];
                        disttoNonfree = disttoObs;
                    }
                    else {
                        disttoObs = distuprighttoleft_[dir] + disttoObs_incells[nbrx][nbry];
                        disttoNonfree = distuprighttoleft_[dir] + disttoNonfree_incells[nbrx][nbry];
                    }

                    if (disttoObs < mindisttoObs) mindisttoObs = disttoObs;
                    if (disttoNonfree < mindisttoNonfree) mindisttoNonfree = disttoNonfree;
                }//over preds

                disttoObs_incells[x][y] = mindisttoObs;
                disttoNonfree_incells[x][y] = mindisttoNonfree;
            }//for y
        }
        else {
            // move from left to right
            for (y = 0; y < height_y; y++) {
                // initialize to current distance
                mindisttoObs = disttoObs_incells[x][y];
                mindisttoNonfree = disttoNonfree_incells[x][y];

                //iterate over predecessors
                for (dir = 0; dir < NUMOF2DQUASIDIRS; dir++) {
                    nbrx = x + dxuplefttoright_[dir];
                    nbry = y + dyuplefttoright_[dir];

                    //make sure it is inside the map and has no obstacle
                    // compute min cost to an obstacle for each cell, using
                    // *just* the cells already computed this pass for checking distance
                    if (nbrx < 0 || nbrx >= width_x || nbry < 0 || nbry >= height_y) {
                        disttoObs = distuplefttoright_[dir];
                        disttoNonfree = disttoObs;
                    }
                    else {
                        disttoObs = distuplefttoright_[dir] + disttoObs_incells[nbrx][nbry];
                        disttoNonfree = distuplefttoright_[dir] + disttoNonfree_incells[nbrx][nbry];
                    }

                    if (disttoObs < mindisttoObs) mindisttoObs = disttoObs;
                    if (disttoNonfree < mindisttoNonfree) mindisttoNonfree = disttoNonfree;
                }

                disttoObs_incells[x][y] = mindisttoObs;
                disttoNonfree_incells[x][y] = mindisttoNonfree;
            }//over y
        }//direction
    }//over x
}

void get_2d_motion_cells(vector<sbpl_2Dpt_t> polygon, vector<sbpl_xy_theta_pt_t> poses, vector<sbpl_2Dcell_t>* cells,
                         double res)
{
    // can't find any motion cells if there are no poses
    if (poses.empty()) {
        return;
    }

    //get first footprint set
    set<sbpl_2Dcell_t> first_cell_set;
    get_2d_footprint_cells(polygon, &first_cell_set, poses[0], res);

    //duplicate first footprint set into motion set
    set<sbpl_2Dcell_t> cell_set = first_cell_set;

    //call get footprint on the rest of the points
    for (unsigned int i = 1; i < poses.size(); i++) {
        get_2d_footprint_cells(polygon, &cell_set, poses[i], res);
    }

    //convert the motion set to a vector but don't include the cells in the first footprint set
    cells->reserve(cell_set.size() - first_cell_set.size());
    for (set<sbpl_2Dcell_t>::iterator it = cell_set.begin(); it != cell_set.end(); it++) {
        if (first_cell_set.find(*it) == first_cell_set.end()) {
            cells->push_back(*it);
        }
    }
}

//This function is inefficient and should be avoided if possible (you should
//use overloaded functions that uses a set for the cells)!
void get_2d_footprint_cells(vector<sbpl_2Dpt_t> polygon, vector<sbpl_2Dcell_t>* cells, sbpl_xy_theta_pt_t pose,
                            double res)
{
    set<sbpl_2Dcell_t> cell_set;
    for (unsigned int i = 0; i < cells->size(); i++)
        cell_set.insert(cells->at(i));
    get_2d_footprint_cells(polygon, &cell_set, pose, res);
    cells->clear();
    cells->reserve(cell_set.size());
    //for(set<sbpl_2Dcell_t>::iterator it; it==cell_set.begin(); it++)
    //  cells->push_back(*it);
    for (set<sbpl_2Dcell_t>::iterator it = cell_set.begin(); it != cell_set.end(); it++) {
        cells->push_back(*it);
    }
}

void get_2d_footprint_cells(vector<sbpl_2Dpt_t> polygon, set<sbpl_2Dcell_t>* cells, sbpl_xy_theta_pt_t pose, double res)
{
    //special case for point robot
    if (polygon.size() <= 1) {
        sbpl_2Dcell_t cell;
        cell.x = CONTXY2DISC(pose.x, res);
        cell.y = CONTXY2DISC(pose.y, res);

        cells->insert(cell);
        return;
    }

    //run bressenham line algorithm around the polygon (add them to the cells set)
    //while doing that find the min and max (x,y) and the average x and y
    double cth = cos(pose.theta);
    double sth = sin(pose.theta);

    vector<pair<int, int> > disc_polygon;
    disc_polygon.reserve(polygon.size() + 1);
    int minx = INFINITECOST;
    int maxx = -INFINITECOST;
    int miny = INFINITECOST;
    int maxy = -INFINITECOST;

    //find the bounding box of the polygon
    for (unsigned int i = 0; i < polygon.size(); i++) {
        pair<int, int> p;
        //		p.first = CONTXY2DISC(cth*polygon[i].x - sth*polygon[i].y + pose.x, res);
        double cx = (cth * polygon[i].x - sth * polygon[i].y + pose.x);
        double cy = (sth * polygon[i].x + cth * polygon[i].y + pose.y);
        p.first = (int)(cx > 0 ? cx / res + 0.5 : cx / res - 0.5); //(int)(cx / res + 0.5 * sign(c);
        //		p.second = CONTXY2DISC(sth*polygon[i].x + cth*polygon[i].y + pose.y, res);
        p.second = (int)(cy > 0 ? cy / res + 0.5 : cy / res - 0.5);//(int)(cy / res + 0.5);
        disc_polygon.push_back(p);
        if (p.first < minx) minx = p.first;
        if (p.first > maxx) maxx = p.first;
        if (p.second < miny) miny = p.second;
        if (p.second > maxy) maxy = p.second;
    }
    disc_polygon.push_back(disc_polygon.front());

    //make a grid big enough for the footprint
    int sizex = (maxx - minx + 1) + 2;
    int sizey = (maxy - miny + 1) + 2;
    int** grid = new int*[sizex];
    for (int i = 0; i < sizex; i++) {
        grid[i] = new int[sizey];
        for (int j = 0; j < sizey; j++)
            grid[i][j] = 0;
    }

    //plot line points on the grid
    for (unsigned int i = 1; i < disc_polygon.size(); i++) {
        int x0 = disc_polygon[i - 1].first - minx + 1;
        int y0 = disc_polygon[i - 1].second - miny + 1;
        int x1 = disc_polygon[i].first - minx + 1;
        int y1 = disc_polygon[i].second - miny + 1;

        //bressenham (add the line cells to the set and to a vector)
        bool steep = abs(y1 - y0) > abs(x1 - x0);
        if (steep) {
            int temp = x0;
            x0 = y0;
            y0 = temp;
            temp = x1;
            x1 = y1;
            y1 = temp;
        }
        if (x0 > x1) {
            int temp = x0;
            x0 = x1;
            x1 = temp;
            temp = y0;
            y0 = y1;
            y1 = temp;
        }
        int deltax = x1 - x0;
        int deltay = abs(y1 - y0);
        int error = deltax / 2;
        int ystep = (y0 < y1 ? 1 : -1);
        int y = y0;
        for (int x = x0; x <= x1; x++) {
            if (steep) {
                grid[y][x] = 1;
                cells->insert(sbpl_2Dcell_t(y - 1 + minx, x - 1 + miny));
            }
            else {
                grid[x][y] = 1;
                cells->insert(sbpl_2Dcell_t(x - 1 + minx, y - 1 + miny));
            }
            int last_error = error;
            error -= deltay;
            if (error < 0 && x != x1) {
                //make sure we can't have a diagonal line (the 8-connected bfs will leak through)

                int tempy = y;
                int tempx = x;
                if (last_error < -error)
                    tempy += ystep;
                else
                    tempx += 1;
                if (steep) {
                    grid[tempy][tempx] = 1;
                    cells->insert(sbpl_2Dcell_t(tempy - 1 + minx, tempx - 1 + miny));
                }
                else {
                    grid[tempx][tempy] = 1;
                    cells->insert(sbpl_2Dcell_t(tempx - 1 + minx, tempy - 1 + miny));
                }

                y += ystep;
                error += deltax;
            }
        }
    }

    //run a 2d bfs from the average (x,y)
    sbpl_bfs_2d bfs(sizex, sizey, 1);
    bfs.compute_distance_from_point(grid, 0, 0);

    for (int i = 0; i < sizex; i++)
        delete[] grid[i];
    delete[] grid;

    //add all cells expanded to the cells set
    for (int i = 1; i < sizex - 1; i++) {
        for (int j = 1; j < sizey - 1; j++) {
            if (bfs.get_distance(i, j) < 0) cells->insert(sbpl_2Dcell_t(i - 1 + minx, j - 1 + miny));
        }
    }
}

void writePlannerStats(vector<PlannerStats> s, FILE* fout)
{
    fprintf(fout, "%%eps time expands cost\n");
    for (unsigned int i = 0; i < s.size(); i++) {
        fprintf(fout, "%f %f %d %d\n", s[i].eps, s[i].time, s[i].expands, s[i].cost);
    }
}
