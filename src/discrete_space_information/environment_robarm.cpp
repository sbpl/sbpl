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
#include <cstring>
#include <queue>
#include <sbpl/discrete_space_information/environment_robarm.h>
#include <sbpl/planners/planner.h>
#include <sbpl/utils/key.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/mdpconfig.h>
#include <sbpl/utils/utils.h>

using namespace std;

#define COSTMULT 1000

//static clock_t time3_addallout = 0;
//static clock_t time_gethash = 0;
//static clock_t time_createhash = 0;

#define XYTO2DIND(x,y) ((x) + (y)*EnvROBARMCfg.EnvWidth_c)

#define DIRECTIONS 8
int dx[DIRECTIONS] = {1, 1, 1, 0, 0, -1, -1, -1};
int dy[DIRECTIONS] = {1, 0, -1, 1, -1, -1, 0, 1};

//-------------------state access functions---------------------

static unsigned int inthash(unsigned int key)
{
    key += (key << 12);
    key ^= (key >> 22);
    key += (key << 4);
    key ^= (key >> 9);
    key += (key << 10);
    key ^= (key >> 2);
    key += (key << 7);
    key ^= (key >> 12);
    return key;
}

//examples of hash functions: map state coordinates onto a hash value
//#define GETHASHBIN(X, Y) (Y*WIDTH_Y+X) 
//here we have state coord: <X1, X2, X3, X4>
unsigned int EnvironmentROBARM::GETHASHBIN(short unsigned int* coord, int numofcoord)
{
    int val = 0;

    for (int i = 0; i < numofcoord; i++) {
        val += inthash(coord[i]) << i;
    }

    return inthash(val) & (EnvROBARM.HashTableSize - 1);
}

void EnvironmentROBARM::PrintHashTableHist()
{
    int s0 = 0, s1 = 0, s50 = 0, s100 = 0, s200 = 0, s300 = 0, slarge = 0;

    for (int j = 0; j < EnvROBARM.HashTableSize; j++) {
        if ((int)EnvROBARM.Coord2StateIDHashTable[j].size() == 0)
            s0++;
        else if ((int)EnvROBARM.Coord2StateIDHashTable[j].size() < 50)
            s1++;
        else if ((int)EnvROBARM.Coord2StateIDHashTable[j].size() < 100)
            s50++;
        else if ((int)EnvROBARM.Coord2StateIDHashTable[j].size() < 200)
            s100++;
        else if ((int)EnvROBARM.Coord2StateIDHashTable[j].size() < 300)
            s200++;
        else if ((int)EnvROBARM.Coord2StateIDHashTable[j].size() < 400)
            s300++;
        else
            slarge++;
    }
    SBPL_PRINTF("hash table histogram: 0:%d, <50:%d, <100:%d, <200:%d, <300:%d, <400:%d >400:%d\n", s0, s1, s50, s100,
                s200, s300, slarge);
}

EnvROBARMHashEntry_t* EnvironmentROBARM::GetHashEntry(short unsigned int* coord, int numofcoord, bool bIsGoal)
{
    //clock_t currenttime = clock();

    //if it is goal
    if (bIsGoal) {
        return EnvROBARM.goalHashEntry;
    }

    int binid = GETHASHBIN(coord, numofcoord);

#if DEBUG
    if ((int)EnvROBARM.Coord2StateIDHashTable[binid].size() > 500)
    {
        SBPL_PRINTF("WARNING: Hash table has a bin %d (coord0=%d) of size %d\n",
            binid, coord[0], (int)EnvROBARM.Coord2StateIDHashTable[binid].size());

        PrintHashTableHist();
    }
#endif

    //iterate over the states in the bin and select the perfect match
    for (int ind = 0; ind < (int)EnvROBARM.Coord2StateIDHashTable[binid].size(); ind++) {
        int j = 0;
        for (j = 0; j < numofcoord; j++) {
            if (EnvROBARM.Coord2StateIDHashTable[binid][ind]->coord[j] != coord[j]) {
                break;
            }
        }
        if (j == numofcoord) {
            //time_gethash += clock()-currenttime;
            return EnvROBARM.Coord2StateIDHashTable[binid][ind];
        }
    }

    //time_gethash += clock()-currenttime;

    return NULL;
}

EnvROBARMHashEntry_t* EnvironmentROBARM::CreateNewHashEntry(short unsigned int* coord, int numofcoord,
                                                            short unsigned int endeffx, short unsigned int endeffy)
{
    int i;

    //clock_t currenttime = clock();

    EnvROBARMHashEntry_t* HashEntry = new EnvROBARMHashEntry_t;

    memcpy(HashEntry->coord, coord, numofcoord * sizeof(short unsigned int));
    HashEntry->endeffx = endeffx;
    HashEntry->endeffy = endeffy;

    HashEntry->stateID = EnvROBARM.StateID2CoordTable.size();

    //insert into the tables
    EnvROBARM.StateID2CoordTable.push_back(HashEntry);

    //get the hash table bin
    i = GETHASHBIN(HashEntry->coord, numofcoord);

    //insert the entry into the bin
    EnvROBARM.Coord2StateIDHashTable[i].push_back(HashEntry);

    //insert into and initialize the mappings
    int* entry = new int[NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(entry);
    for (i = 0; i < NUMOFINDICES_STATEID2IND; i++) {
        StateID2IndexMapping[HashEntry->stateID][i] = -1;
    }

    if (HashEntry->stateID != (int)StateID2IndexMapping.size() - 1) {
        SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID\n");
        throw new SBPL_Exception();
    }

    //time_createhash += clock()-currenttime;

    return HashEntry;
}

//-----------------------------------------------------------------------------

//---------------heuristic values computation-------------------------

int EnvironmentROBARM::distanceincoord(unsigned short* statecoord1, unsigned short* statecoord2)
{
    int totaldiff = 0;

    for (int i = 0; i < NUMOFLINKS; i++) {
        int diff = abs(statecoord1[i] - statecoord2[i]);
        //totaldiff += __min(diff, RobArmStateSpace.anglevals[i] - diff);
        totaldiff = __max(totaldiff, __min(diff, EnvROBARMCfg.anglevals[i] - diff));
    }

    return totaldiff;
}

void EnvironmentROBARM::ReInitializeState2D(State2D* state)
{
    state->g = INFINITECOST;
    state->iterationclosed = 0;
}

void EnvironmentROBARM::InitializeState2D(State2D* state, short unsigned int x, short unsigned int y)
{
    state->g = INFINITECOST;
    state->iterationclosed = 0;
    state->x = x;
    state->y = y;
}

/*
void EnvironmentROBARM::Search2DwithHeap(State** statespace, int searchstartx, int searchstarty)
{
    CKey key;
    CHeap* heap;
    State* ExpState;
    int newx, newy,x,y;

    //create a heap
    heap = new CHeap;

    //initialize to infinity all
    for (x = 0; x < EnvROBARMCfg.EnvWidth_c; x++)
    {
        for (y = 0; y < EnvROBARMCfg.EnvHeight_c; y++)
        {
            HeurGrid[x][y] = INFINITECOST;
            ReInitializeState2D(&statespace[x][y]);
        }
    }

    //initialization
    statespace[searchstartx][searchstarty].g = 0;
    key.key[0] = 0;
    heap->insertheap(&statespace[searchstartx][searchstarty], key);

    //expand all of the states
    while(!heap->emptyheap())
    {
        //get the state to expand
        ExpState = 	heap->deleteminheap();

        //set the corresponding heuristics
        HeurGrid[ExpState->statecoord[0]][ExpState->statecoord[1]] = ExpState->g;

        //iterate through neighbors
        for(int d = 0; d < DIRECTIONS; d++)
        {
            newx = ExpState->statecoord[0] + dx[d];
            newy = ExpState->statecoord[1] + dy[d];

            //make sure it is inside the map
            if(0 > newx || newx >= EnvROBARMCfg.EnvWidth_c ||
                    0 > newy || newy >= EnvROBARMCfg.EnvHeight_c)
                continue;

            if(statespace[newx][newy].g > ExpState->g + 1 &&
                    EnvROBARMCfg.Grid2D[newx][newy] == 0)
            {
                statespace[newx][newy].g = ExpState->g + 1;
                key.key[0] = statespace[newx][newy].g;
                if(statespace[newx][newy].heapindex == 0)
                    heap->insertheap(&statespace[newx][newy], key);
                else
                    heap->updateheap(&statespace[newx][newy], key);
            }
        }
    }

    //delete the heap
    delete heap;
}
*/

void EnvironmentROBARM::Search2DwithQueue(State2D** statespace, int* HeurGrid, int searchstartx, int searchstarty)
{
    State2D* ExpState;
    int newx, newy, x, y;

    //create a queue
    queue<State2D*> Queue;

    //initialize to infinity all
    for (x = 0; x < EnvROBARMCfg.EnvWidth_c; x++) {
        for (y = 0; y < EnvROBARMCfg.EnvHeight_c; y++) {
            HeurGrid[XYTO2DIND(x,y)] = INFINITECOST;
            ReInitializeState2D(&statespace[x][y]);
        }
    }

    //initialization
    statespace[searchstartx][searchstarty].g = 0;
    Queue.push(&statespace[searchstartx][searchstarty]);

    //expand all of the states
    while ((int)Queue.size() > 0) {
        //get the state to expand
        ExpState = Queue.front();
        Queue.pop();

        //it may be that the state is already closed
        if (ExpState->iterationclosed == 1) continue;

        //close it
        ExpState->iterationclosed = 1;

        //set the corresponding heuristics
        HeurGrid[XYTO2DIND(ExpState->x, ExpState->y)] = ExpState->g;

        //iterate through neighbors
        for (int d = 0; d < DIRECTIONS; d++) {
            newx = ExpState->x + dx[d];
            newy = ExpState->y + dy[d];

            //make sure it is inside the map and has no obstacle
            if (0 > newx || newx >= EnvROBARMCfg.EnvWidth_c || 0 > newy || newy >= EnvROBARMCfg.EnvHeight_c
                || EnvROBARMCfg.Grid2D[newx][newy] == 1) continue;

            //check
            if (statespace[newx][newy].g != INFINITECOST && statespace[newx][newy].g > ExpState->g + 1) {
                SBPL_ERROR("ERROR: incorrect heuristic computation\n");
                throw new SBPL_Exception();
            }

            if (statespace[newx][newy].iterationclosed == 0 && statespace[newx][newy].g == INFINITECOST) {
                //insert into the stack
                Queue.push(&statespace[newx][newy]);

                //set the g-value
                statespace[newx][newy].g = ExpState->g + 1;
            }
        }
    }
}

void EnvironmentROBARM::Create2DStateSpace(State2D*** statespace2D)
{
    int x, y;

    //allocate a statespace for 2D search
    *statespace2D = new State2D*[EnvROBARMCfg.EnvWidth_c];
    for (x = 0; x < EnvROBARMCfg.EnvWidth_c; x++) {
        (*statespace2D)[x] = new State2D[EnvROBARMCfg.EnvHeight_c];
        for (y = 0; y < EnvROBARMCfg.EnvWidth_c; y++) {
            InitializeState2D(&(*statespace2D)[x][y], x, y);
        }
    }
}

void EnvironmentROBARM::Delete2DStateSpace(State2D*** statespace2D)
{
    int x;

    //delete the 2D statespace
    for (x = 0; x < EnvROBARMCfg.EnvWidth_c; x++) {
        delete[] (*statespace2D)[x];
    }
    delete *statespace2D;
}

void EnvironmentROBARM::ComputeHeuristicValues()
{
    int i;

    SBPL_PRINTF("Running 2D BFS to compute heuristics\n");

    //allocate memory
    int hsize = XYTO2DIND(EnvROBARMCfg.EnvWidth_c-1, EnvROBARMCfg.EnvHeight_c-1) + 1;
    EnvROBARM.Heur = new int*[hsize];
    for (i = 0; i < hsize; i++) {
        EnvROBARM.Heur[i] = new int[hsize];
    }

    //now compute the heuristics for each goal location

    State2D** statespace2D;
    Create2DStateSpace(&statespace2D);

    for (int x = 0; x < EnvROBARMCfg.EnvWidth_c; x++) {
        for (int y = 0; y < EnvROBARMCfg.EnvHeight_c; y++) {
            int hind = XYTO2DIND(x,y);

            //perform the search for pathcosts to x,y and store values in Heur[hind]
            Search2DwithQueue(statespace2D, &EnvROBARM.Heur[hind][0], x, y);
        }
        //SBPL_PRINTF("h for %d computed\n", x);
    }
    Delete2DStateSpace(&statespace2D);

    SBPL_PRINTF("done\n");
}

//----------------------------------------------------------------------

//--------------printing routines---------------------------------------

/*
void EnvironmentROBARM::PrintHeurGrid()
{
    int x,y;

    for (x = 0; x < EnvROBARMCfg.EnvWidth_c; x++)
    {
        for (y = 0; y < EnvROBARMCfg.EnvHeight_c; y++)
        {
            SBPL_PRINTF("HeurGrid[%d][%d]=%u\n", x,y,HeurGrid[x][y]);
        }
        SBPL_PRINTF("\n");
    }
}
*/

void EnvironmentROBARM::printangles(FILE* fOut, short unsigned int* coord, bool bGoal, bool bVerbose, bool bLocal)
{
    double angles[NUMOFLINKS];
    int i;
    short unsigned int x, y;

    ComputeContAngles(coord, angles);
    if (bVerbose) SBPL_FPRINTF(fOut, "angles: ");
    for (i = 0; i < NUMOFLINKS; i++) {
        if (!bLocal)
            SBPL_FPRINTF(fOut, "%f ", angles[i]);
        else {
            if (i > 0)
                SBPL_FPRINTF(fOut, "%f ", angles[i] - angles[i - 1]);
            else
                SBPL_FPRINTF(fOut, "%f ", angles[i]);
        }
    }
    ComputeEndEffectorPos(angles, &x, &y);
    if (bGoal) {
        x = EnvROBARMCfg.EndEffGoalX_c;
        y = EnvROBARMCfg.EndEffGoalY_c;
    }
    if (bVerbose)
        SBPL_FPRINTF(fOut, "endeff: %d %d", x, y);
    else
        SBPL_FPRINTF(fOut, "%d %d", x, y);

    SBPL_FPRINTF(fOut, "\n");
}

/*
void EnvironmentROBARM::PrintPathRec(State* currentstate, State* searchstartstate, int* cost)
{
    if(currentstate == searchstartstate)
    {
        return;
    }

    if(currentstate->g > currentstate->v)
    {
        SBPL_ERROR("ERROR: an underconsistent state is encountered on the path\n");

#if PLANNER_TYPE == ARA_PLANNER_TYPE
        PrintState(currentstate, stdout);
#endif
        throw new SBPL_Exception();
    }

    //cost of the transition to the previous state
    *cost = *cost + currentstate->costtobestnextstate;
    if(*cost > 100000 || currentstate->g < currentstate->bestnextstate->v +
            currentstate->costtobestnextstate ||
            !IsValidCoord(currentstate->bestnextstate->statecoord))
    {
#if PLANNER_TYPE == ARA_PLANNER_TYPE
        SBPL_PRINTF("currentstate\n");
        PrintState(currentstate, stdout);
        SBPL_PRINTF("nextstate\n");
        PrintState(currentstate->bestnextstate, stdout);
#endif
        SBPL_PRINTF("next statecoord IsValid=%d\n",
                IsValidCoord(currentstate->bestnextstate->statecoord));
        SBPL_ERROR("ERROR: cost of the path too high=%d or incorrect transition\n", *cost);
        throw new SBPL_Exception();
    }
    //transition itself
    currentstate = currentstate->bestnextstate;

    //proceed recursively
#if FORWARD_SEARCH
    PrintPathRec(currentstate, searchstartstate, cost);
    printangles(fSol, currentstate);
#else
    printangles(fSol, currentstate);
    PrintPathRec(currentstate, searchstartstate, cost);
#endif
}

//prints found path and some statistics
int EnvironmentROBARM::PrintPathStat(double erreps, int* pathcost)
{
    State *goalstate, *startstate;
    short unsigned int coord[NUMOFLINKS];
    int cost = 0;
    int inconssize;

    goalstate = RobArmStateSpace.goalstate;

    ComputeCoord(RobArmStateSpace.currentangle, coord);
    startstate = GetState(coord);

#if FORWARD_SEARCH
    if(goalstate == NULL || goalstate->bestnextstate == NULL)
    {
        SBPL_PRINTF("No path is found\n");
        *pathcost = INFINITECOST;
        return 0;
    }
    PrintPathRec(goalstate, startstate, &cost);
    printangles(fSol, goalstate);
#else
    if(startstate == NULL || startstate->bestnextstate == NULL)
    {
        SBPL_PRINTF("No path is found\n");
        *pathcost = INFINITECOST;
        return 0;
    }
    int goalh = Heuristic(goalstate, startstate);
    if(startstate->g+startstate->h < goalh)
    {
        SBPL_ERROR("ERROR: invalid heuristic. goalh(updated)=%d startg=%d\n",
                goalh, startstate->g);
#if PLANNER_TYPE == ARA_PLANNER_TYPE
        PrintState(startstate, stdout);
#endif
        throw new SBPL_Exception();
    }
    else
        SBPL_PRINTF("goalh=%d startg=%d\n", goalstate->h, startstate->g);
    printangles(fSol, startstate);
    PrintPathRec(startstate, goalstate, &cost);
#endif

    SBPL_FPRINTF(fSol, "path cost=%d at eps=%f\n", cost, erreps);
    SBPL_PRINTF("path cost=%d\n", cost);

#if PLANNER_TYPE == ANYASTAR_PLANNER_TYPE
    inconssize = 0;
#else
    inconssize = RobArmStateSpace.inconslist->currentsize;
#endif

    SBPL_PRINTF("open size=%d, incons size=%d\n",
            RobArmStateSpace.heap->currentsize, inconssize);
    SBPL_FPRINTF(fSol, "open size=%d, incons size=%d\n",
            RobArmStateSpace.heap->currentsize, inconssize);

    SBPL_FPRINTF(fSol1, "**************\n");
    SBPL_FFLUSH(fStat);

    *pathcost = cost;

    return 1;
}

void EnvironmentROBARM::PrintInfo()
{
    int i;
    unsigned int statespace_size = 1;
    double fsize = 1.0;

    SBPL_FPRINTF(fSol, "statespace dim=%d size= <", NUMOFLINKS);
    SBPL_FPRINTF(fSol1, "%d\n", NUMOFLINKS);
    for(i = 0; i < NUMOFLINKS; i++)
    {
        statespace_size *= RobArmStateSpace.anglevals[i];
        SBPL_FPRINTF(fSol, "%d ", RobArmStateSpace.anglevals[i]);
        SBPL_FPRINTF(fSol1, "%f ", EnvROBARMCfg.LinkLength_m[i]);
        fsize = fsize*RobArmStateSpace.anglevals[i];
    }
    SBPL_FPRINTF(fSol, "> => %g\n", fsize);
    SBPL_FPRINTF(fSol1, "\n");

    SBPL_PRINTF("dimensionality: %d statespace: %u (over 10^%d)\n", NUMOFLINKS, statespace_size,
            (int)(log10(fsize)));
}

void PrintCoord(short unsigned int coord[NUMOFLINKS], FILE* fOut)
{
    for(int i = 0; i < NUMOFLINKS; i++)
        SBPL_FPRINTF(fOut, "%d ", coord[i]);
    SBPL_FPRINTF(fOut, "\n");
}
*/

//----------------------------------------------------------------------

//--------------------Additional domain specific functions--------------------------------------------------

void EnvironmentROBARM::ReadConfiguration(FILE* fCfg)
{
    char sTemp[1024];
    int dTemp;
    int x, y, i;

    //environmentsize(meters)
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvROBARMCfg.EnvWidth_m = atof(sTemp);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvROBARMCfg.EnvHeight_m = atof(sTemp);

    //discretization(cells)
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvROBARMCfg.EnvWidth_c = atoi(sTemp);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvROBARMCfg.EnvHeight_c = atoi(sTemp);

    //basex(cells):
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvROBARMCfg.BaseX_c = atoi(sTemp);

    //linklengths(meters):
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    for (i = 0; i < NUMOFLINKS; i++) {
        if (fscanf(fCfg, "%s", sTemp) != 1) {
            SBPL_ERROR("ERROR: ran out of env file early\n");
            throw new SBPL_Exception();
        }
        EnvROBARMCfg.LinkLength_m[i] = atof(sTemp);
    }

    //linkstartangles(degrees):
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    for (i = 0; i < NUMOFLINKS; i++) {
        if (fscanf(fCfg, "%s", sTemp) != 1) {
            SBPL_ERROR("ERROR: ran out of env file early\n");
            throw new SBPL_Exception();
        }
        EnvROBARMCfg.LinkStartAngles_d[i] = atoi(sTemp);
    }

    //endeffectorgoal(cells) or linkgoalangles(degrees):
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    if (strcmp(sTemp, "endeffectorgoal(cells):") == 0) {
        //only endeffector is specified
        if (fscanf(fCfg, "%s", sTemp) != 1) {
            SBPL_ERROR("ERROR: ran out of env file early\n");
            throw new SBPL_Exception();
        }
        EnvROBARMCfg.EndEffGoalX_c = atoi(sTemp);
        if (fscanf(fCfg, "%s", sTemp) != 1) {
            SBPL_ERROR("ERROR: ran out of env file early\n");
            throw new SBPL_Exception();
        }
        EnvROBARMCfg.EndEffGoalY_c = atoi(sTemp);

        //set goalangle to invalid number
        EnvROBARMCfg.LinkGoalAngles_d[0] = INVALID_NUMBER;
    }
    else if (strcmp(sTemp, "linkgoalangles(degrees):") == 0) {
        double goalangles[NUMOFLINKS];

        //linkgoalangles(degrees): 90.0 180.0 90.0
        for (i = 0; i < NUMOFLINKS; i++) {
            if (fscanf(fCfg, "%s", sTemp) != 1) {
                SBPL_ERROR("ERROR: ran out of env file early\n");
                throw new SBPL_Exception();
            }
            EnvROBARMCfg.LinkGoalAngles_d[i] = atoi(sTemp);
        }
        //compute endeffectorgoal(cells):
        for (i = 0; i < NUMOFLINKS; i++) {
            goalangles[i] = PI_CONST * (EnvROBARMCfg.LinkGoalAngles_d[i] / 180.0);
        }
        ComputeEndEffectorPos(goalangles, &EnvROBARMCfg.EndEffGoalX_c, &EnvROBARMCfg.EndEffGoalY_c);
    }
    else {
        SBPL_ERROR("ERROR: invalid string encountered=%s\n", sTemp);
        throw new SBPL_Exception();
    }

    //allocate the 2D environment
    EnvROBARMCfg.Grid2D = new char*[EnvROBARMCfg.EnvWidth_c];
    for (x = 0; x < EnvROBARMCfg.EnvWidth_c; x++) {
        EnvROBARMCfg.Grid2D[x] = new char[EnvROBARMCfg.EnvHeight_c];
    }

    //environment:
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    for (y = 0; y < EnvROBARMCfg.EnvHeight_c; y++)
        for (x = 0; x < EnvROBARMCfg.EnvWidth_c; x++) {
            if (fscanf(fCfg, "%d", &dTemp) != 1) {
                SBPL_ERROR("ERROR: incorrect format of config file\n");
                throw new SBPL_Exception();
            }
            EnvROBARMCfg.Grid2D[x][y] = dTemp;
        }

    //set additional parameters
    EnvROBARMCfg.GridCellWidth = EnvROBARMCfg.EnvWidth_m / EnvROBARMCfg.EnvWidth_c;
    if (EnvROBARMCfg.GridCellWidth != EnvROBARMCfg.EnvHeight_m / EnvROBARMCfg.EnvHeight_c) {
        SBPL_ERROR("ERROR: The cell should be square\n");
        throw new SBPL_Exception();
    }
}

void EnvironmentROBARM::DiscretizeAngles()
{
    int i;
    double HalfGridCell = EnvROBARMCfg.GridCellWidth / 2.0;

    for (i = 0; i < NUMOFLINKS; i++) {
        EnvROBARMCfg.angledelta[i] = 2 * asin(HalfGridCell / EnvROBARMCfg.LinkLength_m[i]);
        EnvROBARMCfg.anglevals[i] = (int)(2.0 * PI_CONST / EnvROBARMCfg.angledelta[i] + 0.99999999);
    }
}

//angles are counterclokwise from 0 to 360 in radians, 0 is the center of bin 0, ...
void EnvironmentROBARM::ComputeContAngles(short unsigned int coord[NUMOFLINKS], double angle[NUMOFLINKS])
{
    int i;

    for (i = 0; i < NUMOFLINKS; i++) {
        angle[i] = coord[i] * EnvROBARMCfg.angledelta[i];
    }
}

void EnvironmentROBARM::ComputeCoord(double angle[NUMOFLINKS], short unsigned int coord[NUMOFLINKS])
{
    int i;

    for (i = 0; i < NUMOFLINKS; i++) {
        coord[i] = (int)((angle[i] + EnvROBARMCfg.angledelta[i] * 0.5) / EnvROBARMCfg.angledelta[i]);
        if (coord[i] == EnvROBARMCfg.anglevals[i]) coord[i] = 0;
    }
}

unsigned int EnvironmentROBARM::GetHeurBasedonCoord(short unsigned int coord[NUMOFLINKS])
{
    short unsigned int endeffx, endeffy;
    double angles[NUMOFLINKS];
    unsigned int h;

    ComputeContAngles(coord, angles);
    ComputeEndEffectorPos(angles, &endeffx, &endeffy);

#if INFORMED
    h = HeurGrid[endeffx][endeffy];
#else
    h = 0;
#endif

    return h;
}

void EnvironmentROBARM::Cell2ContXY(int x, int y, double *pX, double *pY)
{
    *pX = x * EnvROBARMCfg.GridCellWidth + EnvROBARMCfg.GridCellWidth * 0.5;
    *pY = y * EnvROBARMCfg.GridCellWidth + EnvROBARMCfg.GridCellWidth * 0.5;
}

void EnvironmentROBARM::ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY)
{
    //take the nearest cell
    *pX = (int)(x / EnvROBARMCfg.GridCellWidth);
    if (x < 0) *pX = 0;
    if (*pX >= EnvROBARMCfg.EnvWidth_c) *pX = EnvROBARMCfg.EnvWidth_c - 1;

    *pY = (int)(y / EnvROBARMCfg.GridCellWidth);
    if (y < 0) *pY = 0;
    if (*pY >= EnvROBARMCfg.EnvHeight_c) *pY = EnvROBARMCfg.EnvHeight_c - 1;
}

//returns 1 if end effector within space, 0 otherwise
int EnvironmentROBARM::ComputeEndEffectorPos(double angles[NUMOFLINKS], short unsigned int* pX, short unsigned int* pY)
{
    double x, y;
    int i;
    int retval = 1;

    //start with the base
    Cell2ContXY(EnvROBARMCfg.BaseX_c, EnvROBARMCfg.EnvHeight_c - 1, &x, &y);

    //translate the point along each link
    for (i = 0; i < NUMOFLINKS; i++) {
        x = x + EnvROBARMCfg.LinkLength_m[i] * cos(angles[i]);
        y = y - EnvROBARMCfg.LinkLength_m[i] * sin(angles[i]);
    }

    if (x < 0 || x >= EnvROBARMCfg.EnvWidth_m || y < 0 || y >= EnvROBARMCfg.EnvHeight_m) retval = 0;

    ContXY2Cell(x, y, pX, pY);

    //return 1;
    return retval;
}

//if pTestedCells is NULL, then the tested points are not saved and it is more
//efficient as it returns as soon as it sees first invalid point
int EnvironmentROBARM::IsValidLineSegment(double x0, double y0, double x1, double y1, char **Grid2D,
                                          vector<CELLV>* pTestedCells)

{
    bresenham_param_t params;
    int nX, nY;
    short unsigned int nX0, nY0, nX1, nY1;
    int retvalue = 1;
    CELLV tempcell;

    //make sure the line segment is inside the environment
    if (x0 < 0 || x0 >= EnvROBARMCfg.EnvWidth_m || x1 < 0 || x1 >= EnvROBARMCfg.EnvWidth_m || y0 < 0 ||
        y0 >= EnvROBARMCfg.EnvHeight_m || y1 < 0 || y1 >= EnvROBARMCfg.EnvHeight_m)
    {
        return 0;
    }

    ContXY2Cell(x0, y0, &nX0, &nY0);
    ContXY2Cell(x1, y1, &nX1, &nY1);

    //iterate through the points on the segment
    get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
    do {
        get_current_point(&params, &nX, &nY);
        if (Grid2D[nX][nY] == 1) {
            if (pTestedCells == NULL)
                return 0;
            else
                retvalue = 0;
        }

        //insert the tested point
        if (pTestedCells) {
            tempcell.bIsObstacle = (Grid2D[nX][nY] == 1);
            tempcell.x = nX;
            tempcell.y = nY;
            pTestedCells->push_back(tempcell);
        }
    }
    while (get_next_point(&params));

    return retvalue;
}

int EnvironmentROBARM::IsValidCoord(short unsigned int coord[NUMOFLINKS], char** Grid2D /*=NULL*/,
                                    vector<CELLV>* pTestedCells /*=NULL*/)
{
    double angles[NUMOFLINKS];
    int retvalue = 1;

    if (Grid2D == NULL) Grid2D = EnvROBARMCfg.Grid2D;

#if ENDEFF_CHECK_ONLY
    int endeffx, endeffy;

    //just check whether end effector is in valid position
    ComputeContAngles(coord, angles);
    if(ComputeEndEffectorPos(angles, &endeffx, &endeffy) == false)
    return 0;

    //check that it is inside the grid
    if(endeffx < 0 || endeffx >= EnvROBARMCfg.EnvWidth_c ||
       endeffy < 0 || endeffy >= EnvROBARMCfg.EnvHeight_c)
    {
        return 0;
    }

    if(pTestedCells)
    {
        CELLV tempcell;
        tempcell.IsObstacle = Grid2D[endeffx][endeffy];
        tempcell.x = endeffx;
        tempcell.y = endeffy;

        pTestedCells->push_back(tempcell);
    }

    //check end effector
    if(Grid2D[endeffx][endeffy] == 1)
    return 0;
#else
    double x0, y0, x1, y1;
    int i;

    //full check of all the links
    ComputeContAngles(coord, angles);

    //iterate through all the links
    Cell2ContXY(EnvROBARMCfg.BaseX_c, EnvROBARMCfg.EnvHeight_c - 1, &x1, &y1);
    for (i = 0; i < NUMOFLINKS; i++) {
        //compute the corresponding line segment
        x0 = x1;
        y0 = y1;
        x1 = x0 + EnvROBARMCfg.LinkLength_m[i] * cos(angles[i]);
        y1 = y0 - EnvROBARMCfg.LinkLength_m[i] * sin(angles[i]);

        //check the validity of the corresponding line segment
        if (!IsValidLineSegment(x0, y0, x1, y1, Grid2D, pTestedCells)) {
            if (pTestedCells == NULL)
                return 0;
            else
                retvalue = 0;
        }
    }

#endif

    return retvalue;
}

int EnvironmentROBARM::cost(short unsigned int state1coord[], short unsigned int state2coord[])
{

    if (!IsValidCoord(state1coord) || !IsValidCoord(state2coord)) return INFINITECOST;

#if UNIFORM_COST
    return 1;
#else
    int i;
    //the cost becomes higher as we are closer to the base
    for (i = 0; i < NUMOFLINKS; i++) {
        if (state1coord[i] != state2coord[i]) return (NUMOFLINKS - i);
        //return (NUMOFLINKS-i)*(NUMOFLINKS-i);
    }

    SBPL_ERROR("ERROR: cost on the same states is called:\n");
    //printangles(stdout, state1coord);
    //printangles(stdout, state2coord);

    throw new SBPL_Exception();

#endif
}

void EnvironmentROBARM::InitializeEnvConfig()
{
    //find the discretization for each angle and store the discretization
    DiscretizeAngles();
}

bool EnvironmentROBARM::InitializeEnvironment()
{
    short unsigned int coord[NUMOFLINKS];
    double startangles[NUMOFLINKS];
    double angles[NUMOFLINKS];
    int i;
    short unsigned int endeffx, endeffy;

    //initialize the map from Coord to StateID
    EnvROBARM.HashTableSize = 32 * 1024; //should be power of two
    EnvROBARM.Coord2StateIDHashTable = new vector<EnvROBARMHashEntry_t*> [EnvROBARM.HashTableSize];

    //initialize the map from StateID to Coord
    EnvROBARM.StateID2CoordTable.clear();

    //initialize the angles of the start states
    for (i = 0; i < NUMOFLINKS; i++) {
        startangles[i] = PI_CONST * (EnvROBARMCfg.LinkStartAngles_d[i] / 180.0);
    }

    ComputeCoord(startangles, coord);
    ComputeContAngles(coord, angles);
    ComputeEndEffectorPos(angles, &endeffx, &endeffy);

    //create the start state
    EnvROBARM.startHashEntry = CreateNewHashEntry(coord, NUMOFLINKS, endeffx, endeffy);

    //create the goal state 
    //initialize the coord of goal state
    for (i = 0; i < NUMOFLINKS; i++) {
        coord[i] = 0;
    }
    EnvROBARM.goalHashEntry = CreateNewHashEntry(coord, NUMOFLINKS, EnvROBARMCfg.EndEffGoalX_c,
                                                 EnvROBARMCfg.EndEffGoalY_c);

    //check the validity of both goal and start configurations
    //testing for EnvROBARMCfg.EndEffGoalX_c < 0  and EnvROBARMCfg.EndEffGoalY_c < 0 is useless since they are unsigned 
    if (!IsValidCoord(EnvROBARM.startHashEntry->coord) || EnvROBARMCfg.EndEffGoalX_c >= EnvROBARMCfg.EnvWidth_c
        || EnvROBARMCfg.EndEffGoalY_c >= EnvROBARMCfg.EnvHeight_c) {
        SBPL_PRINTF("Either start or goal configuration is invalid\n");
        return false;
    }

    //for now heuristics are not set
    EnvROBARM.Heur = NULL;

    return true;
}

//----------------------------------------------------------------------

//-----------interface with outside functions-----------------------------------

bool EnvironmentROBARM::InitializeEnv(const char* sEnvFile)
{
    FILE* fCfg = fopen(sEnvFile, "r");
    if (fCfg == NULL) {
        SBPL_ERROR("ERROR: unable to open %s\n", sEnvFile);
        throw new SBPL_Exception();
    }
    ReadConfiguration(fCfg);
    fclose(fCfg);

    //Initialize other parameters of the environment
    InitializeEnvConfig();

    //initialize Environment
    if (InitializeEnvironment() == false) return false;

    //pre-compute heuristics
    ComputeHeuristicValues();

    return true;
}

bool EnvironmentROBARM::InitializeMDPCfg(MDPConfig *MDPCfg)
{
    //initialize MDPCfg with the start and goal ids
    MDPCfg->goalstateid = EnvROBARM.goalHashEntry->stateID;
    MDPCfg->startstateid = EnvROBARM.startHashEntry->stateID;

    return true;
}

int EnvironmentROBARM::GetFromToHeuristic(int FromStateID, int ToStateID)
{
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if(FromStateID >= (int)EnvROBARM.StateID2CoordTable.size()
        || ToStateID >= (int)EnvROBARM.StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in EnvROBARM... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    //get X, Y for the state
    EnvROBARMHashEntry_t* FromHashEntry = EnvROBARM.StateID2CoordTable[FromStateID];
    EnvROBARMHashEntry_t* ToHashEntry = EnvROBARM.StateID2CoordTable[ToStateID];

    int h =
            EnvROBARM.Heur[XYTO2DIND(ToHashEntry->endeffx, ToHashEntry->endeffy)][XYTO2DIND(FromHashEntry->endeffx, FromHashEntry->endeffy)];

    /* 
     if(ToStateID != EnvROBARM.goalHashEntry->stateID)
     {
     //also consider the distanceincoord

     //get the heuristic based on angles
     unsigned int hangles = distanceincoord(FromHashEntry->coord, ToHashEntry->coord);

     //the heuristic is the max of the two
     h = __max(h, hangles);
     }
     */

    return h;
}

int EnvironmentROBARM::GetGoalHeuristic(int stateID)
{
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if (stateID >= (int)EnvROBARM.StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvROBARM... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    //define this function if it used in the planner (heuristic forward search would use it)
    return GetFromToHeuristic(stateID, EnvROBARM.goalHashEntry->stateID);
}

int EnvironmentROBARM::GetStartHeuristic(int stateID)
{
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if (stateID >= (int)EnvROBARM.StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvROBARM... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    //define this function if it used in the planner (heuristic backward search would use it)

    SBPL_ERROR("ERROR in EnvROBARM.. function: GetStartHeuristic is undefined\n");
    throw new SBPL_Exception();

    return 0;
}

void EnvironmentROBARM::SetAllPreds(CMDPSTATE* state)
{
    //implement this if the planner needs access to predecessors

    SBPL_ERROR("ERROR in EnvROBARM... function: SetAllPreds is undefined\n");
    throw new SBPL_Exception();
}

int EnvironmentROBARM::SizeofCreatedEnv()
{
    return EnvROBARM.StateID2CoordTable.size();
}

void EnvironmentROBARM::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{

#if DEBUG
    if(stateID >= (int)EnvROBARM.StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in EnvROBARM... function: stateID illegal (2)\n");
        throw new SBPL_Exception();
    }
#endif

    if (fOut == NULL) fOut = stdout;

    EnvROBARMHashEntry_t* HashEntry = EnvROBARM.StateID2CoordTable[stateID];

    bool bGoal = false;
    if (stateID == EnvROBARM.goalHashEntry->stateID) bGoal = true;

    if (stateID == EnvROBARM.goalHashEntry->stateID && bVerbose) {
        SBPL_FPRINTF(fOut, "the state is a goal state\n");
        bGoal = true;
    }
    
    printangles(fOut, HashEntry->coord, bGoal, bVerbose, false);
}

//get the goal as a successor of source state at given cost
//if costtogoal = -1 then the succ is chosen
void EnvironmentROBARM::PrintSuccGoal(int SourceStateID, int costtogoal, bool bVerbose, bool bLocal /*=false*/,
                                      FILE* fOut /*=NULL*/)
{
    short unsigned int succcoord[NUMOFLINKS];
    double angles[NUMOFLINKS];
    short unsigned int endeffx, endeffy;
    int i, inc;

    if (fOut == NULL) fOut = stdout;

    EnvROBARMHashEntry_t* HashEntry = EnvROBARM.StateID2CoordTable[SourceStateID];

    //default coords of successor
    for (i = 0; i < NUMOFLINKS; i++)
        succcoord[i] = HashEntry->coord[i];

    //iterate through successors of s
    for (i = 0; i < NUMOFLINKS; i++) {
        //increase and decrease in ith angle
        for (inc = -1; inc < 2; inc = inc + 2) {
            if (inc == -1) {
                if (HashEntry->coord[i] == 0)
                    succcoord[i] = EnvROBARMCfg.anglevals[i] - 1;
                else
                    succcoord[i] = HashEntry->coord[i] + inc;
            }
            else {
                succcoord[i] = (HashEntry->coord[i] + inc) % EnvROBARMCfg.anglevals[i];
            }

            //skip invalid successors
            if (!IsValidCoord(succcoord)) continue;

            ComputeContAngles(succcoord, angles);
            ComputeEndEffectorPos(angles, &endeffx, &endeffy);
            if (endeffx == EnvROBARMCfg.EndEffGoalX_c && endeffy == EnvROBARMCfg.EndEffGoalY_c) {
                if (cost(HashEntry->coord, succcoord) == costtogoal || costtogoal == -1) {
                    if (bVerbose) SBPL_FPRINTF(fOut, "the state is a goal state\n");
                    printangles(fOut, succcoord, true, bVerbose, bLocal);
                    return;
                }
            }
        }

        //restore it back
        succcoord[i] = HashEntry->coord[i];
    }
}

void EnvironmentROBARM::PrintEnv_Config(FILE* fOut)
{
    //implement this if the planner needs to print out EnvROBARM. configuration

    SBPL_ERROR("ERROR in EnvROBARM... function: PrintEnv_Config is undefined\n");
    throw new SBPL_Exception();
}

void EnvironmentROBARM::PrintHeader(FILE* fOut)
{
    SBPL_FPRINTF(fOut, "%d\n", NUMOFLINKS);
    for (int i = 0; i < NUMOFLINKS; i++)
        SBPL_FPRINTF(fOut, "%.3f ", EnvROBARMCfg.LinkLength_m[i]);
    SBPL_FPRINTF(fOut, "\n");
}

void EnvironmentROBARM::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
    SBPL_ERROR("ERROR in EnvROBARM..function: SetAllActionsandOutcomes is undefined\n");
    throw new SBPL_Exception();
}

void EnvironmentROBARM::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
    int i, inc;
    short unsigned int succcoord[NUMOFLINKS];
    double angles[NUMOFLINKS];

    //clear the successor array
    SuccIDV->clear();
    CostV->clear();

    //goal state should be absorbing
    if (SourceStateID == EnvROBARM.goalHashEntry->stateID) return;

    //get X, Y for the state
    EnvROBARMHashEntry_t* HashEntry = EnvROBARM.StateID2CoordTable[SourceStateID];

    //default coords of successor
    for (i = 0; i < NUMOFLINKS; i++)
        succcoord[i] = HashEntry->coord[i];

    //iterate through successors of s
    for (i = 0; i < NUMOFLINKS; i++) {
        //increase and decrease in ith angle
        for (inc = -1; inc < 2; inc = inc + 2) {
            if (inc == -1) {
                if (HashEntry->coord[i] == 0)
                    succcoord[i] = EnvROBARMCfg.anglevals[i] - 1;
                else
                    succcoord[i] = HashEntry->coord[i] + inc;
            }
            else {
                succcoord[i] = (HashEntry->coord[i] + inc) % EnvROBARMCfg.anglevals[i];
            }

            //skip invalid successors
            if (!IsValidCoord(succcoord)) continue;

            //get the successor
            EnvROBARMHashEntry_t* OutHashEntry;
            bool bSuccisGoal = false;
            short unsigned int endeffx = 0, endeffy = 0;
            bool bEndEffComputed = false;
            if (abs(HashEntry->endeffx - EnvROBARMCfg.EndEffGoalX_c) < 3 || abs(HashEntry->endeffy
                - EnvROBARMCfg.EndEffGoalY_c) < 3) {
                //do a strict checks on the endeff coordinates of the successor
                ComputeContAngles(succcoord, angles);
                ComputeEndEffectorPos(angles, &endeffx, &endeffy);
                if (endeffx == EnvROBARMCfg.EndEffGoalX_c && endeffy == EnvROBARMCfg.EndEffGoalY_c) {
                    bSuccisGoal = true;
                    //SBPL_PRINTF("goal succ is generated\n");
                }
                bEndEffComputed = true;
            }

            if ((OutHashEntry = GetHashEntry(succcoord, NUMOFLINKS, bSuccisGoal)) == NULL) {
                if (bEndEffComputed == false) {
                    ComputeContAngles(succcoord, angles);
                    ComputeEndEffectorPos(angles, &endeffx, &endeffy);
                }

                //have to create a new entry
                OutHashEntry = CreateNewHashEntry(succcoord, NUMOFLINKS, endeffx, endeffy);
            }
            SuccIDV->push_back(OutHashEntry->stateID);
            CostV->push_back(cost(HashEntry->coord, succcoord));
        }

        //restore it back
        succcoord[i] = HashEntry->coord[i];
    }
}

void EnvironmentROBARM::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{
    SBPL_ERROR("ERROR in EnvROBARM... function: GetPreds is undefined\n");
    throw new SBPL_Exception();
}

//generate succs at some domain-dependent distance - see environment.h for more info
void EnvironmentROBARM::GetRandomSuccsatDistance(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CLowV)
{
    short unsigned int coord[NUMOFLINKS];
    short unsigned int endeffx, endeffy;
    double angles[NUMOFLINKS];

    //clear the successor array
    SuccIDV->clear();
    CLowV->clear();

    //the number of successors
    int numofsuccs = ROBARM_NUMOFRANDSUCCSATDIST;

    //goal state should be absorbing
    if (SourceStateID == EnvROBARM.goalHashEntry->stateID) return;

    //get X, Y for the state
    EnvROBARMHashEntry_t* HashEntry = EnvROBARM.StateID2CoordTable[SourceStateID];

    //iterate through random actions
    for (int succind = 0; succind < numofsuccs; succind++) {
        //pick the coordinate that will have dist = longactiondist
        int maxcoordind = (int)(NUMOFLINKS * (((double)rand()) / RAND_MAX));

        //now iterate over the coordinates
        for (int cind = 0; cind < NUMOFLINKS; cind++) {

            if (cind == maxcoordind) {
                //we know the magnitude and just need to set sign
                if ((((double)rand()) / RAND_MAX) > 0.5) {
                    //positive sign
                    coord[cind] = (HashEntry->coord[cind] + ROBARM_LONGACTIONDIST_CELLS) % EnvROBARMCfg.anglevals[cind];
                }
                else {
                    //negative sign
                    if (HashEntry->coord[cind] < ROBARM_LONGACTIONDIST_CELLS)
                        coord[cind] = HashEntry->coord[cind] + EnvROBARMCfg.anglevals[cind] -
                                      ROBARM_LONGACTIONDIST_CELLS;
                    else
                        coord[cind] = HashEntry->coord[cind] - ROBARM_LONGACTIONDIST_CELLS;
                    //if(coord[cind] < 0)
                    //{
                    //    SBPL_ERROR("ERROR: ROBARM_LONGACTIONDIST_CELLS is too large for dim %d\n", cind);
                    //    throw new SBPL_Exception();
                    //}
                }
            }
            else {
                //any value within ROBARM_LONGACTIONDIST_CELLS from the center
                int offset = (int)(ROBARM_LONGACTIONDIST_CELLS * (((double)rand()) / RAND_MAX));
                if ((((double)rand()) / RAND_MAX) > 0.5) offset = -offset;

                //we know the magnitude and just need to set sign
                if (offset >= 0) {
                    //positive sign
                    coord[cind] = (HashEntry->coord[cind] + offset) % EnvROBARMCfg.anglevals[cind];
                }
                else {
                    //negative sign
                    coord[cind] = (HashEntry->coord[cind] + offset);
                    if (HashEntry->coord[cind] < -offset) coord[cind] = HashEntry->coord[cind]
                        + EnvROBARMCfg.anglevals[cind] + offset;
                    //if(coord[cind] < 0)
                    //{
                    //    SBPL_ERROR("ERROR: ROBARM_LONGACTIONDIST_CELLS is too large for dim %d\n", cind);
                    //    throw new SBPL_Exception();
                    //}
                }
            }//else random offset
        }//over coordinates

        //skip the invalid sample
        if (!IsValidCoord(coord)) continue;

        //clock_t currenttime = clock();

        //compute end effector
        ComputeContAngles(coord, angles);
        ComputeEndEffectorPos(angles, &endeffx, &endeffy);

        //skip the ones whose end-effector is outside
        if (abs(HashEntry->endeffx - endeffx) > ROBARM_LONGACTIONDIST_CELLS || abs(HashEntry->endeffy - endeffy)
            > ROBARM_LONGACTIONDIST_CELLS) continue;

        bool bIsGoal = false;
        if (endeffx == EnvROBARMCfg.EndEffGoalX_c && endeffy == EnvROBARMCfg.EndEffGoalY_c) {
            bIsGoal = true;
            //SBPL_PRINTF("goal succ is generated\n");
        }

        EnvROBARMHashEntry_t* OutHashEntry = NULL;
        if ((OutHashEntry = GetHashEntry(coord, NUMOFLINKS, bIsGoal)) == NULL) {
            //have to create a new entry
            OutHashEntry = CreateNewHashEntry(coord, NUMOFLINKS, endeffx, endeffy);
        }

        //compute clow
        int clow;
        clow = EnvironmentROBARM::GetFromToHeuristic(HashEntry->stateID, OutHashEntry->stateID);

        SuccIDV->push_back(OutHashEntry->stateID);
        CLowV->push_back(clow);

        //time3_addallout += clock()-currenttime;
    }

    //see if the goal belongs to the inside area and if yes then add it to SUCCS as well
    if (abs(EnvROBARMCfg.EndEffGoalX_c - HashEntry->endeffx) <= ROBARM_LONGACTIONDIST_CELLS &&
        abs(EnvROBARMCfg.EndEffGoalY_c - HashEntry->endeffy) <= ROBARM_LONGACTIONDIST_CELLS)
    {
        EnvROBARMHashEntry_t* OutHashEntry = EnvROBARM.goalHashEntry;

        int clow = EnvironmentROBARM::GetFromToHeuristic(HashEntry->stateID, OutHashEntry->stateID);

        SuccIDV->push_back(OutHashEntry->stateID);
        CLowV->push_back(clow);
    }
}

int EnvironmentROBARM::GetEdgeCost(int FromStateID, int ToStateID)
{
#if DEBUG
    if(FromStateID >= (int)EnvROBARM.StateID2CoordTable.size() ||
       ToStateID >= (int)EnvROBARM.StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in EnvROBARM... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    //get X, Y for the state
    EnvROBARMHashEntry_t* FromHashEntry = EnvROBARM.StateID2CoordTable[FromStateID];
    EnvROBARMHashEntry_t* ToHashEntry = EnvROBARM.StateID2CoordTable[ToStateID];

    return cost(FromHashEntry->coord, ToHashEntry->coord);
}

int EnvironmentROBARM::GetRandomState()
{
    short unsigned int coord[NUMOFLINKS];
    double angles[NUMOFLINKS];
    short unsigned int endeffx, endeffy;
    EnvROBARMHashEntry_t* HashEntry = NULL;

    SBPL_PRINTF("picking a random state...\n");
    while (1) {
        //iterate over links
        for (int i = 0; i < NUMOFLINKS; i++) {
            //give it a shot
            coord[i] = (short unsigned int)(EnvROBARMCfg.anglevals[i] * (((double)rand()) / (((double)RAND_MAX) + 1)));
        }

        /*
        //iterate over links
        double endx[NUMOFLINKS];
        double endy[NUMOFLINKS];
        double x0, y0, x1, y1;

        for(int i = 0; i < NUMOFLINKS; i++)
        {
            int j = 0;

            //set the start of the link
            if(i == 0)
                Cell2ContXY(EnvROBARMCfg.BaseX_c, EnvROBARMCfg.EnvHeight_c-1,&x0,&y0);
            else
            {
                x0 = endx[i-1];
                y0 = endy[i-1];
            }

            //try to set the angle
            int numoftrials = 100;
            for(j = 0; j < numoftrials; j++)
            {
                //give it a shot
                coord[i] = EnvROBARMCfg.anglevals[i]*(((double)rand())/(RAND_MAX+1));

                //check the new link
                ComputeContAngles(coord, angles);
                x1 = x0 + EnvROBARMCfg.LinkLength_m[i]*cos(angles[i]);
                y1 = y0 - EnvROBARMCfg.LinkLength_m[i]*sin(angles[i]);

                //check the validity of the corresponding line segment
                if(IsValidLineSegment(x0,y0,x1,y1, EnvROBARMCfg.Grid2D, NULL))
                {
                    break;
                }
            }

            //see if we found it
            if(j == numoftrials)
            {
                //failed, reset
                SBPL_PRINTF("have to reset\n");
                break;
            }
            else
            {
                endx[i] = x1;
                endy[i] = y1;
            }
        }
        */

        //if not valid then try something else
        if (!IsValidCoord(coord)) {
            //SBPL_ERROR("ERROR: we could not have gotten an invalid sample\n");
            continue;
        }
        else
            //done - found
            break;
    }
    SBPL_PRINTF("done\n");

    //compute end effector
    ComputeContAngles(coord, angles);
    ComputeEndEffectorPos(angles, &endeffx, &endeffy);
    bool bIsGoal = false;
    if (endeffx == EnvROBARMCfg.EndEffGoalX_c && endeffy == EnvROBARMCfg.EndEffGoalY_c) {
        bIsGoal = true;
        //SBPL_PRINTF("goal succ is generated\n");
    }

    if ((HashEntry = GetHashEntry(coord, NUMOFLINKS, bIsGoal)) == NULL) {
        //have to create a new entry
        HashEntry = CreateNewHashEntry(coord, NUMOFLINKS, endeffx, endeffy);
    }

    return HashEntry->stateID;
}

bool EnvironmentROBARM::AreEquivalent(int State1ID, int State2ID)
{
    EnvROBARMHashEntry_t* HashEntry1 = EnvROBARM.StateID2CoordTable[State1ID];
    EnvROBARMHashEntry_t* HashEntry2 = EnvROBARM.StateID2CoordTable[State2ID];

    return (HashEntry1->endeffx == HashEntry2->endeffx && HashEntry1->endeffy == HashEntry2->endeffy);
}

//------------------------------------------------------------------------------
