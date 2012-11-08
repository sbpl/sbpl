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

#include <sbpl/discrete_space_information/environment_XXX.h>
#include <sbpl/planners/planner.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/mdpconfig.h>

using namespace std;

//extern clock_t time3_addallout;
//extern clock_t time_gethash;
//extern clock_t time_createhash;

//function prototypes

//-------------------problem specific and local functions---------------------

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
unsigned int EnvironmentXXX::GETHASHBIN(unsigned int X1, unsigned int X2, unsigned int X3, unsigned int X4)
{
    return inthash((inthash(X1) + (inthash(X2) << 1) + (inthash(X3) << 2) + (inthash(X4) << 3))) &
           (EnvXXX.HashTableSize - 1);
}

void EnvironmentXXX::PrintHashTableHist()
{
    int s0 = 0, s1 = 0, s50 = 0, s100 = 0, s200 = 0, s300 = 0, slarge = 0;

    for (int j = 0; j < (int)EnvXXX.HashTableSize; j++) {
        if ((int)EnvXXX.Coord2StateIDHashTable[j].size() == 0)
            s0++;
        else if ((int)EnvXXX.Coord2StateIDHashTable[j].size() < 50)
            s1++;
        else if ((int)EnvXXX.Coord2StateIDHashTable[j].size() < 100)
            s50++;
        else if ((int)EnvXXX.Coord2StateIDHashTable[j].size() < 200)
            s100++;
        else if ((int)EnvXXX.Coord2StateIDHashTable[j].size() < 300)
            s200++;
        else if ((int)EnvXXX.Coord2StateIDHashTable[j].size() < 400)
            s300++;
        else
            slarge++;
    }
    SBPL_PRINTF("hash table histogram: 0:%d, <50:%d, <100:%d, <200:%d, <300:%d, <400:%d >400:%d\n", s0, s1, s50, s100,
                s200, s300, slarge);
}

void EnvironmentXXX::ReadConfiguration(FILE* fCfg)
{
    //read in the configuration of environment and initialize  EnvCfg structure
}

void EnvironmentXXX::InitializeEnvConfig()
{
    //aditional to configuration file initialization of EnvCfg if necessary
}

EnvXXXHashEntry_t* EnvironmentXXX::GetHashEntry(unsigned int X1, unsigned int X2, unsigned int X3, unsigned int X4)
{
    //clock_t currenttime = clock();

    int binid = GETHASHBIN(X1, X2, X3, X4);

#if DEBUG
    if ((int)EnvXXX.Coord2StateIDHashTable[binid].size() > 500)
    {
        SBPL_PRINTF("WARNING: Hash table has a bin %d (X1=%d X2=%d X3=%d X4=%d) of size %d\n",
                    binid, X1, X2, X3, X4, (int)EnvXXX.Coord2StateIDHashTable[binid].size());

        PrintHashTableHist();
    }
#endif

    //iterate over the states in the bin and select the perfect match
    for (int ind = 0; ind < (int)EnvXXX.Coord2StateIDHashTable[binid].size(); ind++) {
        if (EnvXXX.Coord2StateIDHashTable[binid][ind]->X1 == X1 &&
            EnvXXX.Coord2StateIDHashTable[binid][ind]->X2 == X2 &&
            EnvXXX.Coord2StateIDHashTable[binid][ind]->X3 == X3 &&
            EnvXXX.Coord2StateIDHashTable[binid][ind]->X4 == X4)
        {
            //time_gethash += clock()-currenttime;
            return EnvXXX.Coord2StateIDHashTable[binid][ind];
        }
    }

    //time_gethash += clock()-currenttime;

    return NULL;
}

EnvXXXHashEntry_t* EnvironmentXXX::CreateNewHashEntry(unsigned int X1, unsigned int X2, unsigned int X3,
                                                      unsigned int X4)
{
    int i;

    //clock_t currenttime = clock();

    EnvXXXHashEntry_t* HashEntry = new EnvXXXHashEntry_t;

    HashEntry->X1 = X1;
    HashEntry->X2 = X2;
    HashEntry->X3 = X3;
    HashEntry->X4 = X4;

    HashEntry->stateID = EnvXXX.StateID2CoordTable.size();

    //insert into the tables
    EnvXXX.StateID2CoordTable.push_back(HashEntry);

    //get the hash table bin
    i = GETHASHBIN(HashEntry->X1, HashEntry->X2, HashEntry->X3, HashEntry->X4);

    //insert the entry into the bin
    EnvXXX.Coord2StateIDHashTable[i].push_back(HashEntry);

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

void EnvironmentXXX::CreateStartandGoalStates()
{
    EnvXXXHashEntry_t* HashEntry;

    //create start state
    unsigned int X1 = 0;
    unsigned int X2 = 0;
    unsigned int X3 = 0;
    unsigned int X4 = 0;
    HashEntry = CreateNewHashEntry(X1, X2, X3, X4);
    EnvXXX.startstateid = HashEntry->stateID;

    //create goal state
    X1 = X2 = X3 = X4 = 1;
    HashEntry = CreateNewHashEntry(X1, X2, X3, X4);
    EnvXXX.goalstateid = HashEntry->stateID;
}

void EnvironmentXXX::InitializeEnvironment()
{

    //initialize the map from Coord to StateID
    EnvXXX.HashTableSize = 32 * 1024; //should be power of two
    EnvXXX.Coord2StateIDHashTable = new vector<EnvXXXHashEntry_t*> [EnvXXX.HashTableSize];

    //initialize the map from StateID to Coord
    EnvXXX.StateID2CoordTable.clear();

    //create start and goal states
    CreateStartandGoalStates();
}

void EnvironmentXXX::AddAllOutcomes(unsigned int SourceX1, unsigned int SourceX2, unsigned int SourceX3,
                                    unsigned int SourceX4, CMDPACTION* action, int cost)
{
    EnvXXXHashEntry_t* OutHashEntry;
    int i;
    float CumProb = 0.0;

    //iterate over outcomes
    for (i = 0; i < 2; i++) {
        unsigned int newX1 = SourceX1 + i;
        unsigned int newX2 = SourceX2 + i;
        unsigned int newX3 = SourceX3 + i;
        unsigned int newX4 = SourceX4 + i;

        //add the outcome
        if ((OutHashEntry = GetHashEntry(newX1, newX2, newX3, newX4)) == NULL) {
            //have to create a new entry
            OutHashEntry = CreateNewHashEntry(newX1, newX2, newX3, newX4);
        }
        float Prob = 0.5; //probability of the outcome
        action->AddOutcome(OutHashEntry->stateID, cost, Prob);
        CumProb += Prob;

    } //while

    if (CumProb != 1.0) {
        SBPL_ERROR("ERROR in EnvXXX... function: prob. of all action outcomes=%f\n", CumProb);
        throw new SBPL_Exception();
    }
}

//------------------------------------------------------------------------------

//------------------------------Heuristic computation--------------------------

void EnvironmentXXX::ComputeHeuristicValues()
{
    //whatever necessary pre-computation of heuristic values is done here
    SBPL_PRINTF("Precomputing heuristics\n");

    SBPL_PRINTF("done\n");
}

//-----------interface with outside functions-----------------------------------

bool EnvironmentXXX::InitializeEnv(const char* sEnvFile)
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
    InitializeEnvironment();

    //pre-compute heuristics
    ComputeHeuristicValues();

    return true;
}

bool EnvironmentXXX::InitializeMDPCfg(MDPConfig *MDPCfg)
{
    //initialize MDPCfg with the start and goal ids
    MDPCfg->goalstateid = EnvXXX.goalstateid;
    MDPCfg->startstateid = EnvXXX.startstateid;

    return true;
}

int EnvironmentXXX::GetFromToHeuristic(int FromStateID, int ToStateID)
{
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if(FromStateID >= (int)EnvXXX.StateID2CoordTable.size() ||
       ToStateID >= (int)EnvXXX.StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in EnvXXX... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    //define this function if it is used in the planner

    SBPL_ERROR("ERROR in EnvXXX.. function: FromToHeuristic is undefined\n");
    throw new SBPL_Exception();

    return 0;
}

int EnvironmentXXX::GetGoalHeuristic(int stateID)
{
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if (stateID >= (int)EnvXXX.StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvXXX... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    //define this function if it used in the planner (heuristic forward search would use it)

    SBPL_ERROR("ERROR in EnvXXX..function: GetGoalHeuristic is undefined\n");
    throw new SBPL_Exception();
}

int EnvironmentXXX::GetStartHeuristic(int stateID)
{
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if (stateID >= (int)EnvXXX.StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvXXX... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    //define this function if it used in the planner (heuristic backward search would use it)

    SBPL_ERROR("ERROR in EnvXXX.. function: GetStartHeuristic is undefined\n");
    throw new SBPL_Exception();

    return 0;
}

void EnvironmentXXX::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{

#if DEBUG
    if (state->StateID >= (int)EnvXXX.StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvXXX... function: stateID illegal\n");
        throw new SBPL_Exception();
    }

    if ((int)state->Actions.size() != 0) {
        SBPL_ERROR("ERROR in Env_setAllActionsandAllOutcomes: actions already exist for the state\n");
        throw new SBPL_Exception();
    }
#endif

    //if it is goal then no successors
    if (state->StateID == EnvXXX.goalstateid) return;

    //get values for the state
    EnvXXXHashEntry_t* HashEntry = EnvXXX.StateID2CoordTable[state->StateID];

    //iterate through the actions for the state
    for (int aind = 0; aind < XXX_MAXACTIONSWIDTH; aind++) {
        int cost = 1;

        //Add Action
        CMDPACTION* action = state->AddAction(aind);

        //clock_t currenttime = clock();
        //add all the outcomes to the action
        AddAllOutcomes(HashEntry->X1, HashEntry->X2, HashEntry->X3, HashEntry->X4, action, cost);

        //you can break if the number of actual actions is smaller than the maximum possible

        //time3_addallout += clock()-currenttime;
    }
}

void EnvironmentXXX::SetAllPreds(CMDPSTATE* state)
{
    //implement this if the planner needs access to predecessors

    SBPL_ERROR("ERROR in EnvXXX... function: SetAllPreds is undefined\n");
    throw new SBPL_Exception();
}

void EnvironmentXXX::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
    SBPL_ERROR("ERROR in EnvXXX... function: GetSuccs is undefined\n");
    throw new SBPL_Exception();
}

void EnvironmentXXX::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{
    SBPL_ERROR("ERROR in EnvXXX... function: GetPreds is undefined\n");
    throw new SBPL_Exception();
}

int EnvironmentXXX::SizeofCreatedEnv()
{
    return (int)EnvXXX.StateID2CoordTable.size();
}

void EnvironmentXXX::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG
    if(stateID >= (int)EnvXXX.StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in EnvXXX... function: stateID illegal (2)\n");
        throw new SBPL_Exception();
    }
#endif

    if (fOut == NULL) fOut = stdout;

    EnvXXXHashEntry_t* HashEntry = EnvXXX.StateID2CoordTable[stateID];

    if (stateID == EnvXXX.goalstateid) {
        SBPL_FPRINTF(fOut, "the state is a goal state\n");
    }

    SBPL_FPRINTF(fOut, "X1=%d X2=%d X3=%d X4=%d\n", HashEntry->X1, HashEntry->X2, HashEntry->X3, HashEntry->X4);
}

void EnvironmentXXX::PrintEnv_Config(FILE* fOut)
{
    //implement this if the planner needs to print out EnvXXX. configuration

    SBPL_ERROR("ERROR in EnvXXX... function: PrintEnv_Config is undefined\n");
    throw new SBPL_Exception();
}

//------------------------------------------------------------------------------
