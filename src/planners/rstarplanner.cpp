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
#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/planners/rstarplanner.h>
#include <sbpl/utils/list.h>

using namespace std;

//TODO - define RSTAR_DEBUG_PRINTLOWLEVELEXP and RSTAR_DEBUG_PRINTHIGHLEVELEXP. Make them dependent on DEBUG
//use them to print expands into two separate file.

//-----------------------------------------------------------------------------------------------------

RSTARPlanner::RSTARPlanner(DiscreteSpaceInformation* environment, bool bSearchForward)
{
    bforwardsearch = bSearchForward;

    environment_ = environment;

    bsearchuntilfirstsolution = false;
    finitial_eps = RSTAR_DEFAULT_INITIAL_EPS;
    dec_eps = RSTAR_DECREASE_EPS;
    final_epsilon = RSTAR_FINAL_EPS;
    local_expand_thres = RSTAR_EXPTHRESH;
    highlevel_searchexpands = 0;
    lowlevel_searchexpands = 0;
    MaxMemoryCounter = 0;

#ifndef ROS
    const char* debug = "debug.txt";
#endif
    fDeb = SBPL_FOPEN(debug, "w");
    if (fDeb == NULL) {
        SBPL_ERROR("ERROR: could not open planner debug file\n");
        throw new SBPL_Exception();
    }
    SBPL_PRINTF("debug on\n");

    //create global searchstatespace
    pSearchStateSpace = new RSTARSearchStateSpace_t;
    MaxMemoryCounter += sizeof(RSTARSearchStateSpace_t);

    //create local searchstatespace
    pLSearchStateSpace = new RSTARLSearchStateSpace_t;
    MaxMemoryCounter += sizeof(RSTARLSearchStateSpace_t);

    //create the RSTAR planner
    if (CreateSearchStateSpace() != 1) {
        SBPL_ERROR("ERROR: failed to create statespace\n");
        return;
    }

    //set the start and goal states
    if (InitializeSearchStateSpace() != 1) {
        SBPL_ERROR("ERROR: failed to create statespace\n");
        return;
    }
}

RSTARPlanner::~RSTARPlanner()
{
    if (pSearchStateSpace != NULL) {
        //delete the statespace
        DeleteSearchStateSpace();
        delete pSearchStateSpace;
    }
    SBPL_FCLOSE( fDeb);
}

void RSTARPlanner::Initialize_searchinfo(CMDPSTATE* state)
{
    RSTARState* searchstateinfo = (RSTARState*)state->PlannerSpecificData;

    searchstateinfo->MDPstate = state;
    InitializeSearchStateInfo(searchstateinfo);
}

CMDPSTATE* RSTARPlanner::CreateState(int stateID)
{
    CMDPSTATE* state = NULL;

#if DEBUG
    if(environment_->StateID2IndexMapping[stateID][RSTARMDP_STATEID2IND] != -1)
    {
        SBPL_ERROR("ERROR in CreateState: state already created\n");
        throw new SBPL_Exception();
    }
#endif

    //adds to the tail a state
    state = pSearchStateSpace->searchMDP.AddState(stateID);

    //remember the index of the state
    environment_->StateID2IndexMapping[stateID][RSTARMDP_STATEID2IND] =
            pSearchStateSpace->searchMDP.StateArray.size() - 1;

#if DEBUG
    if(state !=
       pSearchStateSpace->searchMDP.StateArray[environment_->StateID2IndexMapping[stateID][RSTARMDP_STATEID2IND]])
    {
        SBPL_ERROR("ERROR in CreateState: invalid state index\n");
        throw new SBPL_Exception();
    }
#endif

    //create search specific info
    state->PlannerSpecificData = new RSTARState;
    MaxMemoryCounter += sizeof(RSTARState);
    Initialize_searchinfo(state);

    return state;
}

CMDPSTATE* RSTARPlanner::GetState(int stateID)
{
    if (stateID >= (int)environment_->StateID2IndexMapping.size()) {
        SBPL_ERROR("ERROR int GetState: stateID %d is invalid\n", stateID);
        throw new SBPL_Exception();
    }

    if (environment_->StateID2IndexMapping[stateID][RSTARMDP_STATEID2IND] == -1)
        return CreateState(stateID);
    else
        return pSearchStateSpace->searchMDP.StateArray[environment_->StateID2IndexMapping[stateID][RSTARMDP_STATEID2IND]];
}

//-----------------------------------------------------------------------------------------------------

//----------------------------------------functions related to local searches-------------------------------------------

void RSTARPlanner::Initialize_rstarlsearchdata(CMDPSTATE* state)
{
    RSTARLSearchState* rstarlsearch_data = (RSTARLSearchState*)state->PlannerSpecificData;

    rstarlsearch_data->bestpredstate = NULL;
    rstarlsearch_data->bestpredstateactioncost = 0;
    rstarlsearch_data->iteration = 0;
    rstarlsearch_data->iterationclosed = 0;
    rstarlsearch_data->g = INFINITECOST;
    rstarlsearch_data->heapindex = 0;
    rstarlsearch_data->listelem[0] = NULL;
    rstarlsearch_data->listelem[1] = NULL;

    //pointer to itself
    rstarlsearch_data->MDPstate = state;
}

CMDPSTATE* RSTARPlanner::CreateLSearchState(int stateID)
{
    CMDPSTATE* state = NULL;

#if DEBUG
    if (environment_->StateID2IndexMapping[stateID][RSTARMDP_LSEARCH_STATEID2IND] != -1) {
        SBPL_ERROR("ERROR in CreateState: state already created\n");
        throw new SBPL_Exception();
    }
#endif

    //adds to the tail a state
    state = pLSearchStateSpace->MDP.AddState(stateID);

    //remember the index of the state
    environment_->StateID2IndexMapping[stateID][RSTARMDP_LSEARCH_STATEID2IND]
        = pLSearchStateSpace->MDP.StateArray.size() - 1;

#if DEBUG
    if(state !=
       pLSearchStateSpace->MDP.StateArray[environment_->StateID2IndexMapping[stateID][RSTARMDP_LSEARCH_STATEID2IND]])
    {
        SBPL_ERROR("ERROR in CreateState: invalid state index\n");
        throw new SBPL_Exception();
    }
#endif

    //create and initialize rstarlsearch_data
    state->PlannerSpecificData = new RSTARLSearchState;
    Initialize_rstarlsearchdata(state);

    return state;
}

CMDPSTATE* RSTARPlanner::GetLSearchState(int stateID)
{
    if (stateID >= (int)environment_->StateID2IndexMapping.size()) {
        SBPL_ERROR("ERROR int GetLSearchState: stateID is invalid\n");
        throw new SBPL_Exception();
    }

    if (environment_->StateID2IndexMapping[stateID][RSTARMDP_LSEARCH_STATEID2IND] == -1)
        return CreateLSearchState(stateID);
    else
        return pLSearchStateSpace->MDP.StateArray[environment_->StateID2IndexMapping[stateID][RSTARMDP_LSEARCH_STATEID2IND]];
}

CKey RSTARPlanner::LocalSearchComputeKey(RSTARLSearchState* rstarlsearchState)
{
    CKey retkey;

    int h;
    if (bforwardsearch)
        h = environment_->GetFromToHeuristic(rstarlsearchState->MDPstate->StateID,
                                             pLSearchStateSpace->GoalState->StateID);
    else
        h = environment_->GetFromToHeuristic(pLSearchStateSpace->GoalState->StateID,
                                             rstarlsearchState->MDPstate->StateID);

    retkey.key[0] = rstarlsearchState->g + (int)(pSearchStateSpace->eps * h);

    return retkey;
}

bool RSTARPlanner::ComputeLocalPath(int StartStateID, int GoalStateID, int maxc, int maxe, int *pCost, int *pCostLow,
                                    int *pExp, vector<int>* pPathIDs, int* pNewGoalStateID, double maxnumofsecs)
{
    vector<int> SuccIDV;
    vector<int> CostV;

    if (pLSearchStateSpace->OPEN == NULL) pLSearchStateSpace->OPEN = new CHeap;
    if (pLSearchStateSpace->INCONS == NULL) pLSearchStateSpace->INCONS = new CList;

    int local_expands = 0;
    *pNewGoalStateID = GoalStateID;

    //increase iteration
    pLSearchStateSpace->iteration++;

    //set the start and goal states
    pLSearchStateSpace->StartState = GetLSearchState(StartStateID);
    pLSearchStateSpace->GoalState = GetLSearchState(GoalStateID);

    RSTARLSearchState* rstarlsearchstate = (RSTARLSearchState*)pLSearchStateSpace->StartState->PlannerSpecificData;
    RSTARLSearchState* rstarlsearchgoalstate = (RSTARLSearchState*)pLSearchStateSpace->GoalState->PlannerSpecificData;

    //OPEN=0 (it shouldn't be necessary since the memory is cleared before each search)
    pLSearchStateSpace->OPEN->makeemptyheap();
    pLSearchStateSpace->INCONS->makeemptylist(RSTAR_INCONS_LIST_ID);

    //CLOSED = 0 because iteration is increased

    //g(start) = 0
    rstarlsearchstate->g = 0;

    //insert start into open
    pLSearchStateSpace->OPEN->insertheap(rstarlsearchstate, LocalSearchComputeKey(rstarlsearchstate));

    //TODO - prove that min_{OPEN and INCONS} (g + eps*h) <= eps*c*. (proof: take min_{OPEN and INCONS} (g+h) <= c* and
    //multiply both sides by eps and then bring eps into min and drop one by
    //g. I still need to implement INCONS and minimum tracker for it - then
    //change the minkey to min over two sets
    while (rstarlsearchgoalstate->g > pLSearchStateSpace->OPEN->getminkeyheap().key[0] && local_expands < maxe &&
           pLSearchStateSpace->OPEN->getminkeyheap().key[0] <= maxc)
    {
        //pop the min element
        rstarlsearchstate = (RSTARLSearchState*)pLSearchStateSpace->OPEN->deleteminheap();

        //close the state
        rstarlsearchstate->iterationclosed = pLSearchStateSpace->iteration;

        //expansion
        local_expands++;
        //environment_->PrintState(rstarlsearchstate->MDPstate->StateID, false, fLowLevelExp);

        //generate SUCCS state
        SuccIDV.clear();
        CostV.clear();
        //this setting makes it to get all successors - since this is a deterministic search
        if (bforwardsearch == false)
            environment_->GetPreds(rstarlsearchstate->MDPstate->StateID, &SuccIDV, &CostV);
        else
            environment_->GetSuccs(rstarlsearchstate->MDPstate->StateID, &SuccIDV, &CostV);

        //iterate over states in SUCCS set
        for (int i = 0; i < (int)SuccIDV.size(); i++) {
            RSTARLSearchState* rstarlsearchSuccState =
                (RSTARLSearchState*)GetLSearchState(SuccIDV.at(i))->PlannerSpecificData;

            //skip if the state is already closed - TODO fix this with INCONS
            //list - it seems to make five times less expansions!
            //if(rstarlsearchSuccState->iterationclosed == pLSearchStateSpace->iteration)
            //continue;

            //see if we can improve g-value of successor
            if (rstarlsearchstate->g + CostV[i] < rstarlsearchSuccState->g) {
                rstarlsearchSuccState->bestpredstate = rstarlsearchstate->MDPstate;
                rstarlsearchSuccState->bestpredstateactioncost = CostV[i];
                rstarlsearchSuccState->g = rstarlsearchstate->g + CostV[i];
                if (rstarlsearchSuccState->heapindex == 0)
                    pLSearchStateSpace->OPEN->insertheap(rstarlsearchSuccState,
                                                         LocalSearchComputeKey(rstarlsearchSuccState));
                else
                    pLSearchStateSpace->OPEN->updateheap(rstarlsearchSuccState,
                                                         LocalSearchComputeKey(rstarlsearchSuccState));

                int goalStateID = pSearchStateSpace->searchgoalstate->StateID;
                if (!environment_->AreEquivalent(rstarlsearchSuccState->MDPstate->StateID, goalStateID) &&
                    environment_->AreEquivalent(rstarlsearchSuccState->MDPstate->StateID,
                                                rstarlsearchgoalstate->MDPstate->StateID) &&
                    rstarlsearchSuccState->g < rstarlsearchgoalstate->g)
                {
                    //swap the goal
                    rstarlsearchgoalstate = rstarlsearchSuccState;
                    GoalStateID = rstarlsearchgoalstate->MDPstate->StateID;
                }
            }
        }

        if (local_expands % 10000 == 0) {
            if ((clock() - TimeStarted) >= maxnumofsecs * (double)CLOCKS_PER_SEC) {
                SBPL_PRINTF("breaking local search because global planning time expires\n");
                break;
            }
        }
    }//while

    lowlevel_searchexpands += local_expands;
    SBPL_FPRINTF(fDeb, "local search: expands=%d\n", local_expands);

    //set the return path and other path related variables
    vector<int> tempPathID;
    pPathIDs->clear();
    if (rstarlsearchgoalstate->g < INFINITECOST) {
        int pathcost = 0;

        //path exists
        rstarlsearchstate = rstarlsearchgoalstate;
        while (rstarlsearchstate->bestpredstate != NULL && rstarlsearchstate->MDPstate
            != pLSearchStateSpace->StartState) {
            tempPathID.push_back(rstarlsearchstate->MDPstate->StateID);
            pathcost += rstarlsearchstate->bestpredstateactioncost;
            rstarlsearchstate = (RSTARLSearchState*)rstarlsearchstate->bestpredstate->PlannerSpecificData;
        }
        //store into pPathIDs so that the path is always forward path w.r.t. the original graph 
        //this requires us to reverse order in case of forward search
        for (int i = 0; i < (int)tempPathID.size(); i++) {
            if (bforwardsearch == false)
                pPathIDs->push_back(tempPathID.at(i));
            else
                pPathIDs->push_back(tempPathID.at(tempPathID.size() - i - 1));
        }
        *pCost = pathcost;
        *pCostLow = rstarlsearchgoalstate->g;
        SBPL_FPRINTF(fDeb, "pathcost=%d while g=%d\n", pathcost, rstarlsearchgoalstate->g);
    }
    else {
        *pCost = INFINITECOST;
        *pCostLow = pLSearchStateSpace->OPEN->getminkeyheap().key[0];
        if (*pCostLow != pLSearchStateSpace->OPEN->getminkeyheap().key[0]) {
            SBPL_FPRINTF(fDeb, "after localsearch clow=%d while keymin=%d\n", (int)(*pCostLow),
                         (int)pLSearchStateSpace->OPEN->getminkeyheap().key[0]);
        }
    }

    //set all other return variables
    *pExp = local_expands;
    *pNewGoalStateID = GoalStateID;

    return true;
}

bool RSTARPlanner::DestroyLocalSearchMemory()
{
    pLSearchStateSpace->OPEN->currentsize = 0;
    pLSearchStateSpace->StartState = pLSearchStateSpace->GoalState = NULL;

    //remove the states in the MDP itself
    for (int i = 0; i < (int)pLSearchStateSpace->MDP.StateArray.size(); i++) {
        CMDPSTATE* state = pLSearchStateSpace->MDP.StateArray.at(i);
        RSTARLSearchState* rstarlsearchstatedata = (RSTARLSearchState*)state->PlannerSpecificData;
        delete rstarlsearchstatedata;
        state->PlannerSpecificData = NULL;
        environment_->StateID2IndexMapping[state->StateID][RSTARMDP_LSEARCH_STATEID2IND] = -1;
    }
    //now we can delete the states themselves
    if (pLSearchStateSpace->MDP.Delete() == false) {
        SBPL_ERROR("ERROR: failed to delete local search MDP\n");
        throw new SBPL_Exception();
    }

    //TODO - ask Env to delete the new memory allocated during the local
    //search (usings the fact that stateID's for local search continuously and
    //no states were allocated for global search before local search exits.

    return true;
}

//----------------------------------------------------------------------------------------------------------------------

int RSTARPlanner::ComputeHeuristic(CMDPSTATE* MDPstate)
{
    //compute heuristic for search

    //the heuristics will be re-computed anyway when searchgoalstate is updated
    if (pSearchStateSpace->searchgoalstate == NULL) return 0; 

    if (bforwardsearch) {
#if MEM_CHECK == 1
        //int WasEn = DisableMemCheck();
#endif

        //forward search: heur = distance from state to searchgoal which is Goal RSTARState
        int retv = environment_->GetFromToHeuristic(MDPstate->StateID, pSearchStateSpace->searchgoalstate->StateID);

#if MEM_CHECK == 1
        //if (WasEn)
        //	EnableMemCheck();
#endif

        return retv;
    }
    else {
        //backward search: heur = distance from searchgoal to state
        return environment_->GetFromToHeuristic(pSearchStateSpace->searchgoalstate->StateID, MDPstate->StateID);
    }
}

//initialization of a state
void RSTARPlanner::InitializeSearchStateInfo(RSTARState* state)
{
    state->g = INFINITECOST;
    state->iterationclosed = 0;
    state->callnumberaccessed = pSearchStateSpace->callnumber;
    state->heapindex = 0;
    state->bestpredaction = NULL;

    //compute heuristics
#if USE_HEUR
    if(pSearchStateSpace->searchgoalstate != NULL)
    state->h = ComputeHeuristic(state->MDPstate);
    else
    state->h = 0;
#else
    state->h = 0;
#endif

    state->predactionV.clear();
}

//re-initialization of a state
void RSTARPlanner::ReInitializeSearchStateInfo(RSTARState* state)
{
    state->g = INFINITECOST;
    state->iterationclosed = 0;
    state->callnumberaccessed = pSearchStateSpace->callnumber;
    state->heapindex = 0;

    state->bestpredaction = NULL;

    //compute heuristics
#if USE_HEUR

    if(pSearchStateSpace->searchgoalstate != NULL)
    {
        state->h = ComputeHeuristic(state->MDPstate);
    }
    else
    state->h = 0;

#else

    state->h = 0;

#endif

    state->predactionV.clear();
    for (int i = 0; i < (int)state->MDPstate->Actions.size(); i++) {
        if (state->MDPstate->Actions.at(i)->PlannerSpecificData != NULL) {
            DeleteSearchActionData((RSTARACTIONDATA*)state->MDPstate->Actions.at(i)->PlannerSpecificData);
            delete (RSTARACTIONDATA*)(state->MDPstate->Actions.at(i)->PlannerSpecificData);
            state->MDPstate->Actions.at(i)->PlannerSpecificData = NULL;
        }
    }
    state->MDPstate->RemoveAllActions();
}

void RSTARPlanner::DeleteSearchStateData(RSTARState* state)
{
    //delete PlannerSpecificData for each action
    state->predactionV.clear();
    for (int i = 0; i < (int)state->MDPstate->Actions.size(); i++) {
        if (state->MDPstate->Actions.at(i)->PlannerSpecificData != NULL) {
            DeleteSearchActionData((RSTARACTIONDATA*)state->MDPstate->Actions.at(i)->PlannerSpecificData);
            delete (RSTARACTIONDATA*)state->MDPstate->Actions.at(i)->PlannerSpecificData;
            state->MDPstate->Actions.at(i)->PlannerSpecificData = NULL;
        }
    }
    state->MDPstate->RemoveAllActions();

    return;
}

void RSTARPlanner::DeleteSearchActionData(RSTARACTIONDATA* actiondata)
{
    //no memory was allocated for actiondata

    return;
}

int RSTARPlanner::GetGVal(int StateID)
{
    CMDPSTATE* cmdp_state = GetState(StateID);
    RSTARState* state = (RSTARState*)cmdp_state->PlannerSpecificData;
    return state->g;
}

void RSTARPlanner::SetBestPredecessor(RSTARState* rstarState, RSTARState* rstarPredState, CMDPACTION* action)
{
    rstarState->bestpredaction = action;
    rstarState->g = rstarPredState->g + ((RSTARACTIONDATA*)(action->PlannerSpecificData))->clow;
    if (rstarState->heapindex == 0)
        pSearchStateSpace->OPEN->insertheap(rstarState, ComputeKey(rstarState));
    else
        pSearchStateSpace->OPEN->updateheap(rstarState, ComputeKey(rstarState));
}

CKey RSTARPlanner::ComputeKey(RSTARState* rstarState)
{
    CKey retkey;

    int h, starttostateh;
    if (bforwardsearch) {
        h = environment_->GetFromToHeuristic(rstarState->MDPstate->StateID,
                                             pSearchStateSpace->searchgoalstate->StateID);
        starttostateh = environment_->GetFromToHeuristic(pSearchStateSpace->searchstartstate->StateID,
                                                         rstarState->MDPstate->StateID);
    }
    else {
        h = environment_->GetFromToHeuristic(pSearchStateSpace->searchgoalstate->StateID,
                                             rstarState->MDPstate->StateID);
        starttostateh = environment_->GetFromToHeuristic(rstarState->MDPstate->StateID,
                                                         pSearchStateSpace->searchstartstate->StateID);
    }

    //compute 2nd element of the key
    retkey.key[1] = rstarState->g + (int)(pSearchStateSpace->eps * h);

    //compute the 1st element
    if (rstarState->g > pSearchStateSpace->eps * starttostateh ||
            (rstarState->bestpredaction != NULL &&
            ((RSTARACTIONDATA*)rstarState->bestpredaction->PlannerSpecificData)->pathIDs.size() == 0 &&
            ((RSTARACTIONDATA*)rstarState->bestpredaction->PlannerSpecificData)->exp >= local_expand_thres))
    {
        retkey.key[0] = 1;
    }
    else {
        retkey.key[0] = 0;
    }

    return retkey;
}

//returns 1 if the solution is found, 0 if the solution does not exist and 2 if it ran out of time
int RSTARPlanner::ImprovePath(double MaxNumofSecs)
{
    int expands;
    RSTARState *rstarstate, *searchgoalstate, *searchstartstate;
    CKey key, minkey;
    CKey goalkey;

    vector<int> SuccIDV;
    vector<int> CLowV;
    int highlevelexpands = 0;

    expands = 0;

    if (pSearchStateSpace->searchgoalstate == NULL) {
        SBPL_ERROR("ERROR searching: no goal state is set\n");
        throw new SBPL_Exception();
    }

    //goal state
    searchgoalstate = (RSTARState*)(pSearchStateSpace->searchgoalstate->PlannerSpecificData);
    if (searchgoalstate->callnumberaccessed != pSearchStateSpace->callnumber) {
        ReInitializeSearchStateInfo( searchgoalstate);
    }

    //get the start state
    searchstartstate = (RSTARState*)(pSearchStateSpace->searchstartstate->PlannerSpecificData);

    //set goal key
    if (searchgoalstate != searchstartstate) {
        goalkey.key[0] = 1;
        goalkey.key[1] = INFINITECOST;
    }
    else {
        goalkey = ComputeKey(searchstartstate);
    }

    //expand states until done
    minkey = pSearchStateSpace->OPEN->getminkeyheap();
    while (!pSearchStateSpace->OPEN->emptyheap() && (clock() - TimeStarted) < MaxNumofSecs * (double)CLOCKS_PER_SEC) {
        //recompute minkey
        minkey = pSearchStateSpace->OPEN->getminkeyheap();

        //recompute goalkey if necessary
        goalkey = ComputeKey(searchgoalstate);

        if (goalkey < minkey) break; //termination condition

        //pop the min element
        rstarstate = (RSTARState*)pSearchStateSpace->OPEN->deleteminheap();

        SBPL_FPRINTF(fDeb, "ComputePath:  selected state %d g=%d (AVOID=%d)\n", rstarstate->MDPstate->StateID,
                     rstarstate->g, (int)minkey.key[0]);
        environment_->PrintState(rstarstate->MDPstate->StateID, true, fDeb);

        if (rstarstate->MDPstate != pSearchStateSpace->searchstartstate &&
            ((RSTARACTIONDATA*)rstarstate->bestpredaction->PlannerSpecificData)->pathIDs.size() == 0)
        {
            SBPL_FPRINTF(fDeb, "re-compute path\n");

            int maxe = INFINITECOST;
            int maxc = INFINITECOST;

            //predecessor
            RSTARState* rstarpredstate =
                (RSTARState*)GetState(rstarstate->bestpredaction->SourceStateID)->PlannerSpecificData;
            CMDPACTION* computedaction = rstarstate->bestpredaction;
            RSTARACTIONDATA* computedactiondata = (RSTARACTIONDATA*)computedaction->PlannerSpecificData;

            if (computedactiondata->exp < local_expand_thres) {
                maxe = local_expand_thres;
            }
            else {
                SBPL_PRINTF("Trying to compute hard-to-find path\n");
                SBPL_FPRINTF(fDeb, "Trying to compute hard-to-find path\n");
                /* TODO
                CKey nextkey = rstarPlanner.OPEN->getminkeyheap();

                if(bforwardsearch)
                h = environment_->GetFromToHeuristic(rstarstate->MDPstate->StateID,
                                                     pSearchStateSpace->GoalState->StateID);
                else
                h = environment_->GetFromToHeuristic(pSearchStateSpace->GoalState->StateID,
                                                     rstarstate->MDPstate->StateID);

                maxc = nextkey[1] -
                (rstarpredstate->g + rstarPlanner.epsilon*h) + RSTAR_COSTDELTA;
                */
            }

            SBPL_FPRINTF(fDeb, "recomputing path from bp %d to state %d with maxc=%d maxe=%d\n",
                         rstarpredstate->MDPstate->StateID, rstarstate->MDPstate->StateID, maxc, maxe);
            SBPL_FPRINTF(fDeb, "bp state:\n");
            environment_->PrintState(rstarpredstate->MDPstate->StateID, true, fDeb);

            //re-compute the path
            int NewGoalStateID = rstarstate->MDPstate->StateID;
            ComputeLocalPath(rstarpredstate->MDPstate->StateID, rstarstate->MDPstate->StateID, maxc, maxe,
                             &computedaction->Costs[0], &computedactiondata->clow, &computedactiondata->exp,
                             &computedactiondata->pathIDs, &NewGoalStateID, MaxNumofSecs);

            SBPL_FPRINTF(fDeb, "return values: pathcost=%d clow=%d exp=%d\n", computedaction->Costs[0],
                         computedactiondata->clow, computedactiondata->exp);
            bool bSwitch = false;
            if (NewGoalStateID != rstarstate->MDPstate->StateID) {
                bSwitch = true;
                SBPL_FPRINTF(fDeb, "targetstate was switched from %d to %d\n", rstarstate->MDPstate->StateID,
                             NewGoalStateID);
                SBPL_FPRINTF(stdout, "targetstate was switched from %d to %d\n", rstarstate->MDPstate->StateID,
                             NewGoalStateID);
                environment_->PrintState(NewGoalStateID, true, fDeb);

                RSTARState* rstarNewTargetState = (RSTARState*)GetState(NewGoalStateID)->PlannerSpecificData;

                //re-initialize the state if necessary
                if (rstarNewTargetState->callnumberaccessed != pSearchStateSpace->callnumber) {
                    ReInitializeSearchStateInfo(rstarNewTargetState);
                }

                SBPL_FPRINTF(fDeb, "predstate.g=%d actual actioncost=%d clow=%d newtartetstate.g=%d\n",
                             rstarpredstate->g, computedaction->Costs[0],
                             ((RSTARACTIONDATA*)computedaction->PlannerSpecificData)->clow, rstarNewTargetState->g);

                //add the successor to our graph
                CMDPACTION* action = rstarpredstate->MDPstate->AddAction(rstarpredstate->MDPstate->Actions.size());
                action->AddOutcome(rstarNewTargetState->MDPstate->StateID, computedaction->Costs[0], 1.0);
                action->PlannerSpecificData = new RSTARACTIONDATA;
                MaxMemoryCounter += sizeof(RSTARACTIONDATA);
                ((RSTARACTIONDATA*)action->PlannerSpecificData)->clow = computedactiondata->clow;
                ((RSTARACTIONDATA*)action->PlannerSpecificData)->exp = computedactiondata->exp;
                ((RSTARACTIONDATA*)action->PlannerSpecificData)->pathIDs = computedactiondata->pathIDs;

                //add the corresponding predaction
                rstarNewTargetState->predactionV.push_back(action);

                //the action was not found to the old state
                computedaction->Costs[0] = INFINITECOST;
                if (bforwardsearch)
                    computedactiondata->clow = environment_->GetFromToHeuristic(rstarpredstate->MDPstate->StateID,
                                                                                rstarstate->MDPstate->StateID);
                else
                    computedactiondata->clow = environment_->GetFromToHeuristic(rstarstate->MDPstate->StateID,
                                                                                rstarpredstate->MDPstate->StateID);

                computedactiondata->pathIDs.clear();

                rstarstate = rstarNewTargetState;
                computedaction = action;
                computedactiondata = (RSTARACTIONDATA*)action->PlannerSpecificData;
            }

            //clean up local search memory
            DestroyLocalSearchMemory();

            RSTARState* stateu = NULL;
            CMDPACTION* utosaction = NULL;
            int hfromstarttostate;
            if (bforwardsearch)
                hfromstarttostate = environment_->GetFromToHeuristic(searchstartstate->MDPstate->StateID,
                                                                     rstarstate->MDPstate->StateID);
            else
                hfromstarttostate = environment_->GetFromToHeuristic(rstarstate->MDPstate->StateID,
                                                                     searchstartstate->MDPstate->StateID);
            if (computedactiondata->pathIDs.size() == 0 ||
                rstarpredstate->g + computedactiondata->clow > pSearchStateSpace->eps * hfromstarttostate)
            {
                SBPL_FPRINTF(fDeb, "selecting best pred\n");
                //SBPL_FPRINTF(stdout, "selecting best pred\n");

                //select other best predecessor
                unsigned int minQ = INFINITECOST;
                for (int i = 0; i < (int)rstarstate->predactionV.size(); i++) {
                    CMDPACTION* predaction = rstarstate->predactionV.at(i);
                    rstarpredstate = (RSTARState*)GetState(predaction->SourceStateID)->PlannerSpecificData;
                    if (minQ >= rstarpredstate->g + ((RSTARACTIONDATA*)predaction->PlannerSpecificData)->clow) {
                        minQ = rstarpredstate->g + ((RSTARACTIONDATA*)predaction->PlannerSpecificData)->clow;
                        stateu = rstarpredstate;
                        utosaction = predaction;
                    }
                }
                if (stateu) SBPL_FPRINTF(fDeb, "best pred stateid: %d\n", stateu->MDPstate->StateID);

                //set the predecessor
                if (minQ < INFINITECOST) SetBestPredecessor(rstarstate, stateu, utosaction);
            }
            else if (rstarpredstate->g + computedactiondata->clow < rstarstate->g || bSwitch == false) {
                SBPL_FPRINTF(fDeb, "keeping the same computedaction\n");
                //SBPL_FPRINTF(stdout, "keeping the same best pred\n");

                stateu = rstarpredstate;
                utosaction = computedaction;

                //set the predecessor
                SetBestPredecessor(rstarstate, stateu, utosaction);
            }
            else {
                SBPL_FPRINTF(fDeb, "keeping the same bestpredaction even though switch of targetstates happened\n");
            }
        }
        else {
            SBPL_FPRINTF(fDeb, "high-level normal expansion of state %d\n", rstarstate->MDPstate->StateID);
            environment_->PrintState(rstarstate->MDPstate->StateID, true, fDeb);
            highlevelexpands++;

            //close the state
            rstarstate->iterationclosed = pSearchStateSpace->searchiteration;

            //expansion
            expands++;

            //generate SUCCS state
            SuccIDV.clear();
            CLowV.clear();
            if (bforwardsearch)
                environment_->GetRandomSuccsatDistance(rstarstate->MDPstate->StateID, &SuccIDV, &CLowV);
            else
                environment_->GetRandomPredsatDistance(rstarstate->MDPstate->StateID, &SuccIDV, &CLowV);

            SBPL_FPRINTF(fDeb, "%d succs were generated at random\n", (unsigned int)SuccIDV.size());

            //iterate over states in SUCCS set
            int notclosed = 0;
            for (int i = 0; i < (int)SuccIDV.size(); i++) {
                RSTARState* rstarSuccState = (RSTARState*)GetState(SuccIDV.at(i))->PlannerSpecificData;

                SBPL_FPRINTF(fDeb, "succ %d:\n", i);
                environment_->PrintState(rstarSuccState->MDPstate->StateID, true, fDeb);

                //re-initialize the state if necessary
                if (rstarSuccState->callnumberaccessed != pSearchStateSpace->callnumber) {
                    ReInitializeSearchStateInfo(rstarSuccState);
                }

                //skip if the state is already closed
                if (rstarSuccState->iterationclosed == pSearchStateSpace->searchiteration) {
                    SBPL_FPRINTF(fDeb, "this succ was already closed -- skipping it\n");
                    continue;
                }
                notclosed++;

                //if(rstarSuccState->MDPstate->StateID == 3838){
                //    SBPL_FPRINTF(fDeb, "generating state %d with g=%d bp=%d:\n",
                //                 rstarSuccState->MDPstate->StateID, rstarSuccState->g, 
                //            (rstarSuccState->bestpredaction==NULL?-1:rstarSuccState->bestpredaction->SourceStateID));
                //    Env_PrintState(rstarSuccState->MDPstate->StateID, true, false, fDeb);
                //}

                //add the successor to our graph
                CMDPACTION* action = rstarstate->MDPstate->AddAction(i);
                action->AddOutcome(rstarSuccState->MDPstate->StateID, INFINITECOST, 1.0);
                action->PlannerSpecificData = new RSTARACTIONDATA;
                MaxMemoryCounter += sizeof(RSTARACTIONDATA);
                ((RSTARACTIONDATA*)action->PlannerSpecificData)->clow = CLowV[i];
                ((RSTARACTIONDATA*)action->PlannerSpecificData)->exp = 0;
                ((RSTARACTIONDATA*)action->PlannerSpecificData)->pathIDs.clear();

                //add the corresponding predaction
                rstarSuccState->predactionV.push_back(action);

                //see if we can improve g-value of successor
                if (rstarSuccState->bestpredaction == NULL || rstarstate->g + CLowV[i] < rstarSuccState->g)
                /*
                 (rstarstate->bestpredaction != NULL &&
                 ((RSTARACTIONDATA*)rstarstate->bestpredaction->PlannerSpecificData)->pathIDs.size() == 0 &&
                 ((RSTARACTIONDATA*)rstarstate->bestpredaction->PlannerSpecificData)->exp >= local_expand_thres) */
                {
                    SetBestPredecessor(rstarSuccState, rstarstate, action);
                    SBPL_FPRINTF(fDeb, "bestpred was set for the succ (clow=%d)\n", CLowV[i]);
                }
                else {
                    SBPL_FPRINTF(fDeb, "bestpred was NOT modified - old one is better\n");
                }

            }
            //SBPL_PRINTF("%d successors were not closed\n", notclosed);
        }//else

        //recompute minkey
        minkey = pSearchStateSpace->OPEN->getminkeyheap();

        //recompute goalkey if necessary
        goalkey = ComputeKey(searchgoalstate);

        if (goalkey.key[0] == 1) {
            SBPL_FPRINTF(fDeb, "goal state is AVOID\n");
        }
        else
            SBPL_FPRINTF(fDeb, "goal state is NON-AVOID\n");

        if (expands % 10 == 0 && expands > 0) {
            SBPL_PRINTF("high-level expands so far=%u\n", expands);
        }
    }
    SBPL_PRINTF("main loop done\n");
    SBPL_FPRINTF(fDeb, "main loop done\n");

    int retv = 1;
    if (searchgoalstate->g == INFINITECOST && pSearchStateSpace->OPEN->emptyheap()) {
        SBPL_PRINTF("solution does not exist: search exited because heap is empty\n");
        retv = 0;
    }
    else if (!pSearchStateSpace->OPEN->emptyheap() && goalkey > minkey) {
        SBPL_PRINTF("search exited because it ran out of time\n");
        retv = 2;
    }
    else if (searchgoalstate->g == INFINITECOST && !pSearchStateSpace->OPEN->emptyheap()) {
        SBPL_PRINTF( "solution does not exist: search exited because all candidates for expansion have "
                    "infinite heuristics\n");
        retv = 0;
    }
    else {
        SBPL_PRINTF("search exited with a solution for eps=%.3f\n", pSearchStateSpace->eps);
        retv = 1;
    }

    SBPL_FPRINTF(fDeb, "high-level expanded=%d\n", expands);

    highlevel_searchexpands += expands;

    return retv;
}

//note this does NOT re-compute heuristics, only re-orders OPEN list based on current eps and h-vals
void RSTARPlanner::Reevaluatefvals()
{
    CKey key;
    int i;
    CHeap* pheap = pSearchStateSpace->OPEN;

    //re-compute priorities for states in OPEN and reorder it
    for (i = 1; i <= pheap->currentsize; ++i) {
        RSTARState* state = (RSTARState*)pheap->heap[i].heapstate;
        pheap->heap[i].key = ComputeKey(state);
    }
    pheap->makeheap();

    pSearchStateSpace->bReevaluatefvals = false;
}

//creates (allocates memory) search state space
//does not initialize search statespace
int RSTARPlanner::CreateSearchStateSpace()
{

    //create a heap
    pSearchStateSpace->OPEN = new CHeap;
    MaxMemoryCounter += sizeof(CHeap);
    //pSearchStateSpace->inconslist = new CList;
    //MaxMemoryCounter += sizeof(CList);

    pSearchStateSpace->searchgoalstate = NULL;
    pSearchStateSpace->searchstartstate = NULL;

    pSearchStateSpace->bReinitializeSearchStateSpace = false;

    return 1;
}

//deallocates memory used by SearchStateSpace
void RSTARPlanner::DeleteSearchStateSpace()
{
    if (pSearchStateSpace->OPEN != NULL) {
        pSearchStateSpace->OPEN->makeemptyheap();
        delete pSearchStateSpace->OPEN;
        pSearchStateSpace->OPEN = NULL;
    }

    //if(pSearchStateSpace->inconslist != NULL)
    //{
    //	pSearchStateSpace->inconslist->makeemptylist(RSTAR_INCONS_LIST_ID);
    //	delete pSearchStateSpace->inconslist;
    //	pSearchStateSpace->inconslist = NULL;
    //}

    //delete the states themselves
    int iend = (int)pSearchStateSpace->searchMDP.StateArray.size();
    for (int i = 0; i < iend; i++) {
        CMDPSTATE* state = pSearchStateSpace->searchMDP.StateArray[i];
        if (state != NULL && state->PlannerSpecificData != NULL) {
            DeleteSearchStateData((RSTARState*)state->PlannerSpecificData);
            delete (RSTARState*)state->PlannerSpecificData;
            state->PlannerSpecificData = NULL;
        }
        if (state != NULL) {
            for (int aind = 0; aind < (int)state->Actions.size(); aind++) {
                if (state->Actions[aind]->PlannerSpecificData != NULL) {
                    DeleteSearchActionData((RSTARACTIONDATA*)state->Actions[aind]->PlannerSpecificData);
                    delete (RSTARACTIONDATA*)state->Actions[aind]->PlannerSpecificData;
                    state->Actions[aind]->PlannerSpecificData = NULL;
                }
            }//over actions
        }
    }

    pSearchStateSpace->searchMDP.Delete();
}

//reset properly search state space
//needs to be done before deleting states
int RSTARPlanner::ResetSearchStateSpace()
{
    pSearchStateSpace->OPEN->makeemptyheap();
    //pSearchStateSpace->inconslist->makeemptylist(RSTAR_INCONS_LIST_ID);

    return 1;
}

//initialization before each search
void RSTARPlanner::ReInitializeSearchStateSpace()
{
    //increase callnumber
    pSearchStateSpace->callnumber++;

    //reset iteration
    pSearchStateSpace->searchiteration = 0;
    pSearchStateSpace->bNewSearchIteration = true;

#if DEBUG
    SBPL_FPRINTF(fDeb, "reinitializing search state-space (new call number=%d search iter=%d)\n",
                 pSearchStateSpace->callnumber,pSearchStateSpace->searchiteration);
#endif

    pSearchStateSpace->OPEN->makeemptyheap();
    //pSearchStateSpace->inconslist->makeemptylist(RSTAR_INCONS_LIST_ID);

    //initialize start state
    RSTARState* startstateinfo = (RSTARState*)(pSearchStateSpace->searchstartstate->PlannerSpecificData);
    if (startstateinfo->callnumberaccessed != pSearchStateSpace->callnumber) {
        ReInitializeSearchStateInfo(startstateinfo);
    }

    startstateinfo->g = 0;

    //insert start state into the heap
    pSearchStateSpace->OPEN->insertheap(startstateinfo, ComputeKey(startstateinfo));

    pSearchStateSpace->bReinitializeSearchStateSpace = false;
    pSearchStateSpace->bReevaluatefvals = false;
}

//very first initialization
int RSTARPlanner::InitializeSearchStateSpace()
{
    if (pSearchStateSpace->OPEN->currentsize != 0)
    //		|| pSearchStateSpace->inconslist->currentsize != 0)
    {
        SBPL_ERROR("ERROR in InitializeSearchStateSpace: OPEN or INCONS is not empty\n");
        throw new SBPL_Exception();
    }

    pSearchStateSpace->eps = this->finitial_eps;
    pSearchStateSpace->eps_satisfied = INFINITECOST;
    pSearchStateSpace->searchiteration = 0;
    pSearchStateSpace->bNewSearchIteration = true;
    pSearchStateSpace->callnumber = 0;
    pSearchStateSpace->bReevaluatefvals = false;

    //create and set the search start state
    pSearchStateSpace->searchgoalstate = NULL;
    //pSearchStateSpace->searchstartstate = GetState(SearchStartStateID);
    pSearchStateSpace->searchstartstate = NULL;

    pSearchStateSpace->bReinitializeSearchStateSpace = true;

    return 1;
}

int RSTARPlanner::SetSearchGoalState(int SearchGoalStateID)
{
    if (pSearchStateSpace->searchgoalstate == NULL ||
        pSearchStateSpace->searchgoalstate->StateID != SearchGoalStateID)
    {
        pSearchStateSpace->searchgoalstate = GetState(SearchGoalStateID);

        //should be new search iteration
        pSearchStateSpace->eps_satisfied = INFINITECOST;
        pSearchStateSpace->bNewSearchIteration = true;
        pSearchStateSpace->eps = this->finitial_eps;

        //recompute heuristic for the heap if heuristics are used
#if USE_HEUR
        for(int i = 0; i < (int)pSearchStateSpace->searchMDP.StateArray.size(); i++)
        {
            CMDPSTATE* MDPstate = pSearchStateSpace->searchMDP.StateArray[i];
            RSTARState* state = (RSTARState*)MDPstate->PlannerSpecificData;
            state->h = ComputeHeuristic(MDPstate);
        }

        pSearchStateSpace->bReevaluatefvals = true;
#endif
    }

    return 1;
}

int RSTARPlanner::SetSearchStartState(int SearchStartStateID)
{
    CMDPSTATE* MDPstate = GetState(SearchStartStateID);

    if (MDPstate != pSearchStateSpace->searchstartstate) {
        pSearchStateSpace->searchstartstate = MDPstate;
        pSearchStateSpace->bReinitializeSearchStateSpace = true;
        pSearchStateSpace->eps_satisfied = INFINITECOST;
    }

    return 1;
}

void RSTARPlanner::PrintSearchState(RSTARState* state, FILE* fOut)
{
    SBPL_FPRINTF(fOut, "state %d: h=%d g=%u iterc=%d callnuma=%d heapind=%d \n", state->MDPstate->StateID, state->h,
                 state->g, state->iterationclosed, state->callnumberaccessed, state->heapindex);
    environment_->PrintState(state->MDPstate->StateID, true, fOut);
}

int RSTARPlanner::getHeurValue(int StateID)
{
    CMDPSTATE* MDPstate = GetState(StateID);
    RSTARState* searchstateinfo = (RSTARState*)MDPstate->PlannerSpecificData;
    return searchstateinfo->h;
}

vector<int> RSTARPlanner::GetSearchPath(int& solcost)
{
    vector<int> wholePathIds;
    RSTARState* rstargoalstate = (RSTARState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData;

    //set the return path and other path related variables
    vector<CMDPACTION*> tempPathID;

    //initially no path
    solcost = INFINITECOST;
    wholePathIds.clear();

    //special case when we are already at the goal
    if (rstargoalstate->MDPstate == pSearchStateSpace->searchstartstate) {
        solcost = 0;
        return wholePathIds;
    }

    if (rstargoalstate->g >= INFINITECOST || rstargoalstate->bestpredaction == NULL ||
        ((RSTARACTIONDATA*)rstargoalstate->bestpredaction->PlannerSpecificData)->pathIDs.size() == 0)
    {
        return wholePathIds; //no path to goal state was found
    }

    //path exists
    int pathcost = 0;
    RSTARState* rstarstate = rstargoalstate;
    while (rstarstate->bestpredaction != NULL && rstarstate->MDPstate != pSearchStateSpace->searchstartstate) {
        //get action data
        RSTARACTIONDATA* bestpredactiondata = (RSTARACTIONDATA*)rstarstate->bestpredaction->PlannerSpecificData;

        //get predecessor in the search tree
        RSTARState* predstate = (RSTARState*)GetState(rstarstate->bestpredaction->SourceStateID)->PlannerSpecificData;

        //check validity
        if (predstate->g + bestpredactiondata->clow != rstarstate->g) {
            SBPL_ERROR("ERROR: clow(=%d) + predstate.g(=%d) = %d != succstate.g = %d (callnum=%d, iter=%d)\n",
                       bestpredactiondata->clow, predstate->g, bestpredactiondata->clow + predstate->g, rstarstate->g,
                       pSearchStateSpace->callnumber, pSearchStateSpace->searchiteration);
            SBPL_PRINTF("predstate: ");
            environment_->PrintState(predstate->MDPstate->StateID, true, stdout);
            SBPL_PRINTF("succstate: ");
            environment_->PrintState(rstarstate->MDPstate->StateID, true, stdout);
            SBPL_PRINTF("PredState: stateID=%d g=%d calln=%d iterc=%d h=%d\n", predstate->MDPstate->StateID,
                        predstate->g, predstate->callnumberaccessed, predstate->iterationclosed, predstate->h);
            SBPL_PRINTF("Succstate: stateID=%d g=%d calln=%d iterc=%d h=%d\n", rstarstate->MDPstate->StateID,
                        rstarstate->g, rstarstate->callnumberaccessed, rstarstate->iterationclosed, rstarstate->h);
            fflush(fDeb);

            throw new SBPL_Exception();
        }

        //store the action and its cost
        tempPathID.push_back(rstarstate->bestpredaction);
        pathcost += rstarstate->bestpredaction->Costs[0];

        //go to the predecessor
        rstarstate = predstate;

        //another check
        if (pathcost + rstarstate->g > rstargoalstate->g) {
            SBPL_ERROR("ERROR: pathcost+rstarstate.g = %d > goalstate.g = %d\n", pathcost + rstarstate->g,
                       rstargoalstate->g);
            throw new SBPL_Exception();
        }
    }

    //now recover the actual path
    RSTARACTIONDATA* actiondata;
    for (int aind = 0; aind < (int)tempPathID.size(); aind++) {
        if (bforwardsearch)
            //getting path in reverse
            actiondata = (RSTARACTIONDATA*)tempPathID.at(tempPathID.size() - aind - 1)->PlannerSpecificData; 
        else
            actiondata = (RSTARACTIONDATA*)tempPathID.at(aind)->PlannerSpecificData; //getting path in reverse

        //get the states that correspond to the high-level action
        for (int j = 0; j < (int)actiondata->pathIDs.size(); j++) {
            //note: path corresponding to the action is already in right direction
            wholePathIds.push_back(actiondata->pathIDs.at(j)); 
        }
    }
    //add the goal state
    if (bforwardsearch)
        wholePathIds.push_back(rstargoalstate->MDPstate->StateID);
    else
        wholePathIds.push_back(pSearchStateSpace->searchstartstate->StateID);

    SBPL_FPRINTF(fDeb, "high-level pathcost=%d and high-level g(searchgoal)=%d\n", pathcost, rstargoalstate->g);

    //get the solcost
    solcost = pathcost;
    return wholePathIds;
}

void RSTARPlanner::PrintSearchPath(FILE* fOut)
{
    vector<int> pathIds;
    int solcost;

    pathIds = GetSearchPath(solcost);

    SBPL_FPRINTF(fOut, "high-level solution cost = %d, solution length=%d\n", solcost, (unsigned int)pathIds.size());
    for (int sind = 0; sind < (int)pathIds.size(); sind++) {
        environment_->PrintState(pathIds.at(sind), false, fOut);
    }
}

bool RSTARPlanner::Search(vector<int>& pathIds, int & PathCost, bool bFirstSolution, bool bOptimalSolution,
                          double MaxNumofSecs)
{
    CKey key;
    TimeStarted = clock();
    highlevel_searchexpands = 0;
    lowlevel_searchexpands = 0;

    //reset the return values
    PathCost = INFINITECOST;
    pathIds.clear();

    //bFirstSolution = false; //TODO-remove this but then fix crashing because later
    //searches within cycle re-initialize g-vals and test path found fails if last search ran out of time
    //so we need to save solutions between iterations
    //also, we need to call change callnumber before each search

#if DEBUG
    SBPL_FPRINTF(fDeb, "new search call (call number=%d)\n", pSearchStateSpace->callnumber);
#endif

    //set epsilons
    pSearchStateSpace->eps = this->finitial_eps;
    pSearchStateSpace->eps_satisfied = INFINITECOST;

    if (pSearchStateSpace->bReinitializeSearchStateSpace == true) {
        //re-initialize state space 
        ReInitializeSearchStateSpace();
    }

    if (bOptimalSolution) {
        pSearchStateSpace->eps = 1;
        MaxNumofSecs = INFINITECOST;
    }
    else if (bFirstSolution) {
        MaxNumofSecs = INFINITECOST;
    }

    //get the size of environment that is already allocated
    int oldenvsize = environment_->StateID2IndexMapping.size() * sizeof(int);

    //the main loop of R*
    int prevexpands = 0;
    clock_t loop_time;
    //TODO - change FINAL_EPS and DECREASE_EPS onto a parameter
    while (pSearchStateSpace->eps_satisfied > final_epsilon &&
           (clock() - TimeStarted) < MaxNumofSecs * (double)CLOCKS_PER_SEC)
    {
        loop_time = clock();

        //decrease eps for all subsequent iterations
        if (fabs(pSearchStateSpace->eps_satisfied - pSearchStateSpace->eps) < ERR_EPS && !bFirstSolution) {
            pSearchStateSpace->eps = pSearchStateSpace->eps - dec_eps;
            if (pSearchStateSpace->eps < final_epsilon) pSearchStateSpace->eps = final_epsilon;

            //the priorities need to be updated
            pSearchStateSpace->bReevaluatefvals = true;

            //it will be a new search. Since R* is non-incremental, it will have to be a new call
            pSearchStateSpace->bNewSearchIteration = true;
            pSearchStateSpace->bReinitializeSearchStateSpace = true;
        }

        //if(pSearchStateSpace->bReinitializeSearchStateSpace == true){
        //re-initialize state space
        ReInitializeSearchStateSpace(); //TODO - we have to do it currently since g-vals from old searches are invalid
        //}

        if (pSearchStateSpace->bNewSearchIteration) {
            pSearchStateSpace->searchiteration++;
            pSearchStateSpace->bNewSearchIteration = false;
        }

        //re-compute f-values if necessary and reorder the heap
        if (pSearchStateSpace->bReevaluatefvals) Reevaluatefvals();

        //improve or compute path
        if (ImprovePath(MaxNumofSecs) == 1) {
            pSearchStateSpace->eps_satisfied = pSearchStateSpace->eps; //note: eps is satisfied probabilistically
        }

        //print the solution cost and eps bound
        SBPL_PRINTF("eps=%f highlevel expands=%d g(searchgoal)=%d time=%.3f\n", pSearchStateSpace->eps_satisfied,
                    highlevel_searchexpands - prevexpands,
                    ((RSTARState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g,
                    double(clock() - loop_time) / CLOCKS_PER_SEC);

#if DEBUG
        SBPL_FPRINTF(fDeb, "eps=%f highlevel expands=%d g(searchgoal)=%d time=%.3f\n",
                     pSearchStateSpace->eps_satisfied, highlevel_searchexpands - prevexpands,
                     ((RSTARState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g,
                     double(clock()-loop_time)/CLOCKS_PER_SEC);
        PrintSearchState((RSTARState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData, fDeb);
#endif
        prevexpands = highlevel_searchexpands;

        //keep track of the best solution so far
        vector<int> CurrentPathIds;
        int CurrentPathCost = ((RSTARState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g;
        if (CurrentPathCost == INFINITECOST ||
            ((RSTARACTIONDATA*)((RSTARState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->bestpredaction->PlannerSpecificData)->pathIDs.size() == 0)
        {
            //the path to the goal is not found, it is just goal has been
            //generated but the last edge to it wasn't computed yet
            CurrentPathCost = INFINITECOST; 
        }
        else {
            //get the found path
            CurrentPathIds = GetSearchPath(CurrentPathCost);
        }
        //keep track of the best solution
        if (CurrentPathCost < PathCost) {
            PathCost = CurrentPathCost;
            pathIds = CurrentPathIds;
        }

        //if just the first solution then we are done
        if (bFirstSolution) break;

        //no solution exists
        if (((RSTARState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g == INFINITECOST) break;
    }

#if DEBUG
    SBPL_FFLUSH(fDeb);
#endif

    MaxMemoryCounter += (oldenvsize - environment_->StateID2IndexMapping.size() * sizeof(int));
    SBPL_PRINTF("MaxMemoryCounter = %d\n", MaxMemoryCounter);

    bool ret = false;
    if (PathCost == INFINITECOST) {
        SBPL_PRINTF("could not find a solution\n");
        ret = false;
    }
    else {
        SBPL_PRINTF("solution is found\n");
        ret = true;
    }

    SBPL_PRINTF("total highlevel expands this call = %d, planning time = %.3f secs, solution cost=%d\n",
                highlevel_searchexpands, (clock() - TimeStarted) / ((double)CLOCKS_PER_SEC), PathCost);

    //SBPL_FPRINTF(fStat, "%d %d\n", highlevel_searchexpands, MinPathCost);

    return ret;
}

//-----------------------------Interface function-----------------------------------------------------
//returns 1 if found a solution, and 0 otherwise
int RSTARPlanner::replan(double allocated_time_secs, vector<int>* solution_stateIDs_V)
{
    int solcost;

    return replan(allocated_time_secs, solution_stateIDs_V, &solcost);
}

//returns 1 if found a solution, and 0 otherwise
int RSTARPlanner::replan(double allocated_time_secs, vector<int>* solution_stateIDs_V, int* psolcost)
{
    vector<int> pathIds;
    bool bFound = false;
    int PathCost;
    bool bFirstSolution = this->bsearchuntilfirstsolution;
    bool bOptimalSolution = false;
    *psolcost = 0;

    SBPL_PRINTF("planner: replan called (bFirstSol=%d, bOptSol=%d)\n", bFirstSolution, bOptimalSolution);

    //plan
    if ((bFound = Search(pathIds, PathCost, bFirstSolution, bOptimalSolution, allocated_time_secs)) == false) {
        SBPL_PRINTF("failed to find a solution\n");
    }

    //copy the solution
    *solution_stateIDs_V = pathIds;
    *psolcost = PathCost;

    return (int)bFound;
}

int RSTARPlanner::set_goal(int goal_stateID)
{
    SBPL_PRINTF("planner: setting goal to %d\n", goal_stateID);
    environment_->PrintState(goal_stateID, true, stdout);

    if (bforwardsearch) {
        if (SetSearchGoalState(goal_stateID) != 1) {
            SBPL_ERROR("ERROR: failed to set search goal state\n");
            return 0;
        }
    }
    else {
        if (SetSearchStartState(goal_stateID) != 1) {
            SBPL_ERROR("ERROR: failed to set search start state\n");
            return 0;
        }
    }

    return 1;
}

int RSTARPlanner::set_start(int start_stateID)
{
    SBPL_PRINTF("planner: setting start to %d\n", start_stateID);
    environment_->PrintState(start_stateID, true, stdout);

    if (bforwardsearch) {
        if (SetSearchStartState(start_stateID) != 1) {
            SBPL_ERROR("ERROR: failed to set search start state\n");
            return 0;
        }
    }
    else {
        if (SetSearchGoalState(start_stateID) != 1) {
            SBPL_ERROR("ERROR: failed to set search goal state\n");
            return 0;
        }
    }

    return 1;
}

void RSTARPlanner::costs_changed(StateChangeQuery const & stateChange)
{
    //since R* is non-incremental
    pSearchStateSpace->bReinitializeSearchStateSpace = true;
}

void RSTARPlanner::costs_changed()
{
    //since R* is non-incremental
    pSearchStateSpace->bReinitializeSearchStateSpace = true;
}

int RSTARPlanner::force_planning_from_scratch()
{
    SBPL_PRINTF("planner: forceplanfromscratch set\n");

    pSearchStateSpace->bReinitializeSearchStateSpace = true;

    return 1;
}

int RSTARPlanner::set_search_mode(bool bSearchUntilFirstSolution)
{
    SBPL_PRINTF("planner: search mode set to %d\n", bSearchUntilFirstSolution);

    bsearchuntilfirstsolution = bSearchUntilFirstSolution;

    return 1;
}

void RSTARPlanner::print_searchpath(FILE* fOut)
{
    PrintSearchPath(fOut);
}

//---------------------------------------------------------------------------------------------------------

