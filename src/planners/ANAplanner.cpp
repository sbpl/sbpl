/*
 * This code was used for generating experimental data for the purpose of understanding the performance of
 * the Anytime Nonparametric A* (ANA*) algorithm.  
 * The authors of this algorithm are Jur van den Berg, Rajat Shah, Arthur Huang and Ken Goldberg.
 * The code is available at http://goldberg.berkeley.edu/ana/
 * 
 */

#include <cmath>
#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/planners/ANAplanner.h>
#include <sbpl/utils/heap.h>
#include <sbpl/utils/list.h>

using namespace std;

//-----------------------------------------------------------------------------------------------------

anaPlanner::anaPlanner(DiscreteSpaceInformation* environment, bool bSearchForward)
{
    bforwardsearch = bSearchForward;

    environment_ = environment;

    bsearchuntilfirstsolution = false;
    finitial_eps = ana_DEFAULT_INITIAL_EPS;
    searchexpands = 0;
    MaxMemoryCounter = 0;

    fDeb = fopen("debug.txt", "w");

    pSearchStateSpace_ = new anaSearchStateSpace_t;

    //create the ana planner
    if (CreateSearchStateSpace(pSearchStateSpace_) != 1) {
        printf("ERROR: failed to create statespace\n");
        return;
    }

    //set the start and goal states
    if (InitializeSearchStateSpace(pSearchStateSpace_) != 1) {
        printf("ERROR: failed to create statespace\n");
        return;
    }
}

anaPlanner::~anaPlanner()
{
    if (pSearchStateSpace_ != NULL) {
        //delete the statespace
        DeleteSearchStateSpace( pSearchStateSpace_);
        delete pSearchStateSpace_;
    }
    fclose( fDeb);
}

void anaPlanner::Initialize_searchinfo(CMDPSTATE* state, anaSearchStateSpace_t* pSearchStateSpace)
{
    anaState* searchstateinfo = (anaState*)state->PlannerSpecificData;

    searchstateinfo->MDPstate = state;
    InitializeSearchStateInfo(searchstateinfo, pSearchStateSpace);
}

CMDPSTATE* anaPlanner::CreateState(int stateID, anaSearchStateSpace_t* pSearchStateSpace)
{
    CMDPSTATE* state = NULL;

#if DEBUG
    if(environment_->StateID2IndexMapping[stateID][anaMDP_STATEID2IND] != -1)
    {
        printf("ERROR in CreateState: state already created\n");
        exit(1);
    }
#endif

    //adds to the tail a state
    state = pSearchStateSpace->searchMDP.AddState(stateID);

    //remember the index of the state
    environment_->StateID2IndexMapping[stateID][anaMDP_STATEID2IND] =
            pSearchStateSpace->searchMDP.StateArray.size() - 1;

#if DEBUG
    if (state != 
        pSearchStateSpace->searchMDP.StateArray[environment_->StateID2IndexMapping[stateID][anaMDP_STATEID2IND]])
    {
        printf("ERROR in CreateState: invalid state index\n");
        exit(1);
    }
#endif

    //create search specific info
    state->PlannerSpecificData = (anaState*)malloc(sizeof(anaState));
    Initialize_searchinfo(state, pSearchStateSpace);
    MaxMemoryCounter += sizeof(anaState);

    return state;
}

CMDPSTATE* anaPlanner::GetState(int stateID, anaSearchStateSpace_t* pSearchStateSpace)
{
    if (stateID >= (int)environment_->StateID2IndexMapping.size()) {
        SBPL_ERROR("ERROR in GetState: stateID %d is invalid\n", stateID);
        throw new SBPL_Exception();
    }

    if (environment_->StateID2IndexMapping[stateID][anaMDP_STATEID2IND] == -1)
        return CreateState(stateID, pSearchStateSpace);
    else
        return pSearchStateSpace->searchMDP.StateArray[environment_->StateID2IndexMapping[stateID][anaMDP_STATEID2IND]];
}

//-----------------------------------------------------------------------------------------------------

int anaPlanner::ComputeHeuristic(CMDPSTATE* MDPstate, anaSearchStateSpace_t* pSearchStateSpace)
{
    //compute heuristic for search

    if (bforwardsearch) {

#if MEM_CHECK == 1
        //int WasEn = DisableMemCheck();
#endif

        //forward search: heur = distance from state to searchgoal which is Goal anaState
        int retv = environment_->GetGoalHeuristic(MDPstate->StateID);

#if MEM_CHECK == 1
        //if (WasEn)
        //	EnableMemCheck();
#endif

        return retv;

    }
    else {
        //backward search: heur = distance from searchgoal to state
        return environment_->GetStartHeuristic(MDPstate->StateID);
    }
}

//initialization of a state
void anaPlanner::InitializeSearchStateInfo(anaState* state, anaSearchStateSpace_t* pSearchStateSpace)
{
    state->g = INFINITECOST;
    state->v = INFINITECOST;
    state->iterationclosed = 0;
    state->callnumberaccessed = pSearchStateSpace->callnumber;
    state->bestnextstate = NULL;
    state->costtobestnextstate = INFINITECOST;
    state->heapindex = 0;
    state->listelem[ana_INCONS_LIST_ID] = 0;
    state->numofexpands = 0;

    state->bestpredstate = NULL;

    //compute heuristics
#if USE_HEUR
    if(pSearchStateSpace->searchgoalstate != NULL)
    state->h = ComputeHeuristic(state->MDPstate, pSearchStateSpace);
    else
    state->h = 0;
#else
    state->h = 0;
#endif
}

//re-initialization of a state
void anaPlanner::ReInitializeSearchStateInfo(anaState* state, anaSearchStateSpace_t* pSearchStateSpace)
{
    state->g = INFINITECOST;
    state->v = INFINITECOST;
    state->iterationclosed = 0;
    state->callnumberaccessed = pSearchStateSpace->callnumber;
    state->bestnextstate = NULL;
    state->costtobestnextstate = INFINITECOST;
    state->heapindex = 0;
    state->listelem[ana_INCONS_LIST_ID] = 0;
    state->numofexpands = 0;

    state->bestpredstate = NULL;

    //compute heuristics
#if USE_HEUR

    if (pSearchStateSpace->searchgoalstate != NULL) {
        state->h = ComputeHeuristic(state->MDPstate, pSearchStateSpace);
    }
    else
    state->h = 0;

#else

    state->h = 0;

#endif

}

void anaPlanner::DeleteSearchStateData(anaState* state)
{
    //no memory was allocated
    MaxMemoryCounter = 0;
    return;
}

double anaPlanner::get_e_value(anaSearchStateSpace_t* pSearchStateSpace, int stateID)
{

    CMDPSTATE* MDPstate = GetState(stateID, pSearchStateSpace);
    anaState* searchstateinfo = (anaState*)MDPstate->PlannerSpecificData;

    //if(!(searchstateinfo->g > pSearchStateSpace->G)) {
    if (searchstateinfo->h == 0) {
        if (searchstateinfo->g >= pSearchStateSpace->G) {
            return 0.0;
        }
        else {
            return (double)INFINITECOST;
        }
    }
    else {
        return ((double)pSearchStateSpace->G - 1.0 * searchstateinfo->g) / (double)searchstateinfo->h;

        //return  0.5*(((double) pSearchStateSpace->G - 1.0*searchstateinfo->g) / (double) searchstateinfo->h);

        //return  1000 + (((double) pSearchStateSpace->G - 1.0*searchstateinfo->g) / (double) searchstateinfo->h);
    }
}

//used for backward search
void anaPlanner::UpdatePreds(anaState* state, anaSearchStateSpace_t* pSearchStateSpace)
{
    vector<int> PredIDV;
    vector<int> CostV;
    CKey key;
    anaState *p;

    environment_->GetPreds(state->MDPstate->StateID, &PredIDV, &CostV);

    //iterate through predecessors of s
    for (int pind = 0; pind < (int)PredIDV.size(); pind++) {
        CMDPSTATE* PredMDPState = GetState(PredIDV[pind], pSearchStateSpace);
        p = (anaState*)(PredMDPState->PlannerSpecificData);
        if (p->callnumberaccessed != pSearchStateSpace->callnumber) ReInitializeSearchStateInfo(p, pSearchStateSpace);

        //see if we can improve the value of p

        if ((p->g > state->g + CostV[pind]) && (state->g + CostV[pind] + p->h < pSearchStateSpace->G)) {
            p->g = state->g + CostV[pind];
            p->bestnextstate = state->MDPstate;
            p->costtobestnextstate = CostV[pind];

            key.key[0] = (long)-get_e_value(pSearchStateSpace, p->MDPstate->StateID);
            if (pSearchStateSpace->heap->inheap(p)) {
                pSearchStateSpace->heap->updateheap(p, key);
            }
            else {
                pSearchStateSpace->heap->insertheap(p, key);
            }
        }
    }
}

//used for forward search
void anaPlanner::UpdateSuccs(anaState* state, anaSearchStateSpace_t* pSearchStateSpace)
{
    vector<int> SuccIDV;
    vector<int> CostV;
    CKey key;
    anaState *n;

    environment_->GetSuccs(state->MDPstate->StateID, &SuccIDV, &CostV);

    //iterate through predecessors of s
    for (int sind = 0; sind < (int)SuccIDV.size(); sind++) {
        CMDPSTATE* SuccMDPState = GetState(SuccIDV[sind], pSearchStateSpace);
        int cost = CostV[sind];

        n = (anaState*)(SuccMDPState->PlannerSpecificData);
        if (n->callnumberaccessed != pSearchStateSpace->callnumber) ReInitializeSearchStateInfo(n, pSearchStateSpace);

        //see if we can improve the value of n
        //taking into account the cost of action
        if ((n->g > state->g + cost) && ((state->g + cost + n->h) < pSearchStateSpace->G)) {
            n->g = state->g + cost;
            n->bestpredstate = state->MDPstate;

            key.key[0] = (long)-get_e_value(pSearchStateSpace, n->MDPstate->StateID);
            /*if(key.key[0] >= -1) {
             printf("inserting on Open with key =%d\n", key.key[0]);
             }*/
            if (pSearchStateSpace->heap->inheap(n)) {
                pSearchStateSpace->heap->updateheap(n, key);
            }
            else {
                pSearchStateSpace->heap->insertheap(n, key);
            }
        }
    }
}

//TODO-debugmax - add obsthresh and other thresholds to other environments in 3dkin
int anaPlanner::GetGVal(int StateID, anaSearchStateSpace_t* pSearchStateSpace)
{
    CMDPSTATE* cmdp_state = GetState(StateID, pSearchStateSpace);
    anaState* state = (anaState*)cmdp_state->PlannerSpecificData;
    return state->g;
}

//returns 1 if the solution is found, 0 if the solution does not exist and 2 if it ran out of time
int anaPlanner::ImprovePath(anaSearchStateSpace_t* pSearchStateSpace, double MaxNumofSecs)
{
    int expands;
    anaState *state, *searchgoalstate;
    CKey key, minkey;
    //CKey goalkey;

    expands = 0;

    if (pSearchStateSpace->searchgoalstate == NULL) {
        SBPL_ERROR("ERROR searching: no goal state is set\n");
        throw new SBPL_Exception();
    }

    //goal state
    searchgoalstate = (anaState*)(pSearchStateSpace->searchgoalstate->PlannerSpecificData);
    if (searchgoalstate->callnumberaccessed != pSearchStateSpace->callnumber) {
        ReInitializeSearchStateInfo( searchgoalstate, pSearchStateSpace);
    }

    //set goal key
    //goalkey.key[0] = -get_e_value(pSearchStateSpace, searchgoalstate->MDPstate->StateID);
    //goalkey.key[1] = searchgoalstate->h;

    //expand states until done
    minkey.key[0] = -(pSearchStateSpace->heap->getminkeyheap().key[0]);
    CKey oldkey = minkey;
    while (!pSearchStateSpace->heap->emptyheap() &&
           (clock() - TimeStarted) < MaxNumofSecs * (double)CLOCKS_PER_SEC) 
           //&& goalkey > minkey && minkey.key[0] <= INFINITECOST
    {
        /*if(minkey.key[0] < 10) {
         printf("Key: %.2f\t", minkey.key[0]);
         }*/
        //printf("%.2f\t", minkey.key[0]);

        //get the state
        state = (anaState*)pSearchStateSpace->heap->deleteminheap();

        if (state->MDPstate->StateID == searchgoalstate->MDPstate->StateID) {
            pSearchStateSpace->G = state->g;
            //minkey.key[0] =
            //printf("search exited with a solution for eps=%.3f\n", pSearchStateSpace->eps);
            //printf("eps=%f time_elapsed=%.3f\n", pSearchStateSpace->eps, double(clock()-TimeStarted)/CLOCKS_PER_SEC);

            searchexpands += expands;
            return 1;// so that it does not declare it as run out of time
        }

        //double e_val = floor(minkey.key[0]*100.0) / 100.0;
        double e_val = minkey.key[0];
        //double save = pSearchStateSpace->eps;
        if (e_val < pSearchStateSpace->eps) { // && e_val>=ana_FINAL_EPS
            pSearchStateSpace->eps = minkey.key[0];
            //if(save - e_val > 0.01)
            //printf("eps=%f time_elapsed=%.6f\n", pSearchStateSpace->eps, double(clock()-TimeStarted)/CLOCKS_PER_SEC);
        }

#if DEBUG
        //fprintf(fDeb, "expanding state(%d): h=%d g=%u key=%u v=%u iterc=%d callnuma=%d expands=%d (g(goal)=%u)\n",
        //	state->MDPstate->StateID, state->h, state->g, state->g+(int)(pSearchStateSpace->eps*state->h), state->v,
        //	state->iterationclosed, state->callnumberaccessed, state->numofexpands, searchgoalstate->g);
        //fprintf(fDeb, "expanding: ");
        //PrintSearchState(state, fDeb);
        if (state->listelem[ana_INCONS_LIST_ID] != NULL) {
            fprintf(fDeb, "ERROR: expanding a state from inconslist\n");
            printf("ERROR: expanding a state from inconslist\n");
            exit(1);
        }
        //fflush(fDeb);
#endif

#if DEBUG
        if (minkey.key[0] < oldkey.key[0] && fabs(this->finitial_eps - 1.0) < ERR_EPS) {
            //printf("WARN in search: the sequence of keys decreases\n");
            //exit(1);
        }
        oldkey = minkey;
#endif

        if (state->v == state->g) {
            printf("ERROR: consistent state is being expanded\n");
#if DEBUG
            fprintf(fDeb, "ERROR: consistent state is being expanded\n");
            exit(1);
#endif
        }

        //recompute state value
        state->v = state->g;
        state->iterationclosed = pSearchStateSpace->searchiteration;

        //new expand
        expands++;
        state->numofexpands++;

        if (bforwardsearch == false)
            UpdatePreds(state, pSearchStateSpace);
        else
            UpdateSuccs(state, pSearchStateSpace);

        //recompute minkey
        minkey.key[0] = -(pSearchStateSpace->heap->getminkeyheap().key[0]);

        //recompute goalkey if necessary

        pSearchStateSpace->G = searchgoalstate->g;

        if (expands % 100000 == 0 && expands > 0) {
            //printf("expands so far=%u\n", expands);
        }

        /*if(state->MDPstate->StateID == searchgoalstate->MDPstate->StateID) {
         goalkey.key[0] = minkey.key[0];
         break;
         }*/
    }

    int retv = 1;
    if (searchgoalstate->g == INFINITECOST && pSearchStateSpace->heap->emptyheap()) {
        printf("solution does not exist: search exited because heap is empty\n");
        retv = 0;
    }
    else if (!pSearchStateSpace->heap->emptyheap() && 0 < minkey.key[0]) {
        printf("search exited because it ran out of time\n");
        //printf("Goalkey=%f and minkey=%f", goalkey.key[0], minkey.key[0]);
        retv = 2;
    }
    else if (searchgoalstate->g == INFINITECOST && !pSearchStateSpace->heap->emptyheap()) {
        printf("solution does not exist: search exited because all "
               "candidates for expansion have infinite heuristics\n");
        retv = 0;
    }
    else {
        //printf("eps=%.3f time=%.3f\n", pSearchStateSpace->eps, double(clock() - TimeStarted)/CLOCKS_PER_SEC);
        retv = 3;
    }

    //fprintf(fDeb, "expanded=%d\n", expands);

    searchexpands += expands;

    return retv;
}

void anaPlanner::Reevaluatefvals(anaSearchStateSpace_t* pSearchStateSpace)
{
    CKey key;
    int i;
    CHeap* pheap = pSearchStateSpace->heap;

    //recompute priorities for states in OPEN and reorder it
    for (i = 1; i <= pheap->currentsize; ++i) {
        //anaState* state = (anaState*)pheap->heap[i].heapstate;

        // CHANGED - cast removed

        pheap->heap[i].key.key[0] = (long)-get_e_value(pSearchStateSpace,
                                                       ((anaState*)pheap->heap[i].heapstate)->MDPstate->StateID);

        //pheap->heap[i].key.key[1] = state->h;
    }
    pheap->makeheap();

    pSearchStateSpace->bReevaluatefvals = false;
}

//creates (allocates memory) search state space
//does not initialize search statespace
int anaPlanner::CreateSearchStateSpace(anaSearchStateSpace_t* pSearchStateSpace)
{
    //create a heap
    pSearchStateSpace->heap = new CHeap;
    //pSearchStateSpace->inconslist = new CHeap;
    MaxMemoryCounter += sizeof(CHeap);
    MaxMemoryCounter += sizeof(CList);

    pSearchStateSpace->searchgoalstate = NULL;
    pSearchStateSpace->searchstartstate = NULL;

    searchexpands = 0;

    pSearchStateSpace->bReinitializeSearchStateSpace = false;

    return 1;
}

//deallocates memory used by SearchStateSpace
void anaPlanner::DeleteSearchStateSpace(anaSearchStateSpace_t* pSearchStateSpace)
{
    if (pSearchStateSpace->heap != NULL) {
        pSearchStateSpace->heap->makeemptyheap();
        delete pSearchStateSpace->heap;
        pSearchStateSpace->heap = NULL;
    }

    /*
    if(pSearchStateSpace->inconslist != NULL)
    {
        pSearchStateSpace->inconslist->makeemptyheap();
        delete pSearchStateSpace->inconslist;
        pSearchStateSpace->inconslist = NULL;
    }
    */

    //delete the states themselves
    int iend = (int)pSearchStateSpace->searchMDP.StateArray.size();
    for (int i = 0; i < iend; i++) {
        CMDPSTATE* state = pSearchStateSpace->searchMDP.StateArray[i];
        if (state != NULL && state->PlannerSpecificData != NULL) {
            DeleteSearchStateData((anaState*)state->PlannerSpecificData);
            free((anaState*)state->PlannerSpecificData);
            state->PlannerSpecificData = NULL;
        }
    }
    pSearchStateSpace->searchMDP.Delete();
}

//reset properly search state space
//needs to be done before deleting states
int anaPlanner::ResetSearchStateSpace(anaSearchStateSpace_t* pSearchStateSpace)
{
    pSearchStateSpace->heap->makeemptyheap();
    //	pSearchStateSpace->inconslist->makeemptyheap();

    return 1;
}

//initialization before each search
void anaPlanner::ReInitializeSearchStateSpace(anaSearchStateSpace_t* pSearchStateSpace)
{
    CKey key;

    //increase callnumber
    pSearchStateSpace->callnumber++;

    //reset iteration
    pSearchStateSpace->searchiteration = 0;
    pSearchStateSpace->bNewSearchIteration = true;
    pSearchStateSpace->G = INFINITECOST;

#if DEBUG
    fprintf(fDeb, "reinitializing search state-space (new call number=%d search iter=%d)\n",
        pSearchStateSpace->callnumber,pSearchStateSpace->searchiteration );
#endif

    pSearchStateSpace->heap->makeemptyheap();
    //pSearchStateSpace->inconslist->makeemptyheap();
    //reset
    pSearchStateSpace->eps = this->finitial_eps;
    pSearchStateSpace->eps_satisfied = INFINITECOST;

    //initialize start state
    anaState* startstateinfo = (anaState*)(pSearchStateSpace->searchstartstate->PlannerSpecificData);
    if (startstateinfo->callnumberaccessed != pSearchStateSpace->callnumber) {
        ReInitializeSearchStateInfo( startstateinfo, pSearchStateSpace);
    }

    startstateinfo->g = 0;

    //insert start state into the heap

    // CHANGED - long int cast removed
    key.key[0] = (long)-get_e_value(pSearchStateSpace, startstateinfo->MDPstate->StateID); 

    //key.key[1] = startstateinfo->h;
    pSearchStateSpace->heap->insertheap(startstateinfo, key);

    pSearchStateSpace->bReinitializeSearchStateSpace = false;
    pSearchStateSpace->bReevaluatefvals = false;
}

//very first initialization
int anaPlanner::InitializeSearchStateSpace(anaSearchStateSpace_t* pSearchStateSpace)
{
    if (pSearchStateSpace->heap->currentsize != 0) {
        SBPL_ERROR("ERROR in InitializeSearchStateSpace: heap or list is not empty\n");
        throw new SBPL_Exception();
    }

    pSearchStateSpace->eps = this->finitial_eps;
    pSearchStateSpace->eps_satisfied = INFINITECOST;
    pSearchStateSpace->searchiteration = 0;
    pSearchStateSpace->bNewSearchIteration = true;
    pSearchStateSpace->callnumber = 0;
    pSearchStateSpace->bReevaluatefvals = false;

    pSearchStateSpace->G = INFINITECOST;

    //create and set the search start state
    pSearchStateSpace->searchgoalstate = NULL;
    //pSearchStateSpace->searchstartstate = GetState(SearchStartStateID, pSearchStateSpace);
    pSearchStateSpace->searchstartstate = NULL;

    pSearchStateSpace->bReinitializeSearchStateSpace = true;

    return 1;
}

int anaPlanner::SetSearchGoalState(int SearchGoalStateID, anaSearchStateSpace_t* pSearchStateSpace)
{
    if (pSearchStateSpace->searchgoalstate == NULL ||
        pSearchStateSpace->searchgoalstate->StateID != SearchGoalStateID)
    {
        pSearchStateSpace->searchgoalstate = GetState(SearchGoalStateID, pSearchStateSpace);

        //should be new search iteration
        pSearchStateSpace->eps_satisfied = INFINITECOST;
        pSearchStateSpace->bNewSearchIteration = true;
        pSearchStateSpace_->eps = this->finitial_eps;

        //recompute heuristic for the heap if heuristics is used
#if USE_HEUR
        for(int i = 0; i < (int)pSearchStateSpace->searchMDP.StateArray.size(); i++)
        {
            CMDPSTATE* MDPstate = pSearchStateSpace->searchMDP.StateArray[i];
            anaState* state = (anaState*)MDPstate->PlannerSpecificData;
            state->h = ComputeHeuristic(MDPstate, pSearchStateSpace);
        }

        pSearchStateSpace->bReevaluatefvals = true;
#endif
    }

    return 1;
}

int anaPlanner::SetSearchStartState(int SearchStartStateID, anaSearchStateSpace_t* pSearchStateSpace)
{
    CMDPSTATE* MDPstate = GetState(SearchStartStateID, pSearchStateSpace);

    if (MDPstate != pSearchStateSpace->searchstartstate) {
        pSearchStateSpace->searchstartstate = MDPstate;
        pSearchStateSpace->bReinitializeSearchStateSpace = true;
    }

    return 1;
}

int anaPlanner::ReconstructPath(anaSearchStateSpace_t* pSearchStateSpace)
{
    if (bforwardsearch) //nothing to do, if search is backward
    {
        CMDPSTATE* MDPstate = pSearchStateSpace->searchgoalstate;
        CMDPSTATE* PredMDPstate;
        anaState *predstateinfo, *stateinfo;

#if DEBUG
        fprintf(fDeb, "reconstructing a path:\n");
#endif

        while (MDPstate != pSearchStateSpace->searchstartstate) {
            stateinfo = (anaState*)MDPstate->PlannerSpecificData;

#if DEBUG
            PrintSearchState(stateinfo, fDeb);
#endif
            if (stateinfo->g == INFINITECOST) {
                //printf("ERROR in ReconstructPath: g of the state on the path is INFINITE\n");
                //exit(1);
                return -1;
            }

            if (stateinfo->bestpredstate == NULL) {
                SBPL_ERROR("ERROR in ReconstructPath: bestpred is NULL\n");
                throw new SBPL_Exception();
            }

            //get the parent state
            PredMDPstate = stateinfo->bestpredstate;
            predstateinfo = (anaState*)PredMDPstate->PlannerSpecificData;

            //set its best next info
            predstateinfo->bestnextstate = MDPstate;

            //check the decrease of g-values along the path
            if (predstateinfo->v >= stateinfo->g) {
                SBPL_ERROR("ERROR in ReconstructPath: g-values are non-decreasing\n");
                PrintSearchState(predstateinfo, fDeb);
                throw new SBPL_Exception();
            }

            //transition back
            MDPstate = PredMDPstate;
        }
    }

    return 1;
}

void anaPlanner::PrintSearchPath(anaSearchStateSpace_t* pSearchStateSpace, FILE* fOut)
{
    anaState* searchstateinfo;
    CMDPSTATE* state;
    int goalID;
    int PathCost;

    if (bforwardsearch) {
        state = pSearchStateSpace->searchstartstate;
        goalID = pSearchStateSpace->searchgoalstate->StateID;
    }
    else {
        state = pSearchStateSpace->searchgoalstate;
        goalID = pSearchStateSpace->searchstartstate->StateID;
    }
    if (fOut == NULL) fOut = stdout;

    PathCost = ((anaState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g;

    fprintf(fOut, "Printing a path from state %d to the goal state %d\n", state->StateID,
            pSearchStateSpace->searchgoalstate->StateID);
    fprintf(fOut, "Path cost = %d:\n", PathCost);

    environment_->PrintState(state->StateID, false, fOut);

    int costFromStart = 0;
    while (state->StateID != goalID) {
        fprintf(fOut, "state %d ", state->StateID);

        if (state->PlannerSpecificData == NULL) {
            fprintf(fOut, "path does not exist since search data does not exist\n");
            break;
        }

        searchstateinfo = (anaState*)state->PlannerSpecificData;

        if (searchstateinfo->bestnextstate == NULL) {
            fprintf(fOut, "path does not exist since bestnextstate == NULL\n");
            break;
        }
        if (searchstateinfo->g == INFINITECOST) {
            fprintf(fOut, "path does not exist since bestnextstate == NULL\n");
            break;
        }

        int costToGoal = PathCost - costFromStart;
        int transcost = searchstateinfo->g - ((anaState*)(searchstateinfo->bestnextstate->PlannerSpecificData))->v;
        if (bforwardsearch) transcost = -transcost;

        costFromStart += transcost;

        fprintf(fOut, "g=%d-->state %d, h = %d ctg = %d  ", searchstateinfo->g,
                searchstateinfo->bestnextstate->StateID, searchstateinfo->h, costToGoal);

        state = searchstateinfo->bestnextstate;

        environment_->PrintState(state->StateID, false, fOut);
    }
}

void anaPlanner::PrintSearchState(anaState* state, FILE* fOut)
{
    fprintf(fOut, "state %d: h=%d g=%u v=%u iterc=%d callnuma=%d expands=%d heapind=%d inconslist=%d\n",
            state->MDPstate->StateID, state->h, state->g, state->v, state->iterationclosed, state->callnumberaccessed,
            state->numofexpands, state->heapindex, state->listelem[ana_INCONS_LIST_ID] ? 1 : 0);
    environment_->PrintState(state->MDPstate->StateID, true, fOut);
}

int anaPlanner::getHeurValue(anaSearchStateSpace_t* pSearchStateSpace, int StateID)
{
    CMDPSTATE* MDPstate = GetState(StateID, pSearchStateSpace);
    anaState* searchstateinfo = (anaState*)MDPstate->PlannerSpecificData;
    return searchstateinfo->h;
}

vector<int> anaPlanner::GetSearchPath(anaSearchStateSpace_t* pSearchStateSpace, int& solcost)
{
    vector<int> SuccIDV;
    vector<int> CostV;
    vector<int> wholePathIds;
    anaState* searchstateinfo;
    CMDPSTATE* state = NULL;
    CMDPSTATE* goalstate = NULL;
    CMDPSTATE* startstate = NULL;

    if (bforwardsearch) {
        startstate = pSearchStateSpace->searchstartstate;
        goalstate = pSearchStateSpace->searchgoalstate;

        //reconstruct the path by setting bestnextstate pointers appropriately
        ReconstructPath(pSearchStateSpace);
    }
    else {
        startstate = pSearchStateSpace->searchgoalstate;
        goalstate = pSearchStateSpace->searchstartstate;
    }

    state = startstate;

    wholePathIds.push_back(state->StateID);
    solcost = 0;

    FILE* fOut = stdout;
    while (state->StateID != goalstate->StateID) {
        if (state->PlannerSpecificData == NULL) {
            fprintf(fOut, "path does not exist since search data does not exist\n");
            break;
        }

        searchstateinfo = (anaState*)state->PlannerSpecificData;

        if (searchstateinfo->bestnextstate == NULL) {
            fprintf(fOut, "path does not exist since bestnextstate == NULL\n");
            break;
        }
        if (searchstateinfo->g == INFINITECOST) {
            fprintf(fOut, "path does not exist since bestnextstate == NULL\n");
            break;
        }

        environment_->GetSuccs(state->StateID, &SuccIDV, &CostV);
        int actioncost = INFINITECOST;
        for (int i = 0; i < (int)SuccIDV.size(); i++) {

            if (SuccIDV.at(i) == searchstateinfo->bestnextstate->StateID) actioncost = CostV.at(i);

        }
        if (actioncost == INFINITECOST) printf("WARNING: actioncost = %d\n", actioncost);

        solcost += actioncost;

        //fprintf(fDeb, "actioncost=%d between states %d and %d\n",
        //        actioncost, state->StateID, searchstateinfo->bestnextstate->StateID);
        //environment_->PrintState(state->StateID, false, fDeb);
        //environment_->PrintState(searchstateinfo->bestnextstate->StateID, false, fDeb);

#if DEBUG
        anaState* nextstateinfo = (anaState*)(searchstateinfo->bestnextstate->PlannerSpecificData);
        if(actioncost != abs((int)(searchstateinfo->g - nextstateinfo->g)) && pSearchStateSpace->eps_satisfied <= 1.001)
        {
            fprintf(fDeb, "ERROR: actioncost=%d is not matching the difference in g-values of %d\n",
                actioncost, abs((int)(searchstateinfo->g - nextstateinfo->g)));
            printf("ERROR: actioncost=%d is not matching the difference in g-values of %d\n",
                actioncost,abs((int)(searchstateinfo->g - nextstateinfo->g)));
            PrintSearchState(searchstateinfo, fDeb);
            PrintSearchState(nextstateinfo, fDeb);
        }
#endif

        state = searchstateinfo->bestnextstate;

        wholePathIds.push_back(state->StateID);
    }

    return wholePathIds;
}

bool anaPlanner::Search(anaSearchStateSpace_t* pSearchStateSpace, vector<int>& pathIds, int & PathCost,
                        bool bFirstSolution, bool bOptimalSolution, double MaxNumofSecs)
{
    CKey key;
    TimeStarted = clock();
    searchexpands = 0;

#if DEBUG
    fprintf(fDeb, "new search call (call number=%d)\n", pSearchStateSpace->callnumber);
#endif

    if (pSearchStateSpace->bReinitializeSearchStateSpace == true) {
        //re-initialize state space 
        ReInitializeSearchStateSpace(pSearchStateSpace);
    }

    if (bOptimalSolution) {
        pSearchStateSpace->eps = 1;
        MaxNumofSecs = INFINITECOST;
    }
    else if (bFirstSolution) {
        MaxNumofSecs = INFINITECOST;
    }

    //ensure heuristics are up-to-date
    environment_->EnsureHeuristicsUpdated((bforwardsearch == true));

    //the main loop of ana*
    int prevexpands = 0;
    clock_t loop_time;

    // CHANGE MADE TO WHILE LOOP to account for open.empty() == FALSE
    while (!pSearchStateSpace->heap->emptyheap() && pSearchStateSpace->eps_satisfied > ana_FINAL_EPS &&
           (clock() - TimeStarted) < MaxNumofSecs * (double)CLOCKS_PER_SEC)
    {
        loop_time = clock();
        //decrease eps for all subsequent iterations
        /*if(fabs(pSearchStateSpace->eps_satisfied - pSearchStateSpace->eps) < ERR_EPS && !bFirstSolution)
         {
         pSearchStateSpace->eps = pSearchStateSpace->eps - ana_DECREASE_EPS;
         if(pSearchStateSpace->eps < ana_FINAL_EPS)
         pSearchStateSpace->eps = ana_FINAL_EPS;

         //the priorities need to be updated
         pSearchStateSpace->bReevaluatefvals = true;

         //it will be a new search
         pSearchStateSpace->bNewSearchIteration = true;

         //build a new open list by merging it with incons one
         BuildNewOPENList(pSearchStateSpace);

         }*/

        pSearchStateSpace->searchiteration++;
        pSearchStateSpace->bNewSearchIteration = false;

        //re-compute f-values if necessary and reorder the heap
        //if(pSearchStateSpace->bReevaluatefvals)
        //	Reevaluatefvals(pSearchStateSpace);

        //improve or compute path
        int retVal = ImprovePath(pSearchStateSpace, MaxNumofSecs);
        anaState* state;
        CKey key;
        CHeap* open = pSearchStateSpace->heap;
        //printf("states expanded: %d\t states considered: %d\t time elapsed: %f\n",searchexpands - prevexpands, pSearchStateSpace->heap->currentsize, double(clock() - TimeStarted)/CLOCKS_PER_SEC);

        double epsprime = 1.0;
        for (int j = 1; j <= open->currentsize;) {
            state = (anaState*)open->heap[j].heapstate;
            double temp_eps = (double)((pSearchStateSpace->G * 1.0) / (double)(state->g + state->h));
            if (temp_eps > epsprime) {
                epsprime = temp_eps;
            }
            double e_val = get_e_value(pSearchStateSpace, state->MDPstate->StateID);
            if (e_val <= 1.0) {

                open->deleteheap_unsafe(state);

            }
            else {
                key.key[0] = (long)-e_val;

                open->updateheap_unsafe(state, key);
                ++j;
            }
            pSearchStateSpace->eps_satisfied = epsprime;
        }
        open->makeheap();

        //print the solution cost and eps bound
        if (retVal == 1) {
            //printf("suboptimality=%f expands=%d g(searchgoal)=%d loop_time=%.3f time_elapsed=%.3f memoryCounter=%d\n", pSearchStateSpace->eps_satisfied, searchexpands - prevexpands, ((anaState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g,double(clock()-loop_time)/CLOCKS_PER_SEC, double(clock() - TimeStarted)/CLOCKS_PER_SEC, MaxMemoryCounter);

            printf("suboptimality=%f g(searchgoal)=%d time_elapsed=%.3f memoryCounter=%d\n",
                   pSearchStateSpace->eps_satisfied,
                   ((anaState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g, double(clock()
                       - TimeStarted) / CLOCKS_PER_SEC, MaxMemoryCounter);

            //printf("states expanded: %d\t states considered: %d\t time elapsed: %f\n",searchexpands - prevexpands, pSearchStateSpace->heap->currentsize, double(clock() - TimeStarted)/CLOCKS_PER_SEC);
        }

#if DEBUG
        fprintf(fDeb, "eps=%f expands=%d g(searchgoal)=%d time=%.3f\n", pSearchStateSpace->eps_satisfied, searchexpands - prevexpands,
            ((anaState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g,double(clock()-loop_time)/CLOCKS_PER_SEC);
        PrintSearchState((anaState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData, fDeb);
#endif
        prevexpands = searchexpands;

        //if just the first solution then we are done
        if (bFirstSolution) break;

        //no solution exists
        if (((anaState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g == INFINITECOST) break;
    }

#if DEBUG
    fflush(fDeb);
#endif

    printf("Suboptimality = %.4f\n", pSearchStateSpace->eps_satisfied);

    PathCost = ((anaState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g;
    MaxMemoryCounter += environment_->StateID2IndexMapping.size() * sizeof(int);

    printf("MaxMemoryCounter = %d\n", MaxMemoryCounter);

    int solcost = INFINITECOST;
    bool ret = false;
    if (PathCost == INFINITECOST) {
        printf("could not find a solution\n");
        ret = false;
    }
    else {
        printf("solution is found\n");
        pathIds = GetSearchPath(pSearchStateSpace, solcost);
        ret = true;
    }

    printf("total expands this call = %d, planning time = %.3f secs, solution cost=%d\n", searchexpands, (clock()
        - TimeStarted) / ((double)CLOCKS_PER_SEC), solcost);

    //fprintf(fStat, "%d %d\n", searchexpands, solcost);

    return ret;
}

//-----------------------------Interface function-----------------------------------------------------

//returns 1 if found a solution, and 0 otherwise
int anaPlanner::replan(double allocated_time_secs, vector<int>* solution_stateIDs_V)
{
    int solcost;

    return replan(allocated_time_secs, solution_stateIDs_V, &solcost);
}

//returns 1 if found a solution, and 0 otherwise
int anaPlanner::replan(double allocated_time_secs, vector<int>* solution_stateIDs_V, int* psolcost)
{
    vector<int> pathIds;
    bool bFound = false;
    int PathCost;
    //bool bFirstSolution = true;
    bool bFirstSolution = this->bsearchuntilfirstsolution;
    bool bOptimalSolution = false;
    *psolcost = 0;

    printf("planner: replan called (bFirstSol=%d, bOptSol=%d)\n", bFirstSolution, bOptimalSolution);

    //plan
    if (!(bFound = Search(pSearchStateSpace_, pathIds, PathCost, bFirstSolution, bOptimalSolution,
                          allocated_time_secs)))
    {
        printf("failed to find a solution\n");
    }

    //copy the solution
    *solution_stateIDs_V = pathIds;
    *psolcost = PathCost;

    return (int)bFound;

}

int anaPlanner::set_goal(int goal_stateID)
{
    printf("planner: setting goal to %d\n", goal_stateID);
    environment_->PrintState(goal_stateID, true, stdout);

    if (bforwardsearch) {
        if (SetSearchGoalState(goal_stateID, pSearchStateSpace_) != 1) {
            printf("ERROR: failed to set search goal state\n");
            return 0;
        }
    }
    else {
        if (SetSearchStartState(goal_stateID, pSearchStateSpace_) != 1) {
            printf("ERROR: failed to set search start state\n");
            return 0;
        }
    }

    return 1;
}

int anaPlanner::set_start(int start_stateID)
{
    printf("planner: setting start to %d\n", start_stateID);
    environment_->PrintState(start_stateID, true, stdout);

    if (bforwardsearch) {

        if (SetSearchStartState(start_stateID, pSearchStateSpace_) != 1) {
            printf("ERROR: failed to set search start state\n");
            return 0;
        }
    }
    else {
        if (SetSearchGoalState(start_stateID, pSearchStateSpace_) != 1) {
            printf("ERROR: failed to set search goal state\n");
            return 0;
        }
    }

    return 1;
}

void anaPlanner::costs_changed(StateChangeQuery const & stateChange)
{
    pSearchStateSpace_->bReinitializeSearchStateSpace = true;
}

void anaPlanner::costs_changed()
{
    pSearchStateSpace_->bReinitializeSearchStateSpace = true;
}

int anaPlanner::force_planning_from_scratch()
{
    printf("planner: forceplanfromscratch set\n");

    pSearchStateSpace_->bReinitializeSearchStateSpace = true;

    return 1;
}

int anaPlanner::set_search_mode(bool bSearchUntilFirstSolution)
{
    printf("planner: search mode set to %d\n", bSearchUntilFirstSolution);

    bsearchuntilfirstsolution = bSearchUntilFirstSolution;

    return 1;
}

void anaPlanner::print_searchpath(FILE* fOut)
{
    PrintSearchPath(pSearchStateSpace_, fOut);
}

//---------------------------------------------------------------------------------------------------------
