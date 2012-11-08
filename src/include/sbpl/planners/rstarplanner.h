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

#ifndef __RSTARPLANNER_H_
#define __RSTARPLANNER_H_

#include <cstdio>
#include <ctime>
#include <sbpl/planners/planner.h>
#include <sbpl/utils/heap.h>
#include <sbpl/utils/key.h>
#include <sbpl/utils/mdp.h>

//---configuration----
//control of EPS
#define RSTAR_DEFAULT_INITIAL_EPS	    3.0
#define RSTAR_DECREASE_EPS				0.2
#define RSTAR_FINAL_EPS					1.0

//the number of states to expand for local search in RSTAR before it declares a hard case and postpones its processing
#define RSTAR_EXPTHRESH  1000 
//---------------------
#define RSTAR_INCONS_LIST_ID 0

class CList;
class StateChangeQuery;

//-------------------------------------------------------------

/**
 * \brief high level states in R* search
 * \note in other words, state structure for high level states in Gamma graph
 */
typedef class RSTARSEARCHSTATEDATA : public AbstractSearchState
{
public:
    /**
     * \brief the MDP state itself
     */
    CMDPSTATE* MDPstate;
    /**
     * \brief RSTAR* relevant data
     */
    unsigned int g;
    /**
     * \brief RSTAR* relevant data
     */
    short unsigned int iterationclosed;
    /**
     * \brief RSTAR* relevant data
     */
    short unsigned int callnumberaccessed;

    /**
     * \brief best predecessor and the action from it
     * 
     * \note note that in R* bestpredaction refers to best predecessor in the
     *       high-level graph constructed by R* this graph is ALWAYS rooted at
     *       searchstart and directed towards searchgoal independently of the actual
     *       direction of search (from start to goal or from goal to start) thus
     *       sourcestate for the action is always at the state closer to the start of
     *       the search tree
     */
    CMDPACTION *bestpredaction;

    /**
     * \brief set of predecessor actions together with some info
     */
    std::vector<CMDPACTION*> predactionV;

    /**
     * \brief RSTAR* relevant data - heuristics
     */
    int h;

public:
    RSTARSEARCHSTATEDATA() { }
    ~RSTARSEARCHSTATEDATA() { }
} RSTARState;

/**
 * \brief action in Gamma graph. Each action in Gamma graph corresponds to a path
 */
typedef struct RSTARACTIONDATA_T
{
    /**
     * \brief lower bound on the cost of the path
     */
    int clow;
    /**
     * \brief number of expansions done so far to compute the path that represents this action
     */
    int exp;
    /**
     * \brief path that the action represents
     * 
     * \note the path is always stored as a valid path w.r.t. the original graph
     *       (not search tree) so if the search is forward, then the path is from
     *       source to target and otherwise, the path is from target toward source
     */
    std::vector<int> pathIDs;
} RSTARACTIONDATA;

/**
 * \brief low-level (local) search state in R*
 */
typedef class RSTARLSEARCHSTATEDATA : public AbstractSearchState
{
public:
    /**
     * \brief the MDP state itself
     */
    CMDPSTATE* MDPstate;
    /**
     * \brief planner relevant data - g-value
     */
    int g;

    /**
     * \brief planner relevant data
     */
    unsigned int iteration;
    /**
     * \brief planner relevant data
     */
    unsigned int iterationclosed;

    /**
     * \brief best predecessors according to the search tree (so, in backward search these are actual successors)
     */
    CMDPSTATE* bestpredstate;
    /**
     * \brief the cost of the best action from the best preds to this state
     */
    int bestpredstateactioncost;

public:
    RSTARLSEARCHSTATEDATA() { }
    ~RSTARLSEARCHSTATEDATA() { }
} RSTARLSearchState;

/**
 * \brief local search statespace in R*
 */
class RSTARLSEARCHSTATESPACE
{
public:
    /**
     * \brief graph constructed by local search
     */
    CMDP MDP;
    /**
     * \brief start state
     */
    CMDPSTATE* StartState;
    /**
     * \brief goal state
     */
    CMDPSTATE* GoalState;
    /**
     * \brief search-related variable
     */
    int iteration;

    /**
     * \brief open list
     */
    CHeap* OPEN;
    /**
     * \brief incons list - used for suboptimal search
     */
    CList* INCONS;

public:
    RSTARLSEARCHSTATESPACE()
    {
        OPEN = NULL;
        INCONS = NULL;
        StartState = NULL;
        GoalState = NULL;
    }

    ~RSTARLSEARCHSTATESPACE()
    {
        if (OPEN != NULL) delete OPEN;
    }
};

typedef class RSTARLSEARCHSTATESPACE RSTARLSearchStateSpace_t;

/**
 * \brief statespace
 */
typedef struct RSTARSEARCHSTATESPACE
{
    /**
     * \brief required epsilon
     */
    double eps;
    /**
     * \brief epsilon that is satisfied
     */
    double eps_satisfied;
    /**
     * \brief open list
     */
    CHeap* OPEN;

    /**
     * \brief searchiteration gets incremented with every R* search (and reset upon every increment of callnumber)
     */
    short unsigned int searchiteration;
    /**
     * \brief callnumber gets incremented with every call to R* (so, it can
     *        be multiple number of times within planning times since R* is executed
     *        multiple times)
     */
    short unsigned int callnumber;
    /**
     * \brief search goal state (not necessarily the actual start state, it depends on search direction)
     */
    CMDPSTATE* searchgoalstate;
    /**
     * \brief search start state
     */
    CMDPSTATE* searchstartstate;

    /**
     * \brief graph constructed by high-level search
     */
    CMDP searchMDP;

    /**
     * \brief need to reevaluate fvals
     */
    bool bReevaluatefvals;
    /**
     * \brief need to reinitialize search space
     */
    bool bReinitializeSearchStateSpace;
    /**
     * \brief set when it is a new search iteration
     */
    bool bNewSearchIteration;
} RSTARSearchStateSpace_t;

/**
 * \brief RSTAR* planner
 */
class RSTARPlanner : public SBPLPlanner
{
public:
    /**
     * \brief replan a path within the allocated time, return the solution in the vector
     */
    int replan(double allocated_time_secs, std::vector<int>* solution_stateIDs_V);

    /**
     * \brief replan a path within the allocated time, return the solution in the vector, also returns solution cost
     */
    int replan(double allocated_time_sec, std::vector<int>* solution_stateIDs_V, int* solcost);

    /**
     * \brief set the goal state
     */
    int set_goal(int goal_stateID);

    /**
     * \brief set the start state
     */
    int set_start(int start_stateID);

    /**
     * \brief inform the search about the new edge costs
     */
    void costs_changed(StateChangeQuery const & stateChange);

    /**
     * \brief inform the search about the new edge costs -
     * 
     * \note since R* is non-incremental, it is sufficient (and more efficient)
     *       to just inform R* of the fact that some costs changed
     */
    void costs_changed();

    /**
     * \brief set a flag to get rid of the previous search efforts, release
     *        the memory and re-initialize the search, when the next replan is called
     */
    int force_planning_from_scratch();

    /**
     * \brief you can either search forwards or backwards
     */
    int set_search_mode(bool bSearchUntilFirstSolution);

    /**
     * \brief obtain probabilistic eps suboptimality bound
     */
    virtual double get_solution_probabilisticeps() const { return pSearchStateSpace->eps_satisfied; }

    /**
     * \brief get number of high level expands
     */
    virtual int get_highlevel_expands() const { return highlevel_searchexpands; }

    /**
     * \brief get number of low level expands
     */
    virtual int get_lowlevel_expands() const { return lowlevel_searchexpands; }

    /**
     * \brief set initial solution epsilon that R* will try to satisfy (probabilistically)
     */
    virtual void set_initialsolution_eps(double initialsolution_eps) { finitial_eps = initialsolution_eps; }

    /**
     * \brief sets the value to decrease from eps at each iteration
     */
    virtual void set_eps_step(double eps) { dec_eps = eps; }

    /**
     * \brief sets the number of states to expand for local search in RSTAR before
     *        it declares a hard case and postpones its processing.
     */
    virtual void set_local_expand_thres(unsigned thres) { local_expand_thres = thres; }

    /**
     * \brief returns the initial epsilon used by the search
     */
    double get_initial_eps() { return finitial_eps; }

    /**
     * \brief returns the final epsilon achieved during the search
     */
    double get_final_epsilon()
    {
        if (pSearchStateSpace != NULL)
            return pSearchStateSpace->eps_satisfied;
        else
            return INFINITECOST;
    }

    /**
     * \brief print out search path
     */
    void print_searchpath(FILE* fOut);

    /**
     * \brief constructor
     */
    RSTARPlanner(DiscreteSpaceInformation* environment, bool bforwardsearch);

    /**
     * \brief destructor
     */
    ~RSTARPlanner();

private:
    //member variables
    double finitial_eps;
    double dec_eps;
    double final_epsilon;
    double local_expand_thres;

    bool bforwardsearch; //if true, then search proceeds forward, otherwise backward

    bool bsearchuntilfirstsolution; //if true, then search until first solution only (see planner.h for search modes)

    RSTARSearchStateSpace_t* pSearchStateSpace;
    RSTARLSearchStateSpace_t* pLSearchStateSpace;

    unsigned int highlevel_searchexpands;
    unsigned int lowlevel_searchexpands;
    int MaxMemoryCounter;
    clock_t TimeStarted;
    FILE *fDeb;

    //member functions
    void Initialize_searchinfo(CMDPSTATE* state);
    CMDPSTATE* CreateState(int stateID);
    CMDPSTATE* GetState(int stateID);

    int ComputeHeuristic(CMDPSTATE* MDPstate);

    //initialization of a state
    void InitializeSearchStateInfo(RSTARState* state);

    //re-initialization of a state
    void ReInitializeSearchStateInfo(RSTARState* state);

    void DeleteSearchStateData(RSTARState* state);
    void DeleteSearchActionData(RSTARACTIONDATA* actiondata);

    int GetGVal(int StateID);

    //returns 1 if the solution is found, 0 if the solution does not exist and 2 if it ran out of time
    int ImprovePath(double MaxNumofSecs);

    //note this does NOT re-compute heuristics, only re-orders OPEN list based on current eps and h-vals
    void Reevaluatefvals();

    //creates (allocates memory) search state space
    //does not initialize search statespace
    int CreateSearchStateSpace();

    //deallocates memory used by SearchStateSpace
    void DeleteSearchStateSpace();

    //debugging
    void PrintSearchState(RSTARState* state, FILE* fOut);

    //reset properly search state space
    //needs to be done before deleting states
    int ResetSearchStateSpace();

    //initialization before each search
    void ReInitializeSearchStateSpace();

    //very first initialization
    int InitializeSearchStateSpace();

    //setting start/goal
    int SetSearchGoalState(int SearchGoalStateID);
    int SetSearchStartState(int SearchStartStateID);

    int getHeurValue(int StateID);

    //get path
    std::vector<int> GetSearchPath(int& solcost);
    void PrintSearchPath(FILE* fOut);

    //the actual search
    bool Search(std::vector<int>& pathIds, int & PathCost, bool bFirstSolution, bool bOptimalSolution,
                double MaxNumofSecs);
    //local search
    bool ComputeLocalPath(int StartStateID, int GoalStateID, int maxc, int maxe, int *pCost, int *pCostLow, int *pExp,
                          std::vector<int>* pPathIDs, int* pNewGoalStateID, double maxnumofsecs);

    //global search functions
    void SetBestPredecessor(RSTARState* rstarState, RSTARState* rstarPredState, CMDPACTION* action);
    CKey ComputeKey(RSTARState* rstarState);

    //local search functions
    void Initialize_rstarlsearchdata(CMDPSTATE* state);
    CMDPSTATE* CreateLSearchState(int stateID);
    CMDPSTATE* GetLSearchState(int stateID);
    bool DestroyLocalSearchMemory();
    CKey LocalSearchComputeKey(RSTARLSearchState* rstarlsearchState);
};

#endif

