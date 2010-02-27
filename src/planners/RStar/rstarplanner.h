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


//---configuration----

//control of EPS
#define RSTAR_DEFAULT_INITIAL_EPS	    5.0
#define RSTAR_DECREASE_EPS				0.2
#define RSTAR_FINAL_EPS					1.0

//the number of states to expand for local search in RSTAR before it declares a hard case and postpones its processing
#define RSTAR_EXPTHRESH  150         //100 //200 TODO - make it a parameter


//---------------------

#define RSTAR_INCONS_LIST_ID 0

class CMDP;
class CMDPSTATE;
class CMDPACTION;
class CHeap;
class CList;


//-------------------------------------------------------------


//state structure for high level states in Gamma graph
typedef class RSTARSEARCHSTATEDATA : public AbstractSearchState
{
public:
	CMDPSTATE* MDPstate; //the MDP state itself
	//RSTAR* relevant data
	unsigned int g;
	short unsigned int iterationclosed;
	short unsigned int callnumberaccessed;

	//best predecessor and the action from it 
	//- note that in R* bestpredaction refers to best predecessor in the high-level graph constructed by R*
	//this graph is ALWAYS rooted at searchstart and directed towards searchgoal independently of the actual direction of search 
	//(from start to goal or from goal to start)
	//thus sourcestate for the action is always at the state closer to the start of the search tree
	CMDPACTION *bestpredaction; 

    //set of predecessor actions together with some info
    vector<CMDPACTION*> predactionV;

	int h;

	
public:
	RSTARSEARCHSTATEDATA() {};	
	~RSTARSEARCHSTATEDATA() {};
} RSTARState;


typedef struct RSTARACTIONDATA_T
{
    int clow;
    int exp;
	//the path is always stored as a valid path w.r.t. the original graph (not search tree) 
	//so if the search is forward, then the path is from source to target and otherwise, the path is from target toward source
    vector<int> pathIDs; 
}RSTARACTIONDATA;


typedef class RSTARLSEARCHSTATEDATA : public AbstractSearchState
{
public:
	CMDPSTATE* MDPstate; //the MDP state itself
	//planner relevant data
	int g;

	unsigned int iteration;
    unsigned int iterationclosed;

	//best predecessors according to the search tree (so, in backward search these are actual successors)
	CMDPSTATE* bestpredstate; 
    int bestpredstateactioncost;

	
public:
	RSTARLSEARCHSTATEDATA() {};	
	~RSTARLSEARCHSTATEDATA() {};
} RSTARLSearchState;

//local search statespace
class RSTARLSEARCHSTATESPACE
{
public:
	CMDP MDP;
	CMDPSTATE* StartState;
	CMDPSTATE* GoalState;
	int iteration;

    CHeap* OPEN;
    CList* INCONS;

public:
    RSTARLSEARCHSTATESPACE(){
        OPEN = NULL;
		INCONS = NULL;
        StartState = NULL;
        GoalState = NULL;
    };
    ~RSTARLSEARCHSTATESPACE(){
        if(OPEN != NULL)
            delete OPEN;
    };
};
typedef class RSTARLSEARCHSTATESPACE RSTARLSearchStateSpace_t;


//statespace
typedef struct RSTARSEARCHSTATESPACE
{
	double eps;
    double eps_satisfied;
	CHeap* OPEN;

	//searchiteration gets incremented with every R* search (and reset upon every increment of callnumber)
	short unsigned int searchiteration; 
	//callnumber gets incremented with every call to R* (so, it can be multiple number of times within planning times since R* is executed multiple times)
	short unsigned int callnumber; 
	CMDPSTATE* searchgoalstate;
	CMDPSTATE* searchstartstate;
	
	CMDP searchMDP;

	bool bReevaluatefvals;
    bool bReinitializeSearchStateSpace;
	bool bNewSearchIteration;

} RSTARSearchStateSpace_t;



//RSTAR* planner
class RSTARPlanner : public SBPLPlanner
{

public:
	int replan(double allocated_time_secs, vector<int>* solution_stateIDs_V);
	int replan(double allocated_time_sec, vector<int>* solution_stateIDs_V, int* solcost);

    int set_goal(int goal_stateID);
    int set_start(int start_stateID);
    void costs_changed(StateChangeQuery const & stateChange);
    void costs_changed();
    int force_planning_from_scratch();

	int set_search_mode(bool bSearchUntilFirstSolution);


	virtual double get_solution_probabilisticeps() const {return pSearchStateSpace->eps_satisfied;};
    virtual int get_highlevel_expands() const { return highlevel_searchexpands; }
    virtual int get_lowlevel_expands() const { return lowlevel_searchexpands; }
	virtual void set_initialsolution_eps(double initialsolution_eps) {finitial_eps = initialsolution_eps;};

	void print_searchpath(FILE* fOut);


	//constructors & destructors
    RSTARPlanner(DiscreteSpaceInformation* environment, bool bforwardsearch);
    ~RSTARPlanner();



private:

	//member variables
	double finitial_eps;

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
	vector<int> GetSearchPath(int& solcost);
	void PrintSearchPath(FILE* fOut);
	
	//the actual search
	bool Search(vector<int>& pathIds, int & PathCost, bool bFirstSolution, bool bOptimalSolution, double MaxNumofSecs);
	//local search
	bool ComputeLocalPath(int StartStateID, int GoalStateID, int maxc, int maxe, int *pCost, int *pCostLow, int *pExp, vector<int>* pPathIDs, int* pNewGoalStateID);

	//global search functions
	void  SetBestPredecessor(RSTARState* rstarState, RSTARState* rstarPredState, CMDPACTION* action);
	CKey ComputeKey(RSTARState* rstarState);

	//local search functions
	void Initialize_rstarlsearchdata(CMDPSTATE* state);
	CMDPSTATE* CreateLSearchState(int stateID);
	CMDPSTATE* GetLSearchState(int stateID);
	bool DestroyLocalSearchMemory();
	CKey LocalSearchComputeKey(RSTARLSearchState* rstarlsearchState);




};


#endif



