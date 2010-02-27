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
#ifndef __ADPLANNER_H_
#define __ADPLANNER_H_


//---configuration----

//control of EPS
#define AD_DEFAULT_INITIAL_EPS	    10.0
#define AD_DECREASE_EPS    0.2
#define AD_FINAL_EPS	    1.0


//---------------------

#define AD_INCONS_LIST_ID 0

class CMDP;
class CMDPSTATE;
class CMDPACTION;
class CHeap;
class CList;


//-------------------------------------------------------------


//state structure
typedef class ADSEARCHSTATEDATA : public AbstractSearchState
{
public:
	CMDPSTATE* MDPstate; //the MDP state itself
	//AD* relevant data
	unsigned int v;
	unsigned int g;
	short unsigned int iterationclosed;
	short unsigned int callnumberaccessed;
	short unsigned int numofexpands;
	//best predecessor and the action from it, used only in forward searches
	CMDPSTATE *bestpredstate;

	//the next state if executing best action
	CMDPSTATE  *bestnextstate;
	unsigned int costtobestnextstate;
	int h;

	
public:
	ADSEARCHSTATEDATA() {};	
	~ADSEARCHSTATEDATA() {};
} ADState;



//statespace
typedef struct ADSEARCHSTATESPACE
{
	double eps;
    double eps_satisfied;
	CHeap* heap;
	CList* inconslist;
	short unsigned int searchiteration;
	short unsigned int callnumber;
	CMDPSTATE* searchgoalstate;
	CMDPSTATE* searchstartstate;
	
	CMDP searchMDP;

	bool bReevaluatefvals;
    bool bReinitializeSearchStateSpace;
	bool bRebuildOpenList;

} ADSearchStateSpace_t;



//AD* planner
class ADPlanner : public SBPLPlanner
{

public:
	int replan(double allocated_time_secs, vector<int>* solution_stateIDs_V);
	int replan(double allocated_time_secs, vector<int>* solution_stateIDs_V, int* solcost);

    int set_goal(int goal_stateID);
    int set_start(int start_stateID);
    int force_planning_from_scratch(); 
	
	int set_search_mode(bool bSearchUntilFirstSolution);
	void costs_changed(StateChangeQuery const & stateChange);


	void update_succs_of_changededges(vector<int>* succsIDV);
	void update_preds_of_changededges(vector<int>* predsIDV);


	virtual double get_solution_eps() const {return pSearchStateSpace_->eps_satisfied;};
    virtual int get_n_expands() const { return searchexpands; }

	virtual void set_initialsolution_eps(double initialsolution_eps) {finitial_eps = initialsolution_eps;};


	//constructors & destructors
    ADPlanner(DiscreteSpaceInformation* environment, bool bForwardSearch);
    ~ADPlanner();



private:

	//member variables
	double finitial_eps;
	MDPConfig* MDPCfg_;

	bool bforwardsearch;
	bool bsearchuntilfirstsolution; //if true, then search until first solution (see planner.h for search modes)

    ADSearchStateSpace_t* pSearchStateSpace_;

	unsigned int searchexpands;
	int MaxMemoryCounter;
	clock_t TimeStarted;
	FILE *fDeb;


	//member functions
	void Initialize_searchinfo(CMDPSTATE* state, ADSearchStateSpace_t* pSearchStateSpace);

	CMDPSTATE* CreateState(int stateID, ADSearchStateSpace_t* pSearchStateSpace);

	CMDPSTATE* GetState(int stateID, ADSearchStateSpace_t* pSearchStateSpace);

	int ComputeHeuristic(CMDPSTATE* MDPstate, ADSearchStateSpace_t* pSearchStateSpace);

	//initialization of a state
	void InitializeSearchStateInfo(ADState* state, ADSearchStateSpace_t* pSearchStateSpace);

	//re-initialization of a state
	void ReInitializeSearchStateInfo(ADState* state, ADSearchStateSpace_t* pSearchStateSpace);

	void DeleteSearchStateData(ADState* state);


	//used for backward search
	void UpdatePredsofOverconsState(ADState* state, ADSearchStateSpace_t* pSearchStateSpace);
	void UpdatePredsofUnderconsState(ADState* state, ADSearchStateSpace_t* pSearchStateSpace);

	//used for forward search
	void UpdateSuccsofOverconsState(ADState* state, ADSearchStateSpace_t* pSearchStateSpace);
	void UpdateSuccsofUnderconsState(ADState* state, ADSearchStateSpace_t* pSearchStateSpace);
	
	void UpdateSetMembership(ADState* state);
	void Recomputegval(ADState* state);


	int GetGVal(int StateID, ADSearchStateSpace_t* pSearchStateSpace);

	//returns 1 if the solution is found, 0 if the solution does not exist and 2 if it ran out of time
	int ComputePath(ADSearchStateSpace_t* pSearchStateSpace, double MaxNumofSecs);

	void BuildNewOPENList(ADSearchStateSpace_t* pSearchStateSpace);

	void Reevaluatefvals(ADSearchStateSpace_t* pSearchStateSpace);

	//creates (allocates memory) search state space
	//does not initialize search statespace
	int CreateSearchStateSpace(ADSearchStateSpace_t* pSearchStateSpace);

	//deallocates memory used by SearchStateSpace
	void DeleteSearchStateSpace(ADSearchStateSpace_t* pSearchStateSpace);


	//reset properly search state space
	//needs to be done before deleting states
	int ResetSearchStateSpace(ADSearchStateSpace_t* pSearchStateSpace);

	//initialization before each search
	void ReInitializeSearchStateSpace(ADSearchStateSpace_t* pSearchStateSpace);

	//very first initialization
	int InitializeSearchStateSpace(ADSearchStateSpace_t* pSearchStateSpace);

	int SetSearchGoalState(int SearchGoalStateID, ADSearchStateSpace_t* pSearchStateSpace);


	int SetSearchStartState(int SearchStartStateID, ADSearchStateSpace_t* pSearchStateSpace);

	//reconstruct path functions are only relevant for forward search
	int ReconstructPath(ADSearchStateSpace_t* pSearchStateSpace);


	void PrintSearchState(ADState* searchstateinfo, FILE* fOut);
	void PrintSearchPath(ADSearchStateSpace_t* pSearchStateSpace, FILE* fOut);

	int getHeurValue(ADSearchStateSpace_t* pSearchStateSpace, int StateID);

	//get path 
	vector<int> GetSearchPath(ADSearchStateSpace_t* pSearchStateSpace, int& solcost);


	bool Search(ADSearchStateSpace_t* pSearchStateSpace, vector<int>& pathIds, int & PathCost, bool bFirstSolution, bool bOptimalSolution, double MaxNumofSecs);

	CKey ComputeKey(ADState* state);

	void Update_SearchSuccs_of_ChangedEdges(vector<int> const * statesIDV);


};


/**
   See comments in sbpl/src/planners/planner.h about the what and why
   of this class.
*/
class StateChangeQuery
{
public:
  virtual ~StateChangeQuery() {}
  virtual std::vector<int> const * getPredecessors() const = 0;
  virtual std::vector<int> const * getSuccessors() const = 0;
};


#endif



