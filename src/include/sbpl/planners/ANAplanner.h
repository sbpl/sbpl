/*
 * This code was used for generating experimental data for the purpose of understanding the performance of
 * the Anytime Nonparametric A* (ANA*) algorithm.  
 * The authors of this algorithm are Jur van den Berg, Rajat Shah, Arthur Huang and Ken Goldberg.
 * The code is available at http://goldberg.berkeley.edu/ana/
 * 
 */

#ifndef __anaPLANNER_H_
#define __anaPLANNER_H_

#include <cstdio>
#include <ctime>
#include <sbpl/planners/planner.h>
#include <sbpl/utils/mdp.h>

//---configuration----
//control of EPS
//initial suboptimality bound (cost solution <= cost(eps*cost optimal solution)
#define ana_DEFAULT_INITIAL_EPS	    100000.0
//as planning time exist, ana* decreases epsilon bound
#define ana_DECREASE_EPS    0.2
//final epsilon bound
#define ana_FINAL_EPS	    1.0
//---------------------
#define ana_INCONS_LIST_ID 0

class CHeap;
class DiscreteSpaceInformation;
class StateChangeQuery;

//-------------------------------------------------------------

/**
 * \brief state structure used in ana* search tree
 */
typedef class anaSEARCHSTATEDATA : public AbstractSearchState
{
public:
    /**
     * \brief the MDP state itself
     */
    CMDPSTATE* MDPstate;
    /**
     * \brief ana* relevant data
     */
    unsigned int v;
    /**
     * \brief ana* relevant data
     */
    unsigned int g;
    /**
     * \brief ana* relevant data
     */
    short unsigned int iterationclosed;
    /**
     * \brief ana* relevant data
     */
    short unsigned int callnumberaccessed;
    /**
     * \brief ana* relevant data
     */
    short unsigned int numofexpands;
    /**
     * \brief best predecessor and the action from it, used only in forward searches
     */
    CMDPSTATE *bestpredstate;
    /**
     * \brief the next state if executing best action
     */
    CMDPSTATE *bestnextstate;
    unsigned int costtobestnextstate;
    int h;

public:
    anaSEARCHSTATEDATA() { }
    ~anaSEARCHSTATEDATA() { }
} anaState;

/**
 * \brief the statespace of ana*
 */
typedef struct anaSEARCHSTATESPACE
{
    unsigned int G;

    double eps;
    double eps_satisfied;
    CHeap* heap;

    short unsigned int searchiteration;
    short unsigned int callnumber;
    CMDPSTATE* searchgoalstate;
    CMDPSTATE* searchstartstate;

    CMDP searchMDP;

    bool bReevaluatefvals;
    bool bReinitializeSearchStateSpace;
    bool bNewSearchIteration;
} anaSearchStateSpace_t;

/**
 * \brief ana* planner
 */
class anaPlanner : public SBPLPlanner
{
public:
    /**
     * \brief replan a path within the allocated time, return the solution in the vector
     */
    int replan(double allocated_time_secs, std::vector<int>* solution_stateIDs_V);

    /**
     * \brief replan a path within the allocated time, return the solution in
     *        the vector, also returns solution cost
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
     * \note since ana* is non-incremental, it is sufficient (and more
     *       efficient) to just inform ana* of the fact that some costs changed
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
     * \brief returns the suboptimality bound on the currently found solution
     */
    virtual double get_solution_eps() const { return pSearchStateSpace_->eps_satisfied; }

    /**
     * \brief returns the number of states expanded so far
     */
    virtual int get_n_expands() const { return searchexpands; }

    /**
     * \brief returns the value of the initial epsilon (suboptimality bound) used
     */
    virtual void set_initialsolution_eps(double initialsolution_eps) { finitial_eps = initialsolution_eps; }

    /**
     * \brief prints out the search path into a file
     */
    void print_searchpath(FILE* fOut);

    /**
     * \brief constructor
     */
    anaPlanner(DiscreteSpaceInformation* environment, bool bforwardsearch);

    /**
     * \brief destructor
     */
    ~anaPlanner();

private:
    //member variables
    double finitial_eps;
    MDPConfig* MDPCfg_;

    bool bforwardsearch; //if true, then search proceeds forward, otherwise backward

    bool bsearchuntilfirstsolution; //if true, then search until first solution only (see planner.h for search modes)

    anaSearchStateSpace_t* pSearchStateSpace_;

    unsigned int searchexpands;
    int MaxMemoryCounter;
    clock_t TimeStarted;
    FILE *fDeb;

    void Initialize_searchinfo(CMDPSTATE* state, anaSearchStateSpace_t* pSearchStateSpace);

    CMDPSTATE* CreateState(int stateID, anaSearchStateSpace_t* pSearchStateSpace);

    CMDPSTATE* GetState(int stateID, anaSearchStateSpace_t* pSearchStateSpace);

    int ComputeHeuristic(CMDPSTATE* MDPstate, anaSearchStateSpace_t* pSearchStateSpace);

    //initialization of a state
    void InitializeSearchStateInfo(anaState* state, anaSearchStateSpace_t* pSearchStateSpace);

    //re-initialization of a state
    void ReInitializeSearchStateInfo(anaState* state, anaSearchStateSpace_t* pSearchStateSpace);

    void DeleteSearchStateData(anaState* state);

    // NEW FUNCTION
    double get_e_value(anaSearchStateSpace_t* pSearchStateSpace, int stateID);

    //used for backward search
    void UpdatePreds(anaState* state, anaSearchStateSpace_t* pSearchStateSpace);

    //used for forward search
    void UpdateSuccs(anaState* state, anaSearchStateSpace_t* pSearchStateSpace);

    int GetGVal(int StateID, anaSearchStateSpace_t* pSearchStateSpace);

    //returns 1 if the solution is found, 0 if the solution does not exist and 2 if it ran out of time
    int ImprovePath(anaSearchStateSpace_t* pSearchStateSpace, double MaxNumofSecs);

    void BuildNewOPENList(anaSearchStateSpace_t* pSearchStateSpace);

    void Reevaluatefvals(anaSearchStateSpace_t* pSearchStateSpace);

    //creates (allocates memory) search state space
    //does not initialize search statespace
    int CreateSearchStateSpace(anaSearchStateSpace_t* pSearchStateSpace);

    //deallocates memory used by SearchStateSpace
    void DeleteSearchStateSpace(anaSearchStateSpace_t* pSearchStateSpace);

    //debugging
    void PrintSearchState(anaState* state, FILE* fOut);

    //reset properly search state space
    //needs to be done before deleting states
    int ResetSearchStateSpace(anaSearchStateSpace_t* pSearchStateSpace);

    //initialization before each search
    void ReInitializeSearchStateSpace(anaSearchStateSpace_t* pSearchStateSpace);

    //very first initialization
    int InitializeSearchStateSpace(anaSearchStateSpace_t* pSearchStateSpace);

    int SetSearchGoalState(int SearchGoalStateID, anaSearchStateSpace_t* pSearchStateSpace);

    int SetSearchStartState(int SearchStartStateID, anaSearchStateSpace_t* pSearchStateSpace);

    //reconstruct path functions are only relevant for forward search
    int ReconstructPath(anaSearchStateSpace_t* pSearchStateSpace);

    void PrintSearchPath(anaSearchStateSpace_t* pSearchStateSpace, FILE* fOut);

    int getHeurValue(anaSearchStateSpace_t* pSearchStateSpace, int StateID);

    //get path
    std::vector<int> GetSearchPath(anaSearchStateSpace_t* pSearchStateSpace, int& solcost);

    bool Search(anaSearchStateSpace_t* pSearchStateSpace, std::vector<int>& pathIds, int & PathCost, bool bFirstSolution,
                bool bOptimalSolution, double MaxNumofSecs);
};

#endif

