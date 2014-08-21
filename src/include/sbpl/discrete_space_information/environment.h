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

#ifndef __ENVIRONMENT_H_
#define __ENVIRONMENT_H_

#include <cstdio>
#include <vector>
#include <sbpl/config.h>
#include <sbpl/sbpl_exception.h>

class CMDPSTATE;
struct MDPConfig;

/**
 * \brief base class for environments defining planning graphs
 * 
 * It is independent of the graph search used
 * The main means of communication between environment and graph search is
 * through stateID.  Each state is uniquely defined by stateID and graph
 * search is ONLY aware of stateIDs. It doesn't know anything about
 * the actual state variables.  Environment, on the * other hand, maintains a
 * mapping from stateID to actual state variables (coordinates) using
 * StateID2IndexMapping array
 */
class DiscreteSpaceInformation
{
public:
    /**
     * \brief mapping from hashentry stateID (used in environment to contain
     *        the coordinates of a state, say x,y or x,y,theta)
     *        to an array of state indices used in searches.
     * 
     * If a single search is done, then it is a single entry.  So
     * StateID2IndexMapping[100][0] = 5 means that hashentry with stateID 100
     * is mapped onto search index = 5 in search 0 The value of -1 means that
     * no search state has been created yet for this hashentry
     */
    std::vector<int*> StateID2IndexMapping;

    /**
     * \brief debugging file
     */
    FILE* fDeb;

    /**
     * \brief initialization environment from file (see .cfg files for examples)
     */
    virtual bool InitializeEnv(const char* sEnvFile) = 0;

    /**
     * \brief initialization of MDP data structure
     */
    virtual bool InitializeMDPCfg(MDPConfig *MDPCfg) = 0;

    /**
     * \brief heuristic estimate from state FromStateID to state ToStateID
     */
    virtual int GetFromToHeuristic(int FromStateID, int ToStateID) = 0;

    /**
     * \brief heuristic estimate from state with stateID to goal state
     */
    virtual int GetGoalHeuristic(int stateID) = 0;

    /**
     * \brief heuristic estimate from start state to state with stateID
     */
    virtual int GetStartHeuristic(int stateID) = 0;

    /** \brief depending on the search used, it may call GetSuccs function
     *         (for forward search) or GetPreds function (for backward search)
     *         or both (for incremental search). At least one of this functions should
     *         be implemented (otherwise, there will be no search to run) Some searches
     *         may also use SetAllActionsandAllOutcomes or SetAllPreds functions if they
     *         keep the pointers to successors (predecessors) but most searches do not
     *         require this, so it is not necessary to support this
     */
    virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV) = 0;

    /**
     * \brief This version is used with lazy planners. The environment must tell which successors have
     *        been evaluated fully (and therefore their true cost is being returned) or if it has not been.
     *        If a successor's cost is not true, then the cost must not overestimate the true cost.
     */
    virtual void GetLazySuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost){
      SBPL_ERROR("ERROR: GetLazySuccs is not implemented for this environment!\n");
      throw new SBPL_Exception();
    };

    /**
     * \brief This version of GetSuccs is needed for E-Graphs. It always returns a unique ID for 
     *        every successor. Generally, this function operates the same as the usual GetSuccs
     *        and only has different behavior when dealing with an underspecified goal condition
     *        so that several different states will have to map to the same id number (most planners
     *        in sbpl use a single goal id number to identify the goal). This function is used 
     *        in conjunction with isGoal.
     */
    virtual void GetSuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV){
      SBPL_ERROR("ERROR: GetSuccsWithUniqueIds is not implemented for this environment!\n");
      throw new SBPL_Exception();
    };

    /**
     * \brief The lazy version of GetSuccsUniqueIds.
     */
    virtual void GetLazySuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost){
      SBPL_ERROR("ERROR: GetLazySuccsWithUniqueIds (lazy) is not implemented for this environment!\n");
      throw new SBPL_Exception();
    };

    /**
     * \brief Used with lazy planners. This evaluates a edge fully that had previously been provided by GetSuccs
     *        without a "true cost". If the edge is found to be invalid, it should return -1
     */
    virtual int GetTrueCost(int parentID, int childID){
      SBPL_ERROR("ERROR: GetTrueCost (used for lazy planning) is not implemented for this environment!\n");
      throw new SBPL_Exception();
      return -1;
    };

    /**
     * \brief This function is generally used with E-Graphs (in conjunction with GetSuccsWithUniqueIds). 
     */
    virtual bool isGoal(int id){
      SBPL_ERROR("ERROR: isGoal is not implemented for this environment!\n");
      throw new SBPL_Exception();
      return false;
    };

    /**
     * \brief see comments for GetSuccs functon
     */
    virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV) = 0;

    /**
     * \brief see comments for GetSuccs functon
     */
    virtual void GetLazyPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost){
      SBPL_ERROR("ERROR: GetLazyPreds is not implemented for this environment!\n");
      throw new SBPL_Exception();
    };

    /**
     * \brief see comments for GetSuccs functon
     */
    virtual void GetPredsWithUniqueIds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV){
      SBPL_ERROR("ERROR: GetPredsWithUniqueIds is not implemented for this environment!\n");
      throw new SBPL_Exception();
    };

    /**
     * \brief see comments for GetSuccs functon
     */
    virtual void GetLazyPredsWithUniqueIds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost){
      SBPL_ERROR("ERROR: GetLazyPredsWithUniqueIds is not implemented for this environment!\n");
      throw new SBPL_Exception();
    };
     
    /**
     * \brief see comments for GetSuccs functon
     */
    virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state) = 0;

    /**
     * \brief see comments for GetSuccs functon
     */
    virtual void SetAllPreds(CMDPSTATE* state) = 0;

    /**
     * \brief returns the number of states (hashentries) created
     */
    virtual int SizeofCreatedEnv() = 0;

    /**
     * \brief prints the state variables for a state with stateID
     */
    virtual void PrintState(int stateID, bool bVerbose, FILE* fOut = NULL) = 0;

    /**
     * \brief prints environment config file
     */
    virtual void PrintEnv_Config(FILE* fOut) = 0;

    /**
     * \brief sets a parameter to a value. The set of supported parameters depends on the particular environment
     */
    virtual bool SetEnvParameter(const char* parameter, int value)
    {
        SBPL_ERROR("ERROR: Environment has no parameters that can be set via SetEnvParameter function\n");
        return false;
    }

    /** NOTE - for debugging door planner - ben 8.31.09 */
    virtual std::vector<int> GetExpandedStates()
    {
        SBPL_ERROR("Error: Not yet defined for any environment other than door environment.\n");
        std::vector<int> list;
        return list;
    }

    /** \brief returns true if two states meet the same condition, brief this
     *         is used in some planners to figure out if two states are the same in
     *         some lower-dimensional manifold for example, in robotarm planning, two
     *         states could be equivalent if their end effectors are at the same
     *         position unless overwritten in a child class, this function is not
     *         implemented
     */
    virtual bool AreEquivalent(int StateID1, int StateID2)
    {
        SBPL_ERROR("ERROR: environment does not support calls to AreEquivalent function\n");
        throw new SBPL_Exception();
    }

    /** \brief the following two functions generate succs/preds at some
     *         domain-dependent distance. The number of generated succs/preds is up
     *         to the environment. NOTE: they MUST generate goal state as a succ/pred if
     *         it is within the distance from the state CLowV is the corresponding
     *         vector of lower bounds on the costs from the state to the successor
     *         states (or vice versa for preds function) unless overwritten in a child
     *         class, this function is not implemented
     */
    virtual void GetRandomSuccsatDistance(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CLowV)
    {
        SBPL_ERROR("ERROR: environment does not support calls to GetRandomSuccsatDistance function\n");
        throw new SBPL_Exception();
    }

    /**
     * \brief see comments for GetRandomSuccsatDistance
     */
    virtual void GetRandomPredsatDistance(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CLowV)
    {
        SBPL_ERROR("ERROR: environment does not support calls to GetRandomPredsatDistance function\n");
        throw new SBPL_Exception();
    }

    /**
     * \brief checks the heuristics for consistency (some environments do not support this debugging call)
     */
    virtual void EnsureHeuristicsUpdated(bool bGoalHeuristics)
    {
        // by default the heuristics are up-to-date, but in some cases, the
        // heuristics are computed only when really needed. For example,
        // xytheta environment uses 2D gridsearch as heuristics, and then
        // re-computes them only when this function is called. This minimizes
        // the number of times heuristics are re-computed which is an expensive
        // operation if bGoalHeuristics == true, then it updates goal
        // heuristics (for forward search), otherwise it updates start
        // heuristics (for backward search)
    }

    /**
     * \brief destructor
     */
    virtual ~DiscreteSpaceInformation()
    {
        SBPL_PRINTF("destroying discretespaceinformation\n");
        for (unsigned int i = 0; i < StateID2IndexMapping.size(); ++i) {
            if (StateID2IndexMapping[i] != NULL) delete[] StateID2IndexMapping[i];
        }
        SBPL_FCLOSE(fDeb);
    }

    /**
     * \brief constructor
     */
    DiscreteSpaceInformation()
    {
#ifndef ROS
        const char* envdebug = "envdebug.txt";
#endif
        if ((fDeb = SBPL_FOPEN(envdebug, "w")) == NULL) {
            SBPL_ERROR("ERROR: failed to open debug file for environment\n");
            throw new SBPL_Exception();
        }
    }
};

#endif

