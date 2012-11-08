/*
 * Copyright (c) 2009, Maxim Likhachev
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

#ifndef __PPCPPLANNER_H_
#define __PPCPPLANNER_H_

#include <cstdio>
#include <sbpl/sbpl_exception.h>
#include <sbpl/planners/planner.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/utils.h>

class DiscreteSpaceInformation;
class StateChangeQuery;

/**
 * \brief a state for PPCP
 */
typedef class PPCPPLANNERSTATEDATA : public AbstractSearchState
{
public:
    /**
     * \brief the MDP state itself
     */
    CMDPSTATE* MDPstate;
    /**
     * \brief planner relevant data
     */
    int v;
    /**
     * \brief planner relevant data
     */
    unsigned int iteration;

    /**
     * \brief best action
     */
    CMDPACTION *bestnextaction;

private:
    //probability of reaching this state (intermediate variable used by the algorithm)
    float Pc;

public:
    PPCPPLANNERSTATEDATA() { }
    ~PPCPPLANNERSTATEDATA() { }
} PPCPState;

/**
 * \brief PPCP statespace
 */
typedef struct PPCPSTATESPACE
{
    /**
     * \brief MDP
     */
    CMDP MDP;
    /**
     * \brief pointer to start state
     */
    CMDPSTATE* StartState;
    /**
     * \brief pointer to goal state
     */
    CMDPSTATE* GoalState;

    int iteration;
    int searchiteration;

    //TODO - vector<PolicyFullState_t*> CurrentPolicy; //current policy
    double currentpolicyconfidence;

    /**
     * \brief set when it is necessary to reset the planner
     */
    bool bReinitializeSearchStateSpace;
} PPCPStateSpace_t;

/**
 * \brief PPCP planner
 *        in explanations, S signifies a fully observable part of the state space H
 *        signifies hidden variables
 */
class PPCPPlanner : public SBPLPlanner
{
public:
    /**
     * \brief planning (replanning) function. Takes in time available for
     *        planning
     *
     * returns policy, expected cost of the solution policy, and probability of
     * successfully reaching the goal (it is < 1, whenever PPCP ran out of time
     * before full convergence
     */
    int replan(double allocated_time_secs, std::vector<sbpl_PolicyStatewithBinaryh_t>* SolutionPolicy,
               float* ExpectedCost, float* ProbofReachGoal);

    /**
     * \brief constructors
     */
    PPCPPlanner(DiscreteSpaceInformation* environment, int sizeofS, int sizeofH);

    /**
     * \brief destructor
     */
    ~PPCPPlanner();

    /**
     * \brief setting goal state in S
     */
    int set_goal(int goal_stateID);

    /**
     * \brief setting start state in S
     */
    int set_start(int start_stateID);

    /**
     * \brief not supported version of replan
     */
    int replan(double allocated_time_sec, std::vector<int>* solution_stateIDs_V)
    {
        SBPL_ERROR("ERROR: this version of replan not supported in PPCP planner\n");
        throw new SBPL_Exception();
    }

    /**
     * \brief not supported version of replan
     */
    int replan(double allocated_time_sec, std::vector<int>* solution_stateIDs_V, int* solcost)
    {
        SBPL_ERROR("ERROR: this version of replan not supported in PPCP planner\n");
        throw new SBPL_Exception();
    }

    /**
     * \brief forgets previous planning efforts and starts planning from scratch next time replan is called
     */
    int force_planning_from_scratch();

    /**
     * \brief sets how to search - not supported in PPCP
     */
    int set_search_mode(bool bSearchUntilFirstSolution)
    {
        SBPL_ERROR("ERROR: set_search_mode not supported in PPCP planner\n");
        throw new SBPL_Exception();
    }

    /**
     * \brief Notifies the planner that costs have changed. May need to be
     *        specialized for different subclasses in terms of what to
     *        do here
     */
    void costs_changed(StateChangeQuery const & stateChange);

    /**
     * \brief notifies the planner that costs have changed
     */
    void costs_changed();

private:
    //member variables
    PPCPStateSpace_t* pStateSpace;
    FILE* fDeb;

    //deallocates memory used by SearchStateSpace
    void DeleteStateSpace(PPCPStateSpace_t* pStateSpace);
    int CreateSearchStateSpace(PPCPStateSpace_t* pStateSpace);
};

#endif
