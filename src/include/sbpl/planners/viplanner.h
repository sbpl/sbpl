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

#ifndef __VIPLANNER_H_
#define __VIPLANNER_H_

#include <cstdio>
#include <ctime>
#include <sbpl/planners/planner.h>
#include <sbpl/utils/mdp.h>

#define MDP_ERRDELTA 0.01

class DiscreteSpaceInformation;
class MDPConfig;

struct VIPLANNER_T
{
    CMDP MDP;
    CMDPSTATE* StartState;
    CMDPSTATE* GoalState;
    int iteration;
};

/**
 * \brief Value Iteration state
 */
typedef class VIPLANNERSTATEDATA : public AbstractSearchState
{
public:
    /**
     * \brief the MDP state itself
     */
    CMDPSTATE* MDPstate;
    /**
     * \brief planner relevant data
     */
    float v;
    float Pc;
    unsigned int iteration;

    /**
     * \brief best action
     */
    CMDPACTION *bestnextaction;

public:
    VIPLANNERSTATEDATA() { }
    ~VIPLANNERSTATEDATA() { }
} VIState;

/**
 * \brief value iteration planner
 */
class VIPlanner : public SBPLPlanner
{
public:
    /**
     * \brief replan a path within the allocated time, return the policy in the solution vector
     */
    virtual int replan(double allocated_time_secs, std::vector<int>* solution_stateIDs_V);

    /**
     * \brief constructors
     */
    VIPlanner(DiscreteSpaceInformation* environment, MDPConfig* MDP_cfg)
    {
        environment_ = environment;
        MDPCfg_ = MDP_cfg;
    }

    /**
     * \brief destructor
     */
    ~VIPlanner();

protected:
    //member variables
    MDPConfig* MDPCfg_;
    VIPLANNER_T viPlanner;

    virtual void Initialize_vidata(CMDPSTATE* state);

    virtual CMDPSTATE* CreateState(int stateID);

    virtual CMDPSTATE* GetState(int stateID);

    virtual void PrintVIData();

    virtual void PrintStatHeader(FILE* fOut);

    virtual void PrintStat(FILE* fOut, clock_t starttime);

    virtual void PrintPolicy(FILE* fPolicy);

    virtual void backup(CMDPSTATE* state);

    virtual void perform_iteration_backward();

    virtual void perform_iteration_forward();

    virtual void InitializePlanner();
};

#endif
