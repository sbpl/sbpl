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

#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/planners/ppcpplanner.h>

using namespace std;

//-----------------------------------------------------------------------------------------------------

PPCPPlanner::PPCPPlanner(DiscreteSpaceInformation* environment, int sizeofS, int sizeofH)
{
    environment_ = environment;

#ifndef ROS
    const char* debug = "debug.txt";
#endif
    fDeb = SBPL_FOPEN(debug, "w");
    if (fDeb == NULL) {
        SBPL_ERROR("ERROR: could not open planner debug file\n");
        throw new SBPL_Exception();
    }

    pStateSpace = new PPCPStateSpace_t;

    //TODO - create and initialize the state-space
}

PPCPPlanner::~PPCPPlanner()
{
    if (pStateSpace != NULL) {
        //delete the statespace
        DeleteStateSpace( pStateSpace);
        delete pStateSpace;
        pStateSpace = NULL;
    }
    SBPL_FCLOSE( fDeb);
}

//deallocates memory used by StateSpace
void PPCPPlanner::DeleteStateSpace(PPCPStateSpace_t* pStateSpace)
{
    //TODO - fill in
}

//creates (allocates memory) search state space
//does not initialize search statespace
int PPCPPlanner::CreateSearchStateSpace(PPCPStateSpace_t* pStateSpace)
{
    //create a heap
    pStateSpace->bReinitializeSearchStateSpace = true;
    pStateSpace->currentpolicyconfidence = 0;
    pStateSpace->GoalState = NULL;
    pStateSpace->StartState = NULL;
    pStateSpace->iteration = 0;
    pStateSpace->searchiteration = 0;

    return 1;
}

//--------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------

//setting start state in S
int PPCPPlanner::set_goal(int goal_stateID)
{
    //TODO
    return 1;
}

//setting goal state in S
int PPCPPlanner::set_start(int start_stateID)
{
    //TODO
    return 1;
}

void PPCPPlanner::costs_changed(StateChangeQuery const & stateChange)
{
    SBPL_PRINTF("planner: costs_changed, state-space reset\n");

    pStateSpace->bReinitializeSearchStateSpace = true;
}

void PPCPPlanner::costs_changed()
{
    SBPL_PRINTF("planner: costs_changed, state-space reset\n");

    pStateSpace->bReinitializeSearchStateSpace = true;
}

int PPCPPlanner::force_planning_from_scratch()
{
    SBPL_PRINTF("planner: forceplanfromscratch set, state-space reset\n");

    pStateSpace->bReinitializeSearchStateSpace = true;

    return 1;
}

int PPCPPlanner::replan(double allocated_time_secs, vector<sbpl_PolicyStatewithBinaryh_t>* SolutionPolicy,
                        float* ExpectedCost, float* ProbofReachGoal)
{
    //TODO
    return 0;
}

//-------------------------------------------------------------------------------------------------

