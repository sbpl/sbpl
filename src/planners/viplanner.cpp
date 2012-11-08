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
#include <sbpl/planners/viplanner.h>
#include <sbpl/utils/key.h>
#include <sbpl/utils/mdpconfig.h>
#include <sbpl/utils/utils.h>

using namespace std;

static unsigned int g_backups;
static clock_t g_runtime = 0;
static double g_belldelta = INFINITECOST;

VIPlanner::~VIPlanner()
{
    //delete the statespace
}

void VIPlanner::Initialize_vidata(CMDPSTATE* state)
{
    VIState* vi_data = (VIState*)state->PlannerSpecificData;

    vi_data->bestnextaction = NULL;
    vi_data->iteration = 0;
    vi_data->v = (float)environment_->GetGoalHeuristic(state->StateID);
}

CMDPSTATE* VIPlanner::CreateState(int stateID)
{
    CMDPSTATE* state = NULL;

#if DEBUG
    if (environment_->StateID2IndexMapping[stateID][VIMDP_STATEID2IND] != -1) {
        SBPL_ERROR("ERROR in CreateState: state already created\n");
        throw new SBPL_Exception();
    }
#endif

    //adds to the tail a state
    state = viPlanner.MDP.AddState(stateID);

    //remember the index of the state
    environment_->StateID2IndexMapping[stateID][VIMDP_STATEID2IND] = viPlanner.MDP.StateArray.size() - 1;

#if DEBUG
    if (state != viPlanner.MDP.StateArray[environment_->StateID2IndexMapping[stateID][VIMDP_STATEID2IND]]) {
        SBPL_ERROR("ERROR in CreateState: invalid state index\n");
        throw new SBPL_Exception();
    }
#endif

    //create and initialize vi_data
    state->PlannerSpecificData = new VIState;
    Initialize_vidata(state);

    return state;
}

CMDPSTATE* VIPlanner::GetState(int stateID)
{
    if (stateID >= (int)environment_->StateID2IndexMapping.size()) {
        SBPL_ERROR("ERROR int GetState: stateID is invalid\n");
        throw new SBPL_Exception();
    }

    if (environment_->StateID2IndexMapping[stateID][VIMDP_STATEID2IND] == -1)
        return CreateState(stateID);
    else
        return viPlanner.MDP.StateArray[environment_->StateID2IndexMapping[stateID][VIMDP_STATEID2IND]];
}

void VIPlanner::PrintVIData()
{
    SBPL_PRINTF("iteration %d: v(start) = %f\n", viPlanner.iteration,
                ((VIState*)(viPlanner.StartState->PlannerSpecificData))->v);
}

void VIPlanner::PrintStatHeader(FILE* fOut)
{
    SBPL_FPRINTF(fOut, "iteration backups v(start)\n");
}

void VIPlanner::PrintStat(FILE* fOut, clock_t starttime)
{
    SBPL_FPRINTF(fOut, "%d %d %f %f %d\n", viPlanner.iteration, g_backups,
                 ((double)(clock() - starttime)) / CLOCKS_PER_SEC,
                 ((VIState*)(viPlanner.StartState->PlannerSpecificData))->v,
                 (unsigned int)viPlanner.MDP.StateArray.size());
}

void VIPlanner::PrintPolicy(FILE* fPolicy)
{
    bool bPrintStatOnly = true;
    vector<CMDPSTATE*> WorkList;
    CMDP PolicyforEvaluation;

    viPlanner.iteration++;
    WorkList.push_back(viPlanner.StartState);
    ((VIState*)viPlanner.StartState->PlannerSpecificData)->iteration = viPlanner.iteration;
    double PolVal = 0.0;
    double Conf = 0;
    bool bCycles = false;
    SBPL_PRINTF("Printing policy...\n");
    while ((int)WorkList.size() > 0) {
        //pop the last state
        CMDPSTATE* state = WorkList.at(WorkList.size() - 1);
        WorkList.pop_back();
        VIState* statedata = (VIState*)state->PlannerSpecificData;

        CMDPSTATE* polstate = PolicyforEvaluation.AddState(state->StateID);

        //print state ID
        if (!bPrintStatOnly) {
            SBPL_FPRINTF(fPolicy, "%d\n", state->StateID);
            environment_->PrintState(state->StateID, false, fPolicy);

            int h = environment_->GetGoalHeuristic(state->StateID);
            SBPL_FPRINTF(fPolicy, "h=%d\n", h);
            if (h > statedata->v) {
                SBPL_FPRINTF(fPolicy, "WARNING h overestimates exp.cost\n");
            }
        }

        if (state->StateID == viPlanner.GoalState->StateID) {
            //goal state
            if (!bPrintStatOnly) SBPL_FPRINTF(fPolicy, "0\n");
            Conf += ((VIState*)state->PlannerSpecificData)->Pc;
        }
        else if (statedata->bestnextaction == NULL) {
            //unexplored
            if (!bPrintStatOnly) {
                //no outcome explored - stay in the same place
                SBPL_FPRINTF(fPolicy, "%d %d %d\n", 1, 0, state->StateID);
            }
        }
        else {
            //get best action
            CMDPACTION* action = statedata->bestnextaction;

            //add action to evaluation MDP
            CMDPACTION* polaction = polstate->AddAction(action->ActionID);

            if (!bPrintStatOnly) SBPL_FPRINTF(fPolicy, "%d ", (unsigned int)action->SuccsID.size());

            //print successors and insert them into the list
            for (int i = 0; i < (int)action->SuccsID.size(); i++) {
                if (!bPrintStatOnly) SBPL_FPRINTF(fPolicy, "%d %d ", action->Costs[i], action->SuccsID[i]);
                polaction->AddOutcome(action->SuccsID[i], action->Costs[i], action->SuccsProb[i]);

                CMDPSTATE* succstate = GetState(action->SuccsID[i]);
                if ((int)((VIState*)succstate->PlannerSpecificData)->iteration != viPlanner.iteration) {
                    ((VIState*)succstate->PlannerSpecificData)->iteration = viPlanner.iteration;
                    WorkList.push_back(succstate);

                    ((VIState*)succstate->PlannerSpecificData)->Pc = action->SuccsProb[i] *
                                                                     ((VIState*)state->PlannerSpecificData)->Pc;
                    PolVal += ((VIState*)succstate->PlannerSpecificData)->Pc * action->Costs[i];
                }
            }
            if (!bPrintStatOnly) SBPL_FPRINTF(fPolicy, "\n");
        }
    }//while worklist not empty
    SBPL_PRINTF("done\n");

    //now evaluate the policy
    double PolicyValue = -1;
    bool bFullPolicy = false;
    double Pcgoal = -1;
    int nMerges = 0;
    EvaluatePolicy(&PolicyforEvaluation, viPlanner.StartState->StateID, viPlanner.GoalState->StateID, &PolicyValue,
                   &bFullPolicy, &Pcgoal, &nMerges, &bCycles);

    SBPL_PRINTF("Policy value = %f FullPolicy=%d Merges=%d Cycles=%d\n", PolicyValue, bFullPolicy, nMerges, bCycles);

    if (!bFullPolicy) SBPL_PRINTF("WARN: POLICY IS ONLY PARTIAL\n");
    if (fabs(PolicyValue - ((VIState*)(viPlanner.StartState->PlannerSpecificData))->v) > MDP_ERRDELTA) {
        SBPL_PRINTF("WARN: POLICY VALUE IS NOT CORRECT\n");
    }

    if (!bPrintStatOnly)
        SBPL_FPRINTF(fPolicy,
                     "backups=%d runtime=%f vstart=%f policyvalue=%f fullpolicy=%d Pc(goal)=%f nMerges=%d bCyc=%d\n",
                     g_backups, (double)g_runtime / CLOCKS_PER_SEC,
                     ((VIState*)(viPlanner.StartState->PlannerSpecificData))->v, PolicyValue, bFullPolicy, Pcgoal,
                     nMerges, bCycles);
    else
        SBPL_FPRINTF(fPolicy, "%d %f %f %f %d %f %d %d\n", g_backups, (double)g_runtime / CLOCKS_PER_SEC,
                     ((VIState*)(viPlanner.StartState->PlannerSpecificData))->v, PolicyValue, bFullPolicy, Pcgoal,
                     nMerges, bCycles);
}

void VIPlanner::backup(CMDPSTATE* state)
{
    int aind, oind;
    CMDPSTATE* succstate;

    g_backups++;

    if (state == viPlanner.GoalState) {
        ((VIState*)(state->PlannerSpecificData))->bestnextaction = NULL;
        ((VIState*)(state->PlannerSpecificData))->v = 0;
        return;
    }

    //iterate through actions
    double minactionQ = INFINITECOST;
    CMDPACTION* minaction = NULL;
    for (aind = 0; aind < (int)state->Actions.size(); aind++) {
        double actionQ = 0;
        CMDPACTION* action = state->Actions[aind];
        for (oind = 0; oind < (int)action->SuccsID.size(); oind++) {
            succstate = GetState(action->SuccsID[oind]);
            actionQ += action->SuccsProb[oind] * (action->Costs[oind] +
                       ((VIState*)(succstate->PlannerSpecificData))->v);
        }

        if (minaction == NULL || actionQ < minactionQ) {
            minactionQ = actionQ;
            minaction = action;
        }
    }

    if (((VIState*)state->PlannerSpecificData)->bestnextaction == NULL)
        g_belldelta = INFINITECOST;
    else if (g_belldelta < fabs(((VIState*)state->PlannerSpecificData)->v - minactionQ))
        g_belldelta = fabs(((VIState*)state->PlannerSpecificData)->v - minactionQ);

    //set state values
    ((VIState*)state->PlannerSpecificData)->bestnextaction = minaction;
    ((VIState*)state->PlannerSpecificData)->v = (float)minactionQ;
}

void VIPlanner::perform_iteration_backward()
{
    CMDPSTATE* state;
    vector<int> Worklist;
    int aind, oind;

    //initialize the worklist
    Worklist.push_back(viPlanner.GoalState->StateID);

    //backup all the states
    while ((int)Worklist.size() > 0) {
        //get the next state to process
        state = GetState(Worklist[Worklist.size() - 1]);
        Worklist.pop_back();

        //add all actions to the state
        if ((int)state->Actions.size() == 0) environment_->SetAllActionsandAllOutcomes(state);

        //backup the state
        backup(state);

        //insert all the not yet processed successors into the worklist
        for (aind = 0; aind < (int)state->Actions.size(); aind++) {
            CMDPACTION* action = state->Actions[aind];
            for (oind = 0; oind < (int)action->SuccsID.size(); oind++) {
                CMDPSTATE* succstate = GetState(action->SuccsID[oind]);

                //skip if already was in the queue
                if ((int)((VIState*)succstate->PlannerSpecificData)->iteration != viPlanner.iteration) {
                    Worklist.push_back(succstate->StateID);

                    //mark it
                    ((VIState*)succstate->PlannerSpecificData)->iteration = viPlanner.iteration;
                }
            }
        }

        //it is not necessary to process the predecessors of the start state
        if (state == viPlanner.StartState) continue;

        //add all predecessor ids to the state
        if ((int)state->PredsID.size() == 0) environment_->SetAllPreds(state);
        //insert all the not yet processed predecessors into the worklist
        for (int pind = 0; pind < (int)state->PredsID.size(); pind++) {
            CMDPSTATE* PredState = GetState(state->PredsID[pind]);

            //skip if already was in the queue
            if ((int)((VIState*)PredState->PlannerSpecificData)->iteration != viPlanner.iteration) {
                Worklist.push_back(PredState->StateID);

                //mark it
                ((VIState*)PredState->PlannerSpecificData)->iteration = viPlanner.iteration;
            }
        }
    } //until empty worklist
}

void VIPlanner::perform_iteration_forward()
{
    CMDPSTATE* state = NULL;
    vector<CMDPSTATE*> Worklist;
    int aind, oind;

    //initialize the worklist
    Worklist.push_back(viPlanner.StartState);

    //backup all the states
    while ((int)Worklist.size() > 0) {
        //get the next state to process from the front
        state = Worklist[Worklist.size() - 1];
        //Env_PrintState(state->StateID);
        Worklist.pop_back();

        //add all actions to the state
        if ((int)state->Actions.size() == 0) environment_->SetAllActionsandAllOutcomes(state);

        //backup the state
        backup(state);

        //insert all the not yet processed successors into the worklist
        for (aind = 0; aind < (int)state->Actions.size(); aind++) {
            //CMDPACTION* action = state->Actions[aind];
            CMDPACTION* action = ((VIState*)state->PlannerSpecificData)->bestnextaction;
            for (oind = 0; action != NULL && oind < (int)action->SuccsID.size(); oind++) {
                CMDPSTATE* succstate = GetState(action->SuccsID[oind]);

                //skip if already was in the queue
                if ((int)((VIState*)succstate->PlannerSpecificData)->iteration != viPlanner.iteration) {
                    Worklist.push_back(succstate);

                    //mark it
                    ((VIState*)succstate->PlannerSpecificData)->iteration = viPlanner.iteration;
                }
            }
        }
    } //until empty worklist
}

void VIPlanner::InitializePlanner()
{
    viPlanner.iteration = 0;

    //create and set up goal and start states
    viPlanner.StartState = GetState(MDPCfg_->startstateid);
    viPlanner.GoalState = GetState(MDPCfg_->goalstateid);
}

//the planning entry point
//returns 1 if path is found, 0 otherwise
int VIPlanner::replan(double allocatedtime, vector<int>* solution_stateIDs_V)
{
#ifndef ROS
    const char* policy = "policy.txt";
    const char* stat = "stat.txt";
#endif
    FILE* fPolicy = SBPL_FOPEN(policy, "w");
    FILE* fStat = SBPL_FOPEN(stat, "w");

    //initialization
    InitializePlanner();

    //start the timer
    clock_t starttime = clock();

    //--------------iterate-------------------------------
    while (((clock() - starttime) / (double)CLOCKS_PER_SEC) < allocatedtime && g_belldelta > MDP_ERRDELTA) {
        viPlanner.iteration++;

        g_belldelta = 0;
        perform_iteration_forward();

        if (viPlanner.iteration % 100 == 0) {
            PrintStat(stdout, starttime);
            PrintStat(fStat, starttime);
        }
    }
    //------------------------------------------------------------------

    g_runtime = clock() - starttime;

    PrintStat(stdout, starttime);
    PrintStat(fStat, starttime);
    SBPL_FFLUSH(fStat);

    PrintPolicy(fPolicy);

    SBPL_FCLOSE(fPolicy);
    SBPL_FCLOSE(fStat);

    return 1;
}

