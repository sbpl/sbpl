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

//MDP.cpp - contains all the functions for MDP classes
#include <cmath>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/utils.h>

using namespace std;

//-------------------------MDPACTION class functions----------

bool CMDPACTION::Delete()
{
    SuccsID.clear();
    Costs.clear();
    SuccsProb.clear();
    return true;
}

bool CMDPACTION::DeleteAllOutcomes()
{
    SuccsID.clear();
    Costs.clear();
    SuccsProb.clear();
    return true;
}

bool CMDPACTION::IsValid()
{
    float Prob = 0;
    for (int i = 0; i < (int)SuccsProb.size(); i++) {
        Prob += SuccsProb[i];
    }

    return (fabs(Prob - 1.0) < EPS_ERROR);
}

void CMDPACTION::AddOutcome(int OutcomeStateID, int OutcomeCost, float OutcomeProb)
{
#if MEM_CHECK
    DisableMemCheck();
#endif

    SuccsID.push_back(OutcomeStateID);
    Costs.push_back(OutcomeCost);
    SuccsProb.push_back(OutcomeProb);

#if MEM_CHECK
    EnableMemCheck();
#endif

}

void CMDPACTION::operator =(const CMDPACTION& rhsaction)
{
    this->ActionID = rhsaction.ActionID;
}

int CMDPACTION::GetIndofMostLikelyOutcome()
{
    double HighestProb = 0;
    int mlind = -1;

    for (int oind = 0; oind < (int)this->SuccsID.size(); oind++) {
        if (this->SuccsProb[oind] >= HighestProb) {
            mlind = oind;
            HighestProb = this->SuccsProb[oind];
        }
    }

    return mlind;
}

int CMDPACTION::GetIndofOutcome(int OutcomeID)
{
    for (int oind = 0; oind < (int)this->SuccsID.size(); oind++) {
        if (this->SuccsID[oind] == OutcomeID) {
            return oind;
        }
    }

    return -1;
}

//----------------------------------------------------------------------

//----------------------------MDPSTATE class functions---------------

bool CMDPSTATE::Delete()
{
    CMDPACTION* action;

    if (this->PlannerSpecificData != NULL) {
        SBPL_ERROR("ERROR deleting state: planner specific data is not deleted\n");
        throw new SBPL_Exception();
    }

    //delete predecessors array
    PredsID.clear();

    //delete actions array
    while ((int)Actions.size() > 0) {
        action = Actions[Actions.size() - 1];
        Actions.pop_back();

        action->Delete();
        delete action;
    }

    return true;
}

CMDPACTION* CMDPSTATE::AddAction(int ID)
{
    CMDPACTION* action = new CMDPACTION(ID, this->StateID);

#if MEM_CHECK
    DisableMemCheck();
#endif

    Actions.push_back(action);

#if MEM_CHECK
    EnableMemCheck();
#endif

    return action;
}

bool CMDPSTATE::AddPred(int stateID)
{
    //add the predecessor
    if (!ContainsPred(stateID)) {
#if MEM_CHECK
        DisableMemCheck();
#endif

        PredsID.push_back(stateID);
#if MEM_CHECK
        EnableMemCheck();
#endif
    }

    return true;
}

bool CMDPSTATE::RemovePred(int stateID)
{
    for (int i = 0; i < (int)this->PredsID.size(); i++) {
        if (this->PredsID.at(i) == stateID) {
            this->PredsID.at(i) = this->PredsID.at(this->PredsID.size() - 1);
            this->PredsID.pop_back();
            return true;
        }
    }

    //can happen when a state is twice a successor
    //SBPL_ERROR("ERROR in RemovePred: no Pred is found\n");
    //throw new SBPL_Exception();

    return false;
}

//requires the deletion of Preds elsewhere
bool CMDPSTATE::RemoveAllActions()
{
    CMDPACTION* action;

    //delete actions array
    while ((int)Actions.size() > 0) {
        action = Actions[Actions.size() - 1];
        Actions.pop_back();

        action->Delete();
        delete action;
    }

    return true;
}

bool CMDPSTATE::ContainsPred(int stateID)
{
    for (int i = 0; i < (int)PredsID.size(); i++) {
        if (PredsID[i] == stateID) return true;
    }
    return false;
}

void CMDPSTATE::operator =(const CMDPSTATE& rhsstate)
{
    this->StateID = rhsstate.StateID;
}

CMDPACTION* CMDPSTATE::GetAction(int actionID)
{
    for (int i = 0; i < (int)Actions.size(); i++) {
        if (Actions[i]->ActionID == actionID) return Actions[i];
    }

    return NULL;
}

//-------------------------------------------------------------------------

//-------------MDP class functions--------------------------------

bool CMDP::empty()
{
    return ((int)StateArray.size() == 0);
}

bool CMDP::full()
{
    return ((int)StateArray.size() >= MAXSTATESPACESIZE);
}

//creates numofstates states. Their ids must be initialized elsewhere
bool CMDP::Create(int numofstates)
{
    CMDPSTATE* state;

    if (numofstates > MAXSTATESPACESIZE) {
        SBPL_ERROR("ERROR in Create: maximum MDP size is reached\n");
        throw new SBPL_Exception();
    }

    for (int i = 0; i < numofstates; i++) {
        state = new CMDPSTATE(-1);

#if MEM_CHECK
        DisableMemCheck();
#endif
        StateArray.push_back(state);
#if MEM_CHECK
        EnableMemCheck();
#endif

    }

    return true;
}

//Adds a new state The id must be initialized elsewhere
CMDPSTATE* CMDP::AddState(int StateID)
{
    CMDPSTATE* state;

    if ((int)StateArray.size() + 1 > MAXSTATESPACESIZE) {
        SBPL_ERROR("ERROR: maximum of states is reached in MDP\n");
        throw new SBPL_Exception();
    }

    state = new CMDPSTATE(StateID);

#if MEM_CHECK
    DisableMemCheck();
#endif
    StateArray.push_back(state);
#if MEM_CHECK
    EnableMemCheck();
#endif

    return state;
}

bool CMDP::Delete()
{
    CMDPSTATE* state;

    while ((int)StateArray.size() > 0) {
        state = StateArray[StateArray.size() - 1];
        StateArray.pop_back();

        state->Delete();
        delete state;
    }

    return true;
}

void CMDP::Print(FILE* fOut)
{
    SBPL_FPRINTF(fOut, "MDP statespace size=%d\n", (unsigned int)StateArray.size());
    for (int i = 0; i < (int)StateArray.size(); i++) {
        SBPL_FPRINTF(fOut, "%d: ", StateArray[i]->StateID);
        for (int j = 0; j < (int)StateArray[i]->Actions.size(); j++) {
            CMDPACTION* action = StateArray[i]->Actions[j];
            SBPL_FPRINTF(fOut, "[%d", action->ActionID);
            for (int outind = 0; outind < (int)action->SuccsID.size(); outind++) {
                SBPL_FPRINTF(fOut, " %d %d %f", action->SuccsID[outind], action->Costs[outind],
                             action->SuccsProb[outind]);
            }
            SBPL_FPRINTF(fOut, "] ");
        }
        SBPL_FPRINTF(fOut, "\n");
    }
}

//--------------------------------------------------------

//----------------other functions-------------------------
