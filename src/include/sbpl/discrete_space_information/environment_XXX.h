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

#ifndef __ENVIRONMENT_XXX_H_
#define __ENVIRONMENT_XXX_H_

#include <cstdio>
#include <vector>
#include <sbpl/discrete_space_information/environment.h>

#define XXX_MAXACTIONSWIDTH 9		

class CMDPACTION;
class CMDPSTATE;
class MDPConfig;

typedef struct ENV_XXX_CONFIG
{
    //parameters that are read from the configuration file
    unsigned int StartX1;
    unsigned int StartX2;
    unsigned int StartX3;
    unsigned int StartX4;
    unsigned int GoalX1;
    unsigned int GoalX2;
    unsigned int GoalX3;
    unsigned int GoalX4;

    //derived and initialized elsewhere parameters
} EnvXXXConfig_t;

typedef struct ENVXXXHASHENTRY
{
    int stateID;
    unsigned int X1;
    unsigned int X2;
    unsigned int X3;
    unsigned int X4;
} EnvXXXHashEntry_t;

typedef struct
{
    int startstateid;
    int goalstateid;

    //hash table of size x_size*y_size. Maps from coords to stateId
    int HashTableSize;
    std::vector<EnvXXXHashEntry_t*>* Coord2StateIDHashTable;

    //vector that maps from stateID to coords
    std::vector<EnvXXXHashEntry_t*> StateID2CoordTable;

    //any additional variables
} EnvironmentXXX_t;

/** \brief this is just an example of environment and can be used (copy and
 *         paste) for creating a more complex environment
 */
class EnvironmentXXX : public DiscreteSpaceInformation
{
public:
    /**
     * \brief see comments on the same function in the parent class
     */
    virtual bool InitializeEnv(const char* sEnvFile);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual bool InitializeMDPCfg(MDPConfig *MDPCfg);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual int GetFromToHeuristic(int FromStateID, int ToStateID);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual int GetGoalHeuristic(int stateID);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual int GetStartHeuristic(int stateID);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual void SetAllPreds(CMDPSTATE* state);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual int SizeofCreatedEnv();

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual void PrintState(int stateID, bool bVerbose, FILE* fOut = NULL);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual void PrintEnv_Config(FILE* fOut);

    ~EnvironmentXXX() { }

protected:
    //member variables
    EnvXXXConfig_t EnvXXXCfg;
    EnvironmentXXX_t EnvXXX;

    virtual void ReadConfiguration(FILE* fCfg);

    virtual void InitializeEnvConfig();

    virtual unsigned int GETHASHBIN(unsigned int X1, unsigned int X2, unsigned int X3, unsigned int X4);

    virtual void PrintHashTableHist();

    virtual EnvXXXHashEntry_t* GetHashEntry(unsigned int X1, unsigned int X2, unsigned int X3, unsigned int X4);

    virtual EnvXXXHashEntry_t* CreateNewHashEntry(unsigned int X1, unsigned int X2, unsigned int X3, unsigned int X4);

    virtual void CreateStartandGoalStates();

    virtual void InitializeEnvironment();

    virtual void AddAllOutcomes(unsigned int SourceX1, unsigned int SourceX2, unsigned int SourceX3,
                                unsigned int SourceX4, CMDPACTION* action, int cost);

    virtual void ComputeHeuristicValues();
};

#endif

