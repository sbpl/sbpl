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

#ifndef __ENVIRONMENT_ROBARM_H_
#define __ENVIRONMENT_ROBARM_H_

#include <cstdio>
#include <sbpl/discrete_space_information/environment.h>

#define NUMOFLINKS 6
//for R* max. distance in coord to a sample point. It should be exactly this,
//for one of the coordinates and this or smaller for the rest
#define ROBARM_LONGACTIONDIST_CELLS 30   
//# of random successors at distance defined above, used by R* search
#define ROBARM_NUMOFRANDSUCCSATDIST 10		 
//if cleared then the intersection of the whole arm against obstacles 
//and bounds is checked
#define ENDEFF_CHECK_ONLY 0
#define UNIFORM_COST 1	//all the actions have the same costs when set
#define INVALID_NUMBER 999

class CMDPSTATE;
class MDPConfig;

typedef struct
{
    short unsigned int x;
    short unsigned int y;
    bool bIsObstacle;
} CELLV;

//state structure
typedef struct STATE2D_t
{
    unsigned int g;
    short unsigned int iterationclosed;
    short unsigned int x;
    short unsigned int y;
} State2D;

typedef struct ENV_ROBARM_CONFIG
{
    double EnvWidth_m;
    double EnvHeight_m;
    int EnvWidth_c;
    int EnvHeight_c;
    int BaseX_c;
    short unsigned int EndEffGoalX_c;
    short unsigned int EndEffGoalY_c;
    double LinkLength_m[NUMOFLINKS];
    double LinkStartAngles_d[NUMOFLINKS];
    double LinkGoalAngles_d[NUMOFLINKS];
    char** Grid2D;
    double GridCellWidth;

    double angledelta[NUMOFLINKS];
    int anglevals[NUMOFLINKS];
} EnvROBARMConfig_t;

typedef struct ENVROBARMHASHENTRY
{
    int stateID;
    //state coordinates
    short unsigned int coord[NUMOFLINKS];
    short unsigned int endeffx;
    short unsigned int endeffy;
} EnvROBARMHashEntry_t;

typedef struct
{
    EnvROBARMHashEntry_t* goalHashEntry;
    EnvROBARMHashEntry_t* startHashEntry;

    //Maps from coords to stateId
    int HashTableSize;
    std::vector<EnvROBARMHashEntry_t*>* Coord2StateIDHashTable;

    //vector that maps from stateID to coords
    std::vector<EnvROBARMHashEntry_t*> StateID2CoordTable;

    //any additional variables
    int** Heur; //h[fromx][fromy][tox][toy] = Heur[to][from], where to= tox+toy*width_c, from = fromx+fromy*width_c
} EnvironmentROBARM_t;

/**
 * \brief planar kinematic robot arm of variable number of degrees of freedom
 */
class EnvironmentROBARM : public DiscreteSpaceInformation
{
public:
    /**
     * \brief initialize environment from a file (see .cfg files in robotarm directory for example)
     */
    virtual bool InitializeEnv(const char* sEnvFile);

    /**
     * \brief initialize MDP config with IDs of start/goal
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

    ~EnvironmentROBARM() { }

protected:
    //member data
    EnvROBARMConfig_t EnvROBARMCfg;
    EnvironmentROBARM_t EnvROBARM;

    virtual void ComputeContAngles(short unsigned int coord[NUMOFLINKS], double angle[NUMOFLINKS]);
    virtual void ComputeCoord(double angle[NUMOFLINKS], short unsigned int coord[NUMOFLINKS]);
    virtual int ComputeEndEffectorPos(double angles[NUMOFLINKS], short unsigned int* pX, short unsigned int* pY);
    virtual int IsValidCoord(short unsigned int coord[NUMOFLINKS], char** Grid2D = NULL,
                             std::vector<CELLV>* pTestedCells = NULL);
    virtual int distanceincoord(unsigned short* statecoord1, unsigned short* statecoord2);
    virtual void ReInitializeState2D(State2D* state);
    virtual void InitializeState2D(State2D* state, short unsigned int x, short unsigned int y);
    virtual void Search2DwithQueue(State2D** statespace, int* HeurGrid, int searchstartx, int searchstarty);
    virtual void Create2DStateSpace(State2D*** statespace2D);
    virtual void Delete2DStateSpace(State2D*** statespace2D);
    virtual void printangles(FILE* fOut, short unsigned int* coord, bool bGoal, bool bVerbose, bool bLocal);
    virtual void DiscretizeAngles();
    virtual void Cell2ContXY(int x, int y, double *pX, double *pY);
    virtual void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY);
    virtual int IsValidLineSegment(double x0, double y0, double x1, double y1, char **Grid2D,
                                   std::vector<CELLV>* pTestedCells);
    virtual void GetRandomSuccsatDistance(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CLowV);
    virtual unsigned int GetHeurBasedonCoord(short unsigned int coord[NUMOFLINKS]);
    virtual void PrintHeader(FILE* fOut);
    virtual int cost(short unsigned int state1coord[], short unsigned int state2coord[]);

    virtual void ReadConfiguration(FILE* fCfg);

    virtual void InitializeEnvConfig();

    virtual unsigned int GETHASHBIN(short unsigned int* coord, int numofcoord);

    virtual void PrintHashTableHist();

    virtual EnvROBARMHashEntry_t* GetHashEntry(short unsigned int* coord, int numofcoord, bool bIsGoal);

    virtual EnvROBARMHashEntry_t* CreateNewHashEntry(short unsigned int* coord, int numofcoord,
                                                     short unsigned int endeffx, short unsigned int endeffy);

    virtual bool InitializeEnvironment();

    virtual void ComputeHeuristicValues();

    virtual int GetEdgeCost(int FromStateID, int ToStateID);
    virtual int GetRandomState();
    virtual bool AreEquivalent(int State1ID, int State2ID);

    virtual void PrintSuccGoal(int SourceStateID, int costtogoal, bool bVerbose, bool bLocal /*=false*/,
                               FILE* fOut /*=NULL*/);
};

#endif

