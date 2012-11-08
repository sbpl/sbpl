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

#ifndef __ENVIRONMENT_NAV2DUU_H_
#define __ENVIRONMENT_NAV2DUU_H_

#include <vector>
#include <cstdio>
#include <sbpl/config.h>
#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/discrete_space_information/environment_nav2D.h>
#include <sbpl/sbpl_exception.h>
#include <sbpl/utils/utils.h>

#define ENVNAV2DUU_COSTMULT 1000
#define NAV2DUU_MAXACTIONSWIDTH 9		
#define ENVNAV2DUU_MAXDIRS 8

class CMDPACTION;
class CMDPSTATE;
class MDPConfig;

typedef struct ENV_NAV2DUU_CONFIG
{
    //parameters that are read from the configuration file
    int EnvWidth_c;
    int EnvHeight_c;
    int StartX_c;
    int StartY_c;
    int EndX_c;
    int EndY_c;
    //cost matrix
    unsigned char** Grid2D;
    //the value at which and above which cells are obstacles in the maps sent from outside
    //the default is defined above
    unsigned char obsthresh;
    //uncertainty matrix (0 defines P(obstacle) = 0, and 1.0 defines P(obstacle) = 1)
    float** UncertaintyGrid2D;
    //matrix of hidden variable IDs
    int** HiddenVariableXY2ID;

    //derived and initialized elsewhere parameters

    //possible transitions
    int dx_[ENVNAV2DUU_MAXDIRS];
    int dy_[ENVNAV2DUU_MAXDIRS];
    //distances of transitions
    int dxy_distance_mm_[ENVNAV2DUU_MAXDIRS];
    //the intermediate cells through which the actions go 
    int dxintersects_[ENVNAV2D_MAXDIRS][2];
    int dyintersects_[ENVNAV2D_MAXDIRS][2];
    int numofdirs; //for now can only be 8

    //size of environment, number of hidden variables
    int sizeofS;
    int sizeofH;
} EnvNAV2DUUConfig_t;

#define NAVNAV2DUU_MAXWIDTHHEIGH 1024
#define ENVNAV2DUU_STATEIDTOY(stateID) (stateID%NAVNAV2DUU_MAXWIDTHHEIGH)
#define ENVNAV2DUU_STATEIDTOX(stateID) (stateID/NAVNAV2DUU_MAXWIDTHHEIGH)
#define ENVNAV2DUU_XYTOSTATEID(X, Y) (X*NAVNAV2DUU_MAXWIDTHHEIGH + Y)

typedef struct
{
    int startstateid;
    int goalstateid;

    //any additional variables
    bool bInitialized;
} EnvironmentNAV2DUU_t;

/**
 * \brief this class is NOT fully yet implemented, please do not use it!
 */
class EnvironmentNAV2DUU : public DiscreteSpaceInformation
{
public:
    /**
     * \brief see comments on the same function in the parent class
     */
    virtual bool InitializeEnv(const char* sEnvFile);

    /**
     * \brief initialize environment. Gridworld is defined as matrix A of
     *        size width by height.
     *        So, internally, it is accessed as A[x][y] with x ranging from 0 to
     *        width-1 and and y from 0 to height-1. Each element in A[x][y] is unsigned
     *        char. A[x][y] = 0 corresponds to fully traversable and cost is just
     *        Euclidean distance. The cost of transition between two neighboring cells
     *        is EuclideanDistance*(max(A[sourcex][sourcey],A[targetx][targety])+1). If
     *        A[x][y] >= obsthresh, then in the above equation it is assumed to be
     *        infinite. mapdata is a pointer to the values of A. If it is null, then A
     *        is initialized to all zeros. Mapping is: A[x][y] = mapdata[x+y*width]
     *        start/goal are given by startx, starty, goalx,goaly. If they are not
     *        known yet, just set them to 0. Later setgoal/setstart can be executed
     *        finally obsthresh defined obstacle threshold, as mentioned above
     *        uncertaintymapdata is set up in the same way as mapdata in terms of the
     *        order in terms of the values, uncertaintymapdata specifies probabilities
     *        of being obstructed
     */
    virtual bool InitializeEnv(int width, int height, const unsigned char* mapdata, const float* uncertaintymapdata,
                               unsigned char obsthresh);

    /**
     * \brief set start location
     */
    virtual int SetStart(int x, int y);

    /**
     * \brief set goal location
     */
    virtual int SetGoal(int x, int y);

    /**
     * \brief update the traversability of a cell<x,y>
     */
    virtual bool UpdateCost(int x, int y, unsigned char newcost);

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
    virtual void PrintState(int stateID, bool bVerbose, FILE* fOut = NULL);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual void PrintEnv_Config(FILE* fOut);

    EnvironmentNAV2DUU();

    ~EnvironmentNAV2DUU()
    {
    }

    /**
     * \brief not fully implemented yet
     */
    virtual void GetPreds(int stateID, const std::vector<sbpl_BinaryHiddenVar_t>* updatedhvaluesV,
                          std::vector<CMDPACTION>* IncomingDetActionV, std::vector<CMDPACTION>* IncomingStochActionV,
                          std::vector<sbpl_BinaryHiddenVar_t>* StochActionNonpreferredOutcomeV);

    /**
     * \brief not fully implemented yet
     */
    virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state)
    {
        SBPL_ERROR("ERROR: SetAllActionsandAllOutcomes not supported in NAV2D UNDER UNCERTAINTY\n");
        throw new SBPL_Exception();
    }

    /**
     * \brief not fully implemented yet
     */
    virtual void SetAllPreds(CMDPSTATE* state)
    {
        SBPL_ERROR("ERROR: SetAllPreds not supported in NAV2D UNDER UNCERTAINTY\n");
        throw new SBPL_Exception();
    }

    /**
     * \brief not fully implemented yet
     */
    virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV)
    {
        SBPL_ERROR("ERROR: GetSuccs not supported in NAV2D UNDER UNCERTAINTY\n");
        throw new SBPL_Exception();
    }

    /**
     * \brief not fully implemented yet
     */
    virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV)
    {
        SBPL_ERROR("ERROR: GetPreds not supported in NAV2D UNDER UNCERTAINTY\n");
        throw new SBPL_Exception();
    }

    /**
     * \brief not fully implemented yet
     */
    virtual int SizeofCreatedEnv();

    /**
     * \brief not fully implemented yet
     */
    virtual int SizeofH();

protected:
    //member variables
    EnvNAV2DUUConfig_t EnvNAV2DUUCfg;
    EnvironmentNAV2DUU_t EnvNAV2DUU;

    //mapdata and uncertaintymapdata is assumed to be organized into a linear array with y being major: map[x+y*width]
    virtual void SetConfiguration(int width, int height, const unsigned char* mapdata, const float* uncertaintymapdata);

    virtual void ReadConfiguration(FILE* fCfg);
    virtual void InitializeEnvConfig();
    virtual void InitializeEnvironment();
    virtual void ComputeHeuristicValues();
    virtual bool InitGeneral();

    virtual bool IsValidRobotPosition(int X, int Y);
    virtual bool IsWithinMapCell(int X, int Y);

    virtual void Computedxy();
};

#endif

