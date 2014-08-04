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

#ifndef __ENVIRONMENT_NAV2D_H_
#define __ENVIRONMENT_NAV2D_H_

#include <cstdio>
#include <vector>
#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/utils/utils.h>

#define ENVNAV2D_COSTMULT 1000
#define ENVNAV2D_DEFAULTOBSTHRESH 1 //253-for willow garage	//see explanation of the value below
#define ENVNAV2D_MAXDIRS 16 //TODO-debugmax - crashes for 8 in debug mode

class CMDPSTATE;
class MDPConfig;

//configuration parameters
typedef struct ENV_NAV2D_CONFIG
{
    int EnvWidth_c;
    int EnvHeight_c;
    int StartX_c;
    int StartY_c;
    int EndX_c;
    int EndY_c;
    unsigned char** Grid2D;
    //the value at which and above which cells are obstacles in the maps sent from outside
    //the default is defined above
    unsigned char obsthresh;

    int dx_[ENVNAV2D_MAXDIRS];
    int dy_[ENVNAV2D_MAXDIRS];
    //the intermediate cells through which the actions go 
    int dxintersects_[ENVNAV2D_MAXDIRS][2];
    int dyintersects_[ENVNAV2D_MAXDIRS][2];
    //distances of transitions
    int dxy_distance_mm_[ENVNAV2D_MAXDIRS];

    int numofdirs; //for now either 8 or 16 (default is 8)
} EnvNAV2DConfig_t;

typedef struct ENVHASHENTRY
{
    int stateID;
    int X;
    int Y;
} EnvNAV2DHashEntry_t;

//variables that dynamically change (e.g., array of states, ...)
typedef struct
{
    int startstateid;
    int goalstateid;

    bool bInitialized;

    //hash table of size x_size*y_size. Maps from coords to stateId
    int HashTableSize;
    std::vector<EnvNAV2DHashEntry_t*>* Coord2StateIDHashTable;

    //vector that maps from stateID to coords
    std::vector<EnvNAV2DHashEntry_t*> StateID2CoordTable;

    //any additional variables
} EnvironmentNAV2D_t;

/**
 * \brief 2D (x,y) grid planning problem. For general structure see comments
 *        on parent class DiscreteSpaceInformation
 */
class EnvironmentNAV2D : public DiscreteSpaceInformation
{
public:
    /**
     * \brief see comments on the same function in the parent class
     */
    virtual bool InitializeEnv(const char* sEnvFile);

    /**
     *\brief see comments on the same function in the parent class
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


    virtual void GetLazySuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost){
      GetSuccs(SourceStateID, SuccIDV, CostV);
      isTrueCost->resize(SuccIDV->size(),true);
    };
    virtual void GetSuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV){
      GetSuccs(SourceStateID, SuccIDV, CostV);
    };
    virtual void GetLazySuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost){
      GetLazySuccs(SourceStateID, SuccIDV, CostV, isTrueCost);
    };

    virtual int GetTrueCost(int parentID, int childID){return -1;};//FIXME: this shouldn't ever be called because we always return true cost...

    virtual bool isGoal(int id){
      return EnvNAV2D.goalstateid == id;
    };

    virtual void GetLazyPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost){
      GetPreds(TargetStateID, PredIDV, CostV);
      isTrueCost->resize(PredIDV->size(),true);
    };
    virtual void GetPredsWithUniqueIds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV){
      GetPreds(TargetStateID, PredIDV, CostV);
    };
    virtual void GetLazyPredsWithUniqueIds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost){
      GetLazyPreds(TargetStateID, PredIDV, CostV, isTrueCost);
    };

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

    /**
     * \brief initialize environment. Gridworld is defined as matrix A of
     *        size width by height.
     *        So, internally, it is accessed as A[x][y] with x ranging from 0 to
     *        width-1 and and y from 0 to height-1. Each element in A[x][y] is unsigned
     *        char. A[x][y] = 0 corresponds to fully traversable and cost is just
     *        Euclidean distance. The cost of transition between two neighboring cells
     *        is EuclideanDistance*(max(A[sourcex][sourcey],A[targetx][targety])+1). If
     *        A[x][y] >= obsthresh, then in the above equation it is assumed to be
     *        infinite.  mapdata is a pointer to the values of A. If it is null, then A
     *        is initialized to all zeros. Mapping is: A[x][y] = mapdata[x+y*width]
     *        start/goal are given by startx, starty, goalx,goaly. If they are not
     *        known yet, just set them to 0. Later setgoal/setstart can be executed
     *        finally obsthresh defined obstacle threshold, as mentioned above
     */
    virtual bool InitializeEnv(int width, int height,

    /** if mapdata is NULL the grid is initialized to all freespace */
    const unsigned char* mapdata, int startx, int starty, int goalx, int goaly, unsigned char obsthresh);

    /**
     * \brief a short version of environment initialization. Here start and goal coordinates will be set to 0s
     * 
     * if mapdata is NULL the grid is initialized to all freespace
     */
    virtual bool InitializeEnv(int width, int height, const unsigned char* mapdata, unsigned char obsthresh);

    /**
     * \brief set start location
     */
    virtual int SetStart(int x, int y);

    /**
     * \brief set goal location
     */
    virtual int SetGoal(int x, int y);

    /**
     * \brief currently, this is not used
     */
    virtual void SetGoalTolerance(double tol_x, double tol_y, double tol_theta); /**< not used yet */

    /**
     * \brief update the traversability of a cell<x,y>
     */
    virtual bool UpdateCost(int x, int y, unsigned char newcost);

    /** \brief this function fill in Predecessor/Successor states of edges
     *         whose costs changed
     *         It takes in an array of cells whose traversability changed, and returns
     *         (in vector preds_of_changededgesIDV) the IDs of all states that have
     *         outgoing edges that go through the changed cells
     */
    virtual void GetPredsofChangedEdges(std::vector<nav2dcell_t> const * changedcellsV,
                                        std::vector<int> *preds_of_changededgesIDV);

    /**
     * \brief same as GetPredsofChangedEdges, but returns successor states.
     *        Both functions need to be present for incremental search
     */
    virtual void GetSuccsofChangedEdges(std::vector<nav2dcell_t> const * changedcellsV,
                                        std::vector<int> *succs_of_changededgesIDV);

    /**
     * \brief returns true if two states meet the same condition - see
     *        environment.h for more info
     */
    virtual bool AreEquivalent(int StateID1, int StateID2);

    /**
     * \brief generates succs at some domain-dependent distance - see
     *        environment.h for more info used by certain searches such as R*
     */
    virtual void GetRandomSuccsatDistance(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CLowV);

    /**
     * \brief generates preds at some domain-dependent distance - see
     *        environment.h for more info
     *        used by certain searches such as R*
     */
    virtual void GetRandomPredsatDistance(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CLowV);

    /**
     * \brief generates nNumofNeighs random neighbors of cell <X,Y> at
     *        distance nDist_c (measured in cells)
     *        it will also generate goal if within this distance as an additional
     *        neighbor
     */
    virtual void GetRandomNeighs(int stateID, std::vector<int>* NeighIDV, std::vector<int>* CLowV, int nNumofNeighs,
                                 int nDist_c, bool bSuccs);

    /** 
     * \brief a direct way to set the configuration of environment - see
     *        InitializeEnv function for details about the parameters
     *        it is not a full way to initialize environment. To fully initialize, one
     *        needs to executed InitGeneral in addition.
     *
     * if mapdata is NULL the grid is initialized to all freespace
     */
    virtual void SetConfiguration(int width, int height, const unsigned char* mapdata,
                                  int startx, int starty,
                                  int goalx, int goaly);

    /**
     * \brief performs initialization of environments. It is usually called
     *        in from InitializeEnv.  But if SetConfiguration is used, then one
     *        should call InitGeneral by himself
     */
    virtual bool InitGeneral();

    /**
     * \brief returns the actual <x,y> associated with state of stateID
     */
    virtual void GetCoordFromState(int stateID, int& x, int& y) const;

    /**
     * \brief returns a stateID associated with coordinates <x,y>
     */
    virtual int GetStateFromCoord(int x, int y);

    /**
     * \brief returns true if <x,y> is obstacle (used by the value of this
     *        cell and obsthresh)
     */
    virtual bool IsObstacle(int x, int y);

    /** 
     * \brief returns the cost associated with <x,y> cell, i.e., A[x][y]
     */
    virtual unsigned char GetMapCost(int x, int y);

    /**
     * \brief returns the parameters associated with the current environment.
     *        This is useful for setting up a copy of an environment (i.e., second
     *        planning problem)
     */
    virtual void GetEnvParms(int *size_x, int *size_y, int* startx, int* starty, int* goalx, int* goaly,
                             unsigned char* obsthresh);

    /**
     * \brief way to set up various parameters. For a list of parameters, see
     *        the body of the function - it is pretty straightforward
     */
    virtual bool SetEnvParameter(const char* parameter, int value);

    /**
     * \brief access to internal configuration data structure
     */
    virtual const EnvNAV2DConfig_t* GetEnvNavConfig();

    EnvironmentNAV2D();
    ~EnvironmentNAV2D();

    /**
     * \brief print some time statistics
     */
    virtual void PrintTimeStat(FILE* fOut);

    /**
     * \brief checks X,Y against map boundaries
     */
    virtual bool IsWithinMapCell(int X, int Y);

protected:
    //member data
    EnvNAV2DConfig_t EnvNAV2DCfg;
    EnvironmentNAV2D_t EnvNAV2D;

    virtual void ReadConfiguration(FILE* fCfg);

    virtual void InitializeEnvConfig();

    virtual unsigned int GETHASHBIN(unsigned int X, unsigned int Y);

    virtual void PrintHashTableHist();

    virtual EnvNAV2DHashEntry_t* GetHashEntry(int X, int Y);

    virtual EnvNAV2DHashEntry_t* CreateNewHashEntry(int X, int Y);

    virtual void InitializeEnvironment();

    virtual void ComputeHeuristicValues();

    virtual bool IsValidCell(int X, int Y);

    virtual void Computedxy();
};

#endif

