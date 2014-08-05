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

#ifndef __ENVIRONMENT_NAVXYTHETALAT_H_
#define __ENVIRONMENT_NAVXYTHETALAT_H_

#include <cstdio>
#include <vector>
#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/utils/utils.h>

//eight-connected grid
#define NAVXYTHETALAT_DXYWIDTH 8
#define ENVNAVXYTHETALAT_DEFAULTOBSTHRESH 254	//see explanation of the value below
//maximum number of states for storing them into lookup (as opposed to hash)
#define SBPL_XYTHETALAT_MAXSTATESFORLOOKUP 100000000 
//definition of theta orientations
//0 - is aligned with X-axis in the positive direction (1,0 in polar coordinates)
//theta increases as we go counterclockwise
//number of theta values - should be power of 2
#define NAVXYTHETALAT_THETADIRS 16
//number of actions per x,y,theta state
//decrease, increase, same angle while moving plus decrease, increase angle while standing.
#define NAVXYTHETALAT_DEFAULT_ACTIONWIDTH 5 
#define NAVXYTHETALAT_COSTMULT_MTOMM 1000

class CMDPSTATE;
class MDPConfig;
class SBPL2DGridSearch;

typedef struct
{
    unsigned char aind; //index of the action (unique for given starttheta)
    char starttheta;
    char dX;
    char dY;
    char endtheta;
    unsigned int cost;
    std::vector<sbpl_2Dcell_t> intersectingcellsV;
    //start at 0,0,starttheta and end at endcell in continuous domain with half-bin less to account for 0,0 start
    std::vector<sbpl_xy_theta_pt_t> intermptV;
    //start at 0,0,starttheta and end at endcell in discrete domain
    std::vector<sbpl_xy_theta_cell_t> interm3DcellsV;
} EnvNAVXYTHETALATAction_t;

typedef struct
{
    int stateID;
    int X;
    int Y;
    char Theta;
    int iteration;
} EnvNAVXYTHETALATHashEntry_t;

typedef struct
{
    int motprimID;
    unsigned char starttheta_c;
    int additionalactioncostmult;
    sbpl_xy_theta_cell_t endcell;
    //intermptV start at 0,0,starttheta and end at endcell in continuous
    //domain with half-bin less to account for 0,0 start
    std::vector<sbpl_xy_theta_pt_t> intermptV;
} SBPL_xytheta_mprimitive;

//variables that dynamically change (e.g., array of states, ...)
typedef struct
{
    int startstateid;
    int goalstateid;

    bool bInitialized;

    //any additional variables
} EnvironmentNAVXYTHETALAT_t;

//configuration parameters
typedef struct ENV_NAVXYTHETALAT_CONFIG
{
    int EnvWidth_c;
    int EnvHeight_c;
    int NumThetaDirs;
    int StartX_c;
    int StartY_c;
    int StartTheta;
    int EndX_c;
    int EndY_c;
    int EndTheta;
    unsigned char** Grid2D;

    // the value at which and above which cells are obstacles in the maps sent from outside
    // the default is defined above
    unsigned char obsthresh;

    // the value at which and above which until obsthresh (not including it)
    // cells have the nearest obstacle at distance smaller than or equal to
    // the inner circle of the robot. In other words, the robot is definitely
    // colliding with the obstacle, independently of its orientation
    // if no such cost is known, then it should be set to obsthresh (if center
    // of the robot collides with obstacle, then the whole robot collides with
    // it independently of its rotation)
    unsigned char cost_inscribed_thresh;

    // the value at which and above which until cost_inscribed_thresh (not including it) cells
    // **may** have a nearest osbtacle within the distance that is in between
    // the robot inner circle and the robot outer circle
    // any cost below this value means that the robot will NOT collide with any
    // obstacle, independently of its orientation
    // if no such cost is known, then it should be set to 0 or -1 (then no cell
    // cost will be lower than it, and therefore the robot's footprint will
    // always be checked)
    int cost_possibly_circumscribed_thresh; // it has to be integer, because -1 means that it is not provided.

    double nominalvel_mpersecs;
    double timetoturn45degsinplace_secs;
    double cellsize_m;

    int dXY[NAVXYTHETALAT_DXYWIDTH][2];

    //array of actions, ActionsV[i][j] - jth action for sourcetheta = i
    EnvNAVXYTHETALATAction_t** ActionsV; 
    //PredActionsV[i] - vector of pointers to the actions that result in a state with theta = i
    std::vector<EnvNAVXYTHETALATAction_t*>* PredActionsV; 

    int actionwidth; //number of motion primitives
    std::vector<SBPL_xytheta_mprimitive> mprimV;

    std::vector<sbpl_2Dpt_t> FootprintPolygon;
} EnvNAVXYTHETALATConfig_t;

class EnvNAVXYTHETALAT_InitParms
{
public:
    unsigned int numThetas;
    const unsigned char* mapdata;
    double startx;
    double starty;
    double starttheta;
    double goalx;
    double goaly;
    double goaltheta;
    double goaltol_x;
    double goaltol_y;
    double goaltol_theta;
};

/** \brief 3D (x,y,theta) planning using lattice-based graph problem. For
 *         general structure see comments on parent class DiscreteSpaceInformation
 *         For info on lattice-based planning used here, you can check out the paper:
 *         Maxim Likhachev and Dave Ferguson, " Planning Long Dynamically-Feasible
 *         Maneuvers for Autonomous Vehicles", IJRR'09
 */
class EnvironmentNAVXYTHETALATTICE : public DiscreteSpaceInformation
{
public:
    EnvironmentNAVXYTHETALATTICE();

    /**
     * \brief initialization of environment from file. See .cfg files for
     *        examples it also takes the perimeter of the robot with respect to some
     *        reference point centered at x=0,y=0 and orientation = 0 (along x axis).
     *        The perimeter is defined in meters as a sequence of vertices of a
     *        polygon defining the perimeter. If vector is of zero size, then robot
     *        is assumed to be point robot (you may want to inflate all obstacles by
     *        its actual radius) Motion primitives file defines the motion primitives
     *        available to the robot
     */
    virtual bool InitializeEnv(const char* sEnvFile, const std::vector<sbpl_2Dpt_t>& perimeterptsV,
                               const char* sMotPrimFile);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual bool InitializeEnv(const char* sEnvFile);

    /**
     * \brief way to set up various parameters. For a list of parameters, see
     *        the body of the function - it is pretty straightforward
     */
    virtual bool SetEnvParameter(const char* parameter, int value);

    /**
     * \brief returns the value of specific parameter - see function body for the list of parameters
     */
    virtual int GetEnvParameter(const char* parameter);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual bool InitializeMDPCfg(MDPConfig *MDPCfg);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual int GetFromToHeuristic(int FromStateID, int ToStateID) = 0;

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual int GetGoalHeuristic(int stateID) = 0;

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual int GetStartHeuristic(int stateID) = 0;

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state) = 0;

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual void SetAllPreds(CMDPSTATE* state);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV);
    virtual void GetLazySuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost);
    virtual void GetSuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV);
    virtual void GetLazySuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV) = 0;

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual void EnsureHeuristicsUpdated(bool bGoalHeuristics);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual void PrintEnv_Config(FILE* fOut);

    /**
     * \brief initialize environment. Gridworld is defined as matrix A of size width by height.
     *        So, internally, it is accessed as A[x][y] with x ranging from 0 to width-1 and and y from 0 to height-1
     *        Each element in A[x][y] is unsigned char. A[x][y] = 0 corresponds to
     *        fully traversable and cost is just Euclidean distance
     *        The cost of transition between two neighboring cells is
     *        EuclideanDistance*(max(A[sourcex][sourcey],A[targetx][targety])+1)
     *        f A[x][y] >= obsthresh, then in the above equation it is assumed to be infinite.
     *        The cost also incorporates the length of a motion primitive and its cost_multiplier (see getcost function)
     *        mapdata is a pointer to the values of A. If it is null, then A is
     *        initialized to all zeros. Mapping is: A[x][y] = mapdata[x+y*width]
     *        start/goal are given by startx, starty, starttheta, goalx,goaly, goaltheta in meters/radians.
     *        If they are not known yet, just set them to 0. Later setgoal/setstart can be executed
     *        finally obsthresh defined obstacle threshold, as mentioned above
     *        goaltolerances are currently ignored
     *        for explanation of perimeter, see comments for InitializeEnv function that reads all from file
     *        cellsize is discretization in meters
     *        nominalvel_mpersecs is assumed velocity of vehicle while moving forward in m/sec
     *        timetoturn45degsinplace_secs is rotational velocity in secs/45 degrees turn
     */
    virtual bool InitializeEnv(int width, int height,
                               /** if mapdata is NULL the grid is initialized to all freespace */
                               const unsigned char* mapdata,
                               double startx, double starty, double starttheta,
                               double goalx, double goaly, double goaltheta,
                               double goaltol_x, double goaltol_y, double goaltol_theta,
                               const std::vector<sbpl_2Dpt_t>& perimeterptsV, double cellsize_m,
                               double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
                               unsigned char obsthresh, const char* sMotPrimFile);

    /**
     * \brief Same as the above InitializeEnv except that only the parameters
     *        that area really needed are required.  The additional (optional)
     *        parameters may be given in the params object (including the ability to
     *        specify the number of thetas)
     */
    virtual bool InitializeEnv(int width, int height, const std::vector<sbpl_2Dpt_t> & perimeterptsV, double cellsize_m,
                               double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
                               unsigned char obsthresh, const char* sMotPrimFile, EnvNAVXYTHETALAT_InitParms params);

    /**
     * \brief update the traversability of a cell<x,y>
     */
    virtual bool UpdateCost(int x, int y, unsigned char newcost);

    /**
     * \brief re-setting the whole 2D map
     *        transform from linear array mapdata to the 2D matrix used internally: Grid2D[x][y] = mapdata[x+y*width]
     */
    virtual bool SetMap(const unsigned char* mapdata);

    /**
     * \brief this function fill in Predecessor/Successor states of edges whose costs changed
     *        It takes in an array of cells whose traversability changed, and
     *        returns (in vector preds_of_changededgesIDV) the IDs of all
     *        states that have outgoing edges that go through the changed
     *        cells
     */
    virtual void GetPredsofChangedEdges(std::vector<nav2dcell_t> const * changedcellsV,
                                        std::vector<int> *preds_of_changededgesIDV) = 0;
    /**
     * \brief same as GetPredsofChangedEdges, but returns successor states.
     *        Both functions need to be present for incremental search
     */
    virtual void GetSuccsofChangedEdges(std::vector<nav2dcell_t> const * changedcellsV,
                                        std::vector<int> *succs_of_changededgesIDV) = 0;

    /**
     * returns true if cell is untraversable
     */
    virtual bool IsObstacle(int x, int y);

    /**
     * \brief returns false if robot intersects obstacles or lies outside of
     *        the map. Note this is pretty expensive operation since it computes the
     *        footprint of the robot based on its x,y,theta
     */
    virtual bool IsValidConfiguration(int X, int Y, int Theta);

    /**
     * \brief returns environment parameters. Useful for creating a copy environment
     */
    virtual void GetEnvParms(int *size_x, int *size_y, double* startx, double* starty, double* starttheta,
                             double* goalx, double* goaly, double* goaltheta, double* cellsize_m,
                             double* nominalvel_mpersecs, double* timetoturn45degsinplace_secs,
                             unsigned char* obsthresh, std::vector<SBPL_xytheta_mprimitive>* motionprimitiveV);

    /**
     * \brief returns environment parameters. Useful for creating a copy environment
     */
    virtual void GetEnvParms(int *size_x, int *size_y, int* num_thetas, double* startx, double* starty,
                             double* starttheta, double* goalx, double* goaly, double* goaltheta, double* cellsize_m,
                             double* nominalvel_mpersecs, double* timetoturn45degsinplace_secs,
                             unsigned char* obsthresh, std::vector<SBPL_xytheta_mprimitive>* motionprimitiveV);

    /**
     * \brief get internal configuration data structure
     */
    virtual const EnvNAVXYTHETALATConfig_t* GetEnvNavConfig();

    virtual ~EnvironmentNAVXYTHETALATTICE();

    /**
     * \brief prints time statistics
     */
    virtual void PrintTimeStat(FILE* fOut);

    /**
     * \brief returns the cost corresponding to the cell <x,y>
     */
    virtual unsigned char GetMapCost(int x, int y);

    /**
     * \brief returns true if cell is within map
     */
    virtual bool IsWithinMapCell(int X, int Y);

    /**
     * \brief Transform a pose into discretized form. The angle 'pth' is
     *        considered to be valid if it lies between -2pi and 2pi (some
     *        people will prefer 0<=pth<2pi, others -pi<pth<=pi, so this
     *        compromise should suit everyone).
     *
     * \note Even if this method returns false, you can still use the
     *       computed indices, for example to figure out how big your map
     *       should have been.
     *
     * \return true if the resulting indices lie within the grid bounds
     *         and the angle was valid.
     */
    virtual bool PoseContToDisc(double px, double py, double pth, int &ix, int &iy, int &ith) const;

    /** \brief Transform grid indices into a continuous pose. The computed
     *         angle lies within 0<=pth<2pi.
     *
     * \note Even if this method returns false, you can still use the
     *      computed indices, for example to figure out poses that lie
     *      outside of your current map.
     *
     * \return true if all the indices are within grid bounds.
     */
    virtual bool PoseDiscToCont(int ix, int iy, int ith, double &px, double &py, double &pth) const;

    /**
     * \brief prints environment variables for debugging
     */
    virtual void PrintVars() { }

protected:
    virtual int GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETALATAction_t* action);

    //member data
    EnvNAVXYTHETALATConfig_t EnvNAVXYTHETALATCfg;
    EnvironmentNAVXYTHETALAT_t EnvNAVXYTHETALAT;
    std::vector<sbpl_xy_theta_cell_t> affectedsuccstatesV; //arrays of states whose outgoing actions cross cell 0,0
    std::vector<sbpl_xy_theta_cell_t> affectedpredstatesV; //arrays of states whose incoming actions cross cell 0,0
    int iteration;

    //2D search for heuristic computations
    bool bNeedtoRecomputeStartHeuristics; //set whenever grid2Dsearchfromstart needs to be re-executed
    bool bNeedtoRecomputeGoalHeuristics; //set whenever grid2Dsearchfromgoal needs to be re-executed
    SBPL2DGridSearch* grid2Dsearchfromstart; //computes h-values that estimate distances from start x,y to all cells
    SBPL2DGridSearch* grid2Dsearchfromgoal; //computes h-values that estimate distances to goal x,y from all cells

    virtual void ReadConfiguration(FILE* fCfg);

    virtual void InitializeEnvConfig(std::vector<SBPL_xytheta_mprimitive>* motionprimitiveV);

    virtual bool CheckQuant(FILE* fOut);

    virtual void SetConfiguration(int width, int height,
                                  /** if mapdata is NULL the grid is initialized to all freespace */
                                  const unsigned char* mapdata,
                                  int startx, int starty, int starttheta,
                                  int goalx, int goaly, int goaltheta,
                                  double cellsize_m, double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
                                  const std::vector<sbpl_2Dpt_t> & robot_perimeterV);

    virtual bool InitGeneral(std::vector<SBPL_xytheta_mprimitive>* motionprimitiveV);
    virtual void PrecomputeActionswithBaseMotionPrimitive(std::vector<SBPL_xytheta_mprimitive>* motionprimitiveV);
    virtual void PrecomputeActionswithCompleteMotionPrimitive(std::vector<SBPL_xytheta_mprimitive>* motionprimitiveV);
    virtual void DeprecatedPrecomputeActions();

    virtual void InitializeEnvironment() = 0;

    virtual void ComputeHeuristicValues();

    virtual bool IsValidCell(int X, int Y);

    virtual void CalculateFootprintForPose(sbpl_xy_theta_pt_t pose, std::vector<sbpl_2Dcell_t>* footprint);
    virtual void CalculateFootprintForPose(sbpl_xy_theta_pt_t pose, std::vector<sbpl_2Dcell_t>* footprint,
                                           const std::vector<sbpl_2Dpt_t>& FootprintPolygon);
    virtual void RemoveSourceFootprint(sbpl_xy_theta_pt_t sourcepose, std::vector<sbpl_2Dcell_t>* footprint);
    virtual void RemoveSourceFootprint(sbpl_xy_theta_pt_t sourcepose, std::vector<sbpl_2Dcell_t>* footprint,
                                       const std::vector<sbpl_2Dpt_t>& FootprintPolygon);

    virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV,
                          std::vector<EnvNAVXYTHETALATAction_t*>* actionindV = NULL) = 0;
    virtual void GetLazySuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost, std::vector<EnvNAVXYTHETALATAction_t*>* actionindV = NULL) = 0;
    virtual void GetSuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<EnvNAVXYTHETALATAction_t*>* actionindV = NULL) = 0;
    virtual void GetLazySuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost, std::vector<EnvNAVXYTHETALATAction_t*>* actionindV = NULL) = 0;
    virtual int GetTrueCost(int parentID, int childID) = 0;
    virtual bool isGoal(int id) = 0;

    virtual double EuclideanDistance_m(int X1, int Y1, int X2, int Y2);

    virtual void ComputeReplanningData();
    virtual void ComputeReplanningDataforAction(EnvNAVXYTHETALATAction_t* action);

    virtual bool ReadMotionPrimitives(FILE* fMotPrims);
    virtual bool ReadinMotionPrimitive(SBPL_xytheta_mprimitive* pMotPrim, FILE* fIn);
    virtual bool ReadinCell(sbpl_xy_theta_cell_t* cell, FILE* fIn);
    virtual bool ReadinPose(sbpl_xy_theta_pt_t* pose, FILE* fIn);

    virtual void PrintHeuristicValues();
};

class EnvironmentNAVXYTHETALAT : public EnvironmentNAVXYTHETALATTICE
{
public:
    EnvironmentNAVXYTHETALAT()
    {
        HashTableSize = 0;
        Coord2StateIDHashTable = NULL;
        Coord2StateIDHashTable_lookup = NULL;
    }

    ~EnvironmentNAVXYTHETALAT();

    /**
     * \brief sets start in meters/radians
     */
    virtual int SetStart(double x, double y, double theta);

    /**
     * \brief sets goal in meters/radians
     */
    virtual int SetGoal(double x, double y, double theta);

    /**
     * \brief sets goal tolerance. (Note goal tolerance is ignored currently)
     */
    virtual void SetGoalTolerance(double tol_x, double tol_y, double tol_theta) { /**< not used yet */ }

    /**
     * \brief returns state coordinates of state with ID=stateID
     */
    virtual void GetCoordFromState(int stateID, int& x, int& y, int& theta) const;

    /**
     * \brief returns stateID for a state with coords x,y,theta
     */
    virtual int GetStateFromCoord(int x, int y, int theta);

    /** \brief converts a path given by stateIDs into a sequence of
     *         coordinates. Note that since motion primitives are short actions
     *         represented as a sequence of points,
     *         the path returned by this function contains much more points than the
     *         number of points in the input path. The returned coordinates are in
     *         meters,meters,radians
     */
    virtual void ConvertStateIDPathintoXYThetaPath(std::vector<int>* stateIDPath,
                                                   std::vector<sbpl_xy_theta_pt_t>* xythetaPath);

    /**
     * \brief prints state info (coordinates) into file
     */
    virtual void PrintState(int stateID, bool bVerbose, FILE* fOut = NULL);

    /**
     * \brief returns all predecessors states and corresponding costs of actions
     */
    virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV);

    /**
     * \brief returns all successors states, costs of corresponding actions
     *        and pointers to corresponding actions, each of which is a motion
     *        primitive
     *        if actionindV is NULL, then pointers to actions are not returned
     */
    virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV,
                          std::vector<EnvNAVXYTHETALATAction_t*>* actionindV = NULL);

    virtual void GetLazySuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost, std::vector<EnvNAVXYTHETALATAction_t*>* actionindV = NULL);
    virtual void GetSuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<EnvNAVXYTHETALATAction_t*>* actionindV = NULL);
    virtual void GetLazySuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost, std::vector<EnvNAVXYTHETALATAction_t*>* actionindV = NULL);
    virtual int GetTrueCost(int parentID, int childID);
    virtual bool isGoal(int id);
    //virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost);
    //virtual void GetPredsWithUniqueIds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV);
    //virtual void GetPredsWithUniqueIds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost);


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
     * \brief see comments on the same function in the parent class
     */
    virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state);

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
    virtual int SizeofCreatedEnv();

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual void PrintVars() { }

protected:
    //hash table of size x_size*y_size. Maps from coords to stateId
    int HashTableSize;
    std::vector<EnvNAVXYTHETALATHashEntry_t*>* Coord2StateIDHashTable;
    //vector that maps from stateID to coords
    std::vector<EnvNAVXYTHETALATHashEntry_t*> StateID2CoordTable;

    EnvNAVXYTHETALATHashEntry_t** Coord2StateIDHashTable_lookup;

    virtual unsigned int GETHASHBIN(unsigned int X, unsigned int Y, unsigned int Theta);

    virtual EnvNAVXYTHETALATHashEntry_t* GetHashEntry_hash(int X, int Y, int Theta);
    virtual EnvNAVXYTHETALATHashEntry_t* CreateNewHashEntry_hash(int X, int Y, int Theta);
    virtual EnvNAVXYTHETALATHashEntry_t* GetHashEntry_lookup(int X, int Y, int Theta);
    virtual EnvNAVXYTHETALATHashEntry_t* CreateNewHashEntry_lookup(int X, int Y, int Theta);

    //pointers to functions
    EnvNAVXYTHETALATHashEntry_t* (EnvironmentNAVXYTHETALAT::*GetHashEntry)(int X, int Y, int Theta);
    EnvNAVXYTHETALATHashEntry_t* (EnvironmentNAVXYTHETALAT::*CreateNewHashEntry)(int X, int Y, int Theta);

    virtual void InitializeEnvironment();

    virtual void PrintHashTableHist(FILE* fOut);
};

#endif

