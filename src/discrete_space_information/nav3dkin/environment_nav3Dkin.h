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
#ifndef __ENVIRONMENT_NAV3DKIN_H_
#define __ENVIRONMENT_NAV3DKIN_H_


//eight-connected grid
#define NAV3DKIN_DXYWIDTH 8

#define ENVNAV3DKIN_DEFAULTOBSTHRESH 254	//see explanation of the value below

// not used yet -- default linear goal tolerance along x and y
#define ENVNAV3DKIN_DEFAULT_TOL_XY 0.3

// not used yet -- default angular goal tolerance
#define ENVNAV3DKIN_DEFAULT_TOL_TH 3.14159265358979323846

//definition of theta orientations
//0 - is aligned with X-axis in the positive direction (1,0 in polar coordinates)
//theta increases as we go counterclockwise
//number of theta values 
#define NAV3DKIN_THETADIRS 8 

//number of actions per x,y,theta state
#define NAV3DKIN_ACTIONWIDTH 5 //decrease, increase, same angle while moving plus decrease, increase angle while standing.

#define NAV3DKIN_COSTMULT_MTOMM 1000

typedef struct{
	double x;
	double y;
} EnvNAV3DKIN2Dpt_t;

typedef struct{
	double x;
	double y;
	double theta;
} EnvNAV3DKIN3Dpt_t;


typedef struct EnvNAV3DKIN3DCELL{
	int x;
	int y;
	int theta;
public:
	bool operator == (EnvNAV3DKIN3DCELL cell) {return (x==cell.x && y==cell.y && theta==cell.theta);}
} EnvNAV3DKIN3Dcell_t;


typedef struct
{
	char starttheta;
	char dX;
	char dY;
	char dTheta;
	unsigned int cost; 
	vector<sbpl_2Dcell_t> intersectingcellsV;
} EnvNAV3DKINAction_t;


//configuration parameters
typedef struct ENV_NAV3DKIN_CONFIG
{
	int EnvWidth_c;
	int EnvHeight_c;
	int StartX_c;
	int StartY_c;
	int StartTheta;
	int EndX_c;
	int EndY_c;
	int EndTheta;
	unsigned char** Grid2D;

	//the value at which and above which cells are obstacles in the maps sent from outside
	//the default is defined above
	unsigned char obsthresh; 


	double nominalvel_mpersecs;
	double timetoturn45degsinplace_secs;
	double cellsize_m;

	int dXY[NAV3DKIN_DXYWIDTH][2];

	EnvNAV3DKINAction_t** ActionsV; //array of actions, ActionsV[i][j] - jth action for sourcetheta = i
	vector<EnvNAV3DKINAction_t*>* PredActionsV; //PredActionsV[i] - vector of pointers to the actions that result in a state with theta = i


	vector<sbpl_2Dpt_t> FootprintPolygon;
} EnvNAV3DKINConfig_t;

typedef struct 
{
	int stateID;
	int X;
	int Y;
	char Theta;
} EnvNAV3DKINHashEntry_t;



//variables that dynamically change (e.g., array of states, ...)
typedef struct
{

	int startstateid;
	int goalstateid;

	//hash table of size x_size*y_size. Maps from coords to stateId	
	int HashTableSize;
	vector<EnvNAV3DKINHashEntry_t*>* Coord2StateIDHashTable;

	//vector that maps from stateID to coords	
	vector<EnvNAV3DKINHashEntry_t*> StateID2CoordTable;

	//any additional variables

}EnvironmentNAV3DKIN_t;

class SBPL2DGridSearch;

class EnvironmentNAV3DKIN : public DiscreteSpaceInformation
{

public:

	EnvironmentNAV3DKIN();

	bool InitializeEnv(const char* sEnvFile);
	bool InitializeEnv(const char* sEnvFile, const vector<sbpl_2Dpt_t> & perimeterptsV);


	bool InitializeMDPCfg(MDPConfig *MDPCfg);
	int  GetFromToHeuristic(int FromStateID, int ToStateID);
	int  GetGoalHeuristic(int stateID);
	int  GetStartHeuristic(int stateID);
	void SetAllActionsandAllOutcomes(CMDPSTATE* state);
	void SetAllPreds(CMDPSTATE* state);
	void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV);
	void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV);

	int	 SizeofCreatedEnv();
	void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL);
	void PrintEnv_Config(FILE* fOut);
  
  bool InitializeEnv(int width, int height,
		     /** if mapdata is NULL the grid is initialized to all freespace */
		     const unsigned char* mapdata,
		     const vector<sbpl_2Dpt_t> & perimeterptsV,
		     double cellsize_m, double nominalvel_mpersecs,
		     double timetoturn45degsinplace_secs, 
		     unsigned char obsthresh);
  
    bool InitializeEnv(int width, int height,
		       /** if mapdata is NULL the grid is initialized to all freespace */
                       const unsigned char* mapdata,
                       double startx, double starty, double starttheta,
                       double goalx, double goaly, double goaltheta,
					   double goaltol_x, double goaltol_y, double goaltol_theta,
					   const vector<sbpl_2Dpt_t> & perimeterptsV,
					   double cellsize_m, double nominalvel_mpersecs, double timetoturn45degsinplace_secs, 
					   unsigned char obsthresh);
    int SetStart(double x, double y, double theta);
    int SetGoal(double x, double y, double theta);
    void SetGoalTolerance(double tol_x, double tol_y, double tol_theta); /**< not used yet */
    bool UpdateCost(int x, int y, unsigned char newcost);
	void GetPredsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *preds_of_changededgesIDV);
	void GetSuccsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *succs_of_changededgesIDV);


	void GetCoordFromState(int stateID, int& x, int& y, int& theta) const;

	int GetStateFromCoord(int x, int y, int theta);

	bool IsObstacle(int x, int y);
	bool IsValidConfiguration(int X, int Y, int Theta);

	void GetEnvParms(int *size_x, int *size_y, double* startx, double* starty, double* starttheta, double* goalx, double* goaly, double* goaltheta,
			double* cellsize_m, double* nominalvel_mpersecs, double* timetoturn45degsinplace_secs, unsigned char* obsthresh);

	const EnvNAV3DKINConfig_t* GetEnvNavConfig();


    ~EnvironmentNAV3DKIN();

    void PrintTimeStat(FILE* fOut);
  
	unsigned char GetMapCost(int x, int y);

  
  bool IsWithinMapCell(int X, int Y);
  
  /** Transform a pose into discretized form. The angle 'pth' is
      considered to be valid if it lies between -2pi and 2pi (some
      people will prefer 0<=pth<2pi, others -pi<pth<=pi, so this
      compromise should suit everyone).
      
      \note Even if this method returns false, you can still use the
      computed indices, for example to figure out how big your map
      should have been.
      
      \return true if the resulting indices lie within the grid bounds
      and the angle was valid.
  */
  bool PoseContToDisc(double px, double py, double pth,
		      int &ix, int &iy, int &ith) const;
  
  /** Transform grid indices into a continuous pose. The computed
      angle lies within 0<=pth<2pi.
      
      \note Even if this method returns false, you can still use the
      computed indices, for example to figure out poses that lie
      outside of your current map.
      
      \return true if all the indices are within grid bounds.
  */
  bool PoseDiscToCont(int ix, int iy, int ith,
		      double &px, double &py, double &pth) const;
  
 private:

	//member data
	EnvNAV3DKINConfig_t EnvNAV3DKINCfg;
	EnvironmentNAV3DKIN_t EnvNAV3DKIN;
	vector<EnvNAV3DKIN3Dcell_t> affectedsuccstatesV; //arrays of states whose outgoing actions cross cell 0,0
	vector<EnvNAV3DKIN3Dcell_t> affectedpredstatesV; //arrays of states whose incoming actions cross cell 0,0
	
	//2D search for heuristic computations
	bool bNeedtoRecomputeStartHeuristics;
	SBPL2DGridSearch* grid2Dsearch;

 	void ReadConfiguration(FILE* fCfg);

	void InitializeEnvConfig();

	unsigned int GETHASHBIN(unsigned int X, unsigned int Y, unsigned int Theta);

	void PrintHashTableHist();
	bool CheckQuant(FILE* fOut);

	void SetConfiguration(int width, int height,
			      /** if mapdata is NULL the grid is initialized to all freespace */
			      const unsigned char* mapdata,
			      int startx, int starty, int starttheta,
			      int goalx, int goaly, int goaltheta,
				  double cellsize_m, double nominalvel_mpersecs, double timetoturn45degsinplace_secs, const vector<sbpl_2Dpt_t> & robot_perimeterV);
	
	bool InitGeneral();



	EnvNAV3DKINHashEntry_t* GetHashEntry(int X, int Y, int Theta);

	EnvNAV3DKINHashEntry_t* CreateNewHashEntry(int X, int Y, int Theta);


	void CreateStartandGoalStates();

	void InitializeEnvironment();

	void ComputeHeuristicValues();

	bool IsValidCell(int X, int Y);

	void CalculateFootprintForPose(EnvNAV3DKIN3Dpt_t pose, vector<sbpl_2Dcell_t>* footprint);
	void RemoveSourceFootprint(EnvNAV3DKIN3Dpt_t sourcepose, vector<sbpl_2Dcell_t>* footprint);

	int GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAV3DKINAction_t* action);

	double EuclideanDistance_m(int X1, int Y1, int X2, int Y2);

	void ComputeReplanningData();
	void ComputeReplanningDataforAction(EnvNAV3DKINAction_t* action);



};

#endif

