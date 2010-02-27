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

#define ENVNAV2DUU_COSTMULT 1000

#define NAV2DUU_MAXACTIONSWIDTH 9		

#define ENVNAV2DUU_MAXDIRS 8

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

}EnvironmentNAV2DUU_t;



class EnvironmentNAV2DUU : public DiscreteSpaceInformation
{

public:

	bool InitializeEnv(const char* sEnvFile);

	//mapdata and uncertaintymapdata is assumed to be organized into a linear array with y being major: map[x+y*width]
	bool InitializeEnv(int width, int height,
					const unsigned char* mapdata, const float* uncertaintymapdata, unsigned char obsthresh);
    int SetStart(int x, int y);
    int SetGoal(int x, int y);
	bool UpdateCost(int x, int y, unsigned char newcost);
	

	bool InitializeMDPCfg(MDPConfig *MDPCfg);
	int  GetFromToHeuristic(int FromStateID, int ToStateID);
	int  GetGoalHeuristic(int stateID);
	int  GetStartHeuristic(int stateID);

	void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL);
	void PrintEnv_Config(FILE* fOut);


	EnvironmentNAV2DUU();
    ~EnvironmentNAV2DUU(){};

	void GetPreds(int stateID, const vector<sbpl_BinaryHiddenVar_t>* updatedhvaluesV, vector<CMDPACTION>* IncomingDetActionV,
								  vector<CMDPACTION>* IncomingStochActionV, vector<sbpl_BinaryHiddenVar_t>* StochActionNonpreferredOutcomeV);


	void SetAllActionsandAllOutcomes(CMDPSTATE* state){
		printf("ERROR: SetAllActionsandAllOutcomes not supported in NAV2D UNDER UNCERTAINTY\n");
		exit(1);
	};
	void SetAllPreds(CMDPSTATE* state){
		printf("ERROR: SetAllPreds not supported in NAV2D UNDER UNCERTAINTY\n");
		exit(1);
	};
	void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV){
		printf("ERROR: GetSuccs not supported in NAV2D UNDER UNCERTAINTY\n");
		exit(1);
	};
	void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV){
		printf("ERROR: GetPreds not supported in NAV2D UNDER UNCERTAINTY\n");
		exit(1);
	};
	
	int	 SizeofCreatedEnv();
	int  SizeofH();



private:

	//member variables
	EnvNAV2DUUConfig_t EnvNAV2DUUCfg;
	EnvironmentNAV2DUU_t EnvNAV2DUU;


	//mapdata and uncertaintymapdata is assumed to be organized into a linear array with y being major: map[x+y*width]
	void SetConfiguration(int width, int height, const unsigned char* mapdata, const float* uncertaintymapdata);
	
	void ReadConfiguration(FILE* fCfg);
	void InitializeEnvConfig();
	void InitializeEnvironment();
	void ComputeHeuristicValues();
	bool InitGeneral();

	bool IsValidRobotPosition(int X, int Y);
	bool IsWithinMapCell(int X, int Y);

	void Computedxy();


};


#endif

