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
#include <iostream>
using namespace std;

#include "../sbpl/headers.h"


void PrintUsage(char *argv[])
{
	SBPL_PRINTF("USAGE: %s <cfg file> [motionprimitivesfile.mprim for xythetalattice planning]\n", argv[0]);
}


int plan2d(int argc, char *argv[])
{

	int bRet = 0;
	double allocated_time_secs = 100.0; //in seconds
	MDPConfig MDPCfg;
	bool bsearchuntilfirstsolution = true;
	
	//Initialize Environment (should be called before initializing anything else)
	EnvironmentNAV2D environment_nav2D;
	if(!environment_nav2D.InitializeEnv(argv[1]))
	{
		SBPL_ERROR("ERROR: InitializeEnv failed\n");
		throw new SBPL_Exception();
	}

	//Initialize MDP Info
	if(!environment_nav2D.InitializeMDPCfg(&MDPCfg))
	{
		SBPL_ERROR("ERROR: InitializeMDPCfg failed\n");
		throw new SBPL_Exception();
	}


	//plan a path
	vector<int> solution_stateIDs_V;
	bool bforwardsearch = false;
	ARAPlanner planner(&environment_nav2D, bforwardsearch);

	//set search mode
	planner.set_search_mode(bsearchuntilfirstsolution);


    if(planner.set_start(MDPCfg.startstateid) == 0)
        {
            SBPL_ERROR("ERROR: failed to set start state\n");
            throw new SBPL_Exception();
        }

    if(planner.set_goal(MDPCfg.goalstateid) == 0)
        {
            SBPL_ERROR("ERROR: failed to set goal state\n");
            throw new SBPL_Exception();
        }

	planner.set_initialsolution_eps(3.0);

    SBPL_PRINTF("start planning...\n");
	bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    SBPL_PRINTF("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_nav2D.PrintTimeStat(stdout);

	/*
    SBPL_PRINTF("start planning...\n");
	bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    SBPL_PRINTF("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_nav2D.PrintTimeStat(stdout);

    SBPL_PRINTF("start planning...\n");
	bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    SBPL_PRINTF("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_nav2D.PrintTimeStat(stdout);

    SBPL_PRINTF("start planning...\n");
	bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    SBPL_PRINTF("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_nav2D.PrintTimeStat(stdout);

    SBPL_PRINTF("start planning...\n");
	bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    SBPL_PRINTF("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_nav2D.PrintTimeStat(stdout);

    SBPL_PRINTF("start planning...\n");
	bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    SBPL_PRINTF("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;
	*/

#ifndef ROS
  const char* sol = "sol.txt";
#endif
  FILE* fSol = SBPL_FOPEN(sol, "w");
  if(fSol == NULL){
    SBPL_ERROR("ERROR: could not open solution file\n");
    throw new SBPL_Exception();
  }
	for(unsigned int i = 0; i < solution_stateIDs_V.size(); i++) {
	  environment_nav2D.PrintState(solution_stateIDs_V[i], false, fSol);
	}
    SBPL_FCLOSE(fSol);

    environment_nav2D.PrintTimeStat(stdout);

	//print a path
	if(bRet)
	{
		//print the solution
		SBPL_PRINTF("Solution is found\n");
	}
	else
		SBPL_PRINTF("Solution does not exist\n");

	SBPL_FFLUSH(NULL);


    return bRet;
}

int plan2duu(int argc, char *argv[])
{

	int bRet = 0;
	double allocated_time_secs = 100.0; //in seconds
	MDPConfig MDPCfg;
	
	//Initialize Environment (should be called before initializing anything else)
	EnvironmentNAV2DUU environment_nav2Duu;
	if(!environment_nav2Duu.InitializeEnv(argv[1]))
	{
		SBPL_ERROR("ERROR: InitializeEnv failed\n");
		throw new SBPL_Exception();
	}

	//Initialize MDP Info
	if(!environment_nav2Duu.InitializeMDPCfg(&MDPCfg))
	{
		SBPL_ERROR("ERROR: InitializeMDPCfg failed\n");
		throw new SBPL_Exception();
	}


	//create the planner
	PPCPPlanner planner(&environment_nav2Duu, environment_nav2Duu.SizeofCreatedEnv(), environment_nav2Duu.SizeofH());
	
	//set start and goal
    if(planner.set_start(MDPCfg.startstateid) == 0)
        {
            SBPL_ERROR("ERROR: failed to set start state\n");
            throw new SBPL_Exception();
        }
    if(planner.set_goal(MDPCfg.goalstateid) == 0)
        {
            SBPL_ERROR("ERROR: failed to set goal state\n");
            throw new SBPL_Exception();
        }


    SBPL_PRINTF("start planning...\n");
	float ExpectedCost, ProbofReachGoal;
	vector<sbpl_PolicyStatewithBinaryh_t> SolutionPolicy;
	bRet = planner.replan(allocated_time_secs, &SolutionPolicy, &ExpectedCost, &ProbofReachGoal);
    SBPL_PRINTF("done planning\n");


	if(bRet)
	{
		//print the solution
		SBPL_PRINTF("Solution is found: exp. cost=%f probofreachgoal=%f\n", ExpectedCost, ProbofReachGoal);
	}
	else
		SBPL_PRINTF("Solution does not exist\n");

	SBPL_FFLUSH(NULL);


    return bRet;
}


int planxythetalat(int argc, char *argv[])
{

	int bRet = 0;
	double allocated_time_secs = 10; //in seconds
	MDPConfig MDPCfg;
	bool bsearchuntilfirstsolution = false;

	//set the perimeter of the robot (it is given with 0,0,0 robot ref. point for which planning is done)
	vector<sbpl_2Dpt_t> perimeterptsV;
	sbpl_2Dpt_t pt_m;
	double halfwidth = 0.01; //0.3;
	double halflength = 0.01; //0.45;
	pt_m.x = -halflength;
	pt_m.y = -halfwidth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = halflength;
	pt_m.y = -halfwidth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = halflength;
	pt_m.y = halfwidth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = -halflength;
	pt_m.y = halfwidth;
	perimeterptsV.push_back(pt_m);
	

	//clear perimeter
	//perimeterptsV.clear();
	//pt_m.x = 0.0;
	//pt_m.y = 0.0;
	//perimeterptsV.push_back(pt_m);

	//Initialize Environment (should be called before initializing anything else)
	EnvironmentNAVXYTHETALAT environment_navxythetalat;

	if(argc == 3)
	{
		if(!environment_navxythetalat.InitializeEnv(argv[1], perimeterptsV, argv[2]))
		{
			SBPL_ERROR("ERROR: InitializeEnv failed\n");
			throw new SBPL_Exception();
		}
	}
	else
	{
		if(!environment_navxythetalat.InitializeEnv(argv[1], perimeterptsV, NULL))
		{
			SBPL_ERROR("ERROR: InitializeEnv failed\n");
			throw new SBPL_Exception();
		}
	}

	//Initialize MDP Info
	if(!environment_navxythetalat.InitializeMDPCfg(&MDPCfg))
	{
		SBPL_ERROR("ERROR: InitializeMDPCfg failed\n");
		throw new SBPL_Exception();
	}


	//plan a path
	vector<int> solution_stateIDs_V;
	bool bforwardsearch = false;
	ADPlanner planner(&environment_navxythetalat, bforwardsearch);

    if(planner.set_start(MDPCfg.startstateid) == 0)
        {
            SBPL_ERROR("ERROR: failed to set start state\n");
            throw new SBPL_Exception();
        }

    if(planner.set_goal(MDPCfg.goalstateid) == 0)
        {
            SBPL_ERROR("ERROR: failed to set goal state\n");
            throw new SBPL_Exception();
        }
	planner.set_initialsolution_eps(3.0);

	//set search mode
	planner.set_search_mode(bsearchuntilfirstsolution);

    SBPL_PRINTF("start planning...\n");
	bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    SBPL_PRINTF("done planning\n");
	SBPL_PRINTF("size of solution=%d\n",(unsigned int)solution_stateIDs_V.size());

    environment_navxythetalat.PrintTimeStat(stdout);

	/*
    if(planner.set_start(environment_navxythetalat.SetStart(1.6,0.5,0)) == 0)
        {
            SBPL_ERROR("ERROR: failed to set start state\n");
            throw new SBPL_Exception();
        }


    SBPL_PRINTF("start planning...\n");
	bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    SBPL_PRINTF("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_navxythetalat.PrintTimeStat(stdout);


    SBPL_PRINTF("start planning...\n");
	bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    SBPL_PRINTF("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_navxythetalat.PrintTimeStat(stdout);

    SBPL_PRINTF("start planning...\n");
	bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    SBPL_PRINTF("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_navxythetalat.PrintTimeStat(stdout);

    SBPL_PRINTF("start planning...\n");
	bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    SBPL_PRINTF("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_navxythetalat.PrintTimeStat(stdout);

    SBPL_PRINTF("start planning...\n");
	bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    SBPL_PRINTF("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;
	*/

#ifndef ROS
  const char* sol = "sol.txt";
#endif
  FILE* fSol = SBPL_FOPEN(sol, "w");
  if(fSol == NULL){
    SBPL_ERROR("ERROR: could not open solution file\n");
    throw new SBPL_Exception();
  }
	vector<EnvNAVXYTHETALAT3Dpt_t> xythetaPath;
	environment_navxythetalat.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs_V, &xythetaPath);
	SBPL_PRINTF("solution size=%d\n", (unsigned int)xythetaPath.size());
	for(unsigned int i = 0; i < xythetaPath.size(); i++) {
		SBPL_FPRINTF(fSol, "%.3f %.3f %.3f\n", xythetaPath.at(i).x, xythetaPath.at(i).y, xythetaPath.at(i).theta);
	}
    SBPL_FCLOSE(fSol);

    environment_navxythetalat.PrintTimeStat(stdout);

	//print a path
	if(bRet)
	{
		//print the solution
		SBPL_PRINTF("Solution is found\n");
	}
	else
		SBPL_PRINTF("Solution does not exist\n");

	SBPL_FFLUSH(NULL);


    return bRet;
}

//planning with multiple levels x,y,theta lattice (for example, base and upper body)
//there are no additional degrees of freedom. 
//It is just that each level may have a different footprint 
//and should be checked against the cost map at corresponding height
//It is useful for doing planning for navigation by a tall ground robot operating in a cluttered 3D map
int planxythetamlevlat(int argc, char *argv[])
{

	int bRet = 0;
	double allocated_time_secs = 10; //in seconds
	MDPConfig MDPCfg;
	bool bsearchuntilfirstsolution = false;

	//set the perimeter of the robot (it is given with 0,0,0 robot ref. point for which planning is done)
	//this is for the default level - base level
	vector<sbpl_2Dpt_t> perimeterptsV;
	sbpl_2Dpt_t pt_m;
	double halfwidth = 0.3; //0.3;
	double halflength = 0.25; //0.45;
	pt_m.x = -halflength;
	pt_m.y = -halfwidth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = halflength;
	pt_m.y = -halfwidth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = halflength;
	pt_m.y = halfwidth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = -halflength;
	pt_m.y = halfwidth;
	perimeterptsV.push_back(pt_m);
	
	//clear perimeter
	//perimeterptsV.clear();
	//pt_m.x = 0.0;
	//pt_m.y = 0.0;
	//perimeterptsV.push_back(pt_m);

	//Initialize Environment (should be called before initializing anything else)
	EnvironmentNAVXYTHETAMLEVLAT environment_navxythetalat;

	if(argc == 3)
	{
		if(!environment_navxythetalat.InitializeEnv(argv[1], perimeterptsV, argv[2]))
		{
			SBPL_ERROR("ERROR: InitializeEnv failed\n");
			throw new SBPL_Exception();
		}
	}
	else
	{
		if(!environment_navxythetalat.InitializeEnv(argv[1], perimeterptsV, NULL))
		{
			SBPL_ERROR("ERROR: InitializeEnv failed\n");
			throw new SBPL_Exception();
		}
	}


	//this is for the second level - upper body level
	vector<sbpl_2Dpt_t> perimeterptsVV[2];
	perimeterptsVV[0].clear();
	halfwidth = 0.3; 
	halflength = 0.75; 
	pt_m.x = -halflength;
	pt_m.y = -halfwidth;
	perimeterptsVV[0].push_back(pt_m);
	pt_m.x = halflength;
	pt_m.y = -halfwidth;
	perimeterptsVV[0].push_back(pt_m);
	pt_m.x = halflength;
	pt_m.y = halfwidth;
	perimeterptsVV[0].push_back(pt_m);
	pt_m.x = -halflength;
	pt_m.y = halfwidth;
	perimeterptsVV[0].push_back(pt_m);


	//enable the second level
	int numofaddlevels = 1;
	SBPL_PRINTF("Number of additional levels = %d\n", numofaddlevels);
	unsigned char cost_inscribed_thresh_addlevels[2]; //size should be at least numofaddlevels
	unsigned char cost_possibly_circumscribed_thresh_addlevels[2]; //size should be at least numofaddlevels
	cost_inscribed_thresh_addlevels[0] = 255; //no costs are indicative of whether a cell is within inner circle
	cost_possibly_circumscribed_thresh_addlevels[0] = 0; //no costs are indicative of whether a cell is within outer circle
	cost_inscribed_thresh_addlevels[1] = 255; //no costs are indicative of whether a cell is within inner circle
	cost_possibly_circumscribed_thresh_addlevels[1] = 0; //no costs are indicative of whether a cell is within outer circle
	if(!environment_navxythetalat.InitializeAdditionalLevels(numofaddlevels, perimeterptsVV, 
		cost_inscribed_thresh_addlevels, cost_possibly_circumscribed_thresh_addlevels))
	{
		SBPL_ERROR("ERROR: InitializeAdditionalLevels failed with numofaddlevels=%d\n", numofaddlevels);
		throw new SBPL_Exception();

	}

	//set the map for the second level (index parameter for the additional levels and is zero based)
	//for this example, we pass in the same map as the map for the base. In general, it can be a totally different map
	//as it corresponds to a different level
	//NOTE: this map has to have costs set correctly with respect to inner and outer radii of the robot
	//if the second level of the robot has these radii different than at the base level, then costs
	//should reflect this 
	//(see explanation for cost_possibly_circumscribed_thresh and cost_inscribed_thresh parameters in environment_navxythetalat.h file)
	int addlevind = 0;
	if(!environment_navxythetalat.Set2DMapforAddLev(
		(const unsigned char**)(environment_navxythetalat.GetEnvNavConfig()->Grid2D),addlevind))
	{
		SBPL_ERROR("ERROR: Setting Map for the Additional Level failed with level %d\n", addlevind);
		throw new SBPL_Exception();

	}

	//Initialize MDP Info
	if(!environment_navxythetalat.InitializeMDPCfg(&MDPCfg))
	{
		SBPL_ERROR("ERROR: InitializeMDPCfg failed\n");
		throw new SBPL_Exception();
	}


	//plan a path
	vector<int> solution_stateIDs_V;
	bool bforwardsearch = false;
	ADPlanner planner(&environment_navxythetalat, bforwardsearch);

    if(planner.set_start(MDPCfg.startstateid) == 0)
        {
            SBPL_ERROR("ERROR: failed to set start state\n");
            throw new SBPL_Exception();
        }

    if(planner.set_goal(MDPCfg.goalstateid) == 0)
        {
            SBPL_ERROR("ERROR: failed to set goal state\n");
            throw new SBPL_Exception();
        }
	planner.set_initialsolution_eps(3.0);

	//set search mode
	planner.set_search_mode(bsearchuntilfirstsolution);

    SBPL_PRINTF("start planning...\n");
	bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    SBPL_PRINTF("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_navxythetalat.PrintTimeStat(stdout);

#ifndef ROS
  const char* sol = "sol.txt";
#endif
  FILE* fSol = SBPL_FOPEN(sol, "w");
  if(fSol == NULL){
    SBPL_ERROR("ERROR: could not open solution file\n");
    throw new SBPL_Exception();
  }
	vector<EnvNAVXYTHETALAT3Dpt_t> xythetaPath;
	environment_navxythetalat.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs_V, &xythetaPath);
	SBPL_PRINTF("solution size=%d\n", (unsigned int)xythetaPath.size());
	for(unsigned int i = 0; i < xythetaPath.size(); i++) {
		SBPL_FPRINTF(fSol, "%.3f %.3f %.3f\n", xythetaPath.at(i).x, xythetaPath.at(i).y, xythetaPath.at(i).theta);
	}
    SBPL_FCLOSE(fSol);

    environment_navxythetalat.PrintTimeStat(stdout);

	//print a path
	if(bRet)
	{
		//print the solution
		SBPL_PRINTF("Solution is found\n");
	}
	else
		SBPL_PRINTF("Solution does not exist\n");

	SBPL_FFLUSH(NULL);


    return bRet;
}




int planandnavigate2d(int argc, char *argv[])
{
	double allocated_time_secs_foreachplan = 0.2; //in seconds
	MDPConfig MDPCfg;
	EnvironmentNAV2D environment_nav2D;
	EnvironmentNAV2D trueenvironment_nav2D;
  int size_x=-1,size_y=-1;
  int startx = 0, starty = 0;
  int goalx=-1, goaly=-1;
#ifndef ROS
  const char* sol = "sol.txt";
#endif
  FILE* fSol = SBPL_FOPEN(sol, "w");
  if(fSol == NULL){
    SBPL_ERROR("ERROR: could not open solution file\n");
    throw new SBPL_Exception();
  }
  int dx[8] = {-1, -1, -1,  0,  0,  1,  1,  1};
  int dy[8] = {-1,  0,  1, -1,  1, -1,  0,  1};
	bool bPrint = true;
	int x,y;
	vector<int> preds_of_changededgesIDV;
	vector<nav2dcell_t> changedcellsV;
	nav2dcell_t nav2dcell;
	int i;
	unsigned char obsthresh = 0;
	//srand(0);
	int plantime_over1secs=0, plantime_over0p5secs=0, plantime_over0p1secs=0, plantime_over0p05secs=0, plantime_below0p05secs=0;

	//set parameters - should be done before initialization 
	if(!trueenvironment_nav2D.SetEnvParameter("is16connected", 1))
	{
		SBPL_ERROR("ERROR: failed to set parameters\n");
		throw new SBPL_Exception();
	}
	if(!environment_nav2D.SetEnvParameter("is16connected", 1))
	{
		SBPL_ERROR("ERROR: failed to set parameters\n");
		throw new SBPL_Exception();
	}


    //initialize true map and robot map
	if(!trueenvironment_nav2D.InitializeEnv(argv[1]))
	{
		SBPL_ERROR("ERROR: InitializeEnv failed\n");
		throw new SBPL_Exception();
	}
	trueenvironment_nav2D.GetEnvParms(&size_x, &size_y, &startx, &starty, &goalx, &goaly, &obsthresh);
    unsigned char* map = (unsigned char*)calloc(size_x*size_y, sizeof(unsigned char));

	//print the map
	if(bPrint) SBPL_PRINTF("true map:\n");
	for(y = 0; bPrint && y < size_y; y++){
		for(x = 0; x < size_x; x++){
			SBPL_PRINTF("%d ", (int)trueenvironment_nav2D.IsObstacle(x,y));
		}
		SBPL_PRINTF("\n");
	}
	if(bPrint) SBPL_PRINTF("System Pause (return=%d)\n",system("pause"));

	//Initialize Environment (should be called before initializing anything else)
    if(!environment_nav2D.InitializeEnv(size_x, size_y, map, startx, starty, goalx, goaly, obsthresh)){
		SBPL_ERROR("ERROR: InitializeEnv failed\n");
		throw new SBPL_Exception();
	}

	//Initialize MDP Info
	if(!environment_nav2D.InitializeMDPCfg(&MDPCfg))
	{
		SBPL_ERROR("ERROR: InitializeMDPCfg failed\n");
		throw new SBPL_Exception();
	}


	//create a planner
	vector<int> solution_stateIDs_V;
	bool bforwardsearch = false;
    //ARAPlanner planner(&environment_nav2D, bforwardsearch);
	ADPlanner planner(&environment_nav2D, bforwardsearch);

	planner.set_initialsolution_eps(2.0);


    //set the start and goal configurations
    if(planner.set_start(MDPCfg.startstateid) == 0)
        {
            SBPL_ERROR("ERROR: failed to set start state\n");
            throw new SBPL_Exception();
        }
    if(planner.set_goal(MDPCfg.goalstateid) == 0)
        {
            SBPL_ERROR("ERROR: failed to set goal state\n");
            throw new SBPL_Exception();
        }

    //now comes the main loop
    int goalthresh = 0;
    while(abs(startx - goalx) > goalthresh || abs(starty - goaly) > goalthresh){

        //simulate sensor data update
        bool bChanges = false;
		preds_of_changededgesIDV.clear();
		changedcellsV.clear();
        for(i = 0; i < 8; i++){
            int x = startx + dx[i];
            int y = starty + dy[i];
            if(x < 0 || x >= size_x || y < 0 || y >= size_y)
                continue;
            int index = x + y*size_x;
			unsigned char truecost = trueenvironment_nav2D.GetMapCost(x,y);
            if(map[index] != truecost){
                map[index] = truecost;
                environment_nav2D.UpdateCost(x,y,map[index]);
                SBPL_PRINTF("setting cost[%d][%d] to %d\n", x,y,map[index]);
                bChanges = true;
				//store the changed cells
				nav2dcell.x = x;
				nav2dcell.y = y;
				changedcellsV.push_back(nav2dcell);
            }
        }
		
		double TimeStarted = clock();

        if(bChanges){
            //planner.costs_changed(); //use by ARA* planner (non-incremental)

			//get the affected states
			environment_nav2D.GetPredsofChangedEdges(&changedcellsV, &preds_of_changededgesIDV);
			//let know the incremental planner about them
			planner.update_preds_of_changededges(&preds_of_changededgesIDV); //use by AD* planner (incremental)
        }


        SBPL_FPRINTF(fSol, "%d %d ",  startx, starty);

        //plan a path 
        bool bPlanExists = false;
        while(bPlanExists == false){
            SBPL_PRINTF("new planning...\n");   
            bPlanExists = (planner.replan(allocated_time_secs_foreachplan, &solution_stateIDs_V) == 1);
            SBPL_PRINTF("done with the solution of size=%d\n", (unsigned int)solution_stateIDs_V.size());   
            environment_nav2D.PrintTimeStat(stdout);

            //for(unsigned int i = 0; i < solution_stateIDs_V.size(); i++) {
            //environment_nav2D.PrintState(solution_stateIDs_V[i], true, fSol);
            //}
            //SBPL_FPRINTF(fSol, "*********\n");
        }

		double plantime_secs = (clock()-TimeStarted)/((double)CLOCKS_PER_SEC);
		SBPL_FPRINTF(fSol, "%.5f %.5f\n", plantime_secs, planner.get_solution_eps());
		SBPL_FFLUSH(fSol);
		if(plantime_secs > 1.0)
			plantime_over1secs++;
		else if(plantime_secs > 0.5)
			plantime_over0p5secs++;
		else if(plantime_secs > 0.1)
			plantime_over0p1secs++;
		else if(plantime_secs > 0.05)
			plantime_over0p05secs++;
		else
			plantime_below0p05secs++;

		//print the map
		int startindex = startx + starty*size_x;
		int goalindex = goalx + goaly*size_x;
		for(y = 0; bPrint && y < size_y; y++){
			for(x = 0; x < size_x; x++){
				int index = x + y*size_x;

				//check to see if it is on the path
				bool bOnthePath = false;
				for(int j = 1; j < (int)solution_stateIDs_V.size(); j++)
				{
					int newx, newy;
					environment_nav2D.GetCoordFromState(solution_stateIDs_V[j], newx, newy);
					if(x == newx && y == newy)
						bOnthePath = true;
				}

				if (index != startindex && index != goalindex && !bOnthePath)
					SBPL_PRINTF("%3d ", map[index]);
				else if(index == startindex)
					SBPL_PRINTF("  R ");
				else if(index == goalindex)
					SBPL_PRINTF("  G ");
				else if (bOnthePath)
					SBPL_PRINTF("  * ");
				else
					SBPL_PRINTF("  ? ");
			}
			SBPL_PRINTF("\n");
		}
    if(bPrint) SBPL_PRINTF("System Pause (return=%d)\n",system("pause"));


        //move along the path
        if(bPlanExists && (int)solution_stateIDs_V.size() > 1){
            //get coord of the successor
            int newx, newy;
            environment_nav2D.GetCoordFromState(solution_stateIDs_V[1], newx, newy);

			if(trueenvironment_nav2D.GetMapCost(newx,newy) >= obsthresh)
			{
				SBPL_ERROR("ERROR: robot is commanded to move into an obstacle\n");
				throw new SBPL_Exception();
			}

            //move
            SBPL_PRINTF("moving from %d %d to %d %d\n", startx, starty, newx, newy);
            startx = newx;
            starty = newy;
			
            //update the environment
            environment_nav2D.SetStart(startx, starty);
            
            //update the planner
            if(planner.set_start(solution_stateIDs_V[1]) == 0){               
                SBPL_ERROR("ERROR: failed to update robot pose in the planner\n");
                throw new SBPL_Exception();
            }
        }

    }

	//print stats
	SBPL_PRINTF("stats: plantimes over 1 secs=%d; over 0.5 secs=%d; over 0.1 secs=%d; over 0.05 secs=%d; below 0.05 secs=%d\n",
		plantime_over1secs, plantime_over0p5secs, plantime_over0p1secs, plantime_over0p05secs, plantime_below0p05secs);
	SBPL_FPRINTF(fSol, "stats: plantimes over 1 secs=%d; over 0.5; secs=%d; over 0.1 secs=%d; over 0.05 secs=%d; below 0.05 secs=%d\n",
		plantime_over1secs, plantime_over0p5secs, plantime_over0p1secs, plantime_over0p05secs, plantime_below0p05secs);

	SBPL_FFLUSH(NULL);
  SBPL_FCLOSE(fSol);


    return 1;
}




int planandnavigatexythetalat(int argc, char *argv[])
{

	double allocated_time_secs_foreachplan = 3.0; //in seconds
	MDPConfig MDPCfg;
	EnvironmentNAVXYTHETALAT environment_navxythetalat;
	EnvironmentNAVXYTHETALAT trueenvironment_navxythetalat;
    int size_x=-1,size_y=-1;
    double startx = 0, starty = 0, starttheta = 0;
    double goalx=-1, goaly=-1, goaltheta = -1;
	double goaltol_x = 0.001, goaltol_y = 0.001, goaltol_theta = 0.001;
#ifndef ROS
  const char* sol = "sol.txt";
#endif
  FILE* fSol = SBPL_FOPEN(sol, "w");
  if(fSol == NULL){
    SBPL_ERROR("ERROR: could not open solution file\n");
    throw new SBPL_Exception();
  }
    //int dx[8] = {-1, -1, -1,  0,  0,  1,  1,  1};
    //int dy[8] = {-1,  0,  1, -1,  1, -1,  0,  1};
	bool bPrint = true, bPrintMap = false;
	int x,y;
	vector<int> preds_of_changededgesIDV;
	vector<nav2dcell_t> changedcellsV;
	nav2dcell_t nav2dcell;
	int i;
	double cellsize_m, nominalvel_mpersecs, timetoturn45degsinplace_secs;
	bool bsearchuntilfirstsolution = false; 
	unsigned char obsthresh = 0;
	unsigned char costinscribed_thresh = 0; 
	unsigned char costcircum_thresh = 0; 
	char sMotPrimFilename[1024];
	char* pMotPrimFilename = NULL;

	//get the name of motion primitives file
	if(argc > 2)
	{
		strcpy(sMotPrimFilename, argv[2]);
		pMotPrimFilename = sMotPrimFilename;
		SBPL_PRINTF("using motion primitive file %s\n", pMotPrimFilename);
	}
	else
	{
		pMotPrimFilename = NULL;
		SBPL_PRINTF("no motion primitives file provided\n");
	}

	//srand(0);

    //initialize true map and robot map
	if(!trueenvironment_navxythetalat.InitializeEnv(argv[1]))
	{
		SBPL_ERROR("ERROR: InitializeEnv failed\n");
		throw new SBPL_Exception();
	}

	//get environment parameters
	vector<SBPL_xytheta_mprimitive> motionprimitiveV;
	trueenvironment_navxythetalat.GetEnvParms(&size_x, &size_y, &startx, &starty, &starttheta, &goalx, &goaly, &goaltheta, &cellsize_m, 
		&nominalvel_mpersecs, &timetoturn45degsinplace_secs, &obsthresh, &motionprimitiveV);	
	costinscribed_thresh = trueenvironment_navxythetalat.GetEnvParameter("cost_inscribed_thresh");
	costcircum_thresh = trueenvironment_navxythetalat.GetEnvParameter("cost_possibly_circumscribed_thresh");

    unsigned char* map = (unsigned char*)calloc(size_x*size_y, sizeof(unsigned char));

	//print the map
	if(bPrintMap) SBPL_PRINTF("true map:\n");
	for(y = 0; bPrintMap && y < size_y; y++){
		for(x = 0; x < size_x; x++){
			SBPL_PRINTF("%3d ", trueenvironment_navxythetalat.GetMapCost(x,y));
		}
		SBPL_PRINTF("\n");
	}
  if(bPrint) SBPL_PRINTF("System Pause (return=%d)\n",system("pause"));

	SBPL_PRINTF("start: %f %f %f, goal: %f %f %f\n", startx, starty, starttheta, goalx, goaly, goaltheta);

	//set the perimeter of the robot (it is given with 0,0,0 robot ref. point for which planning is done)
	vector<sbpl_2Dpt_t> perimeterptsV;
	sbpl_2Dpt_t pt_m;
	double halfwidth = 0.025; //0.3;
	double halflength = 0.025; //0.45;
	pt_m.x = -halflength;
	pt_m.y = -halfwidth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = halflength;
	pt_m.y = -halfwidth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = halflength;
	pt_m.y = halfwidth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = -halflength;
	pt_m.y = halfwidth;
	perimeterptsV.push_back(pt_m);

	//compute sensing
	double maxx = 0;
	double maxy = 0;
	for(i = 0; i < (int)perimeterptsV.size(); i++)
	{
		if(maxx < fabs(perimeterptsV.at(i).x))
			maxx = fabs(perimeterptsV.at(i).x);
		if(maxy < fabs(perimeterptsV.at(i).y))
			maxy = fabs(perimeterptsV.at(i).y);
	}
	//TODO - when running it, no obstacle show up when it is 20 and not 2 - bug
	int sensingrange_c = (int)(__max(maxx, maxy)/cellsize_m) + 2; //should be big enough to cover the motion primitive length
	SBPL_PRINTF("sensing range=%d cells\n", sensingrange_c);
	vector<sbpl_2Dcell_t> sensecells;
	for(i = -sensingrange_c; i <= sensingrange_c; i++)
	{
		sbpl_2Dcell_t sensecell;

		sensecell.x = i;
		sensecell.y = sensingrange_c;
		sensecells.push_back(sensecell);
		sensecell.x = i;
		sensecell.y = -sensingrange_c;
		sensecells.push_back(sensecell);
		sensecell.x = sensingrange_c;
		sensecell.y = i;
		sensecells.push_back(sensecell);
		sensecell.x = -sensingrange_c;
		sensecell.y = i;
		sensecells.push_back(sensecell);
	}


	//Set environment parameters (should be done before initialize function is called)
	//set parameters - should be done before initialization 
	if(!environment_navxythetalat.SetEnvParameter("cost_inscribed_thresh", costinscribed_thresh))
	{
		SBPL_ERROR("ERROR: failed to set parameters\n");
		throw new SBPL_Exception();
	}
	if(!environment_navxythetalat.SetEnvParameter("cost_possibly_circumscribed_thresh", costcircum_thresh))
	{
		SBPL_ERROR("ERROR: failed to set parameters\n");
		throw new SBPL_Exception();
	}
	
	//Initialize Environment (should be called before initializing anything else)
    if(!environment_navxythetalat.InitializeEnv(size_x, size_y, 0, 0,0,0,0,0,0, 
		goaltol_x, goaltol_y, goaltol_theta, perimeterptsV,
		cellsize_m, nominalvel_mpersecs, timetoturn45degsinplace_secs, 
		obsthresh, pMotPrimFilename)){
			SBPL_ERROR("ERROR: InitializeEnv failed\n");
			throw new SBPL_Exception();
	}

	/*	
    if(!environment_navxythetalat.InitializeEnv(size_x, size_y, map, startx, starty, starttheta, goalx, goaly, goaltheta, 
		goaltol_x, goaltol_y, goaltol_theta, perimeterptsV,
		cellsize_m, nominalvel_mpersecs, timetoturn45degsinplace_secs, 
		obsthresh)){
		SBPL_ERROR("ERROR: InitializeEnv failed\n");
		throw new SBPL_Exception();
	}
	*/
	//set start state
	environment_navxythetalat.SetStart(startx, starty,starttheta);

	//Initialize MDP Info
	if(!environment_navxythetalat.InitializeMDPCfg(&MDPCfg))
	{
		SBPL_ERROR("ERROR: InitializeMDPCfg failed\n");
		throw new SBPL_Exception();
	}


	//create a planner
	vector<int> solution_stateIDs_V;
	bool bforwardsearch = false;
    //ARAPlanner planner(&environment_navxythetalat, bforwardsearch);
	ADPlanner planner(&environment_navxythetalat, bforwardsearch);

	planner.set_initialsolution_eps(3.0); 

	//set search mode
	planner.set_search_mode(bsearchuntilfirstsolution);

    //set the start and goal configurations
    if(planner.set_start(MDPCfg.startstateid) == 0)
        {
            SBPL_ERROR("ERROR: failed to set start state\n");
            throw new SBPL_Exception();
        }


	MDPCfg.goalstateid = environment_navxythetalat.SetGoal(goalx, goaly, goaltheta);


    if(planner.set_goal(MDPCfg.goalstateid) == 0)
        {
            SBPL_ERROR("ERROR: failed to set goal state\n");
            throw new SBPL_Exception();
        }

    //now comes the main loop
	int goalx_c = CONTXY2DISC(goalx, cellsize_m);
	int goaly_c = CONTXY2DISC(goaly, cellsize_m);
	int goaltheta_c = ContTheta2Disc(goaltheta, NAVXYTHETALAT_THETADIRS);
	SBPL_PRINTF("goal_c: %d %d %d\n", goalx_c, goaly_c, goaltheta_c);
	vector<EnvNAVXYTHETALAT3Dpt_t> xythetaPath;
    while(fabs(startx - goalx) > goaltol_x || fabs(starty - goaly) > goaltol_y || fabs(starttheta - goaltheta) > goaltol_theta){

        //simulate sensor data update
        bool bChanges = false;
		preds_of_changededgesIDV.clear();
		changedcellsV.clear();
        for(i = 0; i < (int)sensecells.size(); i++){
            int x = CONTXY2DISC(startx,cellsize_m) + sensecells.at(i).x;
            int y = CONTXY2DISC(starty,cellsize_m) + sensecells.at(i).y;
            if(x < 0 || x >= size_x || y < 0 || y >= size_y)
                continue;
            int index = x + y*size_x;
			unsigned char truecost = trueenvironment_navxythetalat.GetMapCost(x,y);
            if(map[index] != truecost){
                map[index] = truecost;
                environment_navxythetalat.UpdateCost(x,y,map[index]);
                SBPL_PRINTF("setting cost[%d][%d] to %d\n", x,y,map[index]);
                bChanges = true;
				//store the changed cells
				nav2dcell.x = x;
				nav2dcell.y = y;
				changedcellsV.push_back(nav2dcell);
            }
        }

		double TimeStarted = clock();

        if(bChanges){
            //planner.costs_changed(); //use by ARA* planner (non-incremental)

			//get the affected states
			environment_navxythetalat.GetPredsofChangedEdges(&changedcellsV, &preds_of_changededgesIDV);
			//let know the incremental planner about them
			planner.update_preds_of_changededges(&preds_of_changededgesIDV); //use by AD* planner (incremental)
			//SBPL_PRINTF("%d states were affected\n", preds_of_changededgesIDV.size());

        }

		int startx_c = CONTXY2DISC(startx,cellsize_m);
		int starty_c = CONTXY2DISC(starty,cellsize_m);
		int starttheta_c = ContTheta2Disc(starttheta, NAVXYTHETALAT_THETADIRS);
		
        //plan a path 
        bool bPlanExists = false;

		SBPL_PRINTF("new planning...\n");   
        bPlanExists = (planner.replan(allocated_time_secs_foreachplan, &solution_stateIDs_V) == 1);
        SBPL_PRINTF("done with the solution of size=%d and sol. eps=%f\n", (unsigned int)solution_stateIDs_V.size(), planner.get_solution_eps());   
        environment_navxythetalat.PrintTimeStat(stdout);

		SBPL_FPRINTF(fSol, "plan time=%.5f eps=%.2f\n", (clock()-TimeStarted)/((double)CLOCKS_PER_SEC), planner.get_solution_eps());
		SBPL_FFLUSH(fSol);
		
		xythetaPath.clear();
		environment_navxythetalat.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs_V, &xythetaPath);
		SBPL_PRINTF("actual path (with intermediate poses) size=%d\n", (unsigned int)xythetaPath.size());
		for(unsigned int i = 0; i < xythetaPath.size(); i++) {
			SBPL_FPRINTF(fSol, "%.3f %.3f %.3f\n", xythetaPath.at(i).x, xythetaPath.at(i).y, xythetaPath.at(i).theta);
		}
        SBPL_FPRINTF(fSol, "*********\n");

		for(int j = 1; j < (int)solution_stateIDs_V.size(); j++)
		{
			int newx, newy, newtheta=0;
			environment_navxythetalat.GetCoordFromState(solution_stateIDs_V[j], newx, newy, newtheta); 
			SBPL_FPRINTF(fSol, "%d %d %d\n", newx, newy, newtheta); 
		}
		SBPL_FFLUSH(fSol);


		//print the map
		int startindex = startx_c + starty_c*size_x;
		int goalindex = goalx_c + goaly_c*size_x;
		for(y = 0; bPrintMap && y < size_y; y++){
			for(x = 0; x < size_x; x++){
				int index = x + y*size_x;

				//check to see if it is on the path
				bool bOnthePath = false;
				for(int j = 1; j < (int)solution_stateIDs_V.size(); j++)
				{
					int newx, newy, newtheta=0;
					environment_navxythetalat.GetCoordFromState(solution_stateIDs_V[j], newx, newy, newtheta); 
					if(x == newx && y == newy)
						bOnthePath = true;
				}

				if (index != startindex && index != goalindex && !bOnthePath)
					SBPL_PRINTF("%3d ", map[index]);
				else if(index == startindex)
					SBPL_PRINTF("  X ");
				else if(index == goalindex)
					SBPL_PRINTF("  G ");
				else if (bOnthePath)
					SBPL_PRINTF("  * ");
				else
					SBPL_PRINTF("? ");
			}
			SBPL_PRINTF("\n");
		}

        //move along the path
        if(bPlanExists && (int) xythetaPath.size() > 1){
            //get coord of the successor
            int newx, newy, newtheta;
			
			//move until we move into the end of motion primitive
			environment_navxythetalat.GetCoordFromState(solution_stateIDs_V[1], newx, newy, newtheta); 

            SBPL_PRINTF("moving from %d %d %d to %d %d %d\n", startx_c, starty_c, starttheta_c, newx, newy, newtheta);
 
			//this check is weak since true configuration does not know the actual perimeter of the robot
			if(!trueenvironment_navxythetalat.IsValidConfiguration(newx,newy,newtheta)) 
			{
				SBPL_ERROR("ERROR: robot is commanded to move into an invalid configuration according to true environment\n");
				throw new SBPL_Exception();
			}
			if(!trueenvironment_navxythetalat.IsValidConfiguration(newx,newy,newtheta))
			{
				SBPL_ERROR("ERROR: robot is commanded to move into an invalid configuration\n");
				throw new SBPL_Exception();
			}

            //move
            startx = DISCXY2CONT(newx, cellsize_m);
            starty = DISCXY2CONT(newy, cellsize_m);
			starttheta = DiscTheta2Cont(newtheta, NAVXYTHETALAT_THETADIRS);
			
            //update the environment
            int newstartstateID = environment_navxythetalat.SetStart(startx, starty,starttheta);

            //update the planner
            if(planner.set_start(newstartstateID) == 0){               
                SBPL_ERROR("ERROR: failed to update robot pose in the planner\n");
                throw new SBPL_Exception();
            }
        }
		else
		{
			SBPL_PRINTF("No move is made\n");
		}

    if(bPrint) SBPL_PRINTF("System Pause (return=%d)\n",system("pause"));


    }

	SBPL_PRINTF("goal reached!\n");

	SBPL_FFLUSH(NULL);
  SBPL_FCLOSE(fSol);


    return 1;
}


int planrobarm(int argc, char *argv[])
{

	int bRet = 0;
	double allocated_time_secs = 5.0; //in seconds
	MDPConfig MDPCfg;
	
	//Initialize Environment (should be called before initializing anything else)
	EnvironmentROBARM environment_robarm;
	if(!environment_robarm.InitializeEnv(argv[1]))
	{
		SBPL_ERROR("ERROR: InitializeEnv failed\n");
		throw new SBPL_Exception();
	}

	//Initialize MDP Info
	if(!environment_robarm.InitializeMDPCfg(&MDPCfg))
	{
		SBPL_ERROR("ERROR: InitializeMDPCfg failed\n");
		throw new SBPL_Exception();
	}

	//srand(1); //TODO

	//plan a path
	vector<int> solution_stateIDs_V;
	bool bforwardsearch = true;
	//ARAPlanner planner(&environment_robarm, bforwardsearch);
	RSTARPlanner planner(&environment_robarm, bforwardsearch);

    if(planner.set_start(MDPCfg.startstateid) == 0)
        {
            SBPL_ERROR("ERROR: failed to set start state\n");
            throw new SBPL_Exception();
        }

    if(planner.set_goal(MDPCfg.goalstateid) == 0)
        {
            SBPL_ERROR("ERROR: failed to set goal state\n");
            throw new SBPL_Exception();
        }

    SBPL_PRINTF("start planning...\n");
	bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    SBPL_PRINTF("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

#ifndef ROS
  const char* sol = "sol.txt";
#endif
  FILE* fSol = SBPL_FOPEN(sol, "w");
  if(fSol == NULL){
    SBPL_ERROR("ERROR: could not open solution file\n");
    throw new SBPL_Exception();
  }
	for(unsigned int i = 0; i < solution_stateIDs_V.size(); i++) {
	  environment_robarm.PrintState(solution_stateIDs_V[i], true, fSol);
	}
    SBPL_FCLOSE(fSol);


 	//print a path
	if(bRet)
	{
		//print the solution
		SBPL_PRINTF("Solution is found\n");
	}
	else
		SBPL_PRINTF("Solution does not exist\n");

	SBPL_FFLUSH(NULL);


    return bRet;
}






int main(int argc, char *argv[])
{
#ifdef ROS
  ros::init(argc, argv, "sbpl_test");
#endif

	if(argc < 2)
	{
		PrintUsage(argv);
		throw new SBPL_Exception();
	}

    //2D planning
	//usage: exename 2Denvironmentfile.cfg
	//2Denvironmentfile.cfg files can be found sbpl/env_examples/nav2d
    //plan2d(argc, argv);
    //planandnavigate2d(argc, argv);

    //xytheta planning
	//usage: exename 3Denvironmentfile.cfg motionprimitivesfile.mprim 
	//3Denvironmentfile.cfg files can be found sbpl/env_examples/nav3d
	//the motionprimitives files can be found in matlab/mprim directory
	//note: the resolution of the motion primitives files should match the resolution of the cfg files
    planxythetalat(argc, argv);

    //xytheta planning
	//usage: see the comments for planxythetalat() above
    //planandnavigatexythetalat(argc, argv);

	//xytheta with multiple levels (i.e., base of the robot and upper body)
	//usage: see the comments for planxythetalat() above
	planxythetamlevlat(argc, argv);

    //robotarm planning
	//usage: exename robarmenvironmentfile.cfg
	//robarmenvironmentfile.cfg files can be found sbpl/env_examples/robarm
    //planrobarm(argc, argv);

	//plan under uncertainty (with incomplete information to be exact)
	//not functional yet
	//plan2duu(argc, argv); //not fully implemented yet

	return 0;
}





