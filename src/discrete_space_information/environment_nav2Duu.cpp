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

#include <cstdlib>
#include <cstring>
#include <sbpl/discrete_space_information/environment_nav2Duu.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/mdpconfig.h>

using namespace std;

//extern clock_t time3_addallout;
//extern clock_t time_gethash;
//extern clock_t time_createhash;

#define NAV2DUU_ERR_EPS 0.00001

//function prototypes

//-------------------constructors---------------------
EnvironmentNAV2DUU::EnvironmentNAV2DUU()
{
    EnvNAV2DUU.bInitialized = false;
}

//-----------------------------------------------------

//-------------------problem specific and local functions---------------------

void EnvironmentNAV2DUU::ReadConfiguration(FILE* fCfg)
{
    //read in the configuration of environment and initialize  EnvCfg structure
    char sTemp[1024], sTemp1[1024];
    //int dTemp;
    int x, y;
    float fTemp;

    //discretization(cells)
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    strcpy(sTemp1, "discretization(cells):");
    if (strcmp(sTemp1, sTemp) != 0) {
        SBPL_ERROR("ERROR: configuration file has incorrect format\n");
        SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
        throw new SBPL_Exception();
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvNAV2DUUCfg.EnvWidth_c = atoi(sTemp);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvNAV2DUUCfg.EnvHeight_c = atoi(sTemp);

    //obsthresh:
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    strcpy(sTemp1, "obsthresh:");
    if (strcmp(sTemp1, sTemp) != 0) {
        SBPL_ERROR("ERROR: configuration file has incorrect format\n");
        SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
        throw new SBPL_Exception();
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvNAV2DUUCfg.obsthresh = (int)(atof(sTemp));

    //start(cells):
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvNAV2DUUCfg.StartX_c = atoi(sTemp);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvNAV2DUUCfg.StartY_c = atoi(sTemp);

    if (EnvNAV2DUUCfg.StartX_c < 0 || EnvNAV2DUUCfg.StartX_c >= EnvNAV2DUUCfg.EnvWidth_c) {
        SBPL_ERROR("ERROR: illegal start coordinates\n");
        throw new SBPL_Exception();
    }
    if (EnvNAV2DUUCfg.StartY_c < 0 || EnvNAV2DUUCfg.StartY_c >= EnvNAV2DUUCfg.EnvHeight_c) {
        SBPL_ERROR("ERROR: illegal start coordinates\n");
        throw new SBPL_Exception();
    }

    //end(cells):
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvNAV2DUUCfg.EndX_c = atoi(sTemp);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvNAV2DUUCfg.EndY_c = atoi(sTemp);

    if (EnvNAV2DUUCfg.EndX_c < 0 || EnvNAV2DUUCfg.EndX_c >= EnvNAV2DUUCfg.EnvWidth_c) {
        SBPL_ERROR("ERROR: illegal end coordinates\n");
        throw new SBPL_Exception();
    }
    if (EnvNAV2DUUCfg.EndY_c < 0 || EnvNAV2DUUCfg.EndY_c >= EnvNAV2DUUCfg.EnvHeight_c) {
        SBPL_ERROR("ERROR: illegal end coordinates\n");
        throw new SBPL_Exception();
    }

    //allocate the 2D environment and corresponding uncertainty matrix
    EnvNAV2DUUCfg.Grid2D = new unsigned char*[EnvNAV2DUUCfg.EnvWidth_c];
    EnvNAV2DUUCfg.UncertaintyGrid2D = new float*[EnvNAV2DUUCfg.EnvWidth_c];
    for (x = 0; x < EnvNAV2DUUCfg.EnvWidth_c; x++) {
        EnvNAV2DUUCfg.Grid2D[x] = new unsigned char[EnvNAV2DUUCfg.EnvHeight_c];
        EnvNAV2DUUCfg.UncertaintyGrid2D[x] = new float[EnvNAV2DUUCfg.EnvHeight_c];
    }

    //environment:
    EnvNAV2DUUCfg.sizeofH = 0;
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    for (y = 0; y < EnvNAV2DUUCfg.EnvHeight_c; y++)
        for (x = 0; x < EnvNAV2DUUCfg.EnvWidth_c; x++) {
            if (fscanf(fCfg, "%f", &fTemp) != 1) {
                SBPL_ERROR("ERROR: incorrect format of config file\n");
                throw new SBPL_Exception();
            }

            if (fTemp > 1.0 - NAV2DUU_ERR_EPS || fTemp < NAV2DUU_ERR_EPS) {
                //we assume that the value is just a cost
                EnvNAV2DUUCfg.Grid2D[x][y] = (int)fTemp;
                if (EnvNAV2DUUCfg.Grid2D[x][y] >= EnvNAV2DUUCfg.obsthresh)
                    EnvNAV2DUUCfg.UncertaintyGrid2D[x][y] = 1.0;
                else
                    EnvNAV2DUUCfg.UncertaintyGrid2D[x][y] = 0.0;
            }
            else {
                //the value is probability of being free
                EnvNAV2DUUCfg.Grid2D[x][y] = 0; //assume cost is 0 if traversable
                EnvNAV2DUUCfg.UncertaintyGrid2D[x][y] = fTemp;
                EnvNAV2DUUCfg.sizeofH++;
            }
        }

    EnvNAV2DUUCfg.sizeofS = this->EnvNAV2DUUCfg.EnvWidth_c * this->EnvNAV2DUUCfg.EnvHeight_c;

    SBPL_PRINTF("total size of environment=%d, number of unknown cells=%d\n", EnvNAV2DUUCfg.sizeofS,
                EnvNAV2DUUCfg.sizeofH);
}

//mapdata and uncertaintymapdata is assumed to be organized into a linear array with y being major: map[x+y*width]
void EnvironmentNAV2DUU::SetConfiguration(int width, int height, const unsigned char* mapdata,
                                          const float* uncertaintymapdata)
{
    EnvNAV2DUUCfg.EnvWidth_c = width;
    EnvNAV2DUUCfg.EnvHeight_c = height;

    int x, y;

    //set start and goal to zeros for now
    EnvNAV2DUUCfg.StartX_c = 0;
    EnvNAV2DUUCfg.StartY_c = 0;
    EnvNAV2DUUCfg.EndX_c = 0;
    EnvNAV2DUUCfg.EndY_c = 0;

    //environment:
    //allocate the 2D environment and corresponding uncertainty matrix
    EnvNAV2DUUCfg.Grid2D = new unsigned char*[EnvNAV2DUUCfg.EnvWidth_c];
    EnvNAV2DUUCfg.UncertaintyGrid2D = new float*[EnvNAV2DUUCfg.EnvWidth_c];
    for (x = 0; x < EnvNAV2DUUCfg.EnvWidth_c; x++) {
        EnvNAV2DUUCfg.Grid2D[x] = new unsigned char[EnvNAV2DUUCfg.EnvHeight_c];
        EnvNAV2DUUCfg.UncertaintyGrid2D[x] = new float[EnvNAV2DUUCfg.EnvHeight_c];
    }

    //initialize the mape
    EnvNAV2DUUCfg.sizeofH = 0;
    for (y = 0; y < EnvNAV2DUUCfg.EnvHeight_c; y++) {
        for (x = 0; x < EnvNAV2DUUCfg.EnvWidth_c; x++) {
            if (mapdata == NULL) {
                //special case - all is traversable
                EnvNAV2DUUCfg.Grid2D[x][y] = 0;
                EnvNAV2DUUCfg.UncertaintyGrid2D[x][y] = 0;
            }
            else {
                EnvNAV2DUUCfg.Grid2D[x][y] = mapdata[x + y * width];
                EnvNAV2DUUCfg.UncertaintyGrid2D[x][y] = uncertaintymapdata[x + y * width];
                if (EnvNAV2DUUCfg.UncertaintyGrid2D[x][y] >= NAV2DUU_ERR_EPS && EnvNAV2DUUCfg.UncertaintyGrid2D[x][y]
                    <= (1.0 - NAV2DUU_ERR_EPS)) EnvNAV2DUUCfg.sizeofH++;
            }
        }
    }

    EnvNAV2DUUCfg.sizeofS = this->EnvNAV2DUUCfg.EnvWidth_c * this->EnvNAV2DUUCfg.EnvHeight_c;

    SBPL_PRINTF("total size of environment=%d, number of unknown cells=%d\n", EnvNAV2DUUCfg.sizeofS,
                EnvNAV2DUUCfg.sizeofH);
}

void EnvironmentNAV2DUU::InitializeEnvironment()
{
    //initialize goal/start IDs
    EnvNAV2DUU.startstateid = ENVNAV2DUU_XYTOSTATEID(EnvNAV2DUUCfg.StartX_c, EnvNAV2DUUCfg.StartY_c);
    EnvNAV2DUU.goalstateid = ENVNAV2DUU_XYTOSTATEID(EnvNAV2DUUCfg.EndX_c, EnvNAV2DUUCfg.EndY_c);

    //environment initialized
    EnvNAV2DUU.bInitialized = true;
}

void EnvironmentNAV2DUU::InitializeEnvConfig()
{
    //aditional to configuration file initialization if necessary
    Computedxy();

    //compute IDs of hidden variables
    int x, y;
    int idcount = 0;
    EnvNAV2DUUCfg.HiddenVariableXY2ID = new int*[EnvNAV2DUUCfg.EnvWidth_c];
    for (x = 0; x < EnvNAV2DUUCfg.EnvWidth_c; x++) {
        EnvNAV2DUUCfg.HiddenVariableXY2ID[x] = new int[EnvNAV2DUUCfg.EnvHeight_c];
        for (y = 0; y < EnvNAV2DUUCfg.EnvWidth_c; y++) {
            if (EnvNAV2DUUCfg.UncertaintyGrid2D[x][y] >= NAV2DUU_ERR_EPS && EnvNAV2DUUCfg.UncertaintyGrid2D[x][y]
                <= (1.0 - NAV2DUU_ERR_EPS)) {
                EnvNAV2DUUCfg.HiddenVariableXY2ID[x][y] = idcount;
                idcount++;
            }
            else
                EnvNAV2DUUCfg.HiddenVariableXY2ID[x][y] = -1;
        }
    }

    if (idcount != EnvNAV2DUUCfg.sizeofH) {
        SBPL_ERROR("ERROR: idcount not equal to sizeofH\n");
        throw new SBPL_Exception();
    }
}

int EnvironmentNAV2DUU::SizeofCreatedEnv()
{
    return EnvNAV2DUUCfg.sizeofS;
}

int EnvironmentNAV2DUU::SizeofH()
{
    return EnvNAV2DUUCfg.sizeofH;
}

bool EnvironmentNAV2DUU::IsValidRobotPosition(int X, int Y)
{
    return (X >= 0 && X < EnvNAV2DUUCfg.EnvWidth_c && Y >= 0 && Y < EnvNAV2DUUCfg.EnvHeight_c &&
            EnvNAV2DUUCfg.Grid2D[X][Y] < EnvNAV2DUUCfg.obsthresh &&
            EnvNAV2DUUCfg.UncertaintyGrid2D[X][Y] < NAV2DUU_ERR_EPS);
}

bool EnvironmentNAV2DUU::IsWithinMapCell(int X, int Y)
{
    return (X >= 0 && X < EnvNAV2DUUCfg.EnvWidth_c && Y >= 0 && Y < EnvNAV2DUUCfg.EnvHeight_c);
}

void EnvironmentNAV2DUU::Computedxy()
{
    //initialize some constants for 2D search
    EnvNAV2DUUCfg.dx_[0] = 1;
    EnvNAV2DUUCfg.dy_[0] = 1;
    EnvNAV2DUUCfg.dxintersects_[0][0] = 0;
    EnvNAV2DUUCfg.dyintersects_[0][0] = 1;
    EnvNAV2DUUCfg.dxintersects_[0][1] = 1;
    EnvNAV2DUUCfg.dyintersects_[0][1] = 0;
    EnvNAV2DUUCfg.dx_[1] = 1;
    EnvNAV2DUUCfg.dy_[1] = 0;
    EnvNAV2DUUCfg.dxintersects_[1][0] = 0;
    EnvNAV2DUUCfg.dyintersects_[1][0] = 0;
    EnvNAV2DUUCfg.dxintersects_[1][1] = 0;
    EnvNAV2DUUCfg.dyintersects_[1][1] = 0;
    EnvNAV2DUUCfg.dx_[2] = 1;
    EnvNAV2DUUCfg.dy_[2] = -1;
    EnvNAV2DUUCfg.dxintersects_[2][0] = 0;
    EnvNAV2DUUCfg.dyintersects_[2][0] = -1;
    EnvNAV2DUUCfg.dxintersects_[2][1] = 1;
    EnvNAV2DUUCfg.dyintersects_[2][1] = 0;
    EnvNAV2DUUCfg.dx_[3] = 0;
    EnvNAV2DUUCfg.dy_[3] = 1;
    EnvNAV2DUUCfg.dxintersects_[3][0] = 0;
    EnvNAV2DUUCfg.dyintersects_[3][0] = 0;
    EnvNAV2DUUCfg.dxintersects_[3][1] = 0;
    EnvNAV2DUUCfg.dyintersects_[3][1] = 0;
    EnvNAV2DUUCfg.dx_[4] = 0;
    EnvNAV2DUUCfg.dy_[4] = -1;
    EnvNAV2DUUCfg.dxintersects_[4][0] = 0;
    EnvNAV2DUUCfg.dyintersects_[4][0] = 0;
    EnvNAV2DUUCfg.dxintersects_[4][1] = 0;
    EnvNAV2DUUCfg.dyintersects_[4][1] = 0;
    EnvNAV2DUUCfg.dx_[5] = -1;
    EnvNAV2DUUCfg.dy_[5] = 1;
    EnvNAV2DUUCfg.dxintersects_[5][0] = 0;
    EnvNAV2DUUCfg.dyintersects_[5][0] = 1;
    EnvNAV2DUUCfg.dxintersects_[5][1] = -1;
    EnvNAV2DUUCfg.dyintersects_[5][1] = 0;
    EnvNAV2DUUCfg.dx_[6] = -1;
    EnvNAV2DUUCfg.dy_[6] = 0;
    EnvNAV2DUUCfg.dxintersects_[6][0] = 0;
    EnvNAV2DUUCfg.dyintersects_[6][0] = 0;
    EnvNAV2DUUCfg.dxintersects_[6][1] = 0;
    EnvNAV2DUUCfg.dyintersects_[6][1] = 0;
    EnvNAV2DUUCfg.dx_[7] = -1;
    EnvNAV2DUUCfg.dy_[7] = -1;
    EnvNAV2DUUCfg.dxintersects_[7][0] = 0;
    EnvNAV2DUUCfg.dyintersects_[7][0] = -1;
    EnvNAV2DUUCfg.dxintersects_[7][1] = -1;
    EnvNAV2DUUCfg.dyintersects_[7][1] = 0;

    //compute distances
    for (int dind = 0; dind < ENVNAV2DUU_MAXDIRS; dind++) {

        if (EnvNAV2DUUCfg.dx_[dind] != 0 && EnvNAV2DUUCfg.dy_[dind] != 0) {
            if (dind <= 7) {
                //the cost of a diagonal move in millimeters
                EnvNAV2DUUCfg.dxy_distance_mm_[dind] = (int)(ENVNAV2DUU_COSTMULT * 1.414); 
            }
            else {
                //the cost of a move to 1,2 or 2,1 or so on in millimeters
                EnvNAV2DUUCfg.dxy_distance_mm_[dind] = (int)(ENVNAV2DUU_COSTMULT * 2.236); 
            }
        }
        else
            EnvNAV2DUUCfg.dxy_distance_mm_[dind] = ENVNAV2DUU_COSTMULT; //the cost of a horizontal move in millimeters
    }
}

/*
void EnvironmentNAV2DUU::ComputeReversedxy()
{
    int revaind;

    //iterate over actions
    for (int aind = 0; aind < NAVLSENV_ROBOTNAVACTIONSWIDTH; aind++)
    {
        //get the cell location and reverse it
        int dX  = -EnvNAVLSCfg.dXY[aind][0];
        int dY  = -EnvNAVLSCfg.dXY[aind][1];

        //find the index that corresponds to these offsets
        for (revaind = 0; revaind < NAVLSENV_ROBOTNAVACTIONSWIDTH; revaind++)
        {
            if(EnvNAVLSCfg.dXY[revaind][0] == dX && EnvNAVLSCfg.dXY[revaind][1] == dY)
            {
                EnvNAVLSCfg.reversedXY[aind] = revaind;
                break;
            }
        }

        if(revaind == NAVLSENV_ROBOTNAVACTIONSWIDTH)
        {
            SBPL_ERROR("ERROR: can not determine a reversed index for aind=%d (dX=%d dY=%d)\n",
                    aind, EnvNAVLSCfg.dXY[aind][0], EnvNAVLSCfg.dXY[aind][1]);
        }
    }
}
*/

//------------------------------------------------------------------------------

//------------------------------Heuristic computation--------------------------

void EnvironmentNAV2DUU::ComputeHeuristicValues()
{
    //whatever necessary pre-computation of heuristic values is done here
    SBPL_PRINTF("Precomputing heuristics\n");

    SBPL_PRINTF("done\n");
}

//------------------------------------------------------------------------------

//-----------------Printing Routines--------------------------------------------

void EnvironmentNAV2DUU::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG
    if (stateID >= this->EnvNAV2DUUCfg.EnvWidth_c*this->EnvNAV2DUUCfg.EnvHeight_c) {
        SBPL_ERROR("ERROR in EnvNAV2DUU... function: stateID illegal (2)\n");
        throw new SBPL_Exception();
    }
#endif

    if (fOut == NULL) fOut = stdout;

    if (stateID == EnvNAV2DUU.goalstateid && bVerbose) {
        SBPL_FPRINTF(fOut, "the state is a goal state\n");
    }

    if (bVerbose)
        SBPL_FPRINTF(fOut, "X=%d Y=%d\n", ENVNAV2DUU_STATEIDTOX(stateID), ENVNAV2DUU_STATEIDTOY(stateID));
    else
        SBPL_FPRINTF(fOut, "%d %d\n", ENVNAV2DUU_STATEIDTOX(stateID), ENVNAV2DUU_STATEIDTOY(stateID));
}

void EnvironmentNAV2DUU::PrintEnv_Config(FILE* fOut)
{
    //implement this if the planner needs to print out EnvNAV2DUU. configuration

    SBPL_ERROR("ERROR in EnvNAV2DUU... function: PrintEnv_Config is undefined\n");
    throw new SBPL_Exception();
}

//-------------------------------------------------------------------------------


//-----------interface with outside functions-----------------------------------

bool EnvironmentNAV2DUU::InitializeEnv(const char* sEnvFile)
{
    FILE* fCfg = fopen(sEnvFile, "r");
    if (fCfg == NULL) {
        SBPL_ERROR("ERROR: unable to open %s\n", sEnvFile);
        throw new SBPL_Exception();
    }
    ReadConfiguration(fCfg);
    fclose(fCfg);

    InitGeneral();

    return true;
}

bool EnvironmentNAV2DUU::InitializeEnv(int width, int height, const unsigned char* mapdata,
                                       const float* uncertaintymapdata, unsigned char obsthresh)
{
    SBPL_PRINTF("env: initialized with width=%d height=%d, obsthresh=%d\n", width, height, obsthresh);

    EnvNAV2DUUCfg.obsthresh = obsthresh;

    SetConfiguration(width, height, mapdata, uncertaintymapdata);

    InitGeneral();

    return true;
}

bool EnvironmentNAV2DUU::InitGeneral()
{
    //initialize other variables in Cfg
    InitializeEnvConfig();

    //initialize Environment
    InitializeEnvironment();

    //pre-compute heuristics
    ComputeHeuristicValues();

    return true;
}

bool EnvironmentNAV2DUU::InitializeMDPCfg(MDPConfig *MDPCfg)
{
    //initialize MDPCfg with the start and goal ids
    MDPCfg->goalstateid = EnvNAV2DUU.goalstateid;
    MDPCfg->startstateid = EnvNAV2DUU.startstateid;

    return true;
}

int EnvironmentNAV2DUU::GetFromToHeuristic(int FromStateID, int ToStateID)
{
#if USE_HEUR==0
    return 0;
#endif

    //define this function if it is used in the planner

    SBPL_ERROR("ERROR in EnvNAV2DUU.. function: FromToHeuristic is undefined\n");
    throw new SBPL_Exception();

    return 0;
}

int EnvironmentNAV2DUU::GetGoalHeuristic(int stateID)
{
#if USE_HEUR==0
    return 0;
#endif

    //define this function if it used in the planner (heuristic forward search would use it)

    SBPL_ERROR("ERROR in EnvNAV2DUU..function: GetGoalHeuristic is undefined\n");
    throw new SBPL_Exception();
}

int EnvironmentNAV2DUU::GetStartHeuristic(int stateID)
{
#if USE_HEUR==0
    return 0;
#endif

    //define this function if it used in the planner (heuristic backward search would use it)

    SBPL_ERROR("ERROR in EnvNAV2DUU.. function: GetStartHeuristic is undefined\n");
    throw new SBPL_Exception();

    return 0;
}

void EnvironmentNAV2DUU::GetPreds(int stateID, const vector<sbpl_BinaryHiddenVar_t>* updatedhvaluesV,
                                  vector<CMDPACTION>* IncomingDetActionV, vector<CMDPACTION>* IncomingStochActionV,
                                  vector<sbpl_BinaryHiddenVar_t>* StochActionNonpreferredOutcomeV)
{
    int aind;

    //get state coords
    int destx = ENVNAV2DUU_STATEIDTOX(stateID);
    int desty = ENVNAV2DUU_STATEIDTOY(stateID);

    //clear succs
    IncomingDetActionV->clear();
    IncomingStochActionV->clear();
    StochActionNonpreferredOutcomeV->clear();

    //check that the destination is valid
    if (!IsWithinMapCell(destx, desty) || EnvNAV2DUUCfg.Grid2D[destx][desty] >= EnvNAV2DUUCfg.obsthresh) return;

    //see if the destination was originally uncertain
    bool bDet = false;
    //no need to worry about ==1.0 since obstacles are rejected above
    if (EnvNAV2DUUCfg.UncertaintyGrid2D[destx][desty] < NAV2DUU_ERR_EPS) 
    bDet = true;

    //if yes, then figure out the probability of being an obstacle
    float ProbObs = EnvNAV2DUUCfg.UncertaintyGrid2D[destx][desty];
    int desth_ID = EnvNAV2DUUCfg.HiddenVariableXY2ID[destx][desty];
    bool bFreeh = false;

    //iterate over updated h-values to make sure dest cell is not in there
    for (int hind = 0; hind < (int)updatedhvaluesV->size(); hind++) {
        if (updatedhvaluesV->at(hind).Prob < NAV2DUU_ERR_EPS) bFreeh = true; //there are elements that are free

        if (updatedhvaluesV->at(hind).h_ID == desth_ID) {
            ProbObs = updatedhvaluesV->at(hind).Prob; //found
        }
    }

    //if now known to be an obstacle, then no preds
    if (ProbObs > 1.0 - NAV2DUU_ERR_EPS)
        return;
    else if (ProbObs < NAV2DUU_ERR_EPS) bDet = false; //now it is a deterministic cell

    //get the destination costmult
    int destcostmult = EnvNAV2DUUCfg.Grid2D[destx][desty];

#if DEBUG
    if (EnvNAV2DUUCfg.numofdirs > 8) {
        SBPL_ERROR("ERROR: number of directions can not exceed 8 for now\n");
        throw new SBPL_Exception();
    }
#endif

    //iterate over neighbors
    for (aind = 0; aind < EnvNAV2DUUCfg.numofdirs; aind++) {
        //the actions are undirected, so we can use the same array of actions as in getsuccs case
        int predX = destx + EnvNAV2DUUCfg.dx_[aind];
        int predY = desty + EnvNAV2DUUCfg.dy_[aind];

        //skip the invalid cells
        if (!IsWithinMapCell(predX, predY) || EnvNAV2DUUCfg.Grid2D[predX][predY] >= EnvNAV2DUUCfg.obsthresh) continue;

        //running costmult
        int costmult = destcostmult;

        //get the costmultiplier of pred and update total costmult
        costmult = __max(costmult, EnvNAV2DUUCfg.Grid2D[predX][predY]);

        //if diagonal move
        if (predX != destx && predY != desty) {
            //check intersecting cells to make sure they are not uncertain
            if (EnvNAV2DUUCfg.UncertaintyGrid2D[destx][predY] >= NAV2DUU_ERR_EPS) {
                if (!bFreeh) continue;

                //check that it is not known to be free. Otherwise - invalid move
                int tempID = EnvNAV2DUUCfg.HiddenVariableXY2ID[destx][predY];
                bool btempfree = false;
                for (int hind = 0; hind < (int)updatedhvaluesV->size(); hind++) {
                    if (updatedhvaluesV->at(hind).h_ID == tempID) {
                        if (updatedhvaluesV->at(hind).Prob < NAV2DUU_ERR_EPS) {
                            btempfree = true;
                            break;
                        }
                    }
                }
                if (!btempfree) continue;
            }

            if (EnvNAV2DUUCfg.UncertaintyGrid2D[predX][desty] >= NAV2DUU_ERR_EPS) {
                if (!bFreeh) continue;

                //check that it is not known to be free. Otherwise - invalid move
                int tempID = EnvNAV2DUUCfg.HiddenVariableXY2ID[predX][desty];
                bool btempfree = false;
                for (int hind = 0; hind < (int)updatedhvaluesV->size(); hind++) {
                    if (updatedhvaluesV->at(hind).h_ID == tempID) {
                        if (updatedhvaluesV->at(hind).Prob < NAV2DUU_ERR_EPS) {
                            btempfree = true;
                            break;
                        }
                    }
                }
                if (!btempfree) continue;
            }

            //compute the costmultiplier of intersect cells and update total costmult
            costmult = __max(costmult, EnvNAV2DUUCfg.Grid2D[destx][predY]);
            costmult = __max(costmult, EnvNAV2DUUCfg.Grid2D[predX][desty]);
        }

        //make sure cost is still valid
        if (costmult >= EnvNAV2DUUCfg.obsthresh) continue;

        //otherwise compute the actual cost (once again we use the fact that
        //actions are undirected to determine the cost)
        int cost = (((int)costmult) + 1) * EnvNAV2DUUCfg.dxy_distance_mm_[aind];

        //create action
        int predstateID = ENVNAV2DUU_XYTOSTATEID(predX, predY);
        CMDPACTION action(aind, predstateID); //TODO-use reverseindex

        if (bDet) {
            //if dest is known - then form a deterministic action
            action.AddOutcome(stateID, cost, 1.0);
            IncomingDetActionV->push_back(action);
        }
        else {
            //if dest is unknown - then form a stoch action and compute the corresponding belief part
            action.AddOutcome(stateID, cost, 1 - ProbObs); //preferred outcome
            action.AddOutcome(predstateID, 2 * cost, ProbObs); //non-preferred outcome (stateID is untraversable)
            IncomingStochActionV->push_back(action);
            //also insert the corresponding hidden variable value
            sbpl_BinaryHiddenVar_t hval;
            hval.h_ID = desth_ID;
            hval.Prob = 1; //known to be an obstacle
            StochActionNonpreferredOutcomeV->push_back(hval);
        }
    }
}

//returns the stateid if success, and -1 otherwise
int EnvironmentNAV2DUU::SetGoal(int x, int y)
{
    if (!IsWithinMapCell(x, y)) {
        SBPL_ERROR("ERROR: trying to set a goal cell %d %d that is outside of map\n", x, y);
        return -1;
    }

    if (!IsValidRobotPosition(x, y)) {
        SBPL_PRINTF("WARNING: goal cell is invalid\n");
    }

    EnvNAV2DUU.goalstateid = ENVNAV2DUU_XYTOSTATEID(x, y);
    EnvNAV2DUUCfg.EndX_c = x;
    EnvNAV2DUUCfg.EndY_c = y;

    return EnvNAV2DUU.goalstateid;
}

//returns the stateid if success, and -1 otherwise
int EnvironmentNAV2DUU::SetStart(int x, int y)
{
    if (!IsWithinMapCell(x, y)) {
        SBPL_ERROR("ERROR: trying to set a start cell %d %d that is outside of map\n", x, y);
        return -1;
    }

    if (!IsValidRobotPosition(x, y)) {
        SBPL_PRINTF("WARNING: start cell is invalid\n");
    }

    EnvNAV2DUU.startstateid = ENVNAV2DUU_XYTOSTATEID(x, y);
    EnvNAV2DUUCfg.StartX_c = x;
    EnvNAV2DUUCfg.StartY_c = y;

    return EnvNAV2DUU.startstateid;
}

bool EnvironmentNAV2DUU::UpdateCost(int x, int y, unsigned char newcost)
{
    EnvNAV2DUUCfg.Grid2D[x][y] = newcost;

    return true;
}

//------------------------------------------------------------------------------
