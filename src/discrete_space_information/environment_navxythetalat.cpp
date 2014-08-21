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

#include <cmath>
#include <cstring>
#include <ctime>
#include <sbpl/discrete_space_information/environment_navxythetalat.h>
#include <sbpl/utils/2Dgridsearch.h>
#include <sbpl/utils/key.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/mdpconfig.h>

using namespace std;

#if TIME_DEBUG
static clock_t time3_addallout = 0;
static clock_t time_gethash = 0;
static clock_t time_createhash = 0;
static clock_t time_getsuccs = 0;
#endif

static long int checks = 0;

#define XYTHETA2INDEX(X,Y,THETA) (THETA + X*EnvNAVXYTHETALATCfg.NumThetaDirs + \
                                  Y*EnvNAVXYTHETALATCfg.EnvWidth_c*EnvNAVXYTHETALATCfg.NumThetaDirs)

//-----------------constructors/destructors-------------------------------

EnvironmentNAVXYTHETALATTICE::EnvironmentNAVXYTHETALATTICE()
{
    EnvNAVXYTHETALATCfg.obsthresh = ENVNAVXYTHETALAT_DEFAULTOBSTHRESH;
    //the value that pretty much makes it disabled
    EnvNAVXYTHETALATCfg.cost_inscribed_thresh = EnvNAVXYTHETALATCfg.obsthresh; 
    //the value that pretty much makes it disabled
    EnvNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh = -1; 

    grid2Dsearchfromstart = NULL;
    grid2Dsearchfromgoal = NULL;
    bNeedtoRecomputeStartHeuristics = true;
    bNeedtoRecomputeGoalHeuristics = true;
    iteration = 0;

    EnvNAVXYTHETALAT.bInitialized = false;

    EnvNAVXYTHETALATCfg.actionwidth = NAVXYTHETALAT_DEFAULT_ACTIONWIDTH;

    EnvNAVXYTHETALATCfg.NumThetaDirs = NAVXYTHETALAT_THETADIRS;

    //no memory allocated in cfg yet
    EnvNAVXYTHETALATCfg.Grid2D = NULL;
    EnvNAVXYTHETALATCfg.ActionsV = NULL;
    EnvNAVXYTHETALATCfg.PredActionsV = NULL;
}

EnvironmentNAVXYTHETALATTICE::~EnvironmentNAVXYTHETALATTICE()
{
    SBPL_PRINTF("destroying XYTHETALATTICE\n");
    if (grid2Dsearchfromstart != NULL) delete grid2Dsearchfromstart;
    grid2Dsearchfromstart = NULL;

    if (grid2Dsearchfromgoal != NULL) delete grid2Dsearchfromgoal;
    grid2Dsearchfromgoal = NULL;

    if (EnvNAVXYTHETALATCfg.Grid2D != NULL) {
        for (int x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++)
            delete[] EnvNAVXYTHETALATCfg.Grid2D[x];
        delete[] EnvNAVXYTHETALATCfg.Grid2D;
        EnvNAVXYTHETALATCfg.Grid2D = NULL;
    }

    //delete actions
    if (EnvNAVXYTHETALATCfg.ActionsV != NULL) {
        for (int tind = 0; tind < EnvNAVXYTHETALATCfg.NumThetaDirs; tind++)
            delete[] EnvNAVXYTHETALATCfg.ActionsV[tind];
        delete[] EnvNAVXYTHETALATCfg.ActionsV;
        EnvNAVXYTHETALATCfg.ActionsV = NULL;
    }
    if (EnvNAVXYTHETALATCfg.PredActionsV != NULL) {
        delete[] EnvNAVXYTHETALATCfg.PredActionsV;
        EnvNAVXYTHETALATCfg.PredActionsV = NULL;
    }
}

//---------------------------------------------------------------------

//-------------------problem specific and local functions---------------------

static unsigned int inthash(unsigned int key)
{
    key += (key << 12);
    key ^= (key >> 22);
    key += (key << 4);
    key ^= (key >> 9);
    key += (key << 10);
    key ^= (key >> 2);
    key += (key << 7);
    key ^= (key >> 12);
    return key;
}

void EnvironmentNAVXYTHETALATTICE::SetConfiguration(int width, int height, const unsigned char* mapdata, int startx,
                                                    int starty, int starttheta, int goalx, int goaly, int goaltheta,
                                                    double cellsize_m, double nominalvel_mpersecs,
                                                    double timetoturn45degsinplace_secs,
                                                    const vector<sbpl_2Dpt_t> & robot_perimeterV)
{
    EnvNAVXYTHETALATCfg.EnvWidth_c = width;
    EnvNAVXYTHETALATCfg.EnvHeight_c = height;
    EnvNAVXYTHETALATCfg.StartX_c = startx;
    EnvNAVXYTHETALATCfg.StartY_c = starty;
    EnvNAVXYTHETALATCfg.StartTheta = starttheta;

    if (EnvNAVXYTHETALATCfg.StartX_c < 0 || EnvNAVXYTHETALATCfg.StartX_c >= EnvNAVXYTHETALATCfg.EnvWidth_c) {
        SBPL_ERROR("ERROR: illegal start coordinates\n");
        throw new SBPL_Exception();
    }
    if (EnvNAVXYTHETALATCfg.StartY_c < 0 || EnvNAVXYTHETALATCfg.StartY_c >= EnvNAVXYTHETALATCfg.EnvHeight_c) {
        SBPL_ERROR("ERROR: illegal start coordinates\n");
        throw new SBPL_Exception();
    }
    if (EnvNAVXYTHETALATCfg.StartTheta < 0 || EnvNAVXYTHETALATCfg.StartTheta >= EnvNAVXYTHETALATCfg.NumThetaDirs) {
        SBPL_ERROR("ERROR: illegal start coordinates for theta\n");
        throw new SBPL_Exception();
    }

    EnvNAVXYTHETALATCfg.EndX_c = goalx;
    EnvNAVXYTHETALATCfg.EndY_c = goaly;
    EnvNAVXYTHETALATCfg.EndTheta = goaltheta;

    if (EnvNAVXYTHETALATCfg.EndX_c < 0 || EnvNAVXYTHETALATCfg.EndX_c >= EnvNAVXYTHETALATCfg.EnvWidth_c) {
        SBPL_ERROR("ERROR: illegal goal coordinates\n");
        throw new SBPL_Exception();
    }
    if (EnvNAVXYTHETALATCfg.EndY_c < 0 || EnvNAVXYTHETALATCfg.EndY_c >= EnvNAVXYTHETALATCfg.EnvHeight_c) {
        SBPL_ERROR("ERROR: illegal goal coordinates\n");
        throw new SBPL_Exception();
    }
    if (EnvNAVXYTHETALATCfg.EndTheta < 0 || EnvNAVXYTHETALATCfg.EndTheta >= EnvNAVXYTHETALATCfg.NumThetaDirs) {
        SBPL_ERROR("ERROR: illegal goal coordinates for theta\n");
        throw new SBPL_Exception();
    }

    EnvNAVXYTHETALATCfg.FootprintPolygon = robot_perimeterV;

    EnvNAVXYTHETALATCfg.nominalvel_mpersecs = nominalvel_mpersecs;
    EnvNAVXYTHETALATCfg.cellsize_m = cellsize_m;
    EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs = timetoturn45degsinplace_secs;

    //allocate the 2D environment
    EnvNAVXYTHETALATCfg.Grid2D = new unsigned char*[EnvNAVXYTHETALATCfg.EnvWidth_c];
    for (int x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++) {
        EnvNAVXYTHETALATCfg.Grid2D[x] = new unsigned char[EnvNAVXYTHETALATCfg.EnvHeight_c];
    }

    //environment:
    if (0 == mapdata) {
        for (int y = 0; y < EnvNAVXYTHETALATCfg.EnvHeight_c; y++) {
            for (int x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++) {
                EnvNAVXYTHETALATCfg.Grid2D[x][y] = 0;
            }
        }
    }
    else {
        for (int y = 0; y < EnvNAVXYTHETALATCfg.EnvHeight_c; y++) {
            for (int x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++) {
                EnvNAVXYTHETALATCfg.Grid2D[x][y] = mapdata[x + y * width];
            }
        }
    }
}

void EnvironmentNAVXYTHETALATTICE::ReadConfiguration(FILE* fCfg)
{
    //read in the configuration of environment and initialize  EnvNAVXYTHETALATCfg structure
    char sTemp[1024], sTemp1[1024];
    int dTemp;
    int x, y;

    //discretization(cells)
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early (discretization)\n");
        throw new SBPL_Exception();
    }
    strcpy(sTemp1, "discretization(cells):");
    if (strcmp(sTemp1, sTemp) != 0) {
        SBPL_ERROR("ERROR: configuration file has incorrect format (discretization)\n");
        SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
        throw new SBPL_Exception();
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early (discretization)\n");
        throw new SBPL_Exception();
    }
    EnvNAVXYTHETALATCfg.EnvWidth_c = atoi(sTemp);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early (discretization)\n");
        throw new SBPL_Exception();
    }
    EnvNAVXYTHETALATCfg.EnvHeight_c = atoi(sTemp);

    // Scan for optional NumThetaDirs parameter. Check for following obsthresh.
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    strcpy(sTemp1, "NumThetaDirs:");
    if (strcmp(sTemp1, sTemp) != 0) {
        // optional NumThetaDirs not available; default is NAVXYTHETALAT_THETADIRS (16)
        strcpy(sTemp1, "obsthresh:");
        if (strcmp(sTemp1, sTemp) != 0) {
            SBPL_ERROR("ERROR: configuration file has incorrect format\n");
            SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
            throw new SBPL_Exception();
        }
        else {
            EnvNAVXYTHETALATCfg.NumThetaDirs = NAVXYTHETALAT_THETADIRS;
        }
    }
    else {
        if (fscanf(fCfg, "%s", sTemp) != 1) {
            SBPL_ERROR("ERROR: ran out of env file early (NumThetaDirs)\n");
            throw new SBPL_Exception();
        }
        EnvNAVXYTHETALATCfg.NumThetaDirs = atoi(sTemp);

        //obsthresh: 
        if (fscanf(fCfg, "%s", sTemp) != 1) {
            SBPL_ERROR("ERROR: ran out of env file early (obsthresh)\n");
            throw new SBPL_Exception();
        }
        strcpy(sTemp1, "obsthresh:");
        if (strcmp(sTemp1, sTemp) != 0) {
            SBPL_ERROR("ERROR: configuration file has incorrect format\n");
            SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
            SBPL_PRINTF("see existing examples of env files for the right format of heading\n");
            throw new SBPL_Exception();
        }
    }

    // obsthresh
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvNAVXYTHETALATCfg.obsthresh = atoi(sTemp);
    SBPL_PRINTF("obsthresh = %d\n", EnvNAVXYTHETALATCfg.obsthresh);

    //cost_inscribed_thresh: 
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    strcpy(sTemp1, "cost_inscribed_thresh:");
    if (strcmp(sTemp1, sTemp) != 0) {
        SBPL_ERROR("ERROR: configuration file has incorrect format\n");
        SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
        SBPL_PRINTF("see existing examples of env files for the right format of heading\n");
        throw new SBPL_Exception();
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvNAVXYTHETALATCfg.cost_inscribed_thresh = atoi(sTemp);
    SBPL_PRINTF("cost_inscribed_thresh = %d\n", EnvNAVXYTHETALATCfg.cost_inscribed_thresh);

    //cost_possibly_circumscribed_thresh: 
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    strcpy(sTemp1, "cost_possibly_circumscribed_thresh:");
    if (strcmp(sTemp1, sTemp) != 0) {
        SBPL_ERROR("ERROR: configuration file has incorrect format\n");
        SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
        SBPL_PRINTF("see existing examples of env files for the right format of heading\n");
        throw new SBPL_Exception();
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh = atoi(sTemp);
    SBPL_PRINTF("cost_possibly_circumscribed_thresh = %d\n", EnvNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh);

    //cellsize
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    strcpy(sTemp1, "cellsize(meters):");
    if (strcmp(sTemp1, sTemp) != 0) {
        SBPL_ERROR("ERROR: configuration file has incorrect format\n");
        SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
        throw new SBPL_Exception();
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvNAVXYTHETALATCfg.cellsize_m = atof(sTemp);

    //speeds
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    strcpy(sTemp1, "nominalvel(mpersecs):");
    if (strcmp(sTemp1, sTemp) != 0) {
        SBPL_ERROR("ERROR: configuration file has incorrect format\n");
        SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
        throw new SBPL_Exception();
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvNAVXYTHETALATCfg.nominalvel_mpersecs = atof(sTemp);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    strcpy(sTemp1, "timetoturn45degsinplace(secs):");
    if (strcmp(sTemp1, sTemp) != 0) {
        SBPL_ERROR("ERROR: configuration file has incorrect format\n");
        SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
        throw new SBPL_Exception();
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs = atof(sTemp);

    //start(meters,rads):
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvNAVXYTHETALATCfg.StartX_c = CONTXY2DISC(atof(sTemp), EnvNAVXYTHETALATCfg.cellsize_m);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvNAVXYTHETALATCfg.StartY_c = CONTXY2DISC(atof(sTemp), EnvNAVXYTHETALATCfg.cellsize_m);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvNAVXYTHETALATCfg.StartTheta = ContTheta2Disc(atof(sTemp), EnvNAVXYTHETALATCfg.NumThetaDirs);

    if (EnvNAVXYTHETALATCfg.StartX_c < 0 || EnvNAVXYTHETALATCfg.StartX_c >= EnvNAVXYTHETALATCfg.EnvWidth_c) {
        SBPL_ERROR("ERROR: illegal start coordinates\n");
        throw new SBPL_Exception();
    }
    if (EnvNAVXYTHETALATCfg.StartY_c < 0 || EnvNAVXYTHETALATCfg.StartY_c >= EnvNAVXYTHETALATCfg.EnvHeight_c) {
        SBPL_ERROR("ERROR: illegal start coordinates\n");
        throw new SBPL_Exception();
    }
    if (EnvNAVXYTHETALATCfg.StartTheta < 0 || EnvNAVXYTHETALATCfg.StartTheta >= EnvNAVXYTHETALATCfg.NumThetaDirs) {
        SBPL_ERROR("ERROR: illegal start coordinates for theta\n");
        throw new SBPL_Exception();
    }

    //end(meters,rads):
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvNAVXYTHETALATCfg.EndX_c = CONTXY2DISC(atof(sTemp), EnvNAVXYTHETALATCfg.cellsize_m);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvNAVXYTHETALATCfg.EndY_c = CONTXY2DISC(atof(sTemp), EnvNAVXYTHETALATCfg.cellsize_m);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvNAVXYTHETALATCfg.EndTheta = ContTheta2Disc(atof(sTemp), EnvNAVXYTHETALATCfg.NumThetaDirs);
    ;

    if (EnvNAVXYTHETALATCfg.EndX_c < 0 || EnvNAVXYTHETALATCfg.EndX_c >= EnvNAVXYTHETALATCfg.EnvWidth_c) {
        SBPL_ERROR("ERROR: illegal end coordinates\n");
        throw new SBPL_Exception();
    }
    if (EnvNAVXYTHETALATCfg.EndY_c < 0 || EnvNAVXYTHETALATCfg.EndY_c >= EnvNAVXYTHETALATCfg.EnvHeight_c) {
        SBPL_ERROR("ERROR: illegal end coordinates\n");
        throw new SBPL_Exception();
    }
    if (EnvNAVXYTHETALATCfg.EndTheta < 0 || EnvNAVXYTHETALATCfg.EndTheta >= EnvNAVXYTHETALATCfg.NumThetaDirs) {
        SBPL_ERROR("ERROR: illegal goal coordinates for theta\n");
        throw new SBPL_Exception();
    }

    //allocate the 2D environment
    EnvNAVXYTHETALATCfg.Grid2D = new unsigned char*[EnvNAVXYTHETALATCfg.EnvWidth_c];
    for (x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++) {
        EnvNAVXYTHETALATCfg.Grid2D[x] = new unsigned char[EnvNAVXYTHETALATCfg.EnvHeight_c];
    }

    //environment:
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    for (y = 0; y < EnvNAVXYTHETALATCfg.EnvHeight_c; y++)
        for (x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++) {
            if (fscanf(fCfg, "%d", &dTemp) != 1) {
                SBPL_ERROR("ERROR: incorrect format of config file\n");
                throw new SBPL_Exception();
            }
            EnvNAVXYTHETALATCfg.Grid2D[x][y] = dTemp;
        }
}

bool EnvironmentNAVXYTHETALATTICE::ReadinCell(sbpl_xy_theta_cell_t* cell, FILE* fIn)
{
    char sTemp[60];

    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    cell->x = atoi(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    cell->y = atoi(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    cell->theta = atoi(sTemp);

    //normalize the angle
    cell->theta = NORMALIZEDISCTHETA(cell->theta, EnvNAVXYTHETALATCfg.NumThetaDirs);

    return true;
}

bool EnvironmentNAVXYTHETALATTICE::ReadinPose(sbpl_xy_theta_pt_t* pose, FILE* fIn)
{
    char sTemp[60];

    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    pose->x = atof(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    pose->y = atof(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    pose->theta = atof(sTemp);

    pose->theta = normalizeAngle(pose->theta);

    return true;
}

bool EnvironmentNAVXYTHETALATTICE::ReadinMotionPrimitive(SBPL_xytheta_mprimitive* pMotPrim, FILE* fIn)
{
    char sTemp[1024];
    int dTemp;
    char sExpected[1024];
    int numofIntermPoses;

    //read in actionID
    strcpy(sExpected, "primID:");
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &pMotPrim->motprimID) != 1) return false;

    //read in start angle
    strcpy(sExpected, "startangle_c:");
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &dTemp) == 0) {
        SBPL_ERROR("ERROR reading startangle\n");
        return false;
    }
    pMotPrim->starttheta_c = dTemp;

    //read in end pose
    strcpy(sExpected, "endpose_c:");
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }

    if (ReadinCell(&pMotPrim->endcell, fIn) == false) {
        SBPL_ERROR("ERROR: failed to read in endsearchpose\n");
        return false;
    }

    //read in action cost
    strcpy(sExpected, "additionalactioncostmult:");
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &dTemp) != 1) return false;
    pMotPrim->additionalactioncostmult = dTemp;

    //read in intermediate poses
    strcpy(sExpected, "intermediateposes:");
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &numofIntermPoses) != 1) return false;
    //all intermposes should be with respect to 0,0 as starting pose since it will be added later and should be done
    //after the action is rotated by initial orientation
    for (int i = 0; i < numofIntermPoses; i++) {
        sbpl_xy_theta_pt_t intermpose;
        if (ReadinPose(&intermpose, fIn) == false) {
            SBPL_ERROR("ERROR: failed to read in intermediate poses\n");
            return false;
        }
        pMotPrim->intermptV.push_back(intermpose);
    }

    //check that the last pose corresponds correctly to the last pose
    sbpl_xy_theta_pt_t sourcepose;
    sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
    sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
    sourcepose.theta = DiscTheta2Cont(pMotPrim->starttheta_c, EnvNAVXYTHETALATCfg.NumThetaDirs);
    double mp_endx_m = sourcepose.x + pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].x;
    double mp_endy_m = sourcepose.y + pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].y;
    double mp_endtheta_rad = pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].theta;
    int endx_c = CONTXY2DISC(mp_endx_m, EnvNAVXYTHETALATCfg.cellsize_m);
    int endy_c = CONTXY2DISC(mp_endy_m, EnvNAVXYTHETALATCfg.cellsize_m);
    int endtheta_c = ContTheta2Disc(mp_endtheta_rad, EnvNAVXYTHETALATCfg.NumThetaDirs);
    if (endx_c != pMotPrim->endcell.x || endy_c != pMotPrim->endcell.y || endtheta_c != pMotPrim->endcell.theta) {
        SBPL_ERROR( "ERROR: incorrect primitive %d with startangle=%d "
                   "last interm point %f %f %f does not match end pose %d %d %d\n",
                   pMotPrim->motprimID, pMotPrim->starttheta_c,
                   pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].x,
                   pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].y,
                   pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].theta,
                   pMotPrim->endcell.x, pMotPrim->endcell.y,
                   pMotPrim->endcell.theta);
        return false;
    }

    return true;
}

bool EnvironmentNAVXYTHETALATTICE::ReadMotionPrimitives(FILE* fMotPrims)
{
    char sTemp[1024], sExpected[1024];
    float fTemp;
    int dTemp;
    int totalNumofActions = 0;

    SBPL_PRINTF("Reading in motion primitives...");

    //read in the resolution
    strcpy(sExpected, "resolution_m:");
    if (fscanf(fMotPrims, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fMotPrims, "%f", &fTemp) == 0) return false;
    if (fabs(fTemp - EnvNAVXYTHETALATCfg.cellsize_m) > ERR_EPS) {
        SBPL_ERROR("ERROR: invalid resolution %f (instead of %f) in the dynamics file\n", fTemp,
                   EnvNAVXYTHETALATCfg.cellsize_m);
        return false;
    }

    //read in the angular resolution
    strcpy(sExpected, "numberofangles:");
    if (fscanf(fMotPrims, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fMotPrims, "%d", &dTemp) == 0) return false;
    if (dTemp != EnvNAVXYTHETALATCfg.NumThetaDirs) {
        SBPL_ERROR("ERROR: invalid angular resolution %d angles (instead of %d angles) in the motion primitives file\n",
                   dTemp, EnvNAVXYTHETALATCfg.NumThetaDirs);
        return false;
    }

    //read in the total number of actions
    strcpy(sExpected, "totalnumberofprimitives:");
    if (fscanf(fMotPrims, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fMotPrims, "%d", &totalNumofActions) == 0) {
        return false;
    }

    for (int i = 0; i < totalNumofActions; i++) {
        SBPL_xytheta_mprimitive motprim;

        if (EnvironmentNAVXYTHETALATTICE::ReadinMotionPrimitive(&motprim, fMotPrims) == false) return false;

        EnvNAVXYTHETALATCfg.mprimV.push_back(motprim);
    }
    SBPL_PRINTF("done ");

    return true;
}

void EnvironmentNAVXYTHETALATTICE::ComputeReplanningDataforAction(EnvNAVXYTHETALATAction_t* action)
{
    int j;

    //iterate over all the cells involved in the action
    sbpl_xy_theta_cell_t startcell3d, endcell3d;
    for (int i = 0; i < (int)action->intersectingcellsV.size(); i++) {
        //compute the translated affected search Pose - what state has an
        //outgoing action whose intersecting cell is at 0,0
        startcell3d.theta = action->starttheta;
        startcell3d.x = -action->intersectingcellsV.at(i).x;
        startcell3d.y = -action->intersectingcellsV.at(i).y;

        //compute the translated affected search Pose - what state has an
        //incoming action whose intersecting cell is at 0,0
        endcell3d.theta = NORMALIZEDISCTHETA(action->endtheta, EnvNAVXYTHETALATCfg.NumThetaDirs);
        endcell3d.x = startcell3d.x + action->dX;
        endcell3d.y = startcell3d.y + action->dY;

        //store the cells if not already there
        for (j = 0; j < (int)affectedsuccstatesV.size(); j++) {
            if (affectedsuccstatesV.at(j) == endcell3d) break;
        }
        if (j == (int)affectedsuccstatesV.size()) affectedsuccstatesV.push_back(endcell3d);

        for (j = 0; j < (int)affectedpredstatesV.size(); j++) {
            if (affectedpredstatesV.at(j) == startcell3d) break;
        }
        if (j == (int)affectedpredstatesV.size()) affectedpredstatesV.push_back(startcell3d);
    }//over intersecting cells

    //add the centers since with h2d we are using these in cost computations
    //---intersecting cell = origin
    //compute the translated affected search Pose - what state has an outgoing action whose intersecting cell is at 0,0
    startcell3d.theta = action->starttheta;
    startcell3d.x = -0;
    startcell3d.y = -0;

    //compute the translated affected search Pose - what state has an incoming action whose intersecting cell is at 0,0
    endcell3d.theta = NORMALIZEDISCTHETA(action->endtheta, EnvNAVXYTHETALATCfg.NumThetaDirs);
    endcell3d.x = startcell3d.x + action->dX;
    endcell3d.y = startcell3d.y + action->dY;

    //store the cells if not already there
    for (j = 0; j < (int)affectedsuccstatesV.size(); j++) {
        if (affectedsuccstatesV.at(j) == endcell3d) break;
    }
    if (j == (int)affectedsuccstatesV.size()) affectedsuccstatesV.push_back(endcell3d);

    for (j = 0; j < (int)affectedpredstatesV.size(); j++) {
        if (affectedpredstatesV.at(j) == startcell3d) break;
    }
    if (j == (int)affectedpredstatesV.size()) affectedpredstatesV.push_back(startcell3d);

    //---intersecting cell = outcome state
    //compute the translated affected search Pose - what state has an outgoing action whose intersecting cell is at 0,0
    startcell3d.theta = action->starttheta;
    startcell3d.x = -action->dX;
    startcell3d.y = -action->dY;

    //compute the translated affected search Pose - what state has an incoming action whose intersecting cell is at 0,0
    endcell3d.theta = NORMALIZEDISCTHETA(action->endtheta, EnvNAVXYTHETALATCfg.NumThetaDirs);
    endcell3d.x = startcell3d.x + action->dX;
    endcell3d.y = startcell3d.y + action->dY;

    for (j = 0; j < (int)affectedsuccstatesV.size(); j++) {
        if (affectedsuccstatesV.at(j) == endcell3d) break;
    }
    if (j == (int)affectedsuccstatesV.size()) affectedsuccstatesV.push_back(endcell3d);

    for (j = 0; j < (int)affectedpredstatesV.size(); j++) {
        if (affectedpredstatesV.at(j) == startcell3d) break;
    }
    if (j == (int)affectedpredstatesV.size()) affectedpredstatesV.push_back(startcell3d);
}

//computes all the 3D states whose outgoing actions are potentially affected
//when cell (0,0) changes its status it also does the same for the 3D states
//whose incoming actions are potentially affected when cell (0,0) changes its
//status
void EnvironmentNAVXYTHETALATTICE::ComputeReplanningData()
{
    //iterate over all actions
    //orientations
    for (int tind = 0; tind < EnvNAVXYTHETALATCfg.NumThetaDirs; tind++) {
        //actions
        for (int aind = 0; aind < EnvNAVXYTHETALATCfg.actionwidth; aind++) {
            //compute replanning data for this action 
            ComputeReplanningDataforAction(&EnvNAVXYTHETALATCfg.ActionsV[tind][aind]);
        }
    }
}

//here motionprimitivevector contains actions only for 0 angle
void EnvironmentNAVXYTHETALATTICE::PrecomputeActionswithBaseMotionPrimitive(
        vector<SBPL_xytheta_mprimitive>* motionprimitiveV)
{
    SBPL_PRINTF("Pre-computing action data using base motion primitives...\n");
    EnvNAVXYTHETALATCfg.ActionsV = new EnvNAVXYTHETALATAction_t*[EnvNAVXYTHETALATCfg.NumThetaDirs];
    EnvNAVXYTHETALATCfg.PredActionsV = new vector<EnvNAVXYTHETALATAction_t*> [EnvNAVXYTHETALATCfg.NumThetaDirs];
    vector<sbpl_2Dcell_t> footprint;

    //iterate over source angles
    for (int tind = 0; tind < EnvNAVXYTHETALATCfg.NumThetaDirs; tind++) {
        SBPL_PRINTF("pre-computing for angle %d out of %d angles\n", tind, EnvNAVXYTHETALATCfg.NumThetaDirs);
        EnvNAVXYTHETALATCfg.ActionsV[tind] = new EnvNAVXYTHETALATAction_t[motionprimitiveV->size()];

        //compute sourcepose
        sbpl_xy_theta_pt_t sourcepose;
        sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
        sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
        sourcepose.theta = DiscTheta2Cont(tind, EnvNAVXYTHETALATCfg.NumThetaDirs);

        //iterate over motion primitives
        for (size_t aind = 0; aind < motionprimitiveV->size(); aind++) {
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].aind = aind;
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].starttheta = tind;
            double mp_endx_m = motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size() - 1].x;
            double mp_endy_m = motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size() - 1].y;
            double mp_endtheta_rad =
                    motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size() - 1].theta;

            double endx = sourcepose.x + (mp_endx_m * cos(sourcepose.theta) - mp_endy_m * sin(sourcepose.theta));
            double endy = sourcepose.y + (mp_endx_m * sin(sourcepose.theta) + mp_endy_m * cos(sourcepose.theta));

            int endx_c = CONTXY2DISC(endx, EnvNAVXYTHETALATCfg.cellsize_m);
            int endy_c = CONTXY2DISC(endy, EnvNAVXYTHETALATCfg.cellsize_m);

            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta = ContTheta2Disc(mp_endtheta_rad + sourcepose.theta,
                                                                               EnvNAVXYTHETALATCfg.NumThetaDirs);
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX = endx_c;
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY = endy_c;
            if (EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY != 0 || EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX != 0)
                EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost = (int)(ceil(NAVXYTHETALAT_COSTMULT_MTOMM
                    * EnvNAVXYTHETALATCfg.cellsize_m / EnvNAVXYTHETALATCfg.nominalvel_mpersecs
                    * sqrt((double)(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX
                        * EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX + EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY
                        * EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY))));
            else
                //cost of turn in place
                EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost = (int)(NAVXYTHETALAT_COSTMULT_MTOMM
                    * EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs
                    * fabs(computeMinUnsignedAngleDiff(mp_endtheta_rad, 0)) / (PI_CONST / 4.0));

            //compute and store interm points as well as intersecting cells
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.clear();
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].interm3DcellsV.clear();
            sbpl_xy_theta_cell_t previnterm3Dcell;
            previnterm3Dcell.theta = previnterm3Dcell.x = previnterm3Dcell.y = 0;

            for (int pind = 0; pind < (int)motionprimitiveV->at(aind).intermptV.size(); pind++) {
                sbpl_xy_theta_pt_t intermpt = motionprimitiveV->at(aind).intermptV[pind];

                //rotate it appropriately
                double rotx = intermpt.x * cos(sourcepose.theta) - intermpt.y * sin(sourcepose.theta);
                double roty = intermpt.x * sin(sourcepose.theta) + intermpt.y * cos(sourcepose.theta);
                intermpt.x = rotx;
                intermpt.y = roty;
                intermpt.theta = normalizeAngle(sourcepose.theta + intermpt.theta);

                //store it (they are with reference to 0,0,stattheta (not
                //sourcepose.x,sourcepose.y,starttheta (that is, half-bin))
                EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.push_back(intermpt);
            }
            //now compute the intersecting cells for this motion (including ignoring the source footprint)
            get_2d_motion_cells(EnvNAVXYTHETALATCfg.FootprintPolygon,
                                EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV,
                                &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV,
                                EnvNAVXYTHETALATCfg.cellsize_m);

#if DEBUG
            SBPL_FPRINTF(fDeb,
                         "action tind=%d aind=%d: dX=%d dY=%d endtheta=%d (%.2f degs -> %.2f degs) "
                         "cost=%d (mprim: %.2f %.2f %.2f)\n",
                         tind,
                         (int)aind,
                         EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX,
                         EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY,
                         EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta,
                         sourcepose.theta * 180.0 / PI_CONST,
                         EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV[EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.size() - 1].theta * 180.0 / PI_CONST,
                         EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost,
                         mp_endx_m,
                         mp_endy_m,
                         mp_endtheta_rad);
#endif

            //add to the list of backward actions
            int targettheta = EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta;
            if (targettheta < 0) targettheta = targettheta + EnvNAVXYTHETALATCfg.NumThetaDirs;
            EnvNAVXYTHETALATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETALATCfg.ActionsV[tind][aind]));
        }
    }

    //set number of actions
    EnvNAVXYTHETALATCfg.actionwidth = motionprimitiveV->size();

    //now compute replanning data
    ComputeReplanningData();

    SBPL_PRINTF("done pre-computing action data based on motion primitives\n");
}

//here motionprimitivevector contains actions for all angles
void EnvironmentNAVXYTHETALATTICE::PrecomputeActionswithCompleteMotionPrimitive(
        vector<SBPL_xytheta_mprimitive>* motionprimitiveV)
{
    SBPL_PRINTF("Pre-computing action data using motion primitives for every angle...\n");
    EnvNAVXYTHETALATCfg.ActionsV = new EnvNAVXYTHETALATAction_t*[EnvNAVXYTHETALATCfg.NumThetaDirs];
    EnvNAVXYTHETALATCfg.PredActionsV = new vector<EnvNAVXYTHETALATAction_t*> [EnvNAVXYTHETALATCfg.NumThetaDirs];
    vector<sbpl_2Dcell_t> footprint;

    if (motionprimitiveV->size() % EnvNAVXYTHETALATCfg.NumThetaDirs != 0) {
        SBPL_ERROR("ERROR: motionprimitives should be uniform across actions\n");
        throw new SBPL_Exception();
    }

    EnvNAVXYTHETALATCfg.actionwidth = ((int)motionprimitiveV->size()) / EnvNAVXYTHETALATCfg.NumThetaDirs;

    //iterate over source angles
    int maxnumofactions = 0;
    for (int tind = 0; tind < EnvNAVXYTHETALATCfg.NumThetaDirs; tind++) {
        SBPL_PRINTF("pre-computing for angle %d out of %d angles\n", tind, EnvNAVXYTHETALATCfg.NumThetaDirs);

        EnvNAVXYTHETALATCfg.ActionsV[tind] = new EnvNAVXYTHETALATAction_t[EnvNAVXYTHETALATCfg.actionwidth];

        //compute sourcepose
        sbpl_xy_theta_pt_t sourcepose;
        sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
        sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
        sourcepose.theta = DiscTheta2Cont(tind, EnvNAVXYTHETALATCfg.NumThetaDirs);

        //iterate over motion primitives
        int numofactions = 0;
        int aind = -1;
        for (int mind = 0; mind < (int)motionprimitiveV->size(); mind++) {
            //find a motion primitive for this angle
            if (motionprimitiveV->at(mind).starttheta_c != tind) continue;

            aind++;
            numofactions++;

            //action index
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].aind = aind;

            //start angle
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].starttheta = tind;

            //compute dislocation
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta = motionprimitiveV->at(mind).endcell.theta;
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX = motionprimitiveV->at(mind).endcell.x;
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY = motionprimitiveV->at(mind).endcell.y;

            //compute and store interm points as well as intersecting cells
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.clear();
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].interm3DcellsV.clear();

            sbpl_xy_theta_cell_t previnterm3Dcell;
            previnterm3Dcell.x = 0;
            previnterm3Dcell.y = 0;

            // Compute all the intersected cells for this action (intermptV and interm3DcellsV)
            for (int pind = 0; pind < (int)motionprimitiveV->at(mind).intermptV.size(); pind++) {
                sbpl_xy_theta_pt_t intermpt = motionprimitiveV->at(mind).intermptV[pind];
                EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.push_back(intermpt);

                // also compute the intermediate discrete cells if not there already
                sbpl_xy_theta_pt_t pose;
                pose.x = intermpt.x + sourcepose.x;
                pose.y = intermpt.y + sourcepose.y;
                pose.theta = intermpt.theta;

                sbpl_xy_theta_cell_t intermediate2dCell;
                intermediate2dCell.x = CONTXY2DISC(pose.x, EnvNAVXYTHETALATCfg.cellsize_m);
                intermediate2dCell.y = CONTXY2DISC(pose.y, EnvNAVXYTHETALATCfg.cellsize_m);

                // add unique cells to the list
                if (EnvNAVXYTHETALATCfg.ActionsV[tind][aind].interm3DcellsV.size() == 0 || intermediate2dCell.x
                    != previnterm3Dcell.x || intermediate2dCell.y != previnterm3Dcell.y) {
                    EnvNAVXYTHETALATCfg.ActionsV[tind][aind].interm3DcellsV.push_back(intermediate2dCell);
                }

                previnterm3Dcell = intermediate2dCell;
            }

            //compute linear and angular time
            double linear_distance = 0;
            for (unsigned int i = 1; i < EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.size(); i++) {
                double x0 = EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV[i - 1].x;
                double y0 = EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV[i - 1].y;
                double x1 = EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV[i].x;
                double y1 = EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV[i].y;
                double dx = x1 - x0;
                double dy = y1 - y0;
                linear_distance += sqrt(dx * dx + dy * dy);
            }
            double linear_time = linear_distance / EnvNAVXYTHETALATCfg.nominalvel_mpersecs;
            double angular_distance =
                    fabs(computeMinUnsignedAngleDiff(DiscTheta2Cont(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta,
                                                                    EnvNAVXYTHETALATCfg.NumThetaDirs),
                                                     DiscTheta2Cont(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].starttheta,
                                                                    EnvNAVXYTHETALATCfg.NumThetaDirs)));
            double angular_time = angular_distance / ((PI_CONST / 4.0) /
                                  EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs);
            //make the cost the max of the two times
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost =
                    (int)(ceil(NAVXYTHETALAT_COSTMULT_MTOMM * max(linear_time, angular_time)));
            //use any additional cost multiplier
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost *= motionprimitiveV->at(mind).additionalactioncostmult;

            //now compute the intersecting cells for this motion (including ignoring the source footprint)
            get_2d_motion_cells(EnvNAVXYTHETALATCfg.FootprintPolygon, motionprimitiveV->at(mind).intermptV,
                                &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV,
                                EnvNAVXYTHETALATCfg.cellsize_m);

#if DEBUG
            SBPL_FPRINTF(fDeb,
                         "action tind=%2d aind=%2d: dX=%3d dY=%3d endtheta=%3d (%6.2f degs -> %6.2f degs) "
                         "cost=%4d (mprimID %3d: %3d %3d %3d) numofintermcells = %d numofintercells=%d\n",
                         tind,
                         aind,
                         EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX,
                         EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY,
                         EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta,
                         EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV[0].theta * 180 / PI_CONST,
                         EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV[EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.size() - 1].theta * 180 / PI_CONST, EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost,
                         motionprimitiveV->at(mind).motprimID, motionprimitiveV->at(mind).endcell.x,
                         motionprimitiveV->at(mind).endcell.y, motionprimitiveV->at(mind).endcell.theta,
                         (int)EnvNAVXYTHETALATCfg.ActionsV[tind][aind].interm3DcellsV.size(),
                         (int)EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.size());
#endif

            //add to the list of backward actions
            int targettheta = EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta;
            if (targettheta < 0) targettheta = targettheta + EnvNAVXYTHETALATCfg.NumThetaDirs;
            EnvNAVXYTHETALATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETALATCfg.ActionsV[tind][aind]));
        }

        if (maxnumofactions < numofactions) maxnumofactions = numofactions;
    }

    //at this point we don't allow nonuniform number of actions
    if (motionprimitiveV->size() != (size_t)(EnvNAVXYTHETALATCfg.NumThetaDirs * maxnumofactions)) {
        SBPL_ERROR("ERROR: nonuniform number of actions is not supported "
                   "(maxnumofactions=%d while motprims=%d thetas=%d\n",
                   maxnumofactions, (unsigned int)motionprimitiveV->size(), EnvNAVXYTHETALATCfg.NumThetaDirs);
        throw new SBPL_Exception();
    }

    //now compute replanning data
    ComputeReplanningData();

    SBPL_PRINTF("done pre-computing action data based on motion primitives\n");
}

void EnvironmentNAVXYTHETALATTICE::DeprecatedPrecomputeActions()
{
    SBPL_PRINTF("Use of DeprecatedPrecomputeActions() is deprecated and probably doesn't work!\n");

    //construct list of actions
    SBPL_PRINTF("Pre-computing action data using the motion primitives for a 3D kinematic planning...\n");
    EnvNAVXYTHETALATCfg.ActionsV = new EnvNAVXYTHETALATAction_t*[EnvNAVXYTHETALATCfg.NumThetaDirs];
    EnvNAVXYTHETALATCfg.PredActionsV = new vector<EnvNAVXYTHETALATAction_t*> [EnvNAVXYTHETALATCfg.NumThetaDirs];
    vector<sbpl_2Dcell_t> footprint;
    //iterate over source angles
    for (int tind = 0; tind < EnvNAVXYTHETALATCfg.NumThetaDirs; tind++) {
        SBPL_PRINTF("processing angle %d\n", tind);
        EnvNAVXYTHETALATCfg.ActionsV[tind] = new EnvNAVXYTHETALATAction_t[EnvNAVXYTHETALATCfg.actionwidth];

        //compute sourcepose
        sbpl_xy_theta_pt_t sourcepose;
        sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
        sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
        sourcepose.theta = DiscTheta2Cont(tind, EnvNAVXYTHETALATCfg.NumThetaDirs);

        //the construction assumes that the robot first turns and then goes along this new theta
        int aind = 0;
        for (; aind < 3; aind++) {
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].aind = aind;
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].starttheta = tind;
            //-1,0,1
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta = (tind + aind - 1) % EnvNAVXYTHETALATCfg.NumThetaDirs; 
            double angle = DiscTheta2Cont(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta,
                                          EnvNAVXYTHETALATCfg.NumThetaDirs);
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX = (int)(cos(angle) + 0.5 * (cos(angle) > 0 ? 1 : -1));
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY = (int)(sin(angle) + 0.5 * (sin(angle) > 0 ? 1 : -1));
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost = (int)(ceil(NAVXYTHETALAT_COSTMULT_MTOMM
                * EnvNAVXYTHETALATCfg.cellsize_m / EnvNAVXYTHETALATCfg.nominalvel_mpersecs
                * sqrt((double)(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX
                    * EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX + EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY
                    * EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY))));

            //compute intersecting cells
            sbpl_xy_theta_pt_t pose;
            pose.x = DISCXY2CONT(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETALATCfg.cellsize_m);
            pose.y = DISCXY2CONT(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY, EnvNAVXYTHETALATCfg.cellsize_m);
            pose.theta = angle;
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.clear();
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
            get_2d_footprint_cells(EnvNAVXYTHETALATCfg.FootprintPolygon,
                                   &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV, pose,
                                   EnvNAVXYTHETALATCfg.cellsize_m);
            RemoveSourceFootprint(sourcepose, &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
            SBPL_PRINTF("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d cost=%d\n",
                        tind, aind, EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta, angle,
                        EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY,
                        EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost);
#endif

            //add to the list of backward actions
            int targettheta = EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta;
            if (targettheta < 0) targettheta = targettheta + EnvNAVXYTHETALATCfg.NumThetaDirs;
            EnvNAVXYTHETALATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETALATCfg.ActionsV[tind][aind]));
        }

        //decrease and increase angle without movement
        aind = 3;
        EnvNAVXYTHETALATCfg.ActionsV[tind][aind].aind = aind;
        EnvNAVXYTHETALATCfg.ActionsV[tind][aind].starttheta = tind;
        EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta = tind - 1;
        if (EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta < 0)
            EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta += EnvNAVXYTHETALATCfg.NumThetaDirs;
        EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX = 0;
        EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY = 0;
        EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost = (int)(NAVXYTHETALAT_COSTMULT_MTOMM *
                                                              EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs);

        //compute intersecting cells
        sbpl_xy_theta_pt_t pose;
        pose.x = DISCXY2CONT(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETALATCfg.cellsize_m);
        pose.y = DISCXY2CONT(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY, EnvNAVXYTHETALATCfg.cellsize_m);
        pose.theta =
                DiscTheta2Cont(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta, EnvNAVXYTHETALATCfg.NumThetaDirs);
        EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.clear();
        EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
        get_2d_footprint_cells(EnvNAVXYTHETALATCfg.FootprintPolygon,
                               &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV, pose,
                               EnvNAVXYTHETALATCfg.cellsize_m);
        RemoveSourceFootprint(sourcepose, &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
        SBPL_PRINTF("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d cost=%d\n",
                    tind, aind, EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta,
                    DiscTheta2Cont(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta, EnvNAVXYTHETALATCfg.NumThetaDirs),
                    EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY,
                    EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost);
#endif

        //add to the list of backward actions
        int targettheta = EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta;
        if (targettheta < 0) targettheta = targettheta + EnvNAVXYTHETALATCfg.NumThetaDirs;
        EnvNAVXYTHETALATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETALATCfg.ActionsV[tind][aind]));

        aind = 4;
        EnvNAVXYTHETALATCfg.ActionsV[tind][aind].aind = aind;
        EnvNAVXYTHETALATCfg.ActionsV[tind][aind].starttheta = tind;
        EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta = (tind + 1) % EnvNAVXYTHETALATCfg.NumThetaDirs;
        EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX = 0;
        EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY = 0;
        EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost = (int)(NAVXYTHETALAT_COSTMULT_MTOMM *
                                                              EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs);

        //compute intersecting cells
        pose.x = DISCXY2CONT(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETALATCfg.cellsize_m);
        pose.y = DISCXY2CONT(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY, EnvNAVXYTHETALATCfg.cellsize_m);
        pose.theta =
                DiscTheta2Cont(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta, EnvNAVXYTHETALATCfg.NumThetaDirs);
        EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.clear();
        EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
        get_2d_footprint_cells(EnvNAVXYTHETALATCfg.FootprintPolygon,
                               &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV, pose,
                               EnvNAVXYTHETALATCfg.cellsize_m);
        RemoveSourceFootprint(sourcepose, &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
        SBPL_PRINTF("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d cost=%d\n",
                    tind, aind, EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta, DiscTheta2Cont(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta, EnvNAVXYTHETALATCfg.NumThetaDirs),
                    EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY,
                    EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost);
#endif

        //add to the list of backward actions
        targettheta = EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta;
        if (targettheta < 0) targettheta = targettheta + EnvNAVXYTHETALATCfg.NumThetaDirs;
        EnvNAVXYTHETALATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETALATCfg.ActionsV[tind][aind]));
    }

    //now compute replanning data
    ComputeReplanningData();

    SBPL_PRINTF("done pre-computing action data\n");
}

void EnvironmentNAVXYTHETALATTICE::InitializeEnvConfig(vector<SBPL_xytheta_mprimitive>* motionprimitiveV)
{
    //aditional to configuration file initialization of EnvNAVXYTHETALATCfg if necessary

    //dXY dirs
    EnvNAVXYTHETALATCfg.dXY[0][0] = -1;
    EnvNAVXYTHETALATCfg.dXY[0][1] = -1;
    EnvNAVXYTHETALATCfg.dXY[1][0] = -1;
    EnvNAVXYTHETALATCfg.dXY[1][1] = 0;
    EnvNAVXYTHETALATCfg.dXY[2][0] = -1;
    EnvNAVXYTHETALATCfg.dXY[2][1] = 1;
    EnvNAVXYTHETALATCfg.dXY[3][0] = 0;
    EnvNAVXYTHETALATCfg.dXY[3][1] = -1;
    EnvNAVXYTHETALATCfg.dXY[4][0] = 0;
    EnvNAVXYTHETALATCfg.dXY[4][1] = 1;
    EnvNAVXYTHETALATCfg.dXY[5][0] = 1;
    EnvNAVXYTHETALATCfg.dXY[5][1] = -1;
    EnvNAVXYTHETALATCfg.dXY[6][0] = 1;
    EnvNAVXYTHETALATCfg.dXY[6][1] = 0;
    EnvNAVXYTHETALATCfg.dXY[7][0] = 1;
    EnvNAVXYTHETALATCfg.dXY[7][1] = 1;

    sbpl_xy_theta_pt_t temppose;
    temppose.x = 0.0;
    temppose.y = 0.0;
    temppose.theta = 0.0;
    vector<sbpl_2Dcell_t> footprint;
    get_2d_footprint_cells(EnvNAVXYTHETALATCfg.FootprintPolygon, &footprint, temppose, EnvNAVXYTHETALATCfg.cellsize_m);
    SBPL_PRINTF("number of cells in footprint of the robot = %d\n", (unsigned int)footprint.size());

    for (vector<sbpl_2Dcell_t>::iterator it = footprint.begin(); it != footprint.end(); ++it) {
        SBPL_PRINTF("Footprint cell at (%d, %d)\n", it->x, it->y);
    }

#if DEBUG
    SBPL_FPRINTF(fDeb, "footprint cells (size=%d):\n", (int)footprint.size());
    for(int i = 0; i < (int) footprint.size(); i++)
    {
        SBPL_FPRINTF(fDeb, "%d %d (cont: %.3f %.3f)\n", footprint.at(i).x, footprint.at(i).y,
                     DISCXY2CONT(footprint.at(i).x, EnvNAVXYTHETALATCfg.cellsize_m),
                     DISCXY2CONT(footprint.at(i).y, EnvNAVXYTHETALATCfg.cellsize_m));
    }
#endif

    if (motionprimitiveV == NULL)
        DeprecatedPrecomputeActions();
    else
        PrecomputeActionswithCompleteMotionPrimitive(motionprimitiveV);
}

bool EnvironmentNAVXYTHETALATTICE::IsValidCell(int X, int Y)
{
    return (X >= 0 && X < EnvNAVXYTHETALATCfg.EnvWidth_c && Y >= 0 && Y < EnvNAVXYTHETALATCfg.EnvHeight_c &&
            EnvNAVXYTHETALATCfg.Grid2D[X][Y] < EnvNAVXYTHETALATCfg.obsthresh);
}

bool EnvironmentNAVXYTHETALATTICE::IsWithinMapCell(int X, int Y)
{
    return (X >= 0 && X < EnvNAVXYTHETALATCfg.EnvWidth_c && Y >= 0 && Y < EnvNAVXYTHETALATCfg.EnvHeight_c);
}

bool EnvironmentNAVXYTHETALATTICE::IsValidConfiguration(int X, int Y, int Theta)
{
    vector<sbpl_2Dcell_t> footprint;
    sbpl_xy_theta_pt_t pose;

    //compute continuous pose
    pose.x = DISCXY2CONT(X, EnvNAVXYTHETALATCfg.cellsize_m);
    pose.y = DISCXY2CONT(Y, EnvNAVXYTHETALATCfg.cellsize_m);
    pose.theta = DiscTheta2Cont(Theta, EnvNAVXYTHETALATCfg.NumThetaDirs);

    //compute footprint cells
    get_2d_footprint_cells(EnvNAVXYTHETALATCfg.FootprintPolygon, &footprint, pose, EnvNAVXYTHETALATCfg.cellsize_m);

    //iterate over all footprint cells
    for (int find = 0; find < (int)footprint.size(); find++) {
        int x = footprint.at(find).x;
        int y = footprint.at(find).y;

        if (x < 0 || x >= EnvNAVXYTHETALATCfg.EnvWidth_c || y < 0 || y >= EnvNAVXYTHETALATCfg.EnvHeight_c ||
            EnvNAVXYTHETALATCfg.Grid2D[x][y] >= EnvNAVXYTHETALATCfg.obsthresh)
        {
            return false;
        }
    }

    return true;
}

int EnvironmentNAVXYTHETALATTICE::GetActionCost(int SourceX, int SourceY, int SourceTheta,
                                                EnvNAVXYTHETALATAction_t* action)
{
    sbpl_2Dcell_t cell;
    sbpl_xy_theta_cell_t interm3Dcell;
    int i;

    //TODO - go over bounding box (minpt and maxpt) to test validity and skip
    //testing boundaries below, also order intersect cells so that the four
    //farthest pts go first

    if (!IsValidCell(SourceX, SourceY)) return INFINITECOST;
    if (!IsValidCell(SourceX + action->dX, SourceY + action->dY)) return INFINITECOST;

    if (EnvNAVXYTHETALATCfg.Grid2D[SourceX + action->dX][SourceY + action->dY] >=
        EnvNAVXYTHETALATCfg.cost_inscribed_thresh) 
    {
        return INFINITECOST;
    }

    //need to iterate over discretized center cells and compute cost based on them
    unsigned char maxcellcost = 0;
    for (i = 0; i < (int)action->interm3DcellsV.size(); i++) {
        interm3Dcell = action->interm3DcellsV.at(i);
        interm3Dcell.x = interm3Dcell.x + SourceX;
        interm3Dcell.y = interm3Dcell.y + SourceY;

        if (interm3Dcell.x < 0 || interm3Dcell.x >= EnvNAVXYTHETALATCfg.EnvWidth_c || interm3Dcell.y < 0
            || interm3Dcell.y >= EnvNAVXYTHETALATCfg.EnvHeight_c) return INFINITECOST;

        maxcellcost = __max(maxcellcost, EnvNAVXYTHETALATCfg.Grid2D[interm3Dcell.x][interm3Dcell.y]);

        //check that the robot is NOT in the cell at which there is no valid orientation
        if (maxcellcost >= EnvNAVXYTHETALATCfg.cost_inscribed_thresh) return INFINITECOST;
    }

    //check collisions that for the particular footprint orientation along the action
    if (EnvNAVXYTHETALATCfg.FootprintPolygon.size() > 1 && (int)maxcellcost >=
        EnvNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh)
    {
        checks++;

        for (i = 0; i < (int)action->intersectingcellsV.size(); i++) {
            //get the cell in the map
            cell = action->intersectingcellsV.at(i);
            cell.x = cell.x + SourceX;
            cell.y = cell.y + SourceY;

            //check validity
            if (!IsValidCell(cell.x, cell.y)) return INFINITECOST;

            //if(EnvNAVXYTHETALATCfg.Grid2D[cell.x][cell.y] > currentmaxcost)
            ////cost computation changed: cost = max(cost of centers of the
            //robot along action)
            //	currentmaxcost = EnvNAVXYTHETALATCfg.Grid2D[cell.x][cell.y];
            //	//intersecting cells are only used for collision checking
        }
    }

    //to ensure consistency of h2D:
    maxcellcost = __max(maxcellcost, EnvNAVXYTHETALATCfg.Grid2D[SourceX][SourceY]);
    int currentmaxcost =
            (int)__max(maxcellcost, EnvNAVXYTHETALATCfg.Grid2D[SourceX + action->dX][SourceY + action->dY]);

    return action->cost * (currentmaxcost + 1); //use cell cost as multiplicative factor
}

double EnvironmentNAVXYTHETALATTICE::EuclideanDistance_m(int X1, int Y1, int X2, int Y2)
{
    int sqdist = ((X1 - X2) * (X1 - X2) + (Y1 - Y2) * (Y1 - Y2));
    return EnvNAVXYTHETALATCfg.cellsize_m * sqrt((double)sqdist);
}

//calculates a set of cells that correspond to the specified footprint
//adds points to it (does not clear it beforehand)
void EnvironmentNAVXYTHETALATTICE::CalculateFootprintForPose(sbpl_xy_theta_pt_t pose, vector<sbpl_2Dcell_t>* footprint,
                                                             const vector<sbpl_2Dpt_t>& FootprintPolygon)
{
    int pind;

#if DEBUG
    //  SBPL_PRINTF("---Calculating Footprint for Pose: %f %f %f---\n",
    //	 pose.x, pose.y, pose.theta);
#endif

    //handle special case where footprint is just a point
    if (FootprintPolygon.size() <= 1) {
        sbpl_2Dcell_t cell;
        cell.x = CONTXY2DISC(pose.x, EnvNAVXYTHETALATCfg.cellsize_m);
        cell.y = CONTXY2DISC(pose.y, EnvNAVXYTHETALATCfg.cellsize_m);

        for (pind = 0; pind < (int)footprint->size(); pind++) {
            if (cell.x == footprint->at(pind).x && cell.y == footprint->at(pind).y) break;
        }
        if (pind == (int)footprint->size()) footprint->push_back(cell);
        return;
    }

    vector<sbpl_2Dpt_t> bounding_polygon;
    unsigned int find;
    double max_x = -INFINITECOST, min_x = INFINITECOST, max_y = -INFINITECOST, min_y = INFINITECOST;
    sbpl_2Dpt_t pt(0, 0);
    for (find = 0; find < FootprintPolygon.size(); find++) {
        //rotate and translate the corner of the robot
        pt = FootprintPolygon[find];

        //rotate and translate the point
        sbpl_2Dpt_t corner;
        corner.x = cos(pose.theta) * pt.x - sin(pose.theta) * pt.y + pose.x;
        corner.y = sin(pose.theta) * pt.x + cos(pose.theta) * pt.y + pose.y;
        bounding_polygon.push_back(corner);
#if DEBUG
        //    SBPL_PRINTF("Pt: %f %f, Corner: %f %f\n", pt.x, pt.y, corner.x, corner.y);
#endif
        if (corner.x < min_x || find == 0) {
            min_x = corner.x;
        }
        if (corner.x > max_x || find == 0) {
            max_x = corner.x;
        }
        if (corner.y < min_y || find == 0) {
            min_y = corner.y;
        }
        if (corner.y > max_y || find == 0) {
            max_y = corner.y;
        }
    }

#if DEBUG
    //  SBPL_PRINTF("Footprint bounding box: %f %f %f %f\n", min_x, max_x, min_y, max_y);
#endif
    //initialize previous values to something that will fail the if condition during the first iteration in the for loop
    int prev_discrete_x = CONTXY2DISC(pt.x, EnvNAVXYTHETALATCfg.cellsize_m) + 1;
    int prev_discrete_y = CONTXY2DISC(pt.y, EnvNAVXYTHETALATCfg.cellsize_m) + 1;
    int prev_inside = 0;
    int discrete_x;
    int discrete_y;

    for (double x = min_x; x <= max_x; x += EnvNAVXYTHETALATCfg.cellsize_m / 3) {
        for (double y = min_y; y <= max_y; y += EnvNAVXYTHETALATCfg.cellsize_m / 3) {
            pt.x = x;
            pt.y = y;
            discrete_x = CONTXY2DISC(pt.x, EnvNAVXYTHETALATCfg.cellsize_m);
            discrete_y = CONTXY2DISC(pt.y, EnvNAVXYTHETALATCfg.cellsize_m);

            //see if we just tested this point
            if (discrete_x != prev_discrete_x || discrete_y != prev_discrete_y || prev_inside == 0) {

#if DEBUG
                //		SBPL_PRINTF("Testing point: %f %f Discrete: %d %d\n", pt.x, pt.y, discrete_x, discrete_y);
#endif

                if (IsInsideFootprint(pt, &bounding_polygon)) {
                    //convert to a grid point

#if DEBUG
                    //			SBPL_PRINTF("Pt Inside %f %f\n", pt.x, pt.y);
#endif

                    sbpl_2Dcell_t cell;
                    cell.x = discrete_x;
                    cell.y = discrete_y;

                    //insert point if not there already
                    int pind = 0;
                    for (pind = 0; pind < (int)footprint->size(); pind++) {
                        if (cell.x == footprint->at(pind).x && cell.y == footprint->at(pind).y) break;
                    }
                    if (pind == (int)footprint->size()) footprint->push_back(cell);

                    prev_inside = 1;

#if DEBUG
                    //			SBPL_PRINTF("Added pt to footprint: %f %f\n", pt.x, pt.y);
#endif
                }
                else {
                    prev_inside = 0;
                }

            }
            else {
#if DEBUG
                //SBPL_PRINTF("Skipping pt: %f %f\n", pt.x, pt.y);
#endif
            }

            prev_discrete_x = discrete_x;
            prev_discrete_y = discrete_y;
        }//over x_min...x_max
    }
}

//calculates a set of cells that correspond to the footprint of the base
//adds points to it (does not clear it beforehand) 
void EnvironmentNAVXYTHETALATTICE::CalculateFootprintForPose(sbpl_xy_theta_pt_t pose, vector<sbpl_2Dcell_t>* footprint)
{
    CalculateFootprintForPose(pose, footprint, EnvNAVXYTHETALATCfg.FootprintPolygon);
}

//removes a set of cells that correspond to the specified footprint at the sourcepose
//adds points to it (does not clear it beforehand) 
void EnvironmentNAVXYTHETALATTICE::RemoveSourceFootprint(sbpl_xy_theta_pt_t sourcepose,
                                                         vector<sbpl_2Dcell_t>* footprint,
                                                         const vector<sbpl_2Dpt_t>& FootprintPolygon)
{
    vector<sbpl_2Dcell_t> sourcefootprint;

    //compute source footprint
    get_2d_footprint_cells(FootprintPolygon, &sourcefootprint, sourcepose, EnvNAVXYTHETALATCfg.cellsize_m);

    //now remove the source cells from the footprint
    for (int sind = 0; sind < (int)sourcefootprint.size(); sind++) {
        for (int find = 0; find < (int)footprint->size(); find++) {
            if (sourcefootprint.at(sind).x == footprint->at(find).x && sourcefootprint.at(sind).y
                == footprint->at(find).y) {
                footprint->erase(footprint->begin() + find);
                break;
            }
        }//over footprint
    }//over source
}

//removes a set of cells that correspond to the footprint of the base at the sourcepose
//adds points to it (does not clear it beforehand) 
void EnvironmentNAVXYTHETALATTICE::RemoveSourceFootprint(sbpl_xy_theta_pt_t sourcepose,
                                                         vector<sbpl_2Dcell_t>* footprint)
{
    RemoveSourceFootprint(sourcepose, footprint, EnvNAVXYTHETALATCfg.FootprintPolygon);
}

//------------------------------------------------------------------------------

//------------------------------Heuristic computation--------------------------

void EnvironmentNAVXYTHETALATTICE::EnsureHeuristicsUpdated(bool bGoalHeuristics)
{
    if (bNeedtoRecomputeStartHeuristics && !bGoalHeuristics) {
        grid2Dsearchfromstart->search(EnvNAVXYTHETALATCfg.Grid2D, EnvNAVXYTHETALATCfg.cost_inscribed_thresh,
                                      EnvNAVXYTHETALATCfg.StartX_c, EnvNAVXYTHETALATCfg.StartY_c,
                                      EnvNAVXYTHETALATCfg.EndX_c, EnvNAVXYTHETALATCfg.EndY_c,
                                      SBPL_2DGRIDSEARCH_TERM_CONDITION_TWOTIMESOPTPATH);
        bNeedtoRecomputeStartHeuristics = false;
        SBPL_PRINTF("2dsolcost_infullunits=%d\n",
                    (int)(grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(EnvNAVXYTHETALATCfg.EndX_c,
                                                                                   EnvNAVXYTHETALATCfg.EndY_c) /
                          EnvNAVXYTHETALATCfg.nominalvel_mpersecs));

    }

    if (bNeedtoRecomputeGoalHeuristics && bGoalHeuristics) {
        grid2Dsearchfromgoal->search(EnvNAVXYTHETALATCfg.Grid2D, EnvNAVXYTHETALATCfg.cost_inscribed_thresh,
                                     EnvNAVXYTHETALATCfg.EndX_c, EnvNAVXYTHETALATCfg.EndY_c,
                                     EnvNAVXYTHETALATCfg.StartX_c, EnvNAVXYTHETALATCfg.StartY_c,
                                     SBPL_2DGRIDSEARCH_TERM_CONDITION_TWOTIMESOPTPATH);
        bNeedtoRecomputeGoalHeuristics = false;
        SBPL_PRINTF("2dsolcost_infullunits=%d\n",
                    (int)(grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(EnvNAVXYTHETALATCfg.StartX_c,
                                                                                  EnvNAVXYTHETALATCfg.StartY_c) /
                          EnvNAVXYTHETALATCfg.nominalvel_mpersecs));
    }
}

void EnvironmentNAVXYTHETALATTICE::ComputeHeuristicValues()
{
    //whatever necessary pre-computation of heuristic values is done here
    SBPL_PRINTF("Precomputing heuristics...\n");

    //allocated 2D grid searches
    grid2Dsearchfromstart = new SBPL2DGridSearch(EnvNAVXYTHETALATCfg.EnvWidth_c, EnvNAVXYTHETALATCfg.EnvHeight_c,
                                                 (float)EnvNAVXYTHETALATCfg.cellsize_m);
    grid2Dsearchfromgoal = new SBPL2DGridSearch(EnvNAVXYTHETALATCfg.EnvWidth_c, EnvNAVXYTHETALATCfg.EnvHeight_c,
                                                (float)EnvNAVXYTHETALATCfg.cellsize_m);

    //set OPEN type to sliding buckets
    grid2Dsearchfromstart->setOPENdatastructure(SBPL_2DGRIDSEARCH_OPENTYPE_SLIDINGBUCKETS);
    grid2Dsearchfromgoal->setOPENdatastructure(SBPL_2DGRIDSEARCH_OPENTYPE_SLIDINGBUCKETS);

    SBPL_PRINTF("done\n");
}

//------------debugging functions---------------------------------------------
bool EnvironmentNAVXYTHETALATTICE::CheckQuant(FILE* fOut)
{
    for (double theta = -10; theta < 10; theta += 2.0 * PI_CONST / EnvNAVXYTHETALATCfg.NumThetaDirs * 0.01) {
        int nTheta = ContTheta2Disc(theta, EnvNAVXYTHETALATCfg.NumThetaDirs);
        double newTheta = DiscTheta2Cont(nTheta, EnvNAVXYTHETALATCfg.NumThetaDirs);
        int nnewTheta = ContTheta2Disc(newTheta, EnvNAVXYTHETALATCfg.NumThetaDirs);

        SBPL_FPRINTF(fOut, "theta=%f(%f)->%d->%f->%d\n", theta, theta * 180 / PI_CONST, nTheta, newTheta, nnewTheta);

        if (nTheta != nnewTheta) {
            SBPL_ERROR("ERROR: invalid quantization\n");
            return false;
        }
    }

    return true;
}

//-----------------------------------------------------------------------------

//-----------interface with outside functions-----------------------------------

bool EnvironmentNAVXYTHETALATTICE::InitializeEnv(const char* sEnvFile, const vector<sbpl_2Dpt_t>& perimeterptsV,
                                                 const char* sMotPrimFile)
{
    EnvNAVXYTHETALATCfg.FootprintPolygon = perimeterptsV;

    FILE* fCfg = fopen(sEnvFile, "r");
    if (fCfg == NULL) {
        SBPL_ERROR("ERROR: unable to open %s\n", sEnvFile);
        throw new SBPL_Exception();
    }
    ReadConfiguration(fCfg);
    fclose(fCfg);

    if (sMotPrimFile != NULL) {
        FILE* fMotPrim = fopen(sMotPrimFile, "r");
        if (fMotPrim == NULL) {
            SBPL_ERROR("ERROR: unable to open %s\n", sMotPrimFile);
            throw new SBPL_Exception();
        }
        if (ReadMotionPrimitives(fMotPrim) == false) {
            SBPL_ERROR("ERROR: failed to read in motion primitive file\n");
            throw new SBPL_Exception();
        }
        InitGeneral(&EnvNAVXYTHETALATCfg.mprimV);
        fclose(fMotPrim);
    }
    else
        InitGeneral( NULL);

    SBPL_PRINTF("size of env: %d by %d\n", EnvNAVXYTHETALATCfg.EnvWidth_c, EnvNAVXYTHETALATCfg.EnvHeight_c);

    return true;
}

bool EnvironmentNAVXYTHETALATTICE::InitializeEnv(const char* sEnvFile)
{
    FILE* fCfg = fopen(sEnvFile, "r");
    if (fCfg == NULL) {
        SBPL_ERROR("ERROR: unable to open %s\n", sEnvFile);
        throw new SBPL_Exception();
    }
    ReadConfiguration(fCfg);
    fclose(fCfg);

    InitGeneral( NULL);

    return true;
}

bool EnvironmentNAVXYTHETALATTICE::InitializeEnv(int width, int height, const vector<sbpl_2Dpt_t> & perimeterptsV,
                                                 double cellsize_m, double nominalvel_mpersecs,
                                                 double timetoturn45degsinplace_secs, unsigned char obsthresh,
                                                 const char* sMotPrimFile, EnvNAVXYTHETALAT_InitParms params)
{
    EnvNAVXYTHETALATCfg.NumThetaDirs = params.numThetas;

    return InitializeEnv(width, height, params.mapdata, params.startx, params.starty, params.starttheta, params.goalx,
                         params.goaly, params.goaltheta, params.goaltol_x, params.goaltol_y, params.goaltol_theta,
                         perimeterptsV, cellsize_m, nominalvel_mpersecs, timetoturn45degsinplace_secs, obsthresh,
                         sMotPrimFile);
}

bool EnvironmentNAVXYTHETALATTICE::InitializeEnv(int width, int height, const unsigned char* mapdata, double startx,
                                                 double starty, double starttheta, double goalx, double goaly,
                                                 double goaltheta, double goaltol_x, double goaltol_y,
                                                 double goaltol_theta, const vector<sbpl_2Dpt_t> & perimeterptsV,
                                                 double cellsize_m, double nominalvel_mpersecs,
                                                 double timetoturn45degsinplace_secs, unsigned char obsthresh,
                                                 const char* sMotPrimFile)
{
    SBPL_PRINTF("env: initialize with width=%d height=%d start=%.3f %.3f %.3f "
                "goalx=%.3f %.3f %.3f cellsize=%.3f nomvel=%.3f timetoturn=%.3f, obsthresh=%d\n",
                width, height, startx, starty, starttheta, goalx, goaly, goaltheta, cellsize_m, nominalvel_mpersecs,
                timetoturn45degsinplace_secs, obsthresh);

    SBPL_PRINTF("NOTE: goaltol parameters currently unused\n");

    SBPL_PRINTF("perimeter has size=%d\n", (unsigned int)perimeterptsV.size());

    for (int i = 0; i < (int)perimeterptsV.size(); i++) {
        SBPL_PRINTF("perimeter(%d) = %.4f %.4f\n", i, perimeterptsV.at(i).x, perimeterptsV.at(i).y);
    }

    EnvNAVXYTHETALATCfg.obsthresh = obsthresh;

    //TODO - need to set the tolerance as well

    SetConfiguration(width, height, mapdata, CONTXY2DISC(startx, cellsize_m), CONTXY2DISC(starty, cellsize_m),
                     ContTheta2Disc(starttheta, EnvNAVXYTHETALATCfg.NumThetaDirs), CONTXY2DISC(goalx, cellsize_m),
                     CONTXY2DISC(goaly, cellsize_m), ContTheta2Disc(goaltheta, EnvNAVXYTHETALATCfg.NumThetaDirs),
                     cellsize_m, nominalvel_mpersecs, timetoturn45degsinplace_secs, perimeterptsV);

    if (sMotPrimFile != NULL) {
        FILE* fMotPrim = fopen(sMotPrimFile, "r");
        if (fMotPrim == NULL) {
            SBPL_ERROR("ERROR: unable to open %s\n", sMotPrimFile);
            throw new SBPL_Exception();
        }

        if (ReadMotionPrimitives(fMotPrim) == false) {
            SBPL_ERROR("ERROR: failed to read in motion primitive file\n");
            throw new SBPL_Exception();
        }
        fclose(fMotPrim);
    }

    if (EnvNAVXYTHETALATCfg.mprimV.size() != 0) {
        InitGeneral(&EnvNAVXYTHETALATCfg.mprimV);
    }
    else
        InitGeneral( NULL);

    return true;
}

bool EnvironmentNAVXYTHETALATTICE::InitGeneral(vector<SBPL_xytheta_mprimitive>* motionprimitiveV)
{
    //Initialize other parameters of the environment
    InitializeEnvConfig(motionprimitiveV);

    //initialize Environment
    InitializeEnvironment();

    //pre-compute heuristics
    ComputeHeuristicValues();

    return true;
}

bool EnvironmentNAVXYTHETALATTICE::InitializeMDPCfg(MDPConfig *MDPCfg)
{
    //initialize MDPCfg with the start and goal ids
    MDPCfg->goalstateid = EnvNAVXYTHETALAT.goalstateid;
    MDPCfg->startstateid = EnvNAVXYTHETALAT.startstateid;

    return true;
}

void EnvironmentNAVXYTHETALATTICE::PrintHeuristicValues()
{
#ifndef ROS
    const char* heur = "heur.txt";
#endif
    FILE* fHeur = SBPL_FOPEN(heur, "w");
    if (fHeur == NULL) {
        SBPL_ERROR("ERROR: could not open debug file to write heuristic\n");
        throw new SBPL_Exception();
    }
    SBPL2DGridSearch* grid2Dsearch = NULL;

    for (int i = 0; i < 2; i++) {
        if (i == 0 && grid2Dsearchfromstart != NULL) {
            grid2Dsearch = grid2Dsearchfromstart;
            SBPL_FPRINTF(fHeur, "start heuristics:\n");
        }
        else if (i == 1 && grid2Dsearchfromgoal != NULL) {
            grid2Dsearch = grid2Dsearchfromgoal;
            SBPL_FPRINTF(fHeur, "goal heuristics:\n");
        }
        else
            continue;

        for (int y = 0; y < EnvNAVXYTHETALATCfg.EnvHeight_c; y++) {
            for (int x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++) {
                if (grid2Dsearch->getlowerboundoncostfromstart_inmm(x, y) < INFINITECOST)
                    SBPL_FPRINTF(fHeur, "%5d ", grid2Dsearch->getlowerboundoncostfromstart_inmm(x, y));
                else
                    SBPL_FPRINTF(fHeur, "XXXXX ");
            }
            SBPL_FPRINTF(fHeur, "\n");
        }
    }
    SBPL_FCLOSE(fHeur);
}

void EnvironmentNAVXYTHETALATTICE::SetAllPreds(CMDPSTATE* state)
{
    //implement this if the planner needs access to predecessors

    SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: SetAllPreds is undefined\n");
    throw new SBPL_Exception();
}

void EnvironmentNAVXYTHETALATTICE::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
    GetSuccs(SourceStateID, SuccIDV, CostV, NULL);
}
void EnvironmentNAVXYTHETALATTICE::GetLazySuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost){
    GetLazySuccs(SourceStateID, SuccIDV, CostV, isTrueCost, NULL);
}
void EnvironmentNAVXYTHETALATTICE::GetSuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV){
    GetSuccsWithUniqueIds(SourceStateID, SuccIDV, CostV, NULL);
}
void EnvironmentNAVXYTHETALATTICE::GetLazySuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost){
    GetLazySuccsWithUniqueIds(SourceStateID, SuccIDV, CostV, isTrueCost, NULL);
}

const EnvNAVXYTHETALATConfig_t* EnvironmentNAVXYTHETALATTICE::GetEnvNavConfig()
{
    return &EnvNAVXYTHETALATCfg;
}

bool EnvironmentNAVXYTHETALATTICE::UpdateCost(int x, int y, unsigned char newcost)
{
#if DEBUG
    //SBPL_FPRINTF(fDeb, "Cost updated for cell %d %d from old cost=%d to new cost=%d\n", x,y,EnvNAVXYTHETALATCfg.Grid2D[x][y], newcost);
#endif

    EnvNAVXYTHETALATCfg.Grid2D[x][y] = newcost;

    bNeedtoRecomputeStartHeuristics = true;
    bNeedtoRecomputeGoalHeuristics = true;

    return true;
}

bool EnvironmentNAVXYTHETALATTICE::SetMap(const unsigned char* mapdata)
{
    int xind = -1, yind = -1;

    for (xind = 0; xind < EnvNAVXYTHETALATCfg.EnvWidth_c; xind++) {
        for (yind = 0; yind < EnvNAVXYTHETALATCfg.EnvHeight_c; yind++) {
            EnvNAVXYTHETALATCfg.Grid2D[xind][yind] = mapdata[xind + yind * EnvNAVXYTHETALATCfg.EnvWidth_c];
        }
    }

    bNeedtoRecomputeStartHeuristics = true;
    bNeedtoRecomputeGoalHeuristics = true;

    return true;
}

void EnvironmentNAVXYTHETALATTICE::PrintEnv_Config(FILE* fOut)
{
    //implement this if the planner needs to print out EnvNAVXYTHETALAT. configuration

    SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: PrintEnv_Config is undefined\n");
    throw new SBPL_Exception();
}

void EnvironmentNAVXYTHETALATTICE::PrintTimeStat(FILE* fOut)
{
#if TIME_DEBUG
    SBPL_FPRINTF(fOut, "time3_addallout = %f secs, time_gethash = %f secs, time_createhash = %f secs, "
                 "time_getsuccs = %f\n",
                 time3_addallout/(double)CLOCKS_PER_SEC, time_gethash/(double)CLOCKS_PER_SEC,
                 time_createhash/(double)CLOCKS_PER_SEC, time_getsuccs/(double)CLOCKS_PER_SEC);
#endif
}

bool EnvironmentNAVXYTHETALATTICE::IsObstacle(int x, int y)
{
#if DEBUG
    SBPL_FPRINTF(fDeb, "Status of cell %d %d is queried. Its cost=%d\n", x,y,EnvNAVXYTHETALATCfg.Grid2D[x][y]);
#endif

    return (EnvNAVXYTHETALATCfg.Grid2D[x][y] >= EnvNAVXYTHETALATCfg.obsthresh);
}

void EnvironmentNAVXYTHETALATTICE::GetEnvParms(int *size_x, int *size_y, int* num_thetas, double* startx,
                                               double* starty, double*starttheta, double* goalx, double* goaly,
                                               double* goaltheta, double* cellsize_m, double* nominalvel_mpersecs,
                                               double* timetoturn45degsinplace_secs, unsigned char* obsthresh, vector<
                                                   SBPL_xytheta_mprimitive>* mprimitiveV)
{
    *num_thetas = EnvNAVXYTHETALATCfg.NumThetaDirs;
    GetEnvParms(size_x, size_y, startx, starty, starttheta, goalx, goaly, goaltheta, cellsize_m, nominalvel_mpersecs,
                timetoturn45degsinplace_secs, obsthresh, mprimitiveV);
}

void EnvironmentNAVXYTHETALATTICE::GetEnvParms(int *size_x, int *size_y, double* startx, double* starty,
                                               double*starttheta, double* goalx, double* goaly, double* goaltheta,
                                               double* cellsize_m, double* nominalvel_mpersecs,
                                               double* timetoturn45degsinplace_secs, unsigned char* obsthresh,
                                               vector<SBPL_xytheta_mprimitive>* mprimitiveV)
{
    *size_x = EnvNAVXYTHETALATCfg.EnvWidth_c;
    *size_y = EnvNAVXYTHETALATCfg.EnvHeight_c;

    *startx = DISCXY2CONT(EnvNAVXYTHETALATCfg.StartX_c, EnvNAVXYTHETALATCfg.cellsize_m);
    *starty = DISCXY2CONT(EnvNAVXYTHETALATCfg.StartY_c, EnvNAVXYTHETALATCfg.cellsize_m);
    *starttheta = DiscTheta2Cont(EnvNAVXYTHETALATCfg.StartTheta, EnvNAVXYTHETALATCfg.NumThetaDirs);
    *goalx = DISCXY2CONT(EnvNAVXYTHETALATCfg.EndX_c, EnvNAVXYTHETALATCfg.cellsize_m);
    *goaly = DISCXY2CONT(EnvNAVXYTHETALATCfg.EndY_c, EnvNAVXYTHETALATCfg.cellsize_m);
    *goaltheta = DiscTheta2Cont(EnvNAVXYTHETALATCfg.EndTheta, EnvNAVXYTHETALATCfg.NumThetaDirs);

    *cellsize_m = EnvNAVXYTHETALATCfg.cellsize_m;
    *nominalvel_mpersecs = EnvNAVXYTHETALATCfg.nominalvel_mpersecs;
    *timetoturn45degsinplace_secs = EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs;

    *obsthresh = EnvNAVXYTHETALATCfg.obsthresh;

    *mprimitiveV = EnvNAVXYTHETALATCfg.mprimV;
}

bool EnvironmentNAVXYTHETALATTICE::PoseContToDisc(double px, double py, double pth, int &ix, int &iy, int &ith) const
{
    ix = CONTXY2DISC(px, EnvNAVXYTHETALATCfg.cellsize_m);
    iy = CONTXY2DISC(py, EnvNAVXYTHETALATCfg.cellsize_m);
    ith = ContTheta2Disc(pth, EnvNAVXYTHETALATCfg.NumThetaDirs); // ContTheta2Disc() normalizes the angle
    return (pth >= -2 * PI_CONST) && (pth <= 2 * PI_CONST) && (ix >= 0) && (ix < EnvNAVXYTHETALATCfg.EnvWidth_c) &&
           (iy >= 0) && (iy < EnvNAVXYTHETALATCfg.EnvHeight_c);
}

bool EnvironmentNAVXYTHETALATTICE::PoseDiscToCont(int ix, int iy, int ith, double &px, double &py, double &pth) const
{
    px = DISCXY2CONT(ix, EnvNAVXYTHETALATCfg.cellsize_m);
    py = DISCXY2CONT(iy, EnvNAVXYTHETALATCfg.cellsize_m);
    pth = normalizeAngle(DiscTheta2Cont(ith, EnvNAVXYTHETALATCfg.NumThetaDirs));
    return (ith >= 0) && (ith < EnvNAVXYTHETALATCfg.NumThetaDirs) && (ix >= 0) &&
           (ix < EnvNAVXYTHETALATCfg.EnvWidth_c) && (iy >= 0) && (iy < EnvNAVXYTHETALATCfg.EnvHeight_c);
}

unsigned char EnvironmentNAVXYTHETALATTICE::GetMapCost(int x, int y)
{
    return EnvNAVXYTHETALATCfg.Grid2D[x][y];
}

bool EnvironmentNAVXYTHETALATTICE::SetEnvParameter(const char* parameter, int value)
{
    if (EnvNAVXYTHETALAT.bInitialized == true) {
        SBPL_ERROR("ERROR: all parameters must be set before initialization of the environment\n");
        return false;
    }

    SBPL_PRINTF("setting parameter %s to %d\n", parameter, value);

    if (strcmp(parameter, "cost_inscribed_thresh") == 0) {
        if (value < 0 || value > 255) {
            SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
            return false;
        }
        EnvNAVXYTHETALATCfg.cost_inscribed_thresh = (unsigned char)value;
    }
    else if (strcmp(parameter, "cost_possibly_circumscribed_thresh") == 0) {
        if (value < 0 || value > 255) {
            SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
            return false;
        }
        EnvNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh = value;
    }
    else if (strcmp(parameter, "cost_obsthresh") == 0) {
        if (value < 0 || value > 255) {
            SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
            return false;
        }
        EnvNAVXYTHETALATCfg.obsthresh = (unsigned char)value;
    }
    else {
        SBPL_ERROR("ERROR: invalid parameter %s\n", parameter);
        return false;
    }

    return true;
}

int EnvironmentNAVXYTHETALATTICE::GetEnvParameter(const char* parameter)
{
    if (strcmp(parameter, "cost_inscribed_thresh") == 0) {
        return (int)EnvNAVXYTHETALATCfg.cost_inscribed_thresh;
    }
    else if (strcmp(parameter, "cost_possibly_circumscribed_thresh") == 0) {
        return (int)EnvNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh;
    }
    else if (strcmp(parameter, "cost_obsthresh") == 0) {
        return (int)EnvNAVXYTHETALATCfg.obsthresh;
    }
    else {
        SBPL_ERROR("ERROR: invalid parameter %s\n", parameter);
        throw new SBPL_Exception();
    }
}

//------------------------------------------------------------------------------

//-----------------XYTHETA Enivornment (child) class---------------------------

EnvironmentNAVXYTHETALAT::~EnvironmentNAVXYTHETALAT()
{
    SBPL_PRINTF("destroying XYTHETALAT\n");

    //delete the states themselves first
    for (int i = 0; i < (int)StateID2CoordTable.size(); i++) {
        delete StateID2CoordTable.at(i);
        StateID2CoordTable.at(i) = NULL;
    }
    StateID2CoordTable.clear();

    //delete hashtable
    if (Coord2StateIDHashTable != NULL) {
        delete[] Coord2StateIDHashTable;
        Coord2StateIDHashTable = NULL;
    }
    if (Coord2StateIDHashTable_lookup != NULL) {
        delete[] Coord2StateIDHashTable_lookup;
        Coord2StateIDHashTable_lookup = NULL;
    }
}

void EnvironmentNAVXYTHETALAT::GetCoordFromState(int stateID, int& x, int& y, int& theta) const
{
    EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    x = HashEntry->X;
    y = HashEntry->Y;
    theta = HashEntry->Theta;
}

int EnvironmentNAVXYTHETALAT::GetStateFromCoord(int x, int y, int theta)
{
    EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
    if ((OutHashEntry = (this->*GetHashEntry)(x, y, theta)) == NULL) {
        //have to create a new entry
        OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta);
    }
    return OutHashEntry->stateID;
}

void EnvironmentNAVXYTHETALAT::ConvertStateIDPathintoXYThetaPath(vector<int>* stateIDPath,
                                                                 vector<sbpl_xy_theta_pt_t>* xythetaPath)
{
    vector<EnvNAVXYTHETALATAction_t*> actionV;
    vector<int> CostV;
    vector<int> SuccIDV;
    int targetx_c, targety_c, targettheta_c;
    int sourcex_c, sourcey_c, sourcetheta_c;

    SBPL_PRINTF("checks=%ld\n", checks);

    xythetaPath->clear();

#if DEBUG
    SBPL_FPRINTF(fDeb, "converting stateid path into coordinates:\n");
#endif

    for (int pind = 0; pind < (int)(stateIDPath->size()) - 1; pind++) {
        int sourceID = stateIDPath->at(pind);
        int targetID = stateIDPath->at(pind + 1);

#if DEBUG
        GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
#endif

        //get successors and pick the target via the cheapest action
        SuccIDV.clear();
        CostV.clear();
        actionV.clear();
        GetSuccs(sourceID, &SuccIDV, &CostV, &actionV);

        int bestcost = INFINITECOST;
        int bestsind = -1;

#if DEBUG
        GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
        GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c);
        SBPL_FPRINTF(fDeb, "looking for %d %d %d -> %d %d %d (numofsuccs=%d)\n", sourcex_c, sourcey_c, sourcetheta_c,
                     targetx_c, targety_c, targettheta_c, (int)SuccIDV.size());
#endif

        for (int sind = 0; sind < (int)SuccIDV.size(); sind++) {
#if DEBUG
            int x_c, y_c, theta_c;
            GetCoordFromState(SuccIDV[sind], x_c, y_c, theta_c);
            SBPL_FPRINTF(fDeb, "succ: %d %d %d\n", x_c, y_c, theta_c);
#endif

            if (SuccIDV[sind] == targetID && CostV[sind] <= bestcost) {
                bestcost = CostV[sind];
                bestsind = sind;
            }
        }
        if (bestsind == -1) {
            SBPL_ERROR("ERROR: successor not found for transition:\n");
            GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
            GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c);
            SBPL_PRINTF("%d %d %d -> %d %d %d\n", sourcex_c, sourcey_c, sourcetheta_c, targetx_c, targety_c,
                        targettheta_c);
            throw new SBPL_Exception();
        }

        //now push in the actual path
        int sourcex_c, sourcey_c, sourcetheta_c;
        GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
        double sourcex, sourcey;
        sourcex = DISCXY2CONT(sourcex_c, EnvNAVXYTHETALATCfg.cellsize_m);
        sourcey = DISCXY2CONT(sourcey_c, EnvNAVXYTHETALATCfg.cellsize_m);
        //TODO - when there are no motion primitives we should still print source state
        for (int ipind = 0; ipind < ((int)actionV[bestsind]->intermptV.size()) - 1; ipind++) {
            //translate appropriately
            sbpl_xy_theta_pt_t intermpt = actionV[bestsind]->intermptV[ipind];
            intermpt.x += sourcex;
            intermpt.y += sourcey;

#if DEBUG
            int nx = CONTXY2DISC(intermpt.x, EnvNAVXYTHETALATCfg.cellsize_m);
            int ny = CONTXY2DISC(intermpt.y, EnvNAVXYTHETALATCfg.cellsize_m);
            SBPL_FPRINTF(fDeb, "%.3f %.3f %.3f (%d %d %d cost=%d) ",
                intermpt.x, intermpt.y, intermpt.theta,
                nx, ny,
                ContTheta2Disc(intermpt.theta, EnvNAVXYTHETALATCfg.NumThetaDirs), EnvNAVXYTHETALATCfg.Grid2D[nx][ny]);
            if(ipind == 0) SBPL_FPRINTF(fDeb, "first (heur=%d)\n", GetStartHeuristic(sourceID));
            else SBPL_FPRINTF(fDeb, "\n");
#endif

            //store
            xythetaPath->push_back(intermpt);
        }
    }
}

//returns the stateid if success, and -1 otherwise
int EnvironmentNAVXYTHETALAT::SetGoal(double x_m, double y_m, double theta_rad)
{
    int x = CONTXY2DISC(x_m, EnvNAVXYTHETALATCfg.cellsize_m);
    int y = CONTXY2DISC(y_m, EnvNAVXYTHETALATCfg.cellsize_m);
    int theta = ContTheta2Disc(theta_rad, EnvNAVXYTHETALATCfg.NumThetaDirs);

    SBPL_PRINTF("env: setting goal to %.3f %.3f %.3f (%d %d %d)\n", x_m, y_m, theta_rad, x, y, theta);

    if (!IsWithinMapCell(x, y)) {
        SBPL_ERROR("ERROR: trying to set a goal cell %d %d that is outside of map\n", x, y);
        return -1;
    }

    if (!IsValidConfiguration(x, y, theta)) {
        SBPL_PRINTF("WARNING: goal configuration is invalid\n");
    }

    EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
    if ((OutHashEntry = (this->*GetHashEntry)(x, y, theta)) == NULL) {
        //have to create a new entry
        OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta);
    }

    //need to recompute start heuristics?
    if (EnvNAVXYTHETALAT.goalstateid != OutHashEntry->stateID) {
        bNeedtoRecomputeStartHeuristics = true; //because termination condition may not plan all the way to the new goal
        bNeedtoRecomputeGoalHeuristics = true; //because goal heuristics change
    }

    EnvNAVXYTHETALAT.goalstateid = OutHashEntry->stateID;

    EnvNAVXYTHETALATCfg.EndX_c = x;
    EnvNAVXYTHETALATCfg.EndY_c = y;
    EnvNAVXYTHETALATCfg.EndTheta = theta;

    return EnvNAVXYTHETALAT.goalstateid;
}

//returns the stateid if success, and -1 otherwise
int EnvironmentNAVXYTHETALAT::SetStart(double x_m, double y_m, double theta_rad)
{
    int x = CONTXY2DISC(x_m, EnvNAVXYTHETALATCfg.cellsize_m);
    int y = CONTXY2DISC(y_m, EnvNAVXYTHETALATCfg.cellsize_m);
    int theta = ContTheta2Disc(theta_rad, EnvNAVXYTHETALATCfg.NumThetaDirs);

    if (!IsWithinMapCell(x, y)) {
        SBPL_ERROR("ERROR: trying to set a start cell %d %d that is outside of map\n", x, y);
        return -1;
    }

    SBPL_PRINTF("env: setting start to %.3f %.3f %.3f (%d %d %d)\n", x_m, y_m, theta_rad, x, y, theta);

    if (!IsValidConfiguration(x, y, theta)) {
        SBPL_PRINTF("WARNING: start configuration %d %d %d is invalid\n", x, y, theta);
    }

    EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
    if ((OutHashEntry = (this->*GetHashEntry)(x, y, theta)) == NULL) {
        //have to create a new entry
        OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta);
    }

    //need to recompute start heuristics?
    if (EnvNAVXYTHETALAT.startstateid != OutHashEntry->stateID) {
        bNeedtoRecomputeStartHeuristics = true;
        //because termination condition can be not all states TODO - make it dependent on term. condition
        bNeedtoRecomputeGoalHeuristics = true; 
    }

    //set start
    EnvNAVXYTHETALAT.startstateid = OutHashEntry->stateID;
    EnvNAVXYTHETALATCfg.StartX_c = x;
    EnvNAVXYTHETALATCfg.StartY_c = y;
    EnvNAVXYTHETALATCfg.StartTheta = theta;

    return EnvNAVXYTHETALAT.startstateid;
}

void EnvironmentNAVXYTHETALAT::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG
    if(stateID >= (int)StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: stateID illegal (2)\n");
        throw new SBPL_Exception();
    }
#endif

    if (fOut == NULL) fOut = stdout;

    EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[stateID];

    if (stateID == EnvNAVXYTHETALAT.goalstateid && bVerbose) {
        SBPL_FPRINTF(fOut, "the state is a goal state\n");
    }

    if (bVerbose)
        SBPL_FPRINTF(fOut, "X=%d Y=%d Theta=%d\n", HashEntry->X, HashEntry->Y, HashEntry->Theta);
    else
        SBPL_FPRINTF(fOut, "%.3f %.3f %.3f\n", DISCXY2CONT(HashEntry->X, EnvNAVXYTHETALATCfg.cellsize_m),
                     DISCXY2CONT(HashEntry->Y, EnvNAVXYTHETALATCfg.cellsize_m),
                     DiscTheta2Cont(HashEntry->Theta, EnvNAVXYTHETALATCfg.NumThetaDirs));
}

EnvNAVXYTHETALATHashEntry_t* EnvironmentNAVXYTHETALAT::GetHashEntry_lookup(int X, int Y, int Theta)
{
    if (X < 0 || X >= EnvNAVXYTHETALATCfg.EnvWidth_c || Y < 0 || Y >= EnvNAVXYTHETALATCfg.EnvHeight_c || Theta < 0 ||
        Theta >= EnvNAVXYTHETALATCfg.NumThetaDirs) return NULL;
    int index = XYTHETA2INDEX(X,Y,Theta);
    return Coord2StateIDHashTable_lookup[index];
}

EnvNAVXYTHETALATHashEntry_t* EnvironmentNAVXYTHETALAT::GetHashEntry_hash(int X, int Y, int Theta)
{
#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    int binid = GETHASHBIN(X, Y, Theta);

#if DEBUG
    if ((int)Coord2StateIDHashTable[binid].size() > 5)
    {
        SBPL_FPRINTF(fDeb, "WARNING: Hash table has a bin %d (X=%d Y=%d) of size %d\n",
            binid, X, Y, (int)Coord2StateIDHashTable[binid].size());

        PrintHashTableHist(fDeb);
    }
#endif

    //iterate over the states in the bin and select the perfect match
    vector<EnvNAVXYTHETALATHashEntry_t*>* binV = &Coord2StateIDHashTable[binid];
    for (int ind = 0; ind < (int)binV->size(); ind++) {
        EnvNAVXYTHETALATHashEntry_t* hashentry = binV->at(ind);
        if (hashentry->X == X && hashentry->Y == Y && hashentry->Theta == Theta) {
#if TIME_DEBUG
            time_gethash += clock()-currenttime;
#endif
            return hashentry;
        }
    }

#if TIME_DEBUG	
    time_gethash += clock()-currenttime;
#endif

    return NULL;
}

EnvNAVXYTHETALATHashEntry_t* EnvironmentNAVXYTHETALAT::CreateNewHashEntry_lookup(int X, int Y, int Theta)
{
    int i;

#if TIME_DEBUG	
    clock_t currenttime = clock();
#endif

    EnvNAVXYTHETALATHashEntry_t* HashEntry = new EnvNAVXYTHETALATHashEntry_t;

    HashEntry->X = X;
    HashEntry->Y = Y;
    HashEntry->Theta = Theta;
    HashEntry->iteration = 0;

    HashEntry->stateID = StateID2CoordTable.size();

    //insert into the tables
    StateID2CoordTable.push_back(HashEntry);

    int index = XYTHETA2INDEX(X,Y,Theta);

#if DEBUG
    if(Coord2StateIDHashTable_lookup[index] != NULL)
    {
        SBPL_ERROR("ERROR: creating hash entry for non-NULL hashentry\n");
        throw new SBPL_Exception();
    }
#endif

    Coord2StateIDHashTable_lookup[index] = HashEntry;

    //insert into and initialize the mappings
    int* entry = new int[NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(entry);
    for (i = 0; i < NUMOFINDICES_STATEID2IND; i++) {
        StateID2IndexMapping[HashEntry->stateID][i] = -1;
    }

    if (HashEntry->stateID != (int)StateID2IndexMapping.size() - 1) {
        SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID\n");
        throw new SBPL_Exception();
    }

#if TIME_DEBUG
    time_createhash += clock()-currenttime;
#endif

    return HashEntry;
}

EnvNAVXYTHETALATHashEntry_t* EnvironmentNAVXYTHETALAT::CreateNewHashEntry_hash(int X, int Y, int Theta)
{
    int i;

#if TIME_DEBUG	
    clock_t currenttime = clock();
#endif

    EnvNAVXYTHETALATHashEntry_t* HashEntry = new EnvNAVXYTHETALATHashEntry_t;

    HashEntry->X = X;
    HashEntry->Y = Y;
    HashEntry->Theta = Theta;
    HashEntry->iteration = 0;

    HashEntry->stateID = StateID2CoordTable.size();

    //insert into the tables
    StateID2CoordTable.push_back(HashEntry);

    //get the hash table bin
    i = GETHASHBIN(HashEntry->X, HashEntry->Y, HashEntry->Theta);

    //insert the entry into the bin
    Coord2StateIDHashTable[i].push_back(HashEntry);

    //insert into and initialize the mappings
    int* entry = new int[NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(entry);
    for (i = 0; i < NUMOFINDICES_STATEID2IND; i++) {
        StateID2IndexMapping[HashEntry->stateID][i] = -1;
    }

    if (HashEntry->stateID != (int)StateID2IndexMapping.size() - 1) {
        SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID\n");
        throw new SBPL_Exception();
    }

#if TIME_DEBUG
    time_createhash += clock()-currenttime;
#endif

    return HashEntry;
}

void EnvironmentNAVXYTHETALAT::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<
    EnvNAVXYTHETALATAction_t*>* actionV /*=NULL*/)
{
    int aind;

#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    //clear the successor array
    SuccIDV->clear();
    CostV->clear();
    SuccIDV->reserve(EnvNAVXYTHETALATCfg.actionwidth);
    CostV->reserve(EnvNAVXYTHETALATCfg.actionwidth);
    if (actionV != NULL) {
        actionV->clear();
        actionV->reserve(EnvNAVXYTHETALATCfg.actionwidth);
    }

    //goal state should be absorbing
    if (SourceStateID == EnvNAVXYTHETALAT.goalstateid) return;

    //get X, Y for the state
    EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];

    //iterate through actions
    for (aind = 0; aind < EnvNAVXYTHETALATCfg.actionwidth; aind++) {
        EnvNAVXYTHETALATAction_t* nav3daction = &EnvNAVXYTHETALATCfg.ActionsV[(unsigned int)HashEntry->Theta][aind];
        int newX = HashEntry->X + nav3daction->dX;
        int newY = HashEntry->Y + nav3daction->dY;
        int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, EnvNAVXYTHETALATCfg.NumThetaDirs);

        //skip the invalid cells
        if (!IsValidCell(newX, newY)) continue;

        //get cost
        int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav3daction);
        if (cost >= INFINITECOST) continue;

        EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
        if ((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta)) == NULL) {
            //have to create a new entry
            OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta);
        }

        SuccIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
        if (actionV != NULL) actionV->push_back(nav3daction);
    }

#if TIME_DEBUG
    time_getsuccs += clock()-currenttime;
#endif
}

void EnvironmentNAVXYTHETALAT::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{
    //TODO- to support tolerance, need:
    // a) generate preds for goal state based on all possible goal state variable settings,
    // b) change goal check condition in gethashentry c) change
    //    getpredsofchangedcells and getsuccsofchangedcells functions

    int aind;

#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    //get X, Y for the state
    EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[TargetStateID];

    //clear the successor array
    PredIDV->clear();
    CostV->clear();
    PredIDV->reserve(EnvNAVXYTHETALATCfg.PredActionsV[(unsigned int)HashEntry->Theta].size());
    CostV->reserve(EnvNAVXYTHETALATCfg.PredActionsV[(unsigned int)HashEntry->Theta].size());

    //iterate through actions
    vector<EnvNAVXYTHETALATAction_t*>* actionsV = &EnvNAVXYTHETALATCfg.PredActionsV[(unsigned int)HashEntry->Theta];
    for (aind = 0; aind < (int)EnvNAVXYTHETALATCfg.PredActionsV[(unsigned int)HashEntry->Theta].size(); aind++) {

        EnvNAVXYTHETALATAction_t* nav3daction = actionsV->at(aind);

        int predX = HashEntry->X - nav3daction->dX;
        int predY = HashEntry->Y - nav3daction->dY;
        int predTheta = nav3daction->starttheta;

        //skip the invalid cells
        if (!IsValidCell(predX, predY)) continue;

        //get cost
        int cost = GetActionCost(predX, predY, predTheta, nav3daction);
        if (cost >= INFINITECOST) continue;

        EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
        if ((OutHashEntry = (this->*GetHashEntry)(predX, predY, predTheta)) == NULL) {
            //have to create a new entry
            OutHashEntry = (this->*CreateNewHashEntry)(predX, predY, predTheta);
        }

        PredIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
    }

#if TIME_DEBUG
    time_getsuccs += clock()-currenttime;
#endif
}

void EnvironmentNAVXYTHETALAT::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
    int cost;

#if DEBUG
    if(state->StateID >= (int)StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in Env... function: stateID illegal\n");
        throw new SBPL_Exception();
    }

    if((int)state->Actions.size() != 0)
    {
        SBPL_ERROR("ERROR in Env_setAllActionsandAllOutcomes: actions already exist for the state\n");
        throw new SBPL_Exception();
    }
#endif

    //goal state should be absorbing
    if (state->StateID == EnvNAVXYTHETALAT.goalstateid) return;

    //get X, Y for the state
    EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[state->StateID];

    //iterate through actions
    for (int aind = 0; aind < EnvNAVXYTHETALATCfg.actionwidth; aind++) {
        EnvNAVXYTHETALATAction_t* nav3daction = &EnvNAVXYTHETALATCfg.ActionsV[(unsigned int)HashEntry->Theta][aind];
        int newX = HashEntry->X + nav3daction->dX;
        int newY = HashEntry->Y + nav3daction->dY;
        int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, EnvNAVXYTHETALATCfg.NumThetaDirs);

        //skip the invalid cells
        if (!IsValidCell(newX, newY)) continue;

        //get cost
        cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav3daction);
        if (cost >= INFINITECOST) continue;

        //add the action
        CMDPACTION* action = state->AddAction(aind);

#if TIME_DEBUG
        clock_t currenttime = clock();
#endif

        EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
        if ((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta)) == NULL) {
            //have to create a new entry
            OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta);
        }
        action->AddOutcome(OutHashEntry->stateID, cost, 1.0);

#if TIME_DEBUG
        time3_addallout += clock()-currenttime;
#endif
    }
}

void EnvironmentNAVXYTHETALAT::GetPredsofChangedEdges(vector<nav2dcell_t> const * changedcellsV,
                                                      vector<int> *preds_of_changededgesIDV)
{
    nav2dcell_t cell;
    sbpl_xy_theta_cell_t affectedcell;
    EnvNAVXYTHETALATHashEntry_t* affectedHashEntry;

    //increment iteration for processing savings
    iteration++;

    for (int i = 0; i < (int)changedcellsV->size(); i++) {
        cell = changedcellsV->at(i);

        //now iterate over all states that could potentially be affected
        for (int sind = 0; sind < (int)affectedpredstatesV.size(); sind++) {
            affectedcell = affectedpredstatesV.at(sind);

            //translate to correct for the offset
            affectedcell.x = affectedcell.x + cell.x;
            affectedcell.y = affectedcell.y + cell.y;

            //insert only if it was actually generated
            affectedHashEntry = (this->*GetHashEntry)(affectedcell.x, affectedcell.y, affectedcell.theta);
            if (affectedHashEntry != NULL && affectedHashEntry->iteration < iteration) {
                preds_of_changededgesIDV->push_back(affectedHashEntry->stateID);
                affectedHashEntry->iteration = iteration; //mark as already inserted
            }
        }
    }
}

void EnvironmentNAVXYTHETALAT::GetSuccsofChangedEdges(vector<nav2dcell_t> const * changedcellsV,
                                                      vector<int> *succs_of_changededgesIDV)
{
    nav2dcell_t cell;
    sbpl_xy_theta_cell_t affectedcell;
    EnvNAVXYTHETALATHashEntry_t* affectedHashEntry;

    SBPL_ERROR("ERROR: getsuccs is not supported currently\n");
    throw new SBPL_Exception();

    //increment iteration for processing savings
    iteration++;

    //TODO - check
    for (int i = 0; i < (int)changedcellsV->size(); i++) {
        cell = changedcellsV->at(i);

        //now iterate over all states that could potentially be affected
        for (int sind = 0; sind < (int)affectedsuccstatesV.size(); sind++) {
            affectedcell = affectedsuccstatesV.at(sind);

            //translate to correct for the offset
            affectedcell.x = affectedcell.x + cell.x;
            affectedcell.y = affectedcell.y + cell.y;

            //insert only if it was actually generated
            affectedHashEntry = (this->*GetHashEntry)(affectedcell.x, affectedcell.y, affectedcell.theta);
            if (affectedHashEntry != NULL && affectedHashEntry->iteration < iteration) {
                succs_of_changededgesIDV->push_back(affectedHashEntry->stateID);
                affectedHashEntry->iteration = iteration; //mark as already inserted
            }
        }
    }
}

void EnvironmentNAVXYTHETALAT::InitializeEnvironment()
{
    EnvNAVXYTHETALATHashEntry_t* HashEntry;

    int maxsize = EnvNAVXYTHETALATCfg.EnvWidth_c * EnvNAVXYTHETALATCfg.EnvHeight_c * EnvNAVXYTHETALATCfg.NumThetaDirs;

    if (maxsize <= SBPL_XYTHETALAT_MAXSTATESFORLOOKUP) {
        SBPL_PRINTF("environment stores states in lookup table\n");

        Coord2StateIDHashTable_lookup = new EnvNAVXYTHETALATHashEntry_t*[maxsize];
        for (int i = 0; i < maxsize; i++)
            Coord2StateIDHashTable_lookup[i] = NULL;
        GetHashEntry = &EnvironmentNAVXYTHETALAT::GetHashEntry_lookup;
        CreateNewHashEntry = &EnvironmentNAVXYTHETALAT::CreateNewHashEntry_lookup;

        //not using hash table
        HashTableSize = 0;
        Coord2StateIDHashTable = NULL;
    }
    else {
        SBPL_PRINTF("environment stores states in hashtable\n");

        //initialize the map from Coord to StateID
        HashTableSize = 4 * 1024 * 1024; //should be power of two
        Coord2StateIDHashTable = new vector<EnvNAVXYTHETALATHashEntry_t*> [HashTableSize];
        GetHashEntry = &EnvironmentNAVXYTHETALAT::GetHashEntry_hash;
        CreateNewHashEntry = &EnvironmentNAVXYTHETALAT::CreateNewHashEntry_hash;

        //not using hash
        Coord2StateIDHashTable_lookup = NULL;
    }

    //initialize the map from StateID to Coord
    StateID2CoordTable.clear();

    //create start state
    if ((HashEntry = (this->*GetHashEntry)(EnvNAVXYTHETALATCfg.StartX_c, EnvNAVXYTHETALATCfg.StartY_c,
                                           EnvNAVXYTHETALATCfg.StartTheta)) == NULL) {
        //have to create a new entry
        HashEntry = (this->*CreateNewHashEntry)(EnvNAVXYTHETALATCfg.StartX_c, EnvNAVXYTHETALATCfg.StartY_c,
                                                EnvNAVXYTHETALATCfg.StartTheta);
    }
    EnvNAVXYTHETALAT.startstateid = HashEntry->stateID;

    //create goal state
    if ((HashEntry = (this->*GetHashEntry)(EnvNAVXYTHETALATCfg.EndX_c, EnvNAVXYTHETALATCfg.EndY_c,
                                           EnvNAVXYTHETALATCfg.EndTheta)) == NULL) {
        //have to create a new entry
        HashEntry = (this->*CreateNewHashEntry)(EnvNAVXYTHETALATCfg.EndX_c, EnvNAVXYTHETALATCfg.EndY_c,
                                                EnvNAVXYTHETALATCfg.EndTheta);
    }
    EnvNAVXYTHETALAT.goalstateid = HashEntry->stateID;

    //initialized
    EnvNAVXYTHETALAT.bInitialized = true;
}

//examples of hash functions: map state coordinates onto a hash value
//#define GETHASHBIN(X, Y) (Y*WIDTH_Y+X) 
//here we have state coord: <X1, X2, X3, X4>
unsigned int EnvironmentNAVXYTHETALAT::GETHASHBIN(unsigned int X1, unsigned int X2, unsigned int Theta)
{
    return inthash(inthash(X1) + (inthash(X2) << 1) + (inthash(Theta) << 2)) & (HashTableSize - 1);
}

void EnvironmentNAVXYTHETALAT::PrintHashTableHist(FILE* fOut)
{
    int s0 = 0, s1 = 0, s50 = 0, s100 = 0, s200 = 0, s300 = 0, slarge = 0;

    for (int j = 0; j < HashTableSize; j++) {
        if ((int)Coord2StateIDHashTable[j].size() == 0)
            s0++;
        else if ((int)Coord2StateIDHashTable[j].size() < 5)
            s1++;
        else if ((int)Coord2StateIDHashTable[j].size() < 25)
            s50++;
        else if ((int)Coord2StateIDHashTable[j].size() < 50)
            s100++;
        else if ((int)Coord2StateIDHashTable[j].size() < 100)
            s200++;
        else if ((int)Coord2StateIDHashTable[j].size() < 400)
            s300++;
        else
            slarge++;
    }
    SBPL_FPRINTF(fOut, "hash table histogram: 0:%d, <5:%d, <25:%d, <50:%d, <100:%d, <400:%d, >400:%d\n", s0, s1, s50,
                 s100, s200, s300, slarge);
}

int EnvironmentNAVXYTHETALAT::GetFromToHeuristic(int FromStateID, int ToStateID)
{
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if(FromStateID >= (int)StateID2CoordTable.size()
        || ToStateID >= (int)StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    //get X, Y for the state
    EnvNAVXYTHETALATHashEntry_t* FromHashEntry = StateID2CoordTable[FromStateID];
    EnvNAVXYTHETALATHashEntry_t* ToHashEntry = StateID2CoordTable[ToStateID];

    //TODO - check if one of the gridsearches already computed and then use it.

    return (int)(NAVXYTHETALAT_COSTMULT_MTOMM * EuclideanDistance_m(FromHashEntry->X, FromHashEntry->Y, ToHashEntry->X,
                                                                    ToHashEntry->Y) /
                 EnvNAVXYTHETALATCfg.nominalvel_mpersecs);

}

int EnvironmentNAVXYTHETALAT::GetGoalHeuristic(int stateID)
{
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if (stateID >= (int)StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    //computes distances from start state that is grid2D, so it is EndX_c EndY_c
    int h2D = grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y); 
    int hEuclid = (int)(NAVXYTHETALAT_COSTMULT_MTOMM * EuclideanDistance_m(HashEntry->X, HashEntry->Y,
                                                                           EnvNAVXYTHETALATCfg.EndX_c,
                                                                           EnvNAVXYTHETALATCfg.EndY_c));

    //define this function if it is used in the planner (heuristic backward search would use it)
    return (int)(((double)__max(h2D, hEuclid)) / EnvNAVXYTHETALATCfg.nominalvel_mpersecs);
}

int EnvironmentNAVXYTHETALAT::GetStartHeuristic(int stateID)
{
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if (stateID >= (int)StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    int h2D = grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y);
    int hEuclid = (int)(NAVXYTHETALAT_COSTMULT_MTOMM * EuclideanDistance_m(EnvNAVXYTHETALATCfg.StartX_c,
                                                                           EnvNAVXYTHETALATCfg.StartY_c, HashEntry->X,
                                                                           HashEntry->Y));

    //define this function if it is used in the planner (heuristic backward search would use it)
    return (int)(((double)__max(h2D, hEuclid)) / EnvNAVXYTHETALATCfg.nominalvel_mpersecs);
}

int EnvironmentNAVXYTHETALAT::SizeofCreatedEnv()
{
    return (int)StateID2CoordTable.size();
}

//------------------------------------------------------------------------------


void EnvironmentNAVXYTHETALAT::GetLazySuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost,
                                        std::vector<EnvNAVXYTHETALATAction_t*>* actionV /*=NULL*/)
{
    int aind;

#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    //clear the successor array
    SuccIDV->clear();
    CostV->clear();
    SuccIDV->reserve(EnvNAVXYTHETALATCfg.actionwidth);
    CostV->reserve(EnvNAVXYTHETALATCfg.actionwidth);
    isTrueCost->reserve(EnvNAVXYTHETALATCfg.actionwidth);
    if (actionV != NULL) {
        actionV->clear();
        actionV->reserve(EnvNAVXYTHETALATCfg.actionwidth);
    }

    //goal state should be absorbing
    if (SourceStateID == EnvNAVXYTHETALAT.goalstateid) return;

    //get X, Y for the state
    EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];

    //iterate through actions
    for (aind = 0; aind < EnvNAVXYTHETALATCfg.actionwidth; aind++) {
        EnvNAVXYTHETALATAction_t* nav3daction = &EnvNAVXYTHETALATCfg.ActionsV[(unsigned int)HashEntry->Theta][aind];
        int newX = HashEntry->X + nav3daction->dX;
        int newY = HashEntry->Y + nav3daction->dY;
        int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, EnvNAVXYTHETALATCfg.NumThetaDirs);

        //skip the invalid cells
        if (!IsValidCell(newX, newY)) continue;

        if(!actionV){//if we are supposed to return the action, then don't do lazy
          EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
          if((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta)) == NULL)
            OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta);
          SuccIDV->push_back(OutHashEntry->stateID);
          CostV->push_back(nav3daction->cost);
          isTrueCost->push_back(false);
          continue;
        }

        //get cost
        int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav3daction);
        if (cost >= INFINITECOST) continue;

        EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
        if ((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta)) == NULL) {
            //have to create a new entry
            OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta);
        }

        SuccIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
        isTrueCost->push_back(true);
        if (actionV != NULL) actionV->push_back(nav3daction);
    }

#if TIME_DEBUG
    time_getsuccs += clock()-currenttime;
#endif
}

int EnvironmentNAVXYTHETALAT::GetTrueCost(int parentID, int childID){
  EnvNAVXYTHETALATHashEntry_t* fromHash = StateID2CoordTable[parentID];
  EnvNAVXYTHETALATHashEntry_t* toHash = StateID2CoordTable[childID];

  for(int i=0; i<EnvNAVXYTHETALATCfg.actionwidth; i++){
    EnvNAVXYTHETALATAction_t* nav3daction = &EnvNAVXYTHETALATCfg.ActionsV[(unsigned int)fromHash->Theta][i];
    int newX = fromHash->X + nav3daction->dX;
    int newY = fromHash->Y + nav3daction->dY;
    int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, EnvNAVXYTHETALATCfg.NumThetaDirs);

    //skip the invalid cells
    if(!IsValidCell(newX, newY))
      continue;

    EnvNAVXYTHETALATHashEntry_t* hash;
    if((hash = (this->*GetHashEntry)(newX, newY, newTheta)) == NULL)
      continue;
    if(hash->stateID != toHash->stateID)
      continue;

    //get cost
    int cost = GetActionCost(fromHash->X, fromHash->Y, fromHash->Theta, nav3daction);

    if(cost >= INFINITECOST)
      return -1;
    return cost;
  }
  printf("this should never happen! we didn't find the state we need to get the true cost for!\n");
  throw new SBPL_Exception();
  return -1;
}

void EnvironmentNAVXYTHETALAT::GetSuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV,
                                                     std::vector<EnvNAVXYTHETALATAction_t*>* actionV /*=NULL*/){
  GetSuccs(SourceStateID, SuccIDV, CostV, actionV);
}

void EnvironmentNAVXYTHETALAT::GetLazySuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost,
                                                     std::vector<EnvNAVXYTHETALATAction_t*>* actionV /*=NULL*/){
  GetLazySuccs(SourceStateID, SuccIDV, CostV, isTrueCost, actionV);
}

bool EnvironmentNAVXYTHETALAT::isGoal(int id){
  return EnvNAVXYTHETALAT.goalstateid == id;
}

//void EnvironmentNAVXYTHETALAT::GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost);
//void EnvironmentNAVXYTHETALAT::GetPredsWithUniqueIds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV);
//void EnvironmentNAVXYTHETALAT::GetPredsWithUniqueIds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost);

