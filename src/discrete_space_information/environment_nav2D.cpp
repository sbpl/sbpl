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
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <sbpl/discrete_space_information/environment_nav2D.h>
#include <sbpl/planners/planner.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/mdpconfig.h>

using namespace std;

#if TIME_DEBUG
static clock_t time3_addallout = 0;
static clock_t time_gethash = 0;
static clock_t time_createhash = 0;
static clock_t time_getsuccs = 0;
#endif

//-------------------constructor--------------------------------------------
EnvironmentNAV2D::EnvironmentNAV2D()
{
    EnvNAV2DCfg.obsthresh = ENVNAV2D_DEFAULTOBSTHRESH;

    EnvNAV2DCfg.numofdirs = 8;
    EnvNAV2D.bInitialized = false;
}

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

//examples of hash functions: map state coordinates onto a hash value
//#define GETHASHBIN(X, Y) (Y*WIDTH_Y+X) 
//here we have state coord: <X1, X2, X3, X4>
unsigned int EnvironmentNAV2D::GETHASHBIN(unsigned int X1, unsigned int X2)
{
    return inthash(inthash(X1) + (inthash(X2) << 1)) & (EnvNAV2D.HashTableSize - 1);
}

void EnvironmentNAV2D::PrintHashTableHist()
{
    int s0 = 0, s1 = 0, s50 = 0, s100 = 0, s200 = 0, s300 = 0, slarge = 0;

    for (int j = 0; j < EnvNAV2D.HashTableSize; j++) {
        if ((int)EnvNAV2D.Coord2StateIDHashTable[j].size() == 0)
            s0++;
        else if ((int)EnvNAV2D.Coord2StateIDHashTable[j].size() < 50)
            s1++;
        else if ((int)EnvNAV2D.Coord2StateIDHashTable[j].size() < 100)
            s50++;
        else if ((int)EnvNAV2D.Coord2StateIDHashTable[j].size() < 200)
            s100++;
        else if ((int)EnvNAV2D.Coord2StateIDHashTable[j].size() < 300)
            s200++;
        else if ((int)EnvNAV2D.Coord2StateIDHashTable[j].size() < 400)
            s300++;
        else
            slarge++;
    }
    SBPL_PRINTF("hash table histogram: 0:%d, <50:%d, <100:%d, <200:%d, <300:%d, <400:%d >400:%d\n", s0, s1, s50, s100,
                s200, s300, slarge);
}

void EnvironmentNAV2D::SetConfiguration(int width, int height, const unsigned char* mapdata, int startx, int starty,
                                        int goalx, int goaly)
{
    EnvNAV2DCfg.EnvWidth_c = width;
    EnvNAV2DCfg.EnvHeight_c = height;
    EnvNAV2DCfg.StartX_c = startx;
    EnvNAV2DCfg.StartY_c = starty;
    int x;

    if (EnvNAV2DCfg.StartX_c < 0 || EnvNAV2DCfg.StartX_c >= EnvNAV2DCfg.EnvWidth_c) {
        SBPL_ERROR("ERROR: illegal start coordinates\n");
        throw new SBPL_Exception();
    }
    if (EnvNAV2DCfg.StartY_c < 0 || EnvNAV2DCfg.StartY_c >= EnvNAV2DCfg.EnvHeight_c) {
        SBPL_ERROR("ERROR: illegal start coordinates\n");
        throw new SBPL_Exception();
    }

    EnvNAV2DCfg.EndX_c = goalx;
    EnvNAV2DCfg.EndY_c = goaly;

    //allocate the 2D environment
    EnvNAV2DCfg.Grid2D = new unsigned char*[EnvNAV2DCfg.EnvWidth_c];
    for (x = 0; x < EnvNAV2DCfg.EnvWidth_c; x++) {
        EnvNAV2DCfg.Grid2D[x] = new unsigned char[EnvNAV2DCfg.EnvHeight_c];
    }

    //environment:
    if (0 == mapdata) {
        for (int y = 0; y < EnvNAV2DCfg.EnvHeight_c; y++) {
            for (int x = 0; x < EnvNAV2DCfg.EnvWidth_c; x++) {
                EnvNAV2DCfg.Grid2D[x][y] = 0;
            }
        }
    }
    else {
        for (int y = 0; y < EnvNAV2DCfg.EnvHeight_c; y++) {
            for (int x = 0; x < EnvNAV2DCfg.EnvWidth_c; x++) {
                unsigned char cval = mapdata[x + y * width];
                EnvNAV2DCfg.Grid2D[x][y] = cval;
            }
        }
    }
}

void EnvironmentNAV2D::ReadConfiguration(FILE* fCfg)
{
    //read in the configuration of environment and initialize  EnvNAV2DCfg structure
    char sTemp[1024], sTemp1[1024];
    int dTemp;
    int x, y;

    //discretization(cells)
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvNAV2DCfg.EnvWidth_c = atoi(sTemp);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvNAV2DCfg.EnvHeight_c = atoi(sTemp);

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
    EnvNAV2DCfg.obsthresh = (int)(atof(sTemp));
    SBPL_PRINTF("obsthresh = %d\n", EnvNAV2DCfg.obsthresh);

    //start(cells): 
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvNAV2DCfg.StartX_c = atoi(sTemp);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvNAV2DCfg.StartY_c = atoi(sTemp);

    if (EnvNAV2DCfg.StartX_c < 0 || EnvNAV2DCfg.StartX_c >= EnvNAV2DCfg.EnvWidth_c) {
        SBPL_ERROR("ERROR: illegal start coordinates\n");
        throw new SBPL_Exception();
    }
    if (EnvNAV2DCfg.StartY_c < 0 || EnvNAV2DCfg.StartY_c >= EnvNAV2DCfg.EnvHeight_c) {
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
    EnvNAV2DCfg.EndX_c = atoi(sTemp);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    EnvNAV2DCfg.EndY_c = atoi(sTemp);

    if (EnvNAV2DCfg.EndX_c < 0 || EnvNAV2DCfg.EndX_c >= EnvNAV2DCfg.EnvWidth_c) {
        SBPL_ERROR("ERROR: illegal end coordinates\n");
        throw new SBPL_Exception();
    }
    if (EnvNAV2DCfg.EndY_c < 0 || EnvNAV2DCfg.EndY_c >= EnvNAV2DCfg.EnvHeight_c) {
        SBPL_ERROR("ERROR: illegal end coordinates\n");
        throw new SBPL_Exception();
    }

    //allocate the 2D environment
    EnvNAV2DCfg.Grid2D = new unsigned char*[EnvNAV2DCfg.EnvWidth_c];
    for (x = 0; x < EnvNAV2DCfg.EnvWidth_c; x++) {
        EnvNAV2DCfg.Grid2D[x] = new unsigned char[EnvNAV2DCfg.EnvHeight_c];
    }

    //environment:
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    for (y = 0; y < EnvNAV2DCfg.EnvHeight_c; y++)
        for (x = 0; x < EnvNAV2DCfg.EnvWidth_c; x++) {
            if (fscanf(fCfg, "%d", &dTemp) != 1) {
                SBPL_ERROR("ERROR: incorrect format of config file\n");
                throw new SBPL_Exception();
            }
            EnvNAV2DCfg.Grid2D[x][y] = dTemp;
        }

    SBPL_PRINTF("start has cost=%d goal has cost=%d\n", EnvNAV2DCfg.Grid2D[EnvNAV2DCfg.StartX_c][EnvNAV2DCfg.StartY_c],
                EnvNAV2DCfg.Grid2D[EnvNAV2DCfg.EndX_c][EnvNAV2DCfg.EndY_c]);
}

void EnvironmentNAV2D::InitializeEnvConfig()
{
    //aditional to configuration file initialization of EnvNAV2DCfg if necessary

    /*
     //actions
     EnvNAV2DCfg.dXY[0][0] = -1;
     EnvNAV2DCfg.dXY[0][1] = -1;
     EnvNAV2DCfg.dXY[1][0] = -1;
     EnvNAV2DCfg.dXY[1][1] = 0;
     EnvNAV2DCfg.dXY[2][0] = -1;
     EnvNAV2DCfg.dXY[2][1] = 1;
     EnvNAV2DCfg.dXY[3][0] = 0;
     EnvNAV2DCfg.dXY[3][1] = -1;
     EnvNAV2DCfg.dXY[4][0] = 0;
     EnvNAV2DCfg.dXY[4][1] = 1;
     EnvNAV2DCfg.dXY[5][0] = 1;
     EnvNAV2DCfg.dXY[5][1] = -1;
     EnvNAV2DCfg.dXY[6][0] = 1;
     EnvNAV2DCfg.dXY[6][1] = 0;
     EnvNAV2DCfg.dXY[7][0] = 1;
     EnvNAV2DCfg.dXY[7][1] = 1;
     */
    Computedxy();
}

void EnvironmentNAV2D::Computedxy()
{
    //initialize some constants for 2D search
    EnvNAV2DCfg.dx_[0] = 1;
    EnvNAV2DCfg.dy_[0] = 1;
    EnvNAV2DCfg.dxintersects_[0][0] = 0;
    EnvNAV2DCfg.dyintersects_[0][0] = 1;
    EnvNAV2DCfg.dxintersects_[0][1] = 1;
    EnvNAV2DCfg.dyintersects_[0][1] = 0;

    EnvNAV2DCfg.dx_[1] = 1;
    EnvNAV2DCfg.dy_[1] = 0;
    EnvNAV2DCfg.dxintersects_[1][0] = 0;
    EnvNAV2DCfg.dyintersects_[1][0] = 0;
    EnvNAV2DCfg.dxintersects_[1][1] = 0;
    EnvNAV2DCfg.dyintersects_[1][1] = 0;

    EnvNAV2DCfg.dx_[2] = 1;
    EnvNAV2DCfg.dy_[2] = -1;
    EnvNAV2DCfg.dxintersects_[2][0] = 0;
    EnvNAV2DCfg.dyintersects_[2][0] = -1;
    EnvNAV2DCfg.dxintersects_[2][1] = 1;
    EnvNAV2DCfg.dyintersects_[2][1] = 0;

    EnvNAV2DCfg.dx_[3] = 0;
    EnvNAV2DCfg.dy_[3] = 1;
    EnvNAV2DCfg.dxintersects_[3][0] = 0;
    EnvNAV2DCfg.dyintersects_[3][0] = 0;
    EnvNAV2DCfg.dxintersects_[3][1] = 0;
    EnvNAV2DCfg.dyintersects_[3][1] = 0;

    EnvNAV2DCfg.dx_[4] = 0;
    EnvNAV2DCfg.dy_[4] = -1;
    EnvNAV2DCfg.dxintersects_[4][0] = 0;
    EnvNAV2DCfg.dyintersects_[4][0] = 0;
    EnvNAV2DCfg.dxintersects_[4][1] = 0;
    EnvNAV2DCfg.dyintersects_[4][1] = 0;

    EnvNAV2DCfg.dx_[5] = -1;
    EnvNAV2DCfg.dy_[5] = 1;
    EnvNAV2DCfg.dxintersects_[5][0] = 0;
    EnvNAV2DCfg.dyintersects_[5][0] = 1;
    EnvNAV2DCfg.dxintersects_[5][1] = -1;
    EnvNAV2DCfg.dyintersects_[5][1] = 0;

    EnvNAV2DCfg.dx_[6] = -1;
    EnvNAV2DCfg.dy_[6] = 0;
    EnvNAV2DCfg.dxintersects_[6][0] = 0;
    EnvNAV2DCfg.dyintersects_[6][0] = 0;
    EnvNAV2DCfg.dxintersects_[6][1] = 0;
    EnvNAV2DCfg.dyintersects_[6][1] = 0;

    EnvNAV2DCfg.dx_[7] = -1;
    EnvNAV2DCfg.dy_[7] = -1;
    EnvNAV2DCfg.dxintersects_[7][0] = 0;
    EnvNAV2DCfg.dyintersects_[7][0] = -1;
    EnvNAV2DCfg.dxintersects_[7][1] = -1;
    EnvNAV2DCfg.dyintersects_[7][1] = 0;

    //Note: these actions have to be starting at 8 and through 15, since they 
    //get multiplied correspondingly in Dijkstra's search based on index
    EnvNAV2DCfg.dx_[8] = 2;
    EnvNAV2DCfg.dy_[8] = 1;
    EnvNAV2DCfg.dxintersects_[8][0] = 1;
    EnvNAV2DCfg.dyintersects_[8][0] = 0;
    EnvNAV2DCfg.dxintersects_[8][1] = 1;
    EnvNAV2DCfg.dyintersects_[8][1] = 1;

    EnvNAV2DCfg.dx_[9] = 1;
    EnvNAV2DCfg.dy_[9] = 2;
    EnvNAV2DCfg.dxintersects_[9][0] = 0;
    EnvNAV2DCfg.dyintersects_[9][0] = 1;
    EnvNAV2DCfg.dxintersects_[9][1] = 1;
    EnvNAV2DCfg.dyintersects_[9][1] = 1;

    EnvNAV2DCfg.dx_[10] = -1;
    EnvNAV2DCfg.dy_[10] = 2;
    EnvNAV2DCfg.dxintersects_[10][0] = 0;
    EnvNAV2DCfg.dyintersects_[10][0] = 1;
    EnvNAV2DCfg.dxintersects_[10][1] = -1;
    EnvNAV2DCfg.dyintersects_[10][1] = 1;

    EnvNAV2DCfg.dx_[11] = -2;
    EnvNAV2DCfg.dy_[11] = 1;
    EnvNAV2DCfg.dxintersects_[11][0] = -1;
    EnvNAV2DCfg.dyintersects_[11][0] = 0;
    EnvNAV2DCfg.dxintersects_[11][1] = -1;
    EnvNAV2DCfg.dyintersects_[11][1] = 1;

    EnvNAV2DCfg.dx_[12] = -2;
    EnvNAV2DCfg.dy_[12] = -1;
    EnvNAV2DCfg.dxintersects_[12][0] = -1;
    EnvNAV2DCfg.dyintersects_[12][0] = 0;
    EnvNAV2DCfg.dxintersects_[12][1] = -1;
    EnvNAV2DCfg.dyintersects_[12][1] = -1;

    EnvNAV2DCfg.dx_[13] = -1;
    EnvNAV2DCfg.dy_[13] = -2;
    EnvNAV2DCfg.dxintersects_[13][0] = 0;
    EnvNAV2DCfg.dyintersects_[13][0] = -1;
    EnvNAV2DCfg.dxintersects_[13][1] = -1;
    EnvNAV2DCfg.dyintersects_[13][1] = -1;

    EnvNAV2DCfg.dx_[14] = 1;
    EnvNAV2DCfg.dy_[14] = -2;
    EnvNAV2DCfg.dxintersects_[14][0] = 0;
    EnvNAV2DCfg.dyintersects_[14][0] = -1;
    EnvNAV2DCfg.dxintersects_[14][1] = 1;
    EnvNAV2DCfg.dyintersects_[14][1] = -1;

    EnvNAV2DCfg.dx_[15] = 2;
    EnvNAV2DCfg.dy_[15] = -1;
    EnvNAV2DCfg.dxintersects_[15][0] = 1;
    EnvNAV2DCfg.dyintersects_[15][0] = 0;
    EnvNAV2DCfg.dxintersects_[15][1] = 1;
    EnvNAV2DCfg.dyintersects_[15][1] = -1;

    //compute distances
    for (int dind = 0; dind < ENVNAV2D_MAXDIRS; dind++) {
        if (EnvNAV2DCfg.dx_[dind] != 0 && EnvNAV2DCfg.dy_[dind] != 0) {
            if (dind <= 7)
                //the cost of a diagonal move in millimeters
                EnvNAV2DCfg.dxy_distance_mm_[dind] = (int)ceil(ENVNAV2D_COSTMULT * 1.414);
            else
                //the cost of a move to 1,2 or 2,1 or so on in millimeters
                EnvNAV2DCfg.dxy_distance_mm_[dind] = (int)ceil(ENVNAV2D_COSTMULT * 2.236);
        }
        else
            EnvNAV2DCfg.dxy_distance_mm_[dind] = ENVNAV2D_COSTMULT; //the cost of a horizontal move in millimeters
    }
}

EnvNAV2DHashEntry_t* EnvironmentNAV2D::GetHashEntry(int X, int Y)
{
#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    int binid = GETHASHBIN(X, Y);

#if DEBUG
    if ((int)EnvNAV2D.Coord2StateIDHashTable[binid].size() > 500) {
        SBPL_PRINTF("WARNING: Hash table has a bin %d (X=%d Y=%d) of size %u\n",
                    binid, X, Y, (unsigned)EnvNAV2D.Coord2StateIDHashTable[binid].size());

        PrintHashTableHist();
    }
#endif

    //iterate over the states in the bin and select the perfect match
    for (int ind = 0; ind < (int)EnvNAV2D.Coord2StateIDHashTable[binid].size(); ind++) {
        if (EnvNAV2D.Coord2StateIDHashTable[binid][ind]->X == X &&
            EnvNAV2D.Coord2StateIDHashTable[binid][ind]->Y == Y)
        {
#if TIME_DEBUG
            time_gethash += clock()-currenttime;
#endif
            return EnvNAV2D.Coord2StateIDHashTable[binid][ind];
        }
    }

#if TIME_DEBUG	
    time_gethash += clock()-currenttime;
#endif

    return NULL;
}

EnvNAV2DHashEntry_t* EnvironmentNAV2D::CreateNewHashEntry(int X, int Y)
{
    int i;

#if TIME_DEBUG	
    clock_t currenttime = clock();
#endif

    EnvNAV2DHashEntry_t* HashEntry = new EnvNAV2DHashEntry_t;

    HashEntry->X = X;
    HashEntry->Y = Y;

    HashEntry->stateID = EnvNAV2D.StateID2CoordTable.size();

    //insert into the tables
    EnvNAV2D.StateID2CoordTable.push_back(HashEntry);

    //get the hash table bin
    i = GETHASHBIN(HashEntry->X, HashEntry->Y);

    //insert the entry into the bin
    EnvNAV2D.Coord2StateIDHashTable[i].push_back(HashEntry);

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

bool EnvironmentNAV2D::IsValidCell(int X, int Y)
{
    return (X >= 0 && X < EnvNAV2DCfg.EnvWidth_c && Y >= 0 && Y < EnvNAV2DCfg.EnvHeight_c &&
            EnvNAV2DCfg.Grid2D[X][Y] < EnvNAV2DCfg.obsthresh);
}

bool EnvironmentNAV2D::IsWithinMapCell(int X, int Y)
{
    return (X >= 0 && X < EnvNAV2DCfg.EnvWidth_c && Y >= 0 && Y < EnvNAV2DCfg.EnvHeight_c);
}

EnvironmentNAV2D::~EnvironmentNAV2D()
{
    if (EnvNAV2D.Coord2StateIDHashTable != NULL) {
        delete[] EnvNAV2D.Coord2StateIDHashTable;
    }

    for (unsigned int i = 0; i < EnvNAV2D.StateID2CoordTable.size(); ++i) {
        if (EnvNAV2D.StateID2CoordTable[i] != NULL) delete EnvNAV2D.StateID2CoordTable[i];
    }

    if (EnvNAV2DCfg.Grid2D != NULL) {
        for (int x = 0; x < EnvNAV2DCfg.EnvWidth_c; x++) {
            if (EnvNAV2DCfg.Grid2D[x] != NULL) delete[] EnvNAV2DCfg.Grid2D[x];
        }
        delete[] EnvNAV2DCfg.Grid2D;
    }
}

void EnvironmentNAV2D::InitializeEnvironment()
{
    EnvNAV2DHashEntry_t* HashEntry;

    //initialize the map from Coord to StateID
    EnvNAV2D.HashTableSize = 64 * 1024; //should be power of two
    EnvNAV2D.Coord2StateIDHashTable = new vector<EnvNAV2DHashEntry_t*> [EnvNAV2D.HashTableSize];

    //initialize the map from StateID to Coord
    EnvNAV2D.StateID2CoordTable.clear();

    //create start state
    if ((HashEntry = GetHashEntry(EnvNAV2DCfg.StartX_c, EnvNAV2DCfg.StartY_c)) == NULL) {
        HashEntry = CreateNewHashEntry(EnvNAV2DCfg.StartX_c, EnvNAV2DCfg.StartY_c);
    }
    EnvNAV2D.startstateid = HashEntry->stateID;

    //create goal state 
    if ((HashEntry = GetHashEntry(EnvNAV2DCfg.EndX_c, EnvNAV2DCfg.EndY_c)) == NULL) {
        HashEntry = CreateNewHashEntry(EnvNAV2DCfg.EndX_c, EnvNAV2DCfg.EndY_c);
    }
    EnvNAV2D.goalstateid = HashEntry->stateID;

    EnvNAV2D.bInitialized = true;
}

static int EuclideanDistance(int X1, int Y1, int X2, int Y2)
{
    int sqdist = ((X1 - X2) * (X1 - X2) + (Y1 - Y2) * (Y1 - Y2));
    double dist = sqrt((double)sqdist);
    return (int)(ENVNAV2D_COSTMULT * dist);
}

//generates nNumofNeighs random neighbors of cell <X,Y> at distance nDist_c (measured in cells)
//it will also generate goal if within this distance as an additional neighbor
//bSuccs is set to true if we are computing successor states, otherwise it is Preds
void EnvironmentNAV2D::GetRandomNeighs(int stateID, std::vector<int>* NeighIDV, std::vector<int>* CLowV,
                                       int nNumofNeighs, int nDist_c, bool bSuccs)
{
    //clear the successor array
    NeighIDV->clear();
    CLowV->clear();

    //get X, Y for the states
    EnvNAV2DHashEntry_t* HashEntry = EnvNAV2D.StateID2CoordTable[stateID];
    int X = HashEntry->X;
    int Y = HashEntry->Y;

    //iterate through random actions
    for (int i = 0, nAttempts = 0; i < nNumofNeighs && nAttempts < 5 * nNumofNeighs; i++, nAttempts++) {
        int dX = 0;
        int dY = 0;

        //pick a direction
        float fDir = (float)(2 * PI_CONST * (((double)rand()) / RAND_MAX));

        //compute the successor that result from following this direction until
        //one of the coordinates reaches the desired distance
        //decide whether |dX| = dist or |dY| = dist
        float fRadius = 0;
        if (fabs(cos(fDir)) > fabs(sin(fDir))) {
            fRadius = (float)((nDist_c + 0.5) / fabs(cos(fDir)));
        }
        else {
            fRadius = (float)((nDist_c + 0.5) / fabs(sin(fDir)));
        }

        dX = (int)(fRadius * cos(fDir));
        dY = (int)(fRadius * sin(fDir));

        if ((fabs((float)dX) < nDist_c && fabs((float)dY) < nDist_c) || fabs((float)dX) > nDist_c ||
            fabs((float)dY) > nDist_c)
        {
            SBPL_ERROR("ERROR in EnvNav2D genneighs function: dX=%d dY=%d\n", dX, dY);
            throw new SBPL_Exception();
        }

        //get the coords of the state
        int newX = X + dX;
        int newY = Y + dY;

        //skip the invalid cells 
        if (!IsValidCell(newX, newY)) {
            i--;
            continue;
        }

        //get the state
        EnvNAV2DHashEntry_t* OutHashEntry;
        if ((OutHashEntry = GetHashEntry(newX, newY)) == NULL) {
            //have to create a new entry
            OutHashEntry = CreateNewHashEntry(newX, newY);
        }

        //compute clow
        int clow;
        if (bSuccs)
            clow = GetFromToHeuristic(stateID, OutHashEntry->stateID);
        else
            clow = GetFromToHeuristic(OutHashEntry->stateID, stateID);

        //insert it into the list
        NeighIDV->push_back(OutHashEntry->stateID);
        CLowV->push_back(clow);
    }

    //see if the goal/start belongs to the inside area and if yes then add it to Neighs as well
    int desX_c = EnvNAV2DCfg.EndX_c;
    int desY_c = EnvNAV2DCfg.EndY_c;
    int desstateID = EnvNAV2D.goalstateid;
    if (bSuccs == false) {
        desX_c = EnvNAV2DCfg.StartX_c;
        desY_c = EnvNAV2DCfg.StartY_c;
        desstateID = EnvNAV2D.startstateid;
    }
    //add it if within the distance
    if (abs(desX_c - X) <= nDist_c && abs(desY_c - Y) <= nDist_c) {
        //compute clow
        int clow;
        if (bSuccs)
            clow = GetFromToHeuristic(stateID, desstateID);
        else
            clow = GetFromToHeuristic(desstateID, stateID);

        NeighIDV->push_back(desstateID);
        CLowV->push_back(clow);
    }
}

//------------------------------------------------------------------------------

//------------------------------Heuristic computation--------------------------

void EnvironmentNAV2D::ComputeHeuristicValues()
{
    //whatever necessary pre-computation of heuristic values is done here 
    SBPL_PRINTF("Precomputing heuristics...\n");

    SBPL_PRINTF("done\n");
}

//-----------interface with outside functions-----------------------------------

bool EnvironmentNAV2D::InitializeEnv(const char* sEnvFile)
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

bool EnvironmentNAV2D::InitializeEnv(int width, int height, const unsigned char* mapdata, unsigned char obsthresh)
{
    return InitializeEnv(width, height, mapdata, 0, 0, 0, 0, // just use (0,0) for start and goal
                         obsthresh);
}

bool EnvironmentNAV2D::InitializeEnv(int width, int height, const unsigned char* mapdata, int startx, int starty,
                                     int goalx, int goaly, unsigned char obsthresh)
{
    SBPL_PRINTF("env: initialized with width=%d height=%d, start=%d %d, goal=%d %d, obsthresh=%d\n", width, height,
                startx, starty, goalx, goaly, obsthresh);

    EnvNAV2DCfg.obsthresh = obsthresh;

    SetConfiguration(width, height, mapdata, startx, starty, goalx, goaly);

    InitGeneral();

    return true;
}

bool EnvironmentNAV2D::InitGeneral()
{
    //Initialize other parameters of the environment
    InitializeEnvConfig();

    //initialize Environment
    InitializeEnvironment();

    //pre-compute heuristics
    ComputeHeuristicValues();

    return true;
}

bool EnvironmentNAV2D::InitializeMDPCfg(MDPConfig *MDPCfg)
{
    //initialize MDPCfg with the start and goal ids	
    MDPCfg->goalstateid = EnvNAV2D.goalstateid;
    MDPCfg->startstateid = EnvNAV2D.startstateid;

    return true;
}

int EnvironmentNAV2D::GetFromToHeuristic(int FromStateID, int ToStateID)
{
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if (FromStateID >= (int)EnvNAV2D.StateID2CoordTable.size() ||
        ToStateID >= (int)EnvNAV2D.StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in EnvNAV2D... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    //get X, Y for the state
    EnvNAV2DHashEntry_t* FromHashEntry = EnvNAV2D.StateID2CoordTable[FromStateID];
    EnvNAV2DHashEntry_t* ToHashEntry = EnvNAV2D.StateID2CoordTable[ToStateID];

    return EuclideanDistance(FromHashEntry->X, FromHashEntry->Y, ToHashEntry->X, ToHashEntry->Y);
}

int EnvironmentNAV2D::GetGoalHeuristic(int stateID)
{
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if (stateID >= (int)EnvNAV2D.StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvNAV2D... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    //define this function if it used in the planner (heuristic forward search would use it)
    return GetFromToHeuristic(stateID, EnvNAV2D.goalstateid);
}

int EnvironmentNAV2D::GetStartHeuristic(int stateID)
{
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if (stateID >= (int)EnvNAV2D.StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvNAV2D... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    //define this function if it used in the planner (heuristic backward search would use it)
    return GetFromToHeuristic(EnvNAV2D.startstateid, stateID);
}

void EnvironmentNAV2D::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
    int cost;

#if DEBUG
    if (state->StateID >= (int)EnvNAV2D.StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in Env... function: stateID illegal\n");
        throw new SBPL_Exception();
    }

    if ((int)state->Actions.size() != 0) {
        SBPL_ERROR("ERROR in Env_setAllActionsandAllOutcomes: actions already exist for the state\n");
        throw new SBPL_Exception();
    }
#endif

    //goal state should be absorbing
    if (state->StateID == EnvNAV2D.goalstateid) return;

    //get X, Y for the state
    EnvNAV2DHashEntry_t* HashEntry = EnvNAV2D.StateID2CoordTable[state->StateID];

    //iterate through actions
    bool bTestBounds = false;
    if (HashEntry->X <= 1 || HashEntry->X >= EnvNAV2DCfg.EnvWidth_c - 2 || HashEntry->Y <= 1 ||
        HashEntry->Y >= EnvNAV2DCfg.EnvHeight_c - 2)
    {
        bTestBounds = true;
    }
    for (int aind = 0; aind < EnvNAV2DCfg.numofdirs; aind++) {
        int newX = HashEntry->X + EnvNAV2DCfg.dx_[aind];
        int newY = HashEntry->Y + EnvNAV2DCfg.dy_[aind];

        //skip the invalid cells
        if (bTestBounds) {
            if (!IsValidCell(newX, newY)) continue;
        }

        //compute cost multiplier
        int costmult = EnvNAV2DCfg.Grid2D[newX][newY];
        //for diagonal move, take max over adjacent cells
        if (newX != HashEntry->X && newY != HashEntry->Y && aind <= 7) {
            //check two more cells through which the action goes
            costmult = __max(costmult, EnvNAV2DCfg.Grid2D[HashEntry->X][newY]);
            costmult = __max(costmult, EnvNAV2DCfg.Grid2D[newX][HashEntry->Y]);
        }
        else if (aind > 7) {
            //check two more cells through which the action goes
            costmult = __max(costmult,
                             EnvNAV2DCfg.Grid2D[HashEntry->X + EnvNAV2DCfg.dxintersects_[aind][0]][HashEntry->Y
                                 + EnvNAV2DCfg.dyintersects_[aind][0]]);
            costmult = __max(costmult,
                             EnvNAV2DCfg.Grid2D[HashEntry->X + EnvNAV2DCfg.dxintersects_[aind][1]][HashEntry->Y
                                 + EnvNAV2DCfg.dyintersects_[aind][1]]);
        }

        //check that it is valid
        if (costmult >= EnvNAV2DCfg.obsthresh) continue;

        //otherwise compute the actual cost
        cost = (costmult + 1) * EnvNAV2DCfg.dxy_distance_mm_[aind];

        //add the action
        CMDPACTION* action = state->AddAction(aind);

#if TIME_DEBUG
        clock_t currenttime = clock();
#endif

        EnvNAV2DHashEntry_t* OutHashEntry;
        if ((OutHashEntry = GetHashEntry(newX, newY)) == NULL) {
            //have to create a new entry
            OutHashEntry = CreateNewHashEntry(newX, newY);
        }
        action->AddOutcome(OutHashEntry->stateID, cost, 1.0);

#if TIME_DEBUG
        time3_addallout += clock()-currenttime;
#endif
    }
}

void EnvironmentNAV2D::SetAllPreds(CMDPSTATE* state)
{
    //implement this if the planner needs access to predecessors

    SBPL_ERROR("ERROR in EnvNAV2D... function: SetAllPreds is undefined\n");
    throw new SBPL_Exception();
}

void EnvironmentNAV2D::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
    int aind;

#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    //clear the successor array
    SuccIDV->clear();
    CostV->clear();
    SuccIDV->reserve(EnvNAV2DCfg.numofdirs);
    CostV->reserve(EnvNAV2DCfg.numofdirs);

    //goal state should be absorbing
    if (SourceStateID == EnvNAV2D.goalstateid) return;

    //get X, Y for the state
    EnvNAV2DHashEntry_t* HashEntry = EnvNAV2D.StateID2CoordTable[SourceStateID];

    //iterate through actions
    bool bTestBounds = false;
    if (HashEntry->X <= 1 || HashEntry->X >= EnvNAV2DCfg.EnvWidth_c - 2 || HashEntry->Y <= 1 ||
        HashEntry->Y >= EnvNAV2DCfg.EnvHeight_c - 2)
    {
        bTestBounds = true;
    }
    for (aind = 0; aind < EnvNAV2DCfg.numofdirs; aind++) {
        int newX = HashEntry->X + EnvNAV2DCfg.dx_[aind];
        int newY = HashEntry->Y + EnvNAV2DCfg.dy_[aind];

        //skip the invalid cells
        if (bTestBounds) {
            if (!IsValidCell(newX, newY)) continue;
        }

        int costmult = EnvNAV2DCfg.Grid2D[newX][newY];

        //for diagonal move, take max over adjacent cells
        if (newX != HashEntry->X && newY != HashEntry->Y && aind <= 7) {
            costmult = __max(costmult, EnvNAV2DCfg.Grid2D[HashEntry->X][newY]);
            costmult = __max(costmult, EnvNAV2DCfg.Grid2D[newX][HashEntry->Y]);
        }
        else if (aind > 7) {
            //check two more cells through which the action goes
            costmult = __max(costmult,
                             EnvNAV2DCfg.Grid2D[HashEntry->X + EnvNAV2DCfg.dxintersects_[aind][0]][HashEntry->Y
                                 + EnvNAV2DCfg.dyintersects_[aind][0]]);
            costmult = __max(costmult,
                             EnvNAV2DCfg.Grid2D[HashEntry->X + EnvNAV2DCfg.dxintersects_[aind][1]][HashEntry->Y
                                 + EnvNAV2DCfg.dyintersects_[aind][1]]);
        }

        //check that it is valid
        if (costmult >= EnvNAV2DCfg.obsthresh) continue;

        //otherwise compute the actual cost
        int cost = (costmult + 1) * EnvNAV2DCfg.dxy_distance_mm_[aind];

        EnvNAV2DHashEntry_t* OutHashEntry;
        if ((OutHashEntry = GetHashEntry(newX, newY)) == NULL) {
            //have to create a new entry
            OutHashEntry = CreateNewHashEntry(newX, newY);
        }

        SuccIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
    }

#if TIME_DEBUG
    time_getsuccs += clock()-currenttime;
#endif
}

void EnvironmentNAV2D::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{
    int aind;

#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    //clear the successor array
    PredIDV->clear();
    CostV->clear();
    PredIDV->reserve(EnvNAV2DCfg.numofdirs);
    CostV->reserve(EnvNAV2DCfg.numofdirs);

    //get X, Y for the state
    EnvNAV2DHashEntry_t* HashEntry = EnvNAV2D.StateID2CoordTable[TargetStateID];

    //no predecessors if obstacle
    if (EnvNAV2DCfg.Grid2D[HashEntry->X][HashEntry->Y] >= EnvNAV2DCfg.obsthresh) return;

    int targetcostmult = EnvNAV2DCfg.Grid2D[HashEntry->X][HashEntry->Y];

    //iterate through actions
    bool bTestBounds = false;
    if (HashEntry->X <= 1 || HashEntry->X >= EnvNAV2DCfg.EnvWidth_c - 2 || HashEntry->Y <= 1 ||
        HashEntry->Y >= EnvNAV2DCfg.EnvHeight_c - 2)
    {
        bTestBounds = true;
    }
    for (aind = 0; aind < EnvNAV2DCfg.numofdirs; aind++) {
        // the actions are undirected, so we can use the same array of actions as in getsuccs case
        int predX = HashEntry->X + EnvNAV2DCfg.dx_[aind];
        int predY = HashEntry->Y + EnvNAV2DCfg.dy_[aind];

        // skip the invalid cells
        if (bTestBounds) {
            if (!IsValidCell(predX, predY)) continue;
        }

        // compute costmult
        int costmult = targetcostmult;
        // for diagonal move, take max over adjacent cells
        if (predX != HashEntry->X && predY != HashEntry->Y && aind <= 7) {
            costmult = __max(costmult, EnvNAV2DCfg.Grid2D[HashEntry->X][predY]);
            costmult = __max(costmult, EnvNAV2DCfg.Grid2D[predX][HashEntry->Y]);
        }
        else if (aind > 7) {
            // check two more cells through which the action goes since actions
            // are undirected, we don't have to figure out what are the
            // intersecting cells on moving from <predX,predY> to <X,Y>.  it is
            // the same cells as moving from <X,Y> to <predX,predY>, which is
            // action aind
            costmult = __max(costmult,
                             EnvNAV2DCfg.Grid2D[HashEntry->X + EnvNAV2DCfg.dxintersects_[aind][0]][HashEntry->Y
                                 + EnvNAV2DCfg.dyintersects_[aind][0]]);
            costmult = __max(costmult,
                             EnvNAV2DCfg.Grid2D[HashEntry->X + EnvNAV2DCfg.dxintersects_[aind][1]][HashEntry->Y
                                 + EnvNAV2DCfg.dyintersects_[aind][1]]);
        }

        // check that it is valid
        if (costmult >= EnvNAV2DCfg.obsthresh) continue;

        // otherwise compute the actual cost (once again we use the fact that
        // actions are undirected to determine the cost)
        int cost = (costmult + 1) * EnvNAV2DCfg.dxy_distance_mm_[aind];

        EnvNAV2DHashEntry_t* OutHashEntry;
        if ((OutHashEntry = GetHashEntry(predX, predY)) == NULL) {
            // have to create a new entry
            OutHashEntry = CreateNewHashEntry(predX, predY);
        }

        PredIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
    }

#if TIME_DEBUG
    time_getsuccs += clock()-currenttime;
#endif
}

int EnvironmentNAV2D::SizeofCreatedEnv()
{
    return (int)EnvNAV2D.StateID2CoordTable.size();
}

void EnvironmentNAV2D::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG
    if (stateID >= (int)EnvNAV2D.StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvNAV2D... function: stateID illegal (2)\n");
        throw new SBPL_Exception();
    }
#endif

    if (fOut == NULL) fOut = stdout;

    EnvNAV2DHashEntry_t* HashEntry = EnvNAV2D.StateID2CoordTable[stateID];

    if (stateID == EnvNAV2D.goalstateid && bVerbose) {
        SBPL_FPRINTF(fOut, "the state is a goal state\n");
    }

    if (bVerbose)
        SBPL_FPRINTF(fOut, "X=%d Y=%d\n", HashEntry->X, HashEntry->Y);
    else
        SBPL_FPRINTF(fOut, "%d %d\n", HashEntry->X, HashEntry->Y);
}

void EnvironmentNAV2D::GetCoordFromState(int stateID, int& x, int& y) const
{
    EnvNAV2DHashEntry_t* HashEntry = EnvNAV2D.StateID2CoordTable[stateID];
    x = HashEntry->X;
    y = HashEntry->Y;
}

int EnvironmentNAV2D::GetStateFromCoord(int x, int y)
{
    EnvNAV2DHashEntry_t* OutHashEntry;
    if ((OutHashEntry = GetHashEntry(x, y)) == NULL) {
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y);
    }
    return OutHashEntry->stateID;
}

const EnvNAV2DConfig_t* EnvironmentNAV2D::GetEnvNavConfig()
{
    return &EnvNAV2DCfg;
}

//returns the stateid if success, and -1 otherwise
int EnvironmentNAV2D::SetGoal(int x, int y)
{
    if (!IsWithinMapCell(x, y)) {
        SBPL_ERROR("ERROR: trying to set a goal cell %d %d that is outside of map\n", x, y);
        return -1;
    }

    if (!IsValidCell(x, y)) {
        SBPL_PRINTF("WARNING: goal cell is invalid\n");
    }

    EnvNAV2DHashEntry_t* OutHashEntry;
    if ((OutHashEntry = GetHashEntry(x, y)) == NULL) {
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y);
    }
    EnvNAV2D.goalstateid = OutHashEntry->stateID;
    EnvNAV2DCfg.EndX_c = x;
    EnvNAV2DCfg.EndY_c = y;

    return EnvNAV2D.goalstateid;
}

void EnvironmentNAV2D::SetGoalTolerance(double tol_x, double tol_y, double tol_theta)
{
    // not used yet
}

//returns the stateid if success, and -1 otherwise
int EnvironmentNAV2D::SetStart(int x, int y)
{
    if (!IsWithinMapCell(x, y)) {
        SBPL_ERROR("ERROR: trying to set a start cell %d %d that is outside of map\n", x, y);
        return -1;
    }

    if (!IsValidCell(x, y)) {
        SBPL_PRINTF("WARNING: start cell is invalid\n");
    }

    EnvNAV2DHashEntry_t* OutHashEntry;
    if ((OutHashEntry = GetHashEntry(x, y)) == NULL) {
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y);
    }
    EnvNAV2D.startstateid = OutHashEntry->stateID;
    EnvNAV2DCfg.StartX_c = x;
    EnvNAV2DCfg.StartY_c = y;

    return EnvNAV2D.startstateid;
}

bool EnvironmentNAV2D::UpdateCost(int x, int y, unsigned char newcost)
{
    EnvNAV2DCfg.Grid2D[x][y] = newcost;

    return true;
}

void EnvironmentNAV2D::PrintEnv_Config(FILE* fOut)
{
    //implement this if the planner needs to print out EnvNAV2D. configuration

    SBPL_ERROR("ERROR in EnvNAV2D... function: PrintEnv_Config is undefined\n");
    throw new SBPL_Exception();
}

void EnvironmentNAV2D::PrintTimeStat(FILE* fOut)
{
#if TIME_DEBUG
    SBPL_FPRINTF(fOut,
                "time3_addallout = %f secs, time_gethash = %f secs, time_createhash = %f secs, time_getsuccs = %f\n",
                time3_addallout/(double)CLOCKS_PER_SEC, time_gethash/(double)CLOCKS_PER_SEC,
                time_createhash/(double)CLOCKS_PER_SEC, time_getsuccs/(double)CLOCKS_PER_SEC);
#endif
}

void EnvironmentNAV2D::GetPredsofChangedEdges(vector<nav2dcell_t> const * changedcellsV,
                                              vector<int> *preds_of_changededgesIDV)
{
    nav2dcell_t cell;
    int aind;

    for (int i = 0; i < (int)changedcellsV->size(); i++) {
        cell = changedcellsV->at(i);
        preds_of_changededgesIDV->push_back(GetStateFromCoord(cell.x, cell.y));

        for (aind = 0; aind < EnvNAV2DCfg.numofdirs; aind++) {
            //the actions are undirected, so we can use the same array of actions as in getsuccs case
            int affx = cell.x + EnvNAV2DCfg.dx_[aind];
            int affy = cell.y + EnvNAV2DCfg.dy_[aind];
            if (affx < 0 || affx >= EnvNAV2DCfg.EnvWidth_c || affy < 0 || affy >= EnvNAV2DCfg.EnvHeight_c) continue;
            preds_of_changededgesIDV->push_back(GetStateFromCoord(affx, affy));
        }
    }
}

// identical to GetPredsofChangedEdges except for changing "preds"
// into "succs"... can probably have just one method.
void EnvironmentNAV2D::GetSuccsofChangedEdges(vector<nav2dcell_t> const * changedcellsV,
                                              vector<int> *succs_of_changededgesIDV)
{
    nav2dcell_t cell;
    int aind;

    for (int i = 0; i < (int)changedcellsV->size(); i++) {
        cell = changedcellsV->at(i);
        succs_of_changededgesIDV->push_back(GetStateFromCoord(cell.x, cell.y));
        for (aind = 0; aind < EnvNAV2DCfg.numofdirs; aind++) {
            int affx = cell.x + EnvNAV2DCfg.dx_[aind];
            int affy = cell.y + EnvNAV2DCfg.dy_[aind];
            if (affx < 0 || affx >= EnvNAV2DCfg.EnvWidth_c || affy < 0 || affy >= EnvNAV2DCfg.EnvHeight_c) continue;
            succs_of_changededgesIDV->push_back(GetStateFromCoord(affx, affy));
        }
    }
}

bool EnvironmentNAV2D::IsObstacle(int x, int y)
{
    return (EnvNAV2DCfg.Grid2D[x][y] >= EnvNAV2DCfg.obsthresh);
}

unsigned char EnvironmentNAV2D::GetMapCost(int x, int y)
{
    return EnvNAV2DCfg.Grid2D[x][y];
}

void EnvironmentNAV2D::GetEnvParms(int *size_x, int *size_y, int* startx, int* starty, int* goalx, int* goaly,
                                   unsigned char* obsthresh)
{
    *size_x = EnvNAV2DCfg.EnvWidth_c;
    *size_y = EnvNAV2DCfg.EnvHeight_c;

    *startx = EnvNAV2DCfg.StartX_c;
    *starty = EnvNAV2DCfg.StartY_c;
    *goalx = EnvNAV2DCfg.EndX_c;
    *goaly = EnvNAV2DCfg.EndY_c;

    *obsthresh = EnvNAV2DCfg.obsthresh;
}

bool EnvironmentNAV2D::SetEnvParameter(const char* parameter, int value)
{
    if (EnvNAV2D.bInitialized == true) {
        SBPL_ERROR("ERROR: all parameters must be set before initialization of the environment\n");
        return false;
    }

    SBPL_PRINTF("setting parameter %s to %d\n", parameter, value);

    if (strcmp(parameter, "is16connected") == 0) {
        if (value != 0)
            EnvNAV2DCfg.numofdirs = 16;
        else
            EnvNAV2DCfg.numofdirs = 8;
    }
    else {
        SBPL_ERROR("ERROR: invalid parameter %s\n", parameter);
        return false;
    }

    return true;
}

//returns true if two states meet the same condition - see environment.h for more info
bool EnvironmentNAV2D::AreEquivalent(int StateID1, int StateID2)
{
#if DEBUG
    if (StateID1 >= (int)EnvNAV2D.StateID2CoordTable.size() || StateID2 >= (int)EnvNAV2D.StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvNAV2D... function: stateID illegal (2)\n");
        throw new SBPL_Exception();
    }
#endif

    //get X, Y for the states
    EnvNAV2DHashEntry_t* HashEntry1 = EnvNAV2D.StateID2CoordTable[StateID1];
    EnvNAV2DHashEntry_t* HashEntry2 = EnvNAV2D.StateID2CoordTable[StateID2];

    if (HashEntry1->X == HashEntry2->X && HashEntry1->Y == HashEntry2->Y) return true;

    return false;
}

//generate succs at some domain-dependent distance - see environment.h for more info
void EnvironmentNAV2D::GetRandomSuccsatDistance(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CLowV)
{
    //number of random neighbors
    int nNumofNeighs = 10;
    //distance at which the neighbors are generated
    int nDist_c = 100;

#if DEBUG
    if (SourceStateID >= (int)EnvNAV2D.StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvNAV2DGetRandSuccs... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    //goal state should be absorbing
    if (SourceStateID == EnvNAV2D.goalstateid) return;

    //get the successors
    bool bSuccs = true;
    GetRandomNeighs(SourceStateID, SuccIDV, CLowV, nNumofNeighs, nDist_c, bSuccs);
}

//generate preds at some domain-dependent distance - see environment.h for more info
void EnvironmentNAV2D::GetRandomPredsatDistance(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CLowV)
{
    //number of random neighbors
    int nNumofNeighs = 10;
    //distance at which the neighbors are generated
    int nDist_c = 5; //TODO-was100;

#if DEBUG
    if (TargetStateID >= (int)EnvNAV2D.StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvNAV2DGetRandSuccs... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    //start state does not have start state
    if (TargetStateID == EnvNAV2D.startstateid) return;

    //get the predecessors
    bool bSuccs = false;
    GetRandomNeighs(TargetStateID, PredIDV, CLowV, nNumofNeighs, nDist_c, bSuccs);
}

//------------------------------------------------------------------------------
