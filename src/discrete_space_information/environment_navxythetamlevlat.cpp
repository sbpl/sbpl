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

#include <cstdio>
#include <ctime>
#include <sbpl/discrete_space_information/environment_navxythetamlevlat.h>
#include <sbpl/utils/key.h>

using namespace std;

#if TIME_DEBUG
static clock_t time3_addallout = 0;
static clock_t time_gethash = 0;
static clock_t time_createhash = 0;
static clock_t time_getsuccs = 0;
#endif

static long int checks = 0;

//-----------------constructors/destructors-------------------------------

EnvironmentNAVXYTHETAMLEVLAT::EnvironmentNAVXYTHETAMLEVLAT()
{
    numofadditionalzlevs = 0; //by default there is only base level, no additional levels
    AddLevelFootprintPolygonV = NULL;
    AdditionalInfoinActionsV = NULL;
    AddLevelGrid2D = NULL;
    AddLevel_cost_possibly_circumscribed_thresh = NULL;
    AddLevel_cost_inscribed_thresh = NULL;
}

EnvironmentNAVXYTHETAMLEVLAT::~EnvironmentNAVXYTHETAMLEVLAT()
{
    if (AddLevelFootprintPolygonV != NULL) {
        delete[] AddLevelFootprintPolygonV;
        AddLevelFootprintPolygonV = NULL;
    }

    if (AdditionalInfoinActionsV != NULL) {
        for (int tind = 0; tind < EnvNAVXYTHETALATCfg.NumThetaDirs; tind++) {
            for (int aind = 0; aind < EnvNAVXYTHETALATCfg.actionwidth; aind++) {
                delete[] AdditionalInfoinActionsV[tind][aind].intersectingcellsV;
            }
            delete[] AdditionalInfoinActionsV[tind];
        }
        delete[] AdditionalInfoinActionsV;
        AdditionalInfoinActionsV = NULL;
    }

    if (AddLevelGrid2D != NULL) {
        for (int levelind = 0; levelind < numofadditionalzlevs; levelind++) {
            for (int xind = 0; xind < EnvNAVXYTHETALATCfg.EnvWidth_c; xind++) {
                delete[] AddLevelGrid2D[levelind][xind];
            }
            delete[] AddLevelGrid2D[levelind];
        }
        delete[] AddLevelGrid2D;
        AddLevelGrid2D = NULL;
    }

    if (AddLevel_cost_possibly_circumscribed_thresh != NULL) {
        delete[] AddLevel_cost_possibly_circumscribed_thresh;
        AddLevel_cost_possibly_circumscribed_thresh = NULL;
    }

    if (AddLevel_cost_inscribed_thresh != NULL) {
        delete[] AddLevel_cost_inscribed_thresh;
        AddLevel_cost_inscribed_thresh = NULL;
    }

    //reset the number of additional levels
    numofadditionalzlevs = 0;
}

//---------------------------------------------------------------------

//-------------------problem specific and local functions---------------------

//returns true if cell is traversable and within map limits - it checks against all levels including the base one
bool EnvironmentNAVXYTHETAMLEVLAT::IsValidCell(int X, int Y)
{
    int levelind;

    if (!EnvironmentNAVXYTHETALAT::IsValidCell(X, Y)) return false;

    //iterate through the additional levels
    for (levelind = 0; levelind < numofadditionalzlevs; levelind++) {
        if (AddLevelGrid2D[levelind][X][Y] >= EnvNAVXYTHETALATCfg.obsthresh) return false;
    }
    //otherwise the cell is valid at all levels
    return true;
}

// returns true if cell is traversable and within map limits for a particular level
bool EnvironmentNAVXYTHETAMLEVLAT::IsValidCell(int X, int Y, int levind)
{
    return (X >= 0 && X < EnvNAVXYTHETALATCfg.EnvWidth_c && Y >= 0 && Y < EnvNAVXYTHETALATCfg.EnvHeight_c && levind <
            numofadditionalzlevs && AddLevelGrid2D[levind][X][Y] < EnvNAVXYTHETALATCfg.obsthresh);
}

//returns true if cell is untraversable at all levels
bool EnvironmentNAVXYTHETAMLEVLAT::IsObstacle(int X, int Y)
{
    int levelind;

    if (EnvironmentNAVXYTHETALAT::IsObstacle(X, Y)) return true;

    //iterate through the additional levels
    for (levelind = 0; levelind < numofadditionalzlevs; levelind++) {
        if (AddLevelGrid2D[levelind][X][Y] >= EnvNAVXYTHETALATCfg.obsthresh) return true;
    }
    //otherwise the cell is obstacle-free at all cells
    return false;
}

//returns true if cell is untraversable in level # levelnum. If levelnum = -1, then it checks all levels
bool EnvironmentNAVXYTHETAMLEVLAT::IsObstacle(int X, int Y, int levind)
{
#if DEBUG
    if(levind >= numofadditionalzlevs)
    {
        SBPL_ERROR("ERROR: IsObstacle invoked at level %d\n", levind);
        SBPL_FPRINTF(fDeb, "ERROR: IsObstacle invoked at level %d\n", levind);
        return false;
    }
#endif

    return (AddLevelGrid2D[levind][X][Y] >= EnvNAVXYTHETALATCfg.obsthresh);
}

// returns the maximum over all levels of the cost corresponding to the cell <x,y>
unsigned char EnvironmentNAVXYTHETAMLEVLAT::GetMapCost(int X, int Y)
{
    unsigned char mapcost = EnvNAVXYTHETALATCfg.Grid2D[X][Y];

    for (int levind = 0; levind < numofadditionalzlevs; levind++) {
        mapcost = __max(mapcost, AddLevelGrid2D[levind][X][Y]);
    }

    return mapcost;
}

// returns the cost corresponding to the cell <x,y> at level levind
unsigned char EnvironmentNAVXYTHETAMLEVLAT::GetMapCost(int X, int Y, int levind)
{
#if DEBUG
    if(levind >= numofadditionalzlevs)
    {
        SBPL_ERROR("ERROR: GetMapCost invoked at level %d\n", levind);
        SBPL_FPRINTF(fDeb, "ERROR: GetMapCost invoked at level %d\n", levind);
        return false;
    }
#endif

    return AddLevelGrid2D[levind][X][Y];
}

//returns false if robot intersects obstacles or lies outside of the map.
bool EnvironmentNAVXYTHETAMLEVLAT::IsValidConfiguration(int X, int Y, int Theta)
{
    //check the base footprint first
    if (!EnvironmentNAVXYTHETALAT::IsValidConfiguration(X, Y, Theta)) return false;

    //check the remaining levels now
    vector<sbpl_2Dcell_t> footprint;
    sbpl_xy_theta_pt_t pose;

    //compute continuous pose
    pose.x = DISCXY2CONT(X, EnvNAVXYTHETALATCfg.cellsize_m);
    pose.y = DISCXY2CONT(Y, EnvNAVXYTHETALATCfg.cellsize_m);
    pose.theta = DiscTheta2Cont(Theta, EnvNAVXYTHETALATCfg.NumThetaDirs);

    //iterate over additional levels
    for (int levind = 0; levind < numofadditionalzlevs; levind++) {

        //compute footprint cells
        footprint.clear();
        get_2d_footprint_cells(AddLevelFootprintPolygonV[levind], &footprint, pose, EnvNAVXYTHETALATCfg.cellsize_m);

        //iterate over all footprint cells
        for (int find = 0; find < (int)footprint.size(); find++) {
            int x = footprint.at(find).x;
            int y = footprint.at(find).y;

            if (x < 0 || x >= EnvNAVXYTHETALATCfg.EnvWidth_c || y < 0 || y >= EnvNAVXYTHETALATCfg.EnvHeight_c
                || AddLevelGrid2D[levind][x][y] >= EnvNAVXYTHETALATCfg.obsthresh) {
                return false;
            }
        }
    }

    return true;
}

int EnvironmentNAVXYTHETAMLEVLAT::GetActionCost(int SourceX, int SourceY, int SourceTheta,
                                                EnvNAVXYTHETALATAction_t* action)
{
    int basecost = EnvironmentNAVXYTHETALAT::GetActionCost(SourceX, SourceY, SourceTheta, action);

    if (basecost >= INFINITECOST) return INFINITECOST;

    int addcost = GetActionCostacrossAddLevels(SourceX, SourceY, SourceTheta, action);

    return __max(basecost, addcost);
}

int EnvironmentNAVXYTHETAMLEVLAT::GetActionCostacrossAddLevels(int SourceX, int SourceY, int SourceTheta,
                                                               EnvNAVXYTHETALATAction_t* action)
{
    sbpl_2Dcell_t cell;
    sbpl_xy_theta_cell_t interm3Dcell;
    int i, levelind = -1;

    if (!IsValidCell(SourceX, SourceY)) return INFINITECOST;
    if (!IsValidCell(SourceX + action->dX, SourceY + action->dY)) return INFINITECOST;

    //case of no levels
    if (numofadditionalzlevs == 0) return 0;

    //iterate through the additional levels
    for (levelind = 0; levelind < numofadditionalzlevs; levelind++) {
        if (AddLevelGrid2D[levelind][SourceX + action->dX][SourceY + action->dY]
            >= AddLevel_cost_inscribed_thresh[levelind]) return INFINITECOST;
    }

    //need to iterate over discretized center cells and compute cost based on them
    unsigned char maxcellcost = 0;
    unsigned char* maxcellcostateachlevel = new unsigned char[numofadditionalzlevs];
    for (levelind = 0; levelind < numofadditionalzlevs; levelind++) {
        maxcellcostateachlevel[levelind] = 0;
    }

    for (i = 0; i < (int)action->interm3DcellsV.size() && maxcellcost < EnvNAVXYTHETALATCfg.obsthresh; i++) {
        interm3Dcell = action->interm3DcellsV.at(i);
        interm3Dcell.x = interm3Dcell.x + SourceX;
        interm3Dcell.y = interm3Dcell.y + SourceY;

        if (interm3Dcell.x < 0 || interm3Dcell.x >= EnvNAVXYTHETALATCfg.EnvWidth_c || interm3Dcell.y < 0 ||
            interm3Dcell.y >= EnvNAVXYTHETALATCfg.EnvHeight_c)
        {
            maxcellcost = EnvNAVXYTHETALATCfg.obsthresh;
            break;
        }

        for (levelind = 0; levelind < numofadditionalzlevs; levelind++) {
            maxcellcost = __max(maxcellcost, AddLevelGrid2D[levelind][interm3Dcell.x][interm3Dcell.y]);
            maxcellcostateachlevel[levelind] = __max(maxcellcostateachlevel[levelind],
                                                     AddLevelGrid2D[levelind][interm3Dcell.x][interm3Dcell.y]);
            //check that the robot is NOT in the cell at which there is no valid orientation
            if (maxcellcostateachlevel[levelind] >= AddLevel_cost_inscribed_thresh[levelind]) {
                maxcellcost = EnvNAVXYTHETALATCfg.obsthresh;
                maxcellcostateachlevel[levelind] = EnvNAVXYTHETALATCfg.obsthresh;
                break;
            }
        }
    }

    //check collisions that for the particular footprint orientation along the action
    for (levelind = 0; levelind < numofadditionalzlevs && (int)maxcellcost < EnvNAVXYTHETALATCfg.obsthresh;
         levelind++)
    {
        if (AddLevelFootprintPolygonV[levelind].size() > 1 && (int)maxcellcostateachlevel[levelind] >=
            AddLevel_cost_possibly_circumscribed_thresh[levelind])
        {
            checks++;

            //get intersecting cells for this level
            vector<sbpl_2Dcell_t>* intersectingcellsV =
                &AdditionalInfoinActionsV[(unsigned int)action->starttheta][action->aind].intersectingcellsV[levelind];
            for (i = 0; i < (int)intersectingcellsV->size(); i++) {
                //get the cell in the map
                cell = intersectingcellsV->at(i);
                cell.x = cell.x + SourceX;
                cell.y = cell.y + SourceY;

                //check validity
                if (!IsValidCell(cell.x, cell.y, levelind)) {
                    maxcellcost = EnvNAVXYTHETALATCfg.obsthresh;
                    break;
                }

                //if(AddLevelGrid2D[levelind][cell.x][cell.y] > maxcellcost)
                ////cost computation changed: cost = max(cost of centers of
                //the robot along action)
                //	maxcellcost = AddLevelGrid2D[levelind][cell.x][cell.y];
                //	//intersecting cells are only used for collision checking
            }
        }
    }

    //no need to max it with grid2D to ensure consistency of h2D since it is done for the base class

    //clean up
    delete[] maxcellcostateachlevel;

    if (maxcellcost >= EnvNAVXYTHETALATCfg.obsthresh)
        return INFINITECOST;
    else
        return action->cost * (((int)maxcellcost) + 1); //use cell cost as multiplicative factor
}

//---------------------------------------------------------------------

//------------debugging functions---------------------------------------------

//-----------------------------------------------------------------------------

//-----------interface with outside functions-----------------------------------

/*
 initialization of additional levels. 0 is the original one. All additional ones will start with index 1
 */
bool EnvironmentNAVXYTHETAMLEVLAT::InitializeAdditionalLevels(int numofadditionalzlevs_in,
                                                              const vector<sbpl_2Dpt_t>* perimeterptsV,
                                                              unsigned char* cost_inscribed_thresh_in,
                                                              unsigned char* cost_possibly_circumscribed_thresh_in)
{
    int levelind = -1, xind = -1, yind = -1;
    sbpl_xy_theta_pt_t temppose;
    temppose.x = 0.0;
    temppose.y = 0.0;
    temppose.theta = 0.0;
    vector<sbpl_2Dcell_t> footprint;

    numofadditionalzlevs = numofadditionalzlevs_in;
    SBPL_PRINTF("Planning with additional z levels. Number of additional z levels = %d\n", numofadditionalzlevs);

    //allocate memory and set FootprintPolygons for additional levels
    AddLevelFootprintPolygonV = new vector<sbpl_2Dpt_t> [numofadditionalzlevs];
    for (levelind = 0; levelind < numofadditionalzlevs; levelind++) {
        AddLevelFootprintPolygonV[levelind] = perimeterptsV[levelind];
    }

    //print out the size of a footprint for each additional level
    for (levelind = 0; levelind < numofadditionalzlevs; levelind++) {
        footprint.clear();
        get_2d_footprint_cells(AddLevelFootprintPolygonV[levelind], &footprint, temppose,
                               EnvNAVXYTHETALATCfg.cellsize_m);
        SBPL_PRINTF("number of cells in footprint for additional level %d = %d\n", levelind,
                    (unsigned int)footprint.size());
    }

    //compute additional levels action info
    SBPL_PRINTF("pre-computing action data for additional levels:\n");
    AdditionalInfoinActionsV = new EnvNAVXYTHETAMLEVLATAddInfoAction_t*[EnvNAVXYTHETALATCfg.NumThetaDirs];
    for (int tind = 0; tind < EnvNAVXYTHETALATCfg.NumThetaDirs; tind++) {
        SBPL_PRINTF("pre-computing for angle %d out of %d angles\n", tind, EnvNAVXYTHETALATCfg.NumThetaDirs);

        //compute sourcepose
        sbpl_xy_theta_pt_t sourcepose;
        sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
        sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
        sourcepose.theta = DiscTheta2Cont(tind, EnvNAVXYTHETALATCfg.NumThetaDirs);

        AdditionalInfoinActionsV[tind] = new EnvNAVXYTHETAMLEVLATAddInfoAction_t[EnvNAVXYTHETALATCfg.actionwidth];

        //iterate over actions for each angle
        for (int aind = 0; aind < EnvNAVXYTHETALATCfg.actionwidth; aind++) {
            EnvNAVXYTHETALATAction_t* nav3daction = &EnvNAVXYTHETALATCfg.ActionsV[tind][aind];

            //initialize delta variables
            AdditionalInfoinActionsV[tind][aind].dX = nav3daction->dX;
            AdditionalInfoinActionsV[tind][aind].dY = nav3daction->dY;
            AdditionalInfoinActionsV[tind][aind].starttheta = tind;
            AdditionalInfoinActionsV[tind][aind].endtheta = nav3daction->endtheta;

            //finally, create the footprint for the action for each level
            AdditionalInfoinActionsV[tind][aind].intersectingcellsV = new vector<sbpl_2Dcell_t> [numofadditionalzlevs];
            for (levelind = 0; levelind < numofadditionalzlevs; levelind++) {
                get_2d_motion_cells(AddLevelFootprintPolygonV[levelind],
                                    EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV,
                                    &AdditionalInfoinActionsV[tind][aind].intersectingcellsV[levelind],
                                    EnvNAVXYTHETALATCfg.cellsize_m);
            }
        }
    }

    //create maps for additional levels and initialize to zeros (freespace)
    AddLevelGrid2D = new unsigned char**[numofadditionalzlevs];
    for (levelind = 0; levelind < numofadditionalzlevs; levelind++) {
        AddLevelGrid2D[levelind] = new unsigned char*[EnvNAVXYTHETALATCfg.EnvWidth_c];
        for (xind = 0; xind < EnvNAVXYTHETALATCfg.EnvWidth_c; xind++) {
            AddLevelGrid2D[levelind][xind] = new unsigned char[EnvNAVXYTHETALATCfg.EnvHeight_c];
            for (yind = 0; yind < EnvNAVXYTHETALATCfg.EnvHeight_c; yind++) {
                AddLevelGrid2D[levelind][xind][yind] = 0;
            }
        }
    }

    //create inscribed and circumscribed cost thresholds
    AddLevel_cost_possibly_circumscribed_thresh = new unsigned char[numofadditionalzlevs];
    AddLevel_cost_inscribed_thresh = new unsigned char[numofadditionalzlevs];
    for (levelind = 0; levelind < numofadditionalzlevs; levelind++) {
        AddLevel_cost_possibly_circumscribed_thresh[levelind] = cost_possibly_circumscribed_thresh_in[levelind];
        AddLevel_cost_inscribed_thresh[levelind] = cost_inscribed_thresh_in[levelind];
    }

    return true;
}

//set 2D map for the additional level levind
bool EnvironmentNAVXYTHETAMLEVLAT::Set2DMapforAddLev(const unsigned char* mapdata, int levind)
{
    int xind = -1, yind = -1;

    if (AddLevelGrid2D == NULL) {
        SBPL_ERROR("ERROR: failed to set2Dmap because the map was not allocated previously\n");
        return false;
    }

    for (xind = 0; xind < EnvNAVXYTHETALATCfg.EnvWidth_c; xind++) {
        for (yind = 0; yind < EnvNAVXYTHETALATCfg.EnvHeight_c; yind++) {
            AddLevelGrid2D[levind][xind][yind] = mapdata[xind + yind * EnvNAVXYTHETALATCfg.EnvWidth_c];
        }
    }

    return true;
}

//set 2D map for the additional level levind
//the version of Set2DMapforAddLev that takes newmap as 2D array instead of one linear array
bool EnvironmentNAVXYTHETAMLEVLAT::Set2DMapforAddLev(const unsigned char** NewGrid2D, int levind)
{
    int xind = -1, yind = -1;

    if (AddLevelGrid2D == NULL) {
        SBPL_ERROR("ERROR: failed to set2Dmap because the map was not allocated previously\n");
        return false;
    }

    for (xind = 0; xind < EnvNAVXYTHETALATCfg.EnvWidth_c; xind++) {
        for (yind = 0; yind < EnvNAVXYTHETALATCfg.EnvHeight_c; yind++) {
            AddLevelGrid2D[levind][xind][yind] = NewGrid2D[xind][yind];
        }
    }

    return true;
}

/*
 update the traversability of a cell<x,y> in addtional level zlev (this is not to update basic level)
 */
bool EnvironmentNAVXYTHETAMLEVLAT::UpdateCostinAddLev(int x, int y, unsigned char newcost, int zlev)
{
#if DEBUG
    //SBPL_FPRINTF(fDeb, "Cost updated for cell %d %d at level %d from old cost=%d to new cost=%d\n", x, y, zlev, AddLevelGrid2D[zlev][x][y], newcost);
#endif

    AddLevelGrid2D[zlev][x][y] = newcost;

    //no need to update heuristics because at this point it is computed solely based on the basic level

    return true;
}

//------------------------------------------------------------------------------
