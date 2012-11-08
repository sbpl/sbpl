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

#ifndef __2DGRIDSEARCH_H_
#define __2DGRIDSEARCH_H_

#include <cstdlib>
#include <sbpl/planners/planner.h>
#include <sbpl/utils/key.h>
#include <sbpl/utils/utils.h>

#define SBPL_2DGRIDSEARCH_NUMOF2DDIRS 16

#define SBPL_2DSEARCH_OPEN_LIST_ID 0

// Forward declaration needed to allow instance of CIntHeap*
class CIntHeap;
class CSlidingBucket;

enum SBPL_2DGRIDSEARCH_TERM_CONDITION
{
    SBPL_2DGRIDSEARCH_TERM_CONDITION_OPTPATHFOUND,
    SBPL_2DGRIDSEARCH_TERM_CONDITION_20PERCENTOVEROPTPATH,
    SBPL_2DGRIDSEARCH_TERM_CONDITION_TWOTIMESOPTPATH,
    SBPL_2DGRIDSEARCH_TERM_CONDITION_THREETIMESOPTPATH,
    SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS
};

enum SBPL_2DGRIDSEARCH_OPENTYPE
{
    SBPL_2DGRIDSEARCH_OPENTYPE_HEAP, SBPL_2DGRIDSEARCH_OPENTYPE_SLIDINGBUCKETS
};

//#define SBPL_2DGRIDSEARCH_HEUR2D(x,y)  ((int)(1000*cellSize_m_*sqrt((double)((x-goalX_)*(x-goalX_)+(y-goalY_)*(y-goalY_)))))
#define SBPL_2DGRIDSEARCH_HEUR2D(x,y)  ((int)(1000*cellSize_m_*__max(abs(x-goalX_),abs(y-goalY_))))

/**
 * \brief search state corresponding to each 2D cell
 */
class SBPL_2DGridSearchState : public AbstractSearchState
{
public:
    /**
     * \brief coordinates
     */
    int x, y;

    /**
     * \brief search relevant data
     */
    int g;
    /**
     * \brief iteration at which the state was accessed (generated) last
     */
    int iterationaccessed;

public:
    SBPL_2DGridSearchState() { iterationaccessed = 0; }
    ~SBPL_2DGridSearchState() { }
};

/**
 * \brief 2D search itself
 */
class SBPL2DGridSearch
{
public:
    SBPL2DGridSearch(int width_x, int height_y, float cellsize_m);
    ~SBPL2DGridSearch()
    {
        destroy();
    }

    /**
     * \brief the priority data structure used - it affects efficiency (can be
     *        either heap or sliding buckets, see definition of
     *        SBPL_2DGRIDSEARCH_OPENTYPE)
     */
    bool setOPENdatastructure(SBPL_2DGRIDSEARCH_OPENTYPE OPENtype);

    /**
     * \brief destroys the search and clears all memory
     */
    void destroy();

    /** \brief performs search itself. All costs are given as cost(cell1,
     *         cell2) = Euclidean distance*1000*(max(cell1,cell2)+1) for adjacent
     *         cells.
     * \note It is infinite if max(cell1,cell2) >= obsthresh For cells that are
     *       not adjacent (which may happen if 16 connected grid is ON), then max is
     *       taken over all the cells center line goes through Search is done from
     *       start to goal. termination_condition specifies when to stop the search.
     *       In particular, one may specify to run it until OPEN is empty. In this
     *       case, the values of all states will be computed.
     */
    bool search(unsigned char** Grid2D, unsigned char obsthresh, int startx_c, int starty_c, int goalx_c, int goaly_c,
                SBPL_2DGRIDSEARCH_TERM_CONDITION termination_condition);

    /**
     * \brief print all the values
     */
    void printvalues();

    /**
     * \brief returns the computed distance from the start to <x,y>. If not computed, then returns lower bound on it.
     */
    inline int getlowerboundoncostfromstart_inmm(int x, int y)
    {
        if (term_condition_usedlast == SBPL_2DGRIDSEARCH_TERM_CONDITION_OPTPATHFOUND) {
            //heuristic search
            int h = SBPL_2DGRIDSEARCH_HEUR2D(x,y);
            //the logic is that if s wasn't expanded, then g(s) + h(s) >=
            //maxcomputed_fval => g(s) >= maxcomputed_fval - h(s)
            return ((searchStates2D_[x][y].iterationaccessed == iteration_ &&
                    searchStates2D_[x][y].g + h <= largestcomputedoptf_)      ? searchStates2D_[x][y].g :
                    largestcomputedoptf_ < INFINITECOST                       ? largestcomputedoptf_ - h :
                                                                                INFINITECOST);
        }
        else {
            //Dijkstra's search
            //the logic is that if s wasn't expanded, then g(s) >= maxcomputed_fval => g(s) >= maxcomputed_fval - h(s)
            return ((searchStates2D_[x][y].iterationaccessed == iteration_ &&
                    searchStates2D_[x][y].g <= largestcomputedoptf_) ? searchStates2D_[x][y].g :
                    largestcomputedoptf_);
        }
    }

    /**
     * \brief returns largest optimal g-value computed by search - a lower
     *        bound on the state values of unexpanded states
     */
    int getlargestcomputedoptimalf_inmm() { return largestcomputedoptf_; }

private:
    inline bool withinMap(int x, int y)
    {
        return (x >= 0 && y >= 0 && x < width_ && y < height_);
    }

    void computedxy();
    inline void initializeSearchState2D(SBPL_2DGridSearchState* state2D);
    bool createSearchStates2D(void);

    bool search_withheap(unsigned char** Grid2D, unsigned char obsthresh, int startx_c, int starty_c, int goalx_c,
                         int goaly_c, SBPL_2DGRIDSEARCH_TERM_CONDITION termination_condition);
    bool search_exp(unsigned char** Grid2D, unsigned char obsthresh, int startx_c, int starty_c, int goalx_c,
                    int goaly_c);
    bool search_withbuckets(unsigned char** Grid2D, unsigned char obsthresh, int startx_c, int starty_c, int goalx_c,
                            int goaly_c);
    bool search_withslidingbuckets(unsigned char** Grid2D, unsigned char obsthresh, int startx_c, int starty_c,
                                   int goalx_c, int goaly_c, SBPL_2DGRIDSEARCH_TERM_CONDITION termination_condition);

    //2D search data
    CSlidingBucket* OPEN2DBLIST_;
    CIntHeap* OPEN2D_;
    SBPL_2DGridSearchState** searchStates2D_;
    int dx_[SBPL_2DGRIDSEARCH_NUMOF2DDIRS];
    int dy_[SBPL_2DGRIDSEARCH_NUMOF2DDIRS];
    //the intermediate cells through which the actions go 
    //(for all the ones with the first index <=7, there are none(they go to direct neighbors) -> initialized to -1)
    int dx0intersects_[SBPL_2DGRIDSEARCH_NUMOF2DDIRS];
    int dx1intersects_[SBPL_2DGRIDSEARCH_NUMOF2DDIRS];
    int dy0intersects_[SBPL_2DGRIDSEARCH_NUMOF2DDIRS];
    int dy1intersects_[SBPL_2DGRIDSEARCH_NUMOF2DDIRS];
    //distances of transitions
    int dxy_distance_mm_[SBPL_2DGRIDSEARCH_NUMOF2DDIRS];

    //OPEN data structure type
    SBPL_2DGRIDSEARCH_OPENTYPE OPENtype_;

    //start and goal configurations
    int startX_, startY_;
    int goalX_, goalY_;

    //map parameters
    int width_, height_;
    float cellSize_m_;

    //search iteration
    int iteration_;

    //largest optimal g-value computed by search
    int largestcomputedoptf_;

    //termination criterion used in the search
    SBPL_2DGRIDSEARCH_TERM_CONDITION term_condition_usedlast;
};

#endif

