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
#ifndef __UTILS_H_
#define __UTILS_H_

#ifndef WIN32
#define __max(x,y) (x>y?x:y)
#define __min(x,y) (x>y?y:x)
#endif


#define NORMALIZEDISCTHETA(THETA, THETADIRS) (((THETA>=0)?((THETA)%(THETADIRS)):(((THETA)%(THETADIRS)+THETADIRS)%THETADIRS)))

#define CONTXY2DISC(X, CELLSIZE) (((X)>=0)?((int)((X)/(CELLSIZE))):((int)((X)/(CELLSIZE))-1))
#define DISCXY2CONT(X, CELLSIZE) ((X)*(CELLSIZE) + (CELLSIZE)/2.0)


#define PI_CONST 3.141592653589793238462643383279502884

#define UNKNOWN_COST 1000000

typedef struct {
  int X1, Y1;
  int X2, Y2;
  int Increment;
  int UsingYIndex;
  int DeltaX, DeltaY;
  int DTerm;
  int IncrE, IncrNE;
  int XIndex, YIndex;
  int Flipped;
} bresenham_param_t;


typedef struct {
	int x;
	int y;
} sbpl_2Dcell_t;


typedef struct {
	double x;
	double y;
} sbpl_2Dpt_t;

typedef struct BINARYHIDDENVARIABLE
{
	int h_ID; //ID of the variable
	unsigned char Prob;

}sbpl_BinaryHiddenVar_t;

typedef struct BELIEFSTATEWITHBINARYHVALS
{
	int s_ID; //ID of S part of state-space
	vector<sbpl_BinaryHiddenVar_t> updatedhvaluesV; //vector of updated h-values, the rest are the same as at the start state
	

}sbpl_BeliefStatewithBinaryh_t;

typedef struct POLICYBELIEFSTATEWITHBINARYHVALS
{

	sbpl_BeliefStatewithBinaryh_t BeliefState; //current belief state

	int nextpolicyactionID; //ID of the next policy action if exists (otherwise -1)
	vector<int> outcomestateIndexV; //indices of the outcome states of the policy action. If outcome state is not in the policy, then it is -1

}sbpl_PolicyStatewithBinaryh_t;




//function prototypes
#if MEM_CHECK == 1
void DisableMemCheck();
void EnableMemCheck();
#endif
void CheckMDP(CMDP* mdp);
void PrintMatrix(int** matrix, int rows, int cols, FILE* fOut);
void EvaluatePolicy(CMDP* PolicyMDP, int StartStateID, int GoalStateID,
					double* PolValue, bool *bFullPolicy, double *Pcgoal, 
					int* nMerges, bool *bCycles);
int ComputeNumofStochasticActions(CMDP* pMDP);

/** \brief one of the three functions that correspond to bresenham algorithm of path following
this function computes bresenham parameters given the start and end points on the line segment
*/
void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params);
/** \brief one of the three functions that correspond to bresenham algorithm of path following
returns current cell on the line segment
*/
void get_current_point(bresenham_param_t *params, int *x, int *y);
/** \brief one of the three functions that correspond to bresenham algorithm of path following
moves to the next point
*/
int get_next_point(bresenham_param_t *params);

/** \brief converts discretized version of angle into continuous (radians)
    \note maps 0->0, 1->delta, 2->2*delta, ...
*/
double DiscTheta2Cont(int nTheta, int NUMOFANGLEVALS);
/** \brief converts continuous (radians) version of angle into discrete
    \note maps 0->0, [delta/2, 3/2*delta)->1, [3/2*delta, 5/2*delta)->2,...
*/
int ContTheta2Disc(double fTheta, int NUMOFANGLEVALS);

/** \note counterclockwise is positive
    \param angle input angle should be in radians
    \return output is an angle in the range of from 0 to 2*PI
*/
double normalizeAngle(double angle);

/** \brief computes minimum unsigned difference between two angles in radians
  */
double computeMinUnsignedAngleDiff(double angle1, double angle2);

/** \brief returns true if 2D point is within the specified polygon given by ordered sequence of 2D points (last point is automatically connected to the first)
  */
bool IsInsideFootprint(sbpl_2Dpt_t pt, vector<sbpl_2Dpt_t>* bounding_polygon);

/** \brief computes 8-connected distances - performs distance transform in two linear passes
  */
void computeDistancestoNonfreeAreas(unsigned char** Grid2D, int width_x, int height_y, unsigned char obsthresh, float** disttoObs_incells, 
									float** disttoNonfree_incells);


#if 0
void CheckSearchMDP(CMDP* mdp, int ExcludeSuccStateID = -1);
void CheckSearchPredSucc(CMDPSTATE* state, int ExcludeSuccStateID = -1);
#endif

#endif
