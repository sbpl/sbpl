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
#ifndef __PLANNER_H_
#define __PLANNER_H_


#define 	GETSTATEIND(stateid, mapid) StateID2IndexMapping[mapid][stateid]


//indices for the StateID2Index mapping
enum STATEID2IND {	STATEID2IND_SLOT0 = 0, 
					STATEID2IND_SLOT1,//add more slots if necessary
				 NUMOFINDICES_STATEID2IND
};

//use the slots above for the mutually exclusive algorithms
#define VIMDP_STATEID2IND STATEID2IND_SLOT0
#define ARAMDP_STATEID2IND STATEID2IND_SLOT0
#define ADMDP_STATEID2IND STATEID2IND_SLOT0
#define RSTARMDP_STATEID2IND STATEID2IND_SLOT0
#define RSTARMDP_LSEARCH_STATEID2IND STATEID2IND_SLOT1

//for example
//#define YYYPLANNER_STATEID2IND STATEID2IND_SLOT0
//#define YYYPLANNER_STATEID2IND STATEID2IND_SLOT1


typedef enum 
	{	//different state types if you have more than one type inside a single planner
		ABSTRACT_STATE = 0,
		ABSTRACT_STATEACTIONPAIR,
		ABSTRACT_GENERALSTATE
} AbstractSearchStateType_t; 

/** \brief base class for a search state
  */
class AbstractSearchState
{

public:
	/** \brief each state can be a member of at most two lists
	indices of lists are given in planner.h (e.g., STATEID2IND_SLOT0)
  */
	struct listelement* listelem[2];
	/** \brief index of the state in the heap, typically used for membership in OPEN
    */
	int heapindex;
	/** \brief type of state. usually it will be general state
    */
	AbstractSearchStateType_t StateType; 

public:
	AbstractSearchState(){StateType = ABSTRACT_GENERALSTATE;listelem[0] = listelem[1] = NULL; };
	~AbstractSearchState(){};
};

class DiscreteSpaceInformation;

/**
   Utility for unified notification of cost changes
   across all SBPLPlanner subtypes. Ideally we would have a simple
   unified interface, such as std::vector<nav2dcell_t>, but the
   current separation of planner and environment representation code
   would be violated if we just included that here.
   
   At the moment, ADPlanner is the only one who really uses the
   detailed information provided by ChangedCellsGetter, so we define
   that class in sbpl/src/planners/ADStar/adplanner.h (to be moved up
   the hierarchy when we generalize).
*/
class StateChangeQuery;

/** \brief pure virtual base class for a generic planner
  */
class SBPLPlanner
{

public:

	/** \brief returns 1 if solution is found, 0 otherwise
    will replan incrementally if possible (e.g., as supported by the planner and not forced to replan from scratch)
	takes in the time available for planner and returns a sequence of stateIDs that corresponds to the solution
  */
  virtual int replan(double allocated_time_sec, std::vector<int>* solution_stateIDs_V) = 0;
  /** \brief works same as replan function with two parameters, but also returns the cost of the solution
    */
  virtual int replan(double allocated_time_sec, std::vector<int>* solution_stateIDs_V, int* solcost) = 0;

    /** \brief sets the goal of search (planner will automatically decide whether it needs to replan from scratch)
      */
    virtual int set_goal(int goal_stateID) = 0;

    /** \brief sets the start of search (planner will automatically decide whether it needs to replan from scratch)
      */
    virtual int set_start(int start_stateID) = 0;

    /** \brief forgets previous planning efforts and starts planning from scratch next time replan is called
      */
    virtual int force_planning_from_scratch() = 0; 

	/** \brief sets the mode for searching

      if bSearchUntilFirstSolution is false, then planner searches for at most allocatime_time_sec, independently of whether it finds a solution or not (default mode)
      if bSearchUntilFirstSolution is true, then planner searches until it finds the first solution. It may be faster than allocated_time or it may be longer
      In other words, in the latter case, the planner does not spend time on improving the solution even if time permits, but may also take longer than allocated_time before returning
      So, normally bSearchUntilFirstSolution should be set to false.
  */
	virtual int set_search_mode(bool bSearchUntilFirstSolution) = 0;

    /** \brief  Notifies the planner that costs have changed. May need to be specialized for different subclasses in terms of what to
     do here
     */
	virtual void costs_changed(StateChangeQuery const & stateChange) = 0;
  
  /** \return The "epsilon" value of the solution last computed by
      replan(), if such an epsilon is used by the planner. The base
      class implementation returns -1 to express that it has no such
      thing. */
   virtual double get_solution_eps() const {SBPL_ERROR("get_solution_eps is unimplemented for this planner\n"); return -1; }

  /** \return The number of states expanded during the last replan()
      operation, or -1 if this information is not available. */
  virtual int get_n_expands() const {SBPL_ERROR("get_n_expands is unimplemented for this planner\n");  return -1; }

	/** \brief returns the initial epsilon
    */
  virtual double get_initial_eps(){SBPL_ERROR("get_initial_eps is unimplemented for this planner\n"); return -1;};

	/** \brief returns the time taken to find the first solution
    */
  virtual double get_initial_eps_planning_time(){SBPL_ERROR("get_initial_eps_planning_time is unimplemented for this planner\n"); return -1;}

	/** \brief returns the time taken to get the final solution
    */
  virtual double get_final_eps_planning_time(){SBPL_ERROR("get_final_eps_planning_time is unimplemented for this planner\n"); return -1;};

	/** \brief returns the number of expands to find the first solution
    */
  virtual int get_n_expands_init_solution(){SBPL_ERROR("get_n_expands_init_solution is unimplemented for this planner\n"); return -1;};

	/** \brief returns the final epsilon achieved during the search
    */
  virtual double get_final_epsilon(){SBPL_ERROR("get_final_epsilon is unimplemented for this planner\n"); return -1;};


	/** \brief setting initial solution eps 
       This parameter is ignored in planners that don't have a notion of eps
       In ARA* / AD*: (cost(initialsolution) <= eps*cost(optimalsolution))
  */
   virtual void set_initialsolution_eps(double initialsolution_eps) {};


    virtual ~SBPLPlanner(){};

protected:
	DiscreteSpaceInformation *environment_;

};



#endif

