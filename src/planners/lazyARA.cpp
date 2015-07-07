/*
 * Copyright (c) 2013, Mike Phillips and Maxim Likhachev
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
 *     * Neither the name of the Carnegie Mellon University nor the names of its
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
#include <sbpl_cpr/planners/lazyARA.h>
using namespace std;

LazyARAPlanner::LazyARAPlanner(DiscreteSpaceInformation* environment, bool bSearchForward) :
  params(0.0) {
  bforwardsearch = bSearchForward;
  environment_ = environment;
  replan_number = 0;

  goal_state_id = -1;
  start_state_id = -1;
}

LazyARAState* LazyARAPlanner::GetState(int id){	
  //if this stateID is out of bounds of our state vector then grow the list
  if(id >= int(states.size())){
    for(int i=states.size(); i<=id; i++)
      states.push_back(NULL);
  }
  //if we have never seen this state then create one
  if(states[id]==NULL){
    states[id] = new LazyARAState();
    states[id]->id = id;
    states[id]->replan_number = -1;
  }
  //initialize the state if it hasn't been for this call to replan
  LazyARAState* s = states[id];
  if(s->replan_number != replan_number){
    s->g = INFINITECOST;
    s->v = INFINITECOST;
    s->iteration_closed = -1;
    s->replan_number = replan_number;
    s->best_parent = NULL;
    s->expanded_best_parent = NULL;
    s->heapindex = 0;
    s->in_incons = false;
    s->isTrueCost = true;
    //clear the lazy list
    while(!s->lazyList.empty())
      s->lazyList.pop();

    //compute heuristics
    if(bforwardsearch)
      s->h = environment_->GetGoalHeuristic(s->id);
    else
      s->h = environment_->GetStartHeuristic(s->id);
  }
  return s;
}

void LazyARAPlanner::ExpandState(LazyARAState* parent){
  bool print = false; //parent->id == goal_state_id; //parent->id == 285566;
  if(print)
    printf("expand %d\n",parent->id);
  vector<int> children;
  vector<int> costs;
  vector<bool> isTrueCost;

  if(bforwardsearch)
    environment_->GetLazySuccs(parent->id, &children, &costs, &isTrueCost);
  else
    environment_->GetLazyPreds(parent->id, &children, &costs, &isTrueCost);

  //printf("expand %d\n",parent->id);
  //iterate through children of the parent
  for(int i=0; i<(int)children.size(); i++){
    //printf("  succ %d\n",children[i]);
    LazyARAState* child = GetState(children[i]);
    insertLazyList(child, parent, costs[i], isTrueCost[i]);
  } 
}

//assumptions:
//state is at the front of the open list
//it's minimum f-value is an underestimate (the edge cost from the parent is a guess and needs to be evaluated properly)
//it hasn't been expanded yet this iteration
void LazyARAPlanner::EvaluateState(LazyARAState* state){
  bool print = false; //state->id == goal_state_id; //state->id == 285566;
  if(print)
    printf("evaluate %d (from %d)\n",state->id, state->best_parent->id);
  LazyARAState* parent = state->best_parent;

  getNextLazyElement(state);
  //printf("state_ptr=%p\n",state);
  //printf("state_id=%d\n",state->id);
  //printf("parent_ptr=%p\n",parent);
  //printf("parent_id=%d\n",parent->id);
  int trueCost = environment_->GetTrueCost(parent->id, state->id);
  //printf("has a true cost of %d\n",trueCost);
  if(trueCost > 0){ //if the evaluated true cost is valid (positive), insert it into the lazy list
    if(print)
      printf("  edge is valid with cost %d added to parent->v=%d\n",trueCost,parent->v);
    insertLazyList(state,parent,trueCost,true);
  }
  else{
    if(print)
      printf("  edge not valid\n");
  }
}

//this should only be used with EvaluateState since it is assuming state hasn't been expanded yet (only evaluated)
void LazyARAPlanner::getNextLazyElement(LazyARAState* state){
  if(state->lazyList.empty()){
    state->g = INFINITECOST;
    state->best_parent = NULL;
    state->isTrueCost = true;
    return;
  }
  LazyListElement elem = state->lazyList.top();
  state->lazyList.pop();
  state->g = elem.parent->v + elem.edgeCost;
  state->best_parent = elem.parent;
  state->isTrueCost = elem.isTrueCost;
  //the new value is cheapest and if the value is also true then we want to throw out all the other options
  if(state->isTrueCost){
    while(!state->lazyList.empty())
      state->lazyList.pop();
  }
  putStateInHeap(state);
}

void LazyARAPlanner::insertLazyList(LazyARAState* state, LazyARAState* parent, int edgeCost, bool isTrueCost){
  bool print = false; //state->id == goal_state_id; //state->id == 285566 || parent->id == 285566;
  if(print)
    printf("insertLazyList state->id=%d parent->id=%d state->g=%d parent->v=%d edgeCost=%d isTrueCost=%d\n",state->id,parent->id,state->g,parent->v,edgeCost,isTrueCost);
  if(state->v <= parent->v + edgeCost)
    return;
  else if(state->g <= parent->v + edgeCost){
    //if the best g-value we have is true and better, then the value we had dominates this one and we don't need it
    if(state->isTrueCost)
      return;
    //insert this guy into the lazy list
    LazyListElement elem(parent,edgeCost,isTrueCost);
    state->lazyList.push(elem);
  }
  else{//the new guy is the cheapest so far
    //should we save what was the previous best?
    if(!isTrueCost && //the better guy's cost is not for sure
       //state->g < INFINITECOST && //we actually have a previous best (we actually don't need this line because of the next one)
       state->g < state->v){ //we're not saving something we already expanded (and is stored in v and expanded_best_parent)
      //we save it by putting it in the lazy list
      LazyListElement elem(state->best_parent, state->g - state->best_parent->v, state->isTrueCost);
      state->lazyList.push(elem);
      //printf("save the previous best\n");
    }

    //the new guy is the cheapest
    state->g = parent->v + edgeCost;
    state->best_parent = parent;
    state->isTrueCost = isTrueCost;

    //the new value is cheapest and if the value is also true then we want to throw out all the other options
    if(isTrueCost){
      //printf("clear the lazy list\n");
      while(!state->lazyList.empty())
        state->lazyList.pop();
    }

    //this function puts the state into the heap (or updates the position) if we haven't expanded
    //if we have expanded, it will put the state in the incons list (if we haven't already)
    putStateInHeap(state);
  }
}

void LazyARAPlanner::putStateInHeap(LazyARAState* state){
  bool print = false; //state->id == goal_state_id; //state->id == 285566;
  //we only allow one expansion per search iteration
  //so insert into heap if not closed yet
  if(state->iteration_closed != search_iteration){
    if(print)
      printf("put state %d in open with state->g=%d and state->isTrueCost=%d\n",state->id,state->g,state->isTrueCost);
    CKey key;
    key.key[0] = state->g + int(eps * state->h);
    //if the state is already in the heap, just update its priority
    if(state->heapindex != 0)
      heap.updateheap(state,key);
    else //otherwise add it to the heap
      heap.insertheap(state,key);
  }
  //if the state has already been expanded once for this iteration
  //then add it to the incons list so we can keep track of states
  //that we know we have better costs for
  else if(!state->in_incons){
    if(print)
      printf("put state %d in incons with state->g and state->isTrueCost=%d\n",state->id,state->g,state->isTrueCost);
    incons.push_back(state);
    state->in_incons = true;
  }
}

//returns 1 if the solution is found, 0 if the solution does not exist and 2 if it ran out of time
int LazyARAPlanner::ImprovePath(){

  //expand states until done
  int expands = 0;
  CKey min_key = heap.getminkeyheap();
  while(!heap.emptyheap() && 
        min_key.key[0] < INFINITECOST && 
        (goal_state->g > min_key.key[0] || !goal_state->isTrueCost) &&
        (goal_state->v > min_key.key[0]) &&
        !outOfTime()){

    //get the state		
    LazyARAState* state = (LazyARAState*)heap.deleteminheap();

    if(state->v == state->g){
      printf("ERROR: consistent state is being expanded\n");
      printf("id=%d v=%d g=%d isTrueCost=%d lazyListSize=%lu\n",
              state->id,state->v,state->g,state->isTrueCost,state->lazyList.size());
      throw new SBPL_Exception();
    }

    if(state->isTrueCost){
      //mark the state as expanded
      state->v = state->g;
      state->expanded_best_parent = state->best_parent;
      state->iteration_closed = search_iteration;
      //expand the state
      expands++;
      ExpandState(state);
      if(expands%100000 == 0)
        printf("expands so far=%u\n", expands);
    }
    else //otherwise the state needs to be evaluated for its true cost
      EvaluateState(state);

    //get the min key for the next iteration
    min_key = heap.getminkeyheap();
    //printf("min_key =%d\n",min_key.key[0]);
  }

  search_expands += expands;

  if(goal_state->v < goal_state->g){
    goal_state->g = goal_state->v;
    goal_state->best_parent = goal_state->expanded_best_parent;
  }

  /*
  if(goal_state->g == INFINITECOST)
    printf("goal g is inf\n");
  if(heap.emptyheap())
    printf("heap empty\n");
  if(min_key.key[0] >= INFINITECOST)
    printf("min key inf\n");
  */
   
  if(goal_state->g == INFINITECOST && (heap.emptyheap() || min_key.key[0] >= INFINITECOST))
    return 0;//solution does not exists
  if(!heap.emptyheap() && goal_state->g > min_key.key[0])
    return 2; //search exited because it ran out of time
  printf("search exited with a solution for eps=%.3f\n", eps);
  if(goal_state->g < goal_state->v){
    goal_state->expanded_best_parent = goal_state->best_parent;
    goal_state->v = goal_state->g;
  }
  return 1;
}

vector<int> LazyARAPlanner::GetSearchPath(int& solcost){
  vector<int> SuccIDV;
  vector<int> CostV;
  vector<bool> isTrueCost;
  vector<int> wholePathIds;

  LazyARAState* state;
  LazyARAState* final_state;
  if(bforwardsearch){
    state = goal_state;
    final_state = start_state;
  }
  else{
    state = start_state;
    final_state = goal_state;
  } 

  wholePathIds.push_back(state->id);
  solcost = 0;

  while(state->id != final_state->id){
    if(state->expanded_best_parent == NULL){
      printf("a state along the path has no parent!\n");
      break;
    }
    if(state->v == INFINITECOST){
      printf("a state along the path has an infinite g-value!\n");
      printf("inf state = %d\n",state->id);
      break;
    }

    if(bforwardsearch)
      environment_->GetLazySuccs(state->expanded_best_parent->id, &SuccIDV, &CostV, &isTrueCost);
    else
      environment_->GetLazyPreds(state->expanded_best_parent->id, &SuccIDV, &CostV, &isTrueCost);
    int actioncost = INFINITECOST;
    //printf("reconstruct expand %d\n",state->expanded_best_parent->id);
    for(unsigned int i=0; i<SuccIDV.size(); i++){
      //printf("  succ %d\n",SuccIDV[i]);
      if(SuccIDV[i] == state->id && CostV[i]<actioncost)
        actioncost = CostV[i];
    }
    if(actioncost == INFINITECOST)
      printf("WARNING: actioncost = %d\n", actioncost);
    solcost += actioncost;

    state = state->expanded_best_parent;
    wholePathIds.push_back(state->id);
  }

  //if we searched forward then the path reconstruction 
  //worked backward from the goal, so we have to reverse the path
  if(bforwardsearch){
    //in place reverse
    for(unsigned int i=0; i<wholePathIds.size()/2; i++){
      int other_idx = wholePathIds.size()-i-1;
      int temp = wholePathIds[i];
      wholePathIds[i] = wholePathIds[other_idx];
      wholePathIds[other_idx] = temp;
    }
  }

  return wholePathIds;
}

bool LazyARAPlanner::outOfTime(){
  //if we are supposed to run until the first solution, then we are never out of time
  if(params.return_first_solution)
    return false;
  double time_used = double(clock() - TimeStarted)/CLOCKS_PER_SEC;
  if(time_used >= params.max_time)
    printf("out of max time\n");
  if(use_repair_time && eps_satisfied != INFINITECOST && time_used >= params.repair_time)
    printf("used all repair time...\n");
  //we are out of time if:
         //we used up the max time limit OR
         //we found some solution and used up the minimum time limit
  return time_used >= params.max_time || 
         (use_repair_time && eps_satisfied != INFINITECOST && time_used >= params.repair_time);
}

void LazyARAPlanner::initializeSearch(){
  //it's a new search, so increment replan_number and reset the search_iteration
  replan_number++;
  search_iteration = 0;
  search_expands = 0;
  totalPlanTime = 0;
  totalExpands = 0;

  //clear open list, incons list, and stats list
  heap.makeemptyheap();
  incons.clear();
  stats.clear();

  //initialize epsilon variable
  eps = params.initial_eps;
  eps_satisfied = INFINITECOST;

  //call get state to initialize the start and goal states
  goal_state = GetState(goal_state_id);
  start_state = GetState(start_state_id);

  //put start state in the heap
  start_state->g = 0;
  CKey key;
  key.key[0] = eps*start_state->h;
  heap.insertheap(start_state, key);

  //ensure heuristics are up-to-date
  environment_->EnsureHeuristicsUpdated((bforwardsearch==true));
}

bool LazyARAPlanner::Search(vector<int>& pathIds, int& PathCost){
  CKey key;
  TimeStarted = clock();

  initializeSearch();

  //the main loop of ARA*
  while(eps_satisfied > params.final_eps && !outOfTime()){

    //run weighted A*
    clock_t before_time = clock();
    int before_expands = search_expands;
    //ImprovePath returns:
    //1 if the solution is found
    //0 if the solution does not exist
    //2 if it ran out of time
    int ret = ImprovePath();
    if(ret == 1) //solution found for this iteration
      eps_satisfied = eps;
    int delta_expands = search_expands - before_expands;
    double delta_time = double(clock()-before_time)/CLOCKS_PER_SEC;

    //print the bound, expands, and time for that iteration
    printf("bound=%f expands=%d cost=%d time=%.3f\n", 
        eps_satisfied, delta_expands, goal_state->g, delta_time);

    //update stats
    totalPlanTime += delta_time;
    totalExpands += delta_expands;
    PlannerStats tempStat;
    tempStat.eps = eps_satisfied;
    tempStat.expands = delta_expands;
    tempStat.time = delta_time;
    tempStat.cost = goal_state->g;
    stats.push_back(tempStat);

    //no solution exists
    if(ret == 0){
      printf("Solution does not exist\n");
      return false;
    }

    //if we're just supposed to find the first solution
    //or if we ran out of time, we're done
    if(params.return_first_solution || ret == 2)
      break;

    prepareNextSearchIteration();
  }

  if(goal_state->g == INFINITECOST){
    printf("could not find a solution (ran out of time)\n");
    return false;
  }
  if(eps_satisfied == INFINITECOST)
    printf("WARNING: a solution was found but we don't have quality bound for it!\n");

  printf("solution found\n");
  clock_t before_reconstruct = clock();
  pathIds = GetSearchPath(PathCost);
  reconstructTime = double(clock()-before_reconstruct)/CLOCKS_PER_SEC;
  totalTime = totalPlanTime + reconstructTime;

  return true;
}

void LazyARAPlanner::prepareNextSearchIteration(){
  //decrease epsilon
  eps -= params.dec_eps;
  if(eps < params.final_eps)
    eps = params.final_eps;

  //dump the inconsistent states into the open list
  CKey key;
  while(!incons.empty()){
    LazyARAState* s = incons.back();
    incons.pop_back();
    s->in_incons = false;
    key.key[0] = s->g + int(eps * s->h);
    heap.insertheap(s,key);
  }

  //recompute priorities for states in OPEN and reorder it
  for (int i=1; i<=heap.currentsize; ++i){
    LazyARAState* state = (LazyARAState*)heap.heap[i].heapstate;
    heap.heap[i].key.key[0] = state->g + int(eps * state->h); 
  }
  heap.makeheap();

  search_iteration++;
}


//-----------------------------Interface function-----------------------------------------------------
int LazyARAPlanner::replan(double allocated_time_secs, std::vector<int>* solution_stateIDs_V){
  int solcost;
  return replan(allocated_time_secs, solution_stateIDs_V, &solcost);
}

int LazyARAPlanner::replan(double allocated_time_sec, std::vector<int>* solution_stateIDs_V, int* solcost){
  params.max_time = allocated_time_sec;
  return replan(solution_stateIDs_V, params, solcost);
}

int LazyARAPlanner::replan(vector<int>* solution_stateIDs_V, ReplanParams p){
  int solcost;
  return replan(solution_stateIDs_V, p, &solcost);
}

int LazyARAPlanner::replan(int start, int goal, vector<int>* solution_stateIDs_V, ReplanParams p, int* solcost){
  set_start(start);
  set_goal(goal);
  return replan(solution_stateIDs_V, p, solcost);
}

int LazyARAPlanner::replan(vector<int>* solution_stateIDs_V, ReplanParams p, int* solcost){
  printf("planner: replan called\n");
  params = p;
  use_repair_time = params.repair_time >= 0;

  if(goal_state_id < 0){
    printf("ERROR searching: no goal state set\n");
    return 0;
  }
  if(start_state_id < 0){
    printf("ERROR searching: no start state set\n");
    return 0;
  }

  //plan
  vector<int> pathIds; 
  int PathCost;
  bool solnFound = Search(pathIds, PathCost);
  printf("total expands=%d planning time=%.3f reconstruct path time=%.3f total time=%.3f solution cost=%d\n", 
      totalExpands, totalPlanTime, reconstructTime, totalTime, goal_state->g);

  //copy the solution
  *solution_stateIDs_V = pathIds;
  *solcost = PathCost;

  start_state_id = -1;
  goal_state_id = -1;

  return (int)solnFound;
}

int LazyARAPlanner::set_goal(int id){
  printf("planner: setting goal to %d\n", id);
  if(bforwardsearch)
    goal_state_id = id;
  else
    start_state_id = id;
  return 1;
}

int LazyARAPlanner::set_start(int id){
  printf("planner: setting start to %d\n", id);
  if(bforwardsearch)
    start_state_id = id;
  else
    goal_state_id = id;
  return 1;
}


//---------------------------------------------------------------------------------------------------------


void LazyARAPlanner::get_search_stats(vector<PlannerStats>* s){
  s->clear();
  s->reserve(stats.size());
  for(unsigned int i=0; i<stats.size(); i++){
    s->push_back(stats[i]);
  }
}

