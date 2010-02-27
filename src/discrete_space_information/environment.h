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
#ifndef __ENVIRONMENT_H_
#define __ENVIRONMENT_H_


class DiscreteSpaceInformation
{

public:

  //data
  std::vector<int*> StateID2IndexMapping;
	

	FILE* fDeb;

	virtual bool InitializeEnv(const char* sEnvFile) = 0;


	virtual bool InitializeMDPCfg(MDPConfig *MDPCfg) = 0;
	virtual int  GetFromToHeuristic(int FromStateID, int ToStateID) = 0;
	virtual int  GetGoalHeuristic(int stateID) = 0;
	virtual int  GetStartHeuristic(int stateID) = 0;
	virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state) = 0;
	virtual void SetAllPreds(CMDPSTATE* state) = 0;
	virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV) = 0;
	virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV) = 0;

	virtual int	 SizeofCreatedEnv() = 0;
	virtual void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL) = 0;
	virtual void PrintEnv_Config(FILE* fOut) = 0;

	virtual bool SetEnvParameter(const char* parameter, int value)
	{
		printf("ERROR: Environment has no parameters that can be set via SetEnvParameter function\n");
		return false;
	}

	/** NOTE - for debugging door planner - ben 8.31.09 */
	virtual std::vector<int> GetExpandedStates()
	{
		printf("Error: Not yet defined for any environment other than door environment.\n");
		std::vector<int> list;
		return list;
	}

	//returns true if two states meet the same condition,
	//this is used in some planners to figure out if two states are the same in some lower-dimensional manifold
	//for example, in robotarm planning, two states could be equivalent if their end effectors are at the same position
	//unless overwritten in a child class, this function is not implemented
	virtual bool AreEquivalent(int StateID1, int StateID2){
		printf("ERROR: environment does not support calls to AreEquivalent function\n");
		exit(1);
	}

	//the following two functions generate succs/preds at some domain-dependent distance. The number of generated succs/preds is up
	//to the environment. NOTE: they MUST generate goal state as a succ/pred if it is within the distance from the state
	//CLowV is the corresponding vector of lower bounds on the costs from the state to the successor states (or vice versa for preds function)
	//unless overwritten in a child class, this function is not implemented
	virtual void GetRandomSuccsatDistance(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CLowV)
	{
		printf("ERROR: environment does not support calls to GetRandomSuccsatDistance function\n");
		exit(1);
	}
	virtual void GetRandomPredsatDistance(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CLowV)
	{
		printf("ERROR: environment does not support calls to GetRandomPredsatDistance function\n");
		exit(1);
	};

    
	virtual void EnsureHeuristicsUpdated(bool bGoalHeuristics)
	{
		//by default the heuristics are up-to-date, but in some cases, the heuristics are computed only when really needed. For example,
		//xytheta environment uses 2D gridsearch as heuristics, and then re-computes them only when this function is called. This 
		//minimizes the number of times heuristics are re-computed which is an expensive operation
		//if bGoalHeuristics == true, then it updates goal heuristics (for forward search), otherwise it updates start heuristics (for backward search)

	};

	//destructor
	virtual ~DiscreteSpaceInformation(){
	printf("destroying discretespaceinformation\n");
    for(unsigned int i = 0; i < StateID2IndexMapping.size(); ++i){
      if(StateID2IndexMapping[i] != NULL)
        delete[] StateID2IndexMapping[i];
    }
  }


	//constructor
	DiscreteSpaceInformation()
	{

	  if((fDeb = fopen("envdebug.txt", "w")) == NULL)
	    {
	      printf("ERROR: failed to open debug file for environment\n");
	      exit(1);
	    }

	}
};



#endif

