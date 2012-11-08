#ifndef __PRECOMPUTED_ADJACENCY_LIST_H_
#define __PRECOMPUTED_ADJACENCY_LIST_H_

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <list>
#include <map>
#include <utility>
#include <vector>
#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/sbpl_exception.h>
#include <sbpl/planners/araplanner.h>
#include <sbpl/planners/planner.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/mdpconfig.h>

struct Adjacency
{
    int neighbor;
    int cost;
};
typedef std::list<Adjacency> Adjacencies;
typedef Adjacencies::iterator AdjListIterator;

/**
 * \brief SBPL Environment represented as an adjacency list graph.
 *
 * \tparam Coords Coords must be a type that 1) has operator<< and
 *                heuristicDistanceTo (const Coords&) const defined on it 2) can be used as a
 *                key of an STL map
 *
 * Nodes of the graph are labelled with Coords.  Edges have integer costs
 * attached to them.  ARA* planning is done on this graph, and the function
 * heuristicDistanceTo is used as the admissible heuristic.
 */
template<class Coords>
class AdjacencyListSBPLEnv : public DiscreteSpaceInformation
{
public:
    AdjacencyListSBPLEnv();
    void writeToStream(std::ostream& str = std::cout);

    /**
     * \brief Add point to roadmap.  Does not check for duplicates.
     */
    void addPoint(const Coords& c);

    /**
     * \brief Does the roadmap contain this point?
     */
    bool hasPoint(const Coords& c);

    /**
     * \brief Remove the last N points added using addPoint (and all their incident edges) in O(N) time
     */
    void removeLastPoints(unsigned int n = 1);

    /**
     * \post An undirected edge exists between c1 and c2 with the given cost
     * cost, if not provided, defaults to the heuristic cost between c1 and c2
     */
    void setCost(const Coords& c1, const Coords& c2, int cost);
    void setCost(const Coords& c1, const Coords& c2);

    void setStartState(const Coords& c);
    void setGoalState(const Coords& c);

    /**
     * \brief Use ARA* to find an optimal path between the currently set start and goal states
     * \return Vector of states on the path
     * \post solution_cost will hold the cost of the returned solution
     */
    std::vector<Coords> findOptimalPath(int* solution_cost);

    // Inherited DiscreteSpaceInformation ops
    bool InitializeEnv(const char* sEnvFile);
    bool InitializeMDPCfg(MDPConfig *MDPCfg);
    int GetFromToHeuristic(int FromStateID, int ToStateID);
    int GetGoalHeuristic(int stateID);
    int GetStartHeuristic(int stateID);
    void SetAllActionsandAllOutcomes(CMDPSTATE* state);
    void SetAllPreds(CMDPSTATE* state);
    void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV);
    void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV);
    int SizeofCreatedEnv();
    void PrintState(int stateID, bool bVerbose, FILE* fOut = NULL);
    void PrintEnv_Config(FILE* fOut);

private:
    void resetStateId2IndexMapping(void);

    // Members
    std::vector<Coords> points_;
    std::map<Coords, int> pointIds_;
    std::vector<Adjacencies> adjacency_vector_;
    int startStateId_;
    int goalStateId_;
};

template<class Coords>
AdjacencyListSBPLEnv<Coords>::AdjacencyListSBPLEnv() :
    startStateId_(-1), goalStateId_(-1)
{
}

template<class Coords>
void AdjacencyListSBPLEnv<Coords>::writeToStream(std::ostream& str)
{
    int numStates = points_.size();
    str << "Adjacency list SBPL Env " << endl;
    for (unsigned int i = 0; i < points_.size(); i++) {
        str << i << ". " << points_[i] << ".  Neighbors: ";
        for (AdjListIterator iter = adjacency_vector_[i].begin(); iter != adjacency_vector_[i].end(); iter++) {
            if (iter->neighbor < numStates) {
                str << "[" << points_[iter->neighbor] << " " << iter->cost << "] ";
            }
            else {
                str << "[To-Be-Deleted Edge] ";
            }
        }
        str << endl;
    }
}

template<class Coords>
void AdjacencyListSBPLEnv<Coords>::addPoint(const Coords& c)
{
    pointIds_[c] = points_.size();
    Adjacencies a;
    points_.push_back(c);
    adjacency_vector_.push_back(a);

    int* entry = new int[NUMOFINDICES_STATEID2IND];
    for (unsigned int i = 0; i < NUMOFINDICES_STATEID2IND; i++) {
        entry[i] = -1;
    }
    StateID2IndexMapping.push_back(entry);
}

template<class Coords>
bool AdjacencyListSBPLEnv<Coords>::hasPoint(const Coords& c)
{
    return !(pointIds_.find(c) == pointIds_.end());
}

template<class Coords>
void AdjacencyListSBPLEnv<Coords>::removeLastPoints(unsigned int n)
{
    assert(n <= points_.size());
    for (unsigned int i = 0; i < n; i++) {
        int num_points = points_.size();
        Adjacencies& a = adjacency_vector_.back();

        // Iterate over neighbors of this currently-being-deleted point
        for (Adjacencies::iterator adj_iter = a.begin(); adj_iter != a.end(); adj_iter++) {
            assert(adj_iter->neighbor < num_points);
            Adjacencies& neighbor_adjacency_list = adjacency_vector_[adj_iter->neighbor];

            // Iterate over neighbors of the neighbor
            for (Adjacencies::iterator neighbor_adj_iter = neighbor_adjacency_list.begin(); neighbor_adj_iter
                != neighbor_adjacency_list.end(); neighbor_adj_iter++) {

                // When we find the entry for the current point, remove it and exit inner loop
                if (neighbor_adj_iter->neighbor == num_points - 1) {
                    neighbor_adjacency_list.erase(neighbor_adj_iter);
                    break;
                }
            }
        }
        adjacency_vector_.pop_back();
        pointIds_.erase(points_.back());
        points_.pop_back();
    }
}

template<class Coords>
void AdjacencyListSBPLEnv<Coords>::setCost(const Coords& c1, const Coords& c2)
{
    setCost(c1, c2, c1.heuristicDistanceTo(c2));
}

template<class Coords>
void AdjacencyListSBPLEnv<Coords>::setCost(const Coords& c1, const Coords& c2, int cost)
{
    // Figure out indices of the given points
    typename std::map<Coords, int>::iterator i1 = pointIds_.find(c1);
    typename std::map<Coords, int>::iterator i2 = pointIds_.find(c2);
    int index1 = i1->second;
    int index2 = i2->second;

    // Loop over which direction edge to add
    for (unsigned int j = 0; j < 2; j++) {
        int ind1, ind2;
        if (j == 0) {
            ind1 = index1;
            ind2 = index2;
        }
        else {
            ind1 = index2;
            ind2 = index1;
        }

        // Now set cost of edge from ind1 to ind2
        Adjacencies& adj = adjacency_vector_[ind1];

        AdjListIterator i, last = adj.end();
        for (i = adj.begin(); (i != last) && (i->neighbor != ind2); i++)
            ;

        // If edge does not exist add it, else just set the cost of the existing edge
        if (i == last) {
            Adjacency a;
            a.neighbor = ind2;
            a.cost = cost;
            adj.insert(i, a);
        }
        else {
            i->cost = cost;
        }
    }
}

template<class Coords>
void AdjacencyListSBPLEnv<Coords>::setStartState(const Coords& c)
{
    typename std::map<Coords, int>::iterator i = pointIds_.find(c);
    assert(i != pointIds_.end());
    startStateId_ = i->second;
}

template<class Coords>
void AdjacencyListSBPLEnv<Coords>::setGoalState(const Coords& c)
{
    typename std::map<Coords, int>::iterator i = pointIds_.find(c);
    assert(i != pointIds_.end());
    goalStateId_ = i->second;
}

template<class Coords>
bool AdjacencyListSBPLEnv<Coords>::InitializeMDPCfg(MDPConfig *MDPCfg)
{
    MDPCfg->goalstateid = goalStateId_;
    MDPCfg->startstateid = startStateId_;
    return true;
}

template<class Coords>
int AdjacencyListSBPLEnv<Coords>::GetFromToHeuristic(int FromStateID, int ToStateID)
{
    return points_[FromStateID].heuristicDistanceTo(points_[ToStateID]);
}

template<class Coords>
int AdjacencyListSBPLEnv<Coords>::GetGoalHeuristic(int stateID)
{
    return GetFromToHeuristic(stateID, goalStateId_);
}

template<class Coords>
int AdjacencyListSBPLEnv<Coords>::GetStartHeuristic(int stateID)
{
    return GetFromToHeuristic(startStateId_, stateID);
}

template<class Coords>
void AdjacencyListSBPLEnv<Coords>::PrintState(int stateID, bool bVerbose, FILE* fOut)
{
    // Note we're ignoring the fOut argument
    std::cout << points_[stateID] << endl;
}

template<class Coords>
void AdjacencyListSBPLEnv<Coords>::PrintEnv_Config(FILE* fOut)
{
    // Note we're ignoring the fOut argument
    std::cout << "Adjacency list env" << endl;
}

template<class Coords>
int AdjacencyListSBPLEnv<Coords>::SizeofCreatedEnv()
{
    return points_.size();
}

template<class Coords>
bool AdjacencyListSBPLEnv<Coords>::InitializeEnv(const char* sEnvFile)
{
    // Nothing to do in initialization
    return true;
}

template<class Coords>
void AdjacencyListSBPLEnv<Coords>::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
    // goal state is absorbing
    if (state->StateID == goalStateId_) {
        return;
    }

    Adjacencies& v = adjacency_vector_[state->StateID];
    int actionIndex = 0;

    for (AdjListIterator i = v.begin(); i != v.end(); i++) {
        CMDPACTION* action = state->AddAction(actionIndex);
        action->AddOutcome(i->neighbor, i->cost, 1.0);
    }
}

template<class Coords>
void AdjacencyListSBPLEnv<Coords>::SetAllPreds(CMDPSTATE* state)
{
    // Apparently this is not always necessary
    std::cout << "Error: SetAllPreds not implemented for adjacency list";
    throw new SBPL_Exception();
}

template<class Coords>
void AdjacencyListSBPLEnv<Coords>::GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV)
{
    SuccIDV->clear();
    CostV->clear();

    if (SourceStateID == goalStateId_) {
        return;
    }

    Adjacencies& v = adjacency_vector_[SourceStateID];
    for (AdjListIterator i = v.begin(); i != v.end(); i++) {
        SuccIDV->push_back(i->neighbor);
        CostV->push_back(i->cost);
    }
}

template<class Coords>
void AdjacencyListSBPLEnv<Coords>::GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV)
{
    std::cout << "Error: GetPreds not currently implemented for adjacency list";
    throw new SBPL_Exception();
}

template<class Coords>
std::vector<Coords> AdjacencyListSBPLEnv<Coords>::findOptimalPath(int* solution_cost)
{
    // Initialize ARA planner
    ARAPlanner p(this, true);
    p.set_start(startStateId_);
    p.set_goal(goalStateId_);
    p.set_initialsolution_eps(1.0);
    std::vector<int> solution;
    p.replan(1.0, &solution, solution_cost);

    std::vector<Coords> solutionPoints;
    for (unsigned int i = 0; i < solution.size(); i++) {
        solutionPoints.push_back(points_[solution[i]]);
    }

    resetStateId2IndexMapping();

    return solutionPoints;
}

// There's some side effect where you have to reset this every time you call the ARA planner
template<class Coords>
void AdjacencyListSBPLEnv<Coords>::resetStateId2IndexMapping(void)
{
    for (unsigned int i = 0; i < StateID2IndexMapping.size(); i++) {
        for (unsigned int j = 0; j < NUMOFINDICES_STATEID2IND; j++) {
            StateID2IndexMapping[i][j] = -1;
        }
    }
}

#endif

// Local variables:
// mode:c++
// End:
