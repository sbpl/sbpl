#ifndef sbpl_EmbeddedHeuristic_h
#define sbpl_EmbeddedHeuristic_h

#include <sbpl/heuristics/heuristic.h>

class EmbeddedHeuristic : public Heuristic
{
public:

    EmbeddedHeuristic(DiscreteSpaceInformation* environment);

    int GetGoalHeuristic(int state_id);
    int GetStartHeuristic(int state_id);
    int GetFromToHeuristic(int from_id, int to_id);
};

#endif
