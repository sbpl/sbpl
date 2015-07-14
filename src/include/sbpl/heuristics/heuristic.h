#ifndef sbpl_Heuristic_h
#define sbpl_Heuristic_h

#include <sbpl/discrete_space_information/environment.h>

class Heuristic
{
public:

    Heuristic(DiscreteSpaceInformation* environment) :
        m_environment(environment)
    { }

    virtual int GetGoalHeuristic(int state_id) = 0;
    virtual int GetStartHeuristic(int state_id) = 0;
    virtual int GetFromToHeuristic(int from_id, int to_id) = 0;

protected:

    DiscreteSpaceInformation* m_environment;
};

#endif
