#include <sbpl/heuristics/embedded_heuristic.h>

EmbeddedHeuristic::EmbeddedHeuristic(DiscreteSpaceInformation* environment) :
    Heuristic(environment)
{
}

int EmbeddedHeuristic::GetGoalHeuristic(int state_id)
{
    return m_environment->GetGoalHeuristic(state_id);
}

int EmbeddedHeuristic::GetStartHeuristic(int state_id)
{
    return m_environment->GetStartHeuristic(state_id);
}

int EmbeddedHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    return m_environment->GetFromToHeuristic(from_id, to_id);
}
