#include <sbpl/planners/mhaplanner.h>

#include <assert.h>
#include <stdlib.h>
#include <time.h>
#include <algorithm>

#include <sbpl/utils/key.h>

MHAPlanner::MHAPlanner(
    DiscreteSpaceInformation* environment,
    Heuristic* hanchor,
    Heuristic** heurs,
    int hcount)
:
    SBPLPlanner(),
//    environment_(environment),
    m_hanchor(hanchor),
    m_heurs(heurs),
    m_hcount(hcount),
    m_params(0.0),
    m_eps_mha(1.0),
    m_eps(1.0),
    m_call_number(0), // uninitialized
    m_start_state(NULL),
    m_goal_state(NULL),
    m_search_states(),
    m_open(NULL)
{
    environment_ = environment;

    m_open = new CHeap[hcount + 1];

    // Overwrite default members for ReplanParams to represent a single optimal
    // search
    m_params.initial_eps = 1.0;
    m_params.final_eps = 1.0;
    m_params.dec_eps = 0.0;
    m_params.return_first_solution = false;
    m_params.max_time = 0.0;
    m_params.repair_time = 0.0;

    /// Four Modes:
    ///     Search Until Solution Bounded
    ///     Search Until Solution Unbounded
    ///     Improve Solution Bounded
    ///     Improve Solution Unbounded
}

MHAPlanner::~MHAPlanner()
{
    clear();

    delete[] m_open;
}

int MHAPlanner::set_start(int start_stateID)
{
    m_start_state = get_state(start_stateID);
    if (!m_start_state) {
        return 0;
    }
    else {
        return 1;
    }
}

int MHAPlanner::set_goal(int goal_stateID)
{
    m_goal_state = get_state(goal_stateID);
    if (!m_goal_state) {
        return 0;
    }
    else {
        return 1;
    }
}

int MHAPlanner::replan(
    double allocated_time_sec,
    std::vector<int>* solution_stateIDs_V)
{
    int solcost;
    return replan(allocated_time_sec, solution_stateIDs_V, &solcost);
}

int MHAPlanner::replan(
    double allocated_time_sec,
    std::vector<int>* solution_stateIDs_V,
    int* solcost)
{
    ReplanParams params = m_params;
    params.max_time = allocated_time_sec;
    return replan(solution_stateIDs_V, params, solcost);
}

int MHAPlanner::replan(
    std::vector<int>* solution_stateIDs_V,
    ReplanParams params)
{
    int solcost;
    return replan(solution_stateIDs_V, params, &solcost);
}

int MHAPlanner::replan(
    std::vector<int>* solution_stateIDs_V,
    ReplanParams params,
    int* solcost)
{
    // TODO: pick up from where last search left off and detect lazy
    // reinitializations
    reinit_search();

    m_params = params;
    m_eps = m_params.initial_eps;

    ++m_call_number;
    reinit_state(m_goal_state);
    reinit_state(m_start_state);
    m_start_state->g = 0;

    // insert start state into all heaps with key(start, i) as priority
    for (int hidx = 0; hidx < num_total_heuristics(); ++hidx) {
        CKey key;
        key.key[0] = compute_key(m_start_state, hidx);
        m_open[hidx].insertheap(&m_start_state->od[hidx].open_state, key);
    }

    // TODO: time and expansion limits
    while (!m_open[0].emptyheap()) { 
        for (int i = 0; i < m_hcount; ++i) {
            // TODO: watch out for heaps exhausting prematurely
            const int hidx = 1 + i;
            if (get_minf(m_open[hidx]) <= m_eps_mha * get_minf(m_open[0])) {
                if (m_goal_state->g <= get_minf(m_open[hidx])) {
                    extract_path(solution_stateIDs_V);
                    return 1;
                }
                else {
                    MHASearchState* s = state_from_open_state(m_open[hidx].getminheap());
                    expand(s, hidx);
                }
            }
            else {
                if (m_goal_state->g <= get_minf(m_open[0])) {
                    // terminate and return path pointed to by bp(g)
                    extract_path(solution_stateIDs_V);
                    return 1;
                }
                else {
                    MHASearchState* s = state_from_open_state(m_open[hidx].getminheap());
                    expand(s, 0);
                }
            }
        }
    }

    return 0;
}

int MHAPlanner::force_planning_from_scratch()
{
    return 0;
}

int MHAPlanner::force_planning_from_scratch_and_free_memory()
{
    return 0;
}

void MHAPlanner::costs_changed(StateChangeQuery const & stateChange)
{
}

void MHAPlanner::costs_changed()
{
}

int MHAPlanner::set_search_mode(bool bSearchUntilFirstSolution)
{
    return 0;
}

void MHAPlanner::set_initialsolution_eps(double eps)
{
    m_params.initial_eps = eps;
}

double MHAPlanner::get_solution_eps() const
{
    return 0.0;
}

int MHAPlanner::get_n_expands() const
{
    return 0;
}

double MHAPlanner::get_initial_eps()
{
    return 0.0;
}

double MHAPlanner::get_initial_eps_planning_time()
{
    return 0.0;
}

double MHAPlanner::get_final_eps_planning_time()
{
    return 0.0;
}

int MHAPlanner::get_n_expands_init_solution()
{
    return 0;
}

double MHAPlanner::get_final_epsilon()
{
    return 0.0;
}

void MHAPlanner::get_search_stats(std::vector<PlannerStats>* s)
{
}

void MHAPlanner::set_mha_eps(double eps)
{
    m_eps_mha = eps;
}

void MHAPlanner::set_final_eps(double eps)
{
    m_params.final_eps = eps;
}

void MHAPlanner::set_dec_eps(double eps)
{
    m_params.dec_eps = eps;
}

void MHAPlanner::set_max_expansions(int expansion_count)
{
    m_num_expansions = expansion_count;
}

void MHAPlanner::set_max_time(double max_time)
{
    m_params.max_time = max_time;
}

double MHAPlanner::get_mha_eps() const
{
    return m_eps_mha;
}

double MHAPlanner::get_final_eps() const
{
    return m_params.final_eps;
}

double MHAPlanner::get_dec_eps() const
{
    return m_params.dec_eps;
}

int MHAPlanner::get_max_expansions() const
{
    return m_max_expansions;
}

double MHAPlanner::get_max_time() const
{
    return m_params.max_time;
}

MHASearchState* MHAPlanner::get_state(int state_id)
{
    assert(state_id >= 0 && state_id < environment_->StateID2IndexMapping.size());
    int* idxs = environment_->StateID2IndexMapping[state_id];
    if (idxs[MHAMDP_STATEID2IND] == -1) {
//        SBPL_DEBUG("Creating new search state for graph state %d", state_id);

        // overallocate search state for appropriate heuristic information
        const size_t state_size =
                sizeof(MHASearchState) +
                sizeof(MHASearchState::HeapData) * (m_hcount);
        MHASearchState* s = (MHASearchState*)malloc(state_size);

        const size_t mha_state_idx = m_search_states.size();
        init_state(s, mha_state_idx, state_id);

        // map graph state to search state
        idxs[MHAMDP_STATEID2IND] = mha_state_idx;
        m_search_states.push_back(s);

        return s;
    }
    else {
        int ssidx = idxs[MHAMDP_STATEID2IND];
        return m_search_states[ssidx];
    }
}

void MHAPlanner::clear()
{
    clear_open_lists();

    // free states
    for (size_t i = 0; i < m_search_states.size(); ++i) {
        // unmap graph to search state
        MHASearchState* search_state = m_search_states[i];
        const int state_id = m_search_states[i]->state_id;
        int* idxs = environment_->StateID2IndexMapping[state_id];
        idxs[MHAMDP_STATEID2IND] = -1;

        // free search state
        free(m_search_states[i]);
    }

    // empty state table
    m_search_states.clear();

    m_start_state = NULL;
    m_goal_state = NULL;
}

void MHAPlanner::init_state(
    MHASearchState* state,
    size_t mha_state_idx,
    int state_id)
{
    state->call_number = 0; // not initialized for any iteration
    state->state_id = state_id;
    for (int i = 0; i < m_hcount + 1; ++i) {
        state->od[i].open_state.heapindex = 0;
        state->od[i].h = compute_heuristic(state->state_id, i);
        state->od[i].closed = false;
        // hijack list element pointers to map back to mha search state
        assert(sizeof(state->od[i].open_state.listelem) >= sizeof(struct listelement*));
        reinterpret_cast<size_t&>(state->od[i].open_state.listelem[0]) = mha_state_idx;
    }
}

void MHAPlanner::reinit_state(MHASearchState* state)
{
    if (state->call_number != m_call_number) {
        state->call_number = m_call_number;
        state->g = INFINITECOST;
        state->bp = NULL;

        for (int i = 0; i < m_hcount + 1; ++i) {
            state->od[i].open_state.heapindex = 0;
            state->od[i].h = compute_heuristic(state->state_id, i);
            state->od[i].closed = false;
        }
    }
}

void MHAPlanner::reinit_search()
{
    clear_open_lists();
}

void MHAPlanner::clear_open_lists()
{
    for (int i = 0; i < num_total_heuristics(); ++i) {
        m_open[i].makeemptyheap();
    }
}

int MHAPlanner::compute_key(MHASearchState* state, int hidx)
{
    return state->g + m_eps * state->od[hidx].h;
}

void MHAPlanner::expand(MHASearchState* state, int hidx)
{
    SBPL_DEBUG("Expanding state %d in search %d", state->state_id, hidx);
    // remove s from all open lists
    for (int hidx = 0; hidx < m_hcount; ++hidx) {
        if (m_open[hidx].inheap(&state->od[hidx].open_state)) {
            m_open[hidx].deleteheap(&state->od[hidx].open_state);
        }
    }

    std::vector<int> succ_ids;
    std::vector<int> costs;
    environment_->GetSuccs(state->state_id, &succ_ids, &costs);
    assert(succ_ids.size() == costs.size());

    // for s' in succ(s) do
    for (size_t sidx = 0; sidx < succ_ids.size(); ++sidx)  {
        const int cost = costs[sidx];
        MHASearchState* succ_state = get_state(succ_ids[sidx]);
        // reinit_state(s')
        reinit_state(succ_state);

        int new_g = state->g + costs[sidx];
        if (new_g < succ_state->g) {
            succ_state->g = new_g;
            succ_state->bp = state;
            if (!succ_state->od[0].closed) {
                const int fanchor = compute_key(succ_state, 0);

                insert_or_update(succ_state, 0, fanchor);

                // if not any closed(s', i) for i = 1, n then
                bool closed = false;
                for (int hidx = 1; hidx < m_hcount + 1; ++hidx) {
                    closed |= succ_state->od[hidx].closed;
                }

                if (!closed) {
                    for (int hidx = 1; hidx < m_hcount + 1; ++hidx) {
                        int fn = compute_key(succ_state, hidx);
                        if (fn <= m_eps_mha * fanchor) {
                            insert_or_update(succ_state, hidx, fn);
                        }
                    }
                }
            }
        }
    }
}

MHASearchState* MHAPlanner::state_from_open_state(
    AbstractSearchState* open_state)
{
    const size_t ssidx = reinterpret_cast<size_t>(open_state->listelem[0]);
    return m_search_states[ssidx];
}

int MHAPlanner::compute_heuristic(int state_id, int hidx)
{
    if (hidx == 0) {
        return m_hanchor->GetGoalHeuristic(state_id);
    }
    else {
        return m_heurs[hidx - 1]->GetGoalHeuristic(state_id);
    }
}

int MHAPlanner::get_minf(CHeap& pq) const
{
    return pq.getminkeyheap().key[0];
}

void MHAPlanner::insert_or_update(MHASearchState* state, int hidx, int f)
{
    CKey new_key;
    new_key.key[0] = f;

    if (state->od[hidx].open_state.heapindex != 0) {
        m_open[hidx].updateheap(&state->od[hidx].open_state, new_key);
    }
    else {
        m_open[hidx].insertheap(&state->od[hidx].open_state, new_key);
    }
}

void MHAPlanner::extract_path(std::vector<int>* solution_path)
{
    SBPL_DEBUG("Extracting path");
    solution_path->clear();
    for (MHASearchState* state = m_goal_state; state; state = state->bp)
    {
        solution_path->push_back(state->state_id);
    }

    // TODO: special cases for backward search
    std::reverse(solution_path->begin(), solution_path->end());
}
