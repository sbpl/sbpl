/*
 * Copyright (c) 2015, Maxim Likhachev
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

#include <sbpl/planners/mhaplanner.h>

#include <assert.h>
#include <stdlib.h>
#include <time.h>
#include <algorithm>

#include <sbpl/utils/key.h>

static double GetTime()
{
    return (double)clock() / (double)CLOCKS_PER_SEC;
}

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
    m_initial_eps_mha(1.0),
    m_max_expansions(0),
    m_eps(1.0),
    m_eps_mha(1.0),
    m_eps_satisfied((double)INFINITECOST),
    m_num_expansions(0),
    m_elapsed(0.0),
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
    if (!check_params(params)) { // errors printed within
        return 0;
    }

    m_params = params;

    SBPL_INFO("Generic Search parameters:");
    SBPL_INFO("  Initial Epsilon: %0.3f", m_params.initial_eps);
    SBPL_INFO("  Final Epsilon: %0.3f", m_params.final_eps);
    SBPL_INFO("  Delta Epsilon: %0.3f", m_params.dec_eps);
    SBPL_INFO("  Return First Solution: %s", m_params.return_first_solution ? "true" : "false");
    SBPL_INFO("  Max Time: %0.3f", m_params.max_time);
    SBPL_INFO("  Repair Time: %0.3f", m_params.repair_time);
    SBPL_INFO("MHA Search parameters:");
    SBPL_INFO("  MHA Epsilon: %0.3f", m_initial_eps_mha);
    SBPL_INFO("  Max Expansions: %d", m_max_expansions);

    environment_->EnsureHeuristicsUpdated(true); // TODO: support backwards search

    // TODO: pick up from where last search left off and detect lazy
    // reinitializations
    reinit_search();

    m_eps = m_params.initial_eps;
    m_eps_mha = m_initial_eps_mha;
    m_eps_satisfied = (double)INFINITECOST;

    // reset time limits
    m_num_expansions = 0;
    m_elapsed = 0.0;

    double start_time, end_time;

    start_time = GetTime();

    ++m_call_number;
    reinit_state(m_goal_state);
    reinit_state(m_start_state);
    m_start_state->g = 0;

    // insert start state into all heaps with key(start, i) as priority
    for (int hidx = 0; hidx < num_heuristics(); ++hidx) {
        CKey key;
        key.key[0] = compute_key(m_start_state, hidx);
        m_open[hidx].insertheap(&m_start_state->od[hidx].open_state, key);
        SBPL_DEBUG("Inserted start state %d into search %d with f = %d", m_start_state->state_id, hidx, key.key[0]);
    }

    end_time = GetTime();
    m_elapsed += (end_time - start_time);

    while (!m_open[0].emptyheap() && !time_limit_reached()) { 
        start_time = GetTime();

        // special case for mha* without additional heuristics
        if (num_heuristics() == 1) { 
            if (m_goal_state->g <= get_minf(m_open[0])) {
                m_eps_satisfied = m_eps * m_eps_mha;
                extract_path(solution_stateIDs_V, solcost);
                return 1;
            }
            else {
                MHASearchState* s = state_from_open_state(m_open[0].getminheap());
                expand(s, 0);
            }
        }

        for (int hidx = 1; hidx < num_heuristics(); ++hidx) {
            if (m_open[0].emptyheap()) {
                break;
            }

            if (!m_open[hidx].emptyheap() && get_minf(m_open[hidx]) <=
                m_eps_mha * get_minf(m_open[0]))
            {
                if (m_goal_state->g <= get_minf(m_open[hidx])) {
                    m_eps_satisfied = m_eps * m_eps_mha;
                    extract_path(solution_stateIDs_V, solcost);
                    return 1;
                }
                else {
                    MHASearchState* s =
                            state_from_open_state(m_open[hidx].getminheap());
                    expand(s, hidx);
                }
            }
            else {
                if (m_goal_state->g <= get_minf(m_open[0])) {
                    m_eps_satisfied = m_eps * m_eps_mha;
                    extract_path(solution_stateIDs_V, solcost);
                    return 1;
                }
                else {
                    MHASearchState* s =
                            state_from_open_state(m_open[0].getminheap());
                    expand(s, 0);
                }
            }
        }
        end_time = GetTime();
        m_elapsed += (end_time - start_time);
    }

    if (m_open[0].emptyheap()) {
        SBPL_DEBUG("Anchor search exhausted");
    }
    if (time_limit_reached()) {
        SBPL_DEBUG("Time limit reached");
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
    return m_params.return_first_solution = bSearchUntilFirstSolution;
}

void MHAPlanner::set_initialsolution_eps(double eps)
{
    m_params.initial_eps = eps;
}

double MHAPlanner::get_initial_eps()
{
    return m_params.initial_eps;
}

double MHAPlanner::get_solution_eps() const
{
    return m_eps_satisfied;
}

double MHAPlanner::get_final_epsilon()
{
    return m_eps_satisfied;
}

double MHAPlanner::get_final_eps_planning_time()
{
    return m_elapsed;
}

double MHAPlanner::get_initial_eps_planning_time()
{
    return m_elapsed;
}

int MHAPlanner::get_n_expands() const
{
    return m_num_expansions;
}

int MHAPlanner::get_n_expands_init_solution()
{
    return m_num_expansions;
}

void MHAPlanner::get_search_stats(std::vector<PlannerStats>* s)
{
}

void MHAPlanner::set_initial_mha_eps(double eps)
{
    m_initial_eps_mha = eps;
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
    m_max_expansions = expansion_count;
}

void MHAPlanner::set_max_time(double max_time)
{
    m_params.max_time = max_time;
}

double MHAPlanner::get_initial_mha_eps() const
{
    return m_initial_eps_mha;
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

bool MHAPlanner::check_params(const ReplanParams& params)
{
    if (params.initial_eps < 1.0) {
        SBPL_ERROR("Initial Epsilon must be greater than or equal to 1");
        return false;
    }

    if (params.final_eps > params.initial_eps) {
        SBPL_ERROR("Final Epsilon must be less than or equal to initial epsilon");
        return false;
    }

    if (params.dec_eps <= 0.0) {
        SBPL_ERROR("Delta epsilon must be strictly positive");
        return false;
    }

    if (m_initial_eps_mha < 1.0) {
        SBPL_ERROR("MHA Epsilon must be greater than or equal to 1");
        return false;
    }

    if (params.return_first_solution &&
        params.max_time <= 0.0 &&
        m_max_expansions <= 0)
    {
        SBPL_ERROR("Max Time or Max Expansions must be positive");
        return false;
    }

    return true;
}

bool MHAPlanner::time_limit_reached() const
{
    if (m_params.return_first_solution) {
        return false;
    }
    else if (m_params.max_time > 0.0 && m_elapsed >= m_params.max_time) {
        return true;
    }
    else if (m_max_expansions > 0 && m_num_expansions >= m_max_expansions) {
        return true;
    }
    else {
        return false;
    }
}

MHASearchState* MHAPlanner::get_state(int state_id)
{
    assert(state_id >= 0 && state_id < environment_->StateID2IndexMapping.size());
    int* idxs = environment_->StateID2IndexMapping[state_id];
    if (idxs[MHAMDP_STATEID2IND] == -1) {
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
    state->closed_in_anc = false;
    state->closed_in_add = false;
    for (int i = 0; i < num_heuristics(); ++i) {
        state->od[i].open_state.heapindex = 0;
        state->od[i].h = compute_heuristic(state->state_id, i);
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

        state->closed_in_anc = false;
        state->closed_in_add = false;

        for (int i = 0; i < num_heuristics(); ++i) {
            state->od[i].open_state.heapindex = 0;
            state->od[i].h = compute_heuristic(state->state_id, i);
        }
    }
}

void MHAPlanner::reinit_search()
{
    clear_open_lists();
}

void MHAPlanner::clear_open_lists()
{
    for (int i = 0; i < num_heuristics(); ++i) {
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

    assert(!closed_in_add_search(state) || !closed_in_anc_search(state));

    if (hidx == 0) {
        state->closed_in_anc = true;
    }
    else {
        state->closed_in_add = true;
    }
    ++m_num_expansions;

    // remove s from all open lists
    for (int hidx = 0; hidx < num_heuristics(); ++hidx) {
        if (m_open[hidx].inheap(&state->od[hidx].open_state)) {
            m_open[hidx].deleteheap(&state->od[hidx].open_state);
        }
    }

    std::vector<int> succ_ids;
    std::vector<int> costs;
    environment_->GetSuccs(state->state_id, &succ_ids, &costs);
    assert(succ_ids.size() == costs.size());

    for (size_t sidx = 0; sidx < succ_ids.size(); ++sidx)  {
        const int cost = costs[sidx];
        MHASearchState* succ_state = get_state(succ_ids[sidx]);
        reinit_state(succ_state);

        SBPL_DEBUG(" Successor %d", succ_state->state_id);

        int new_g = state->g + costs[sidx];
        if (new_g < succ_state->g) {
            succ_state->g = new_g;
            succ_state->bp = state;
            if (!closed_in_anc_search(succ_state)) {
                const int fanchor = compute_key(succ_state, 0);
                insert_or_update(succ_state, 0, fanchor);
                SBPL_DEBUG("  Update in search %d with f = %d", 0, fanchor);

                if (!closed_in_add_search(succ_state)) {
                    for (int hidx = 1; hidx < num_heuristics(); ++hidx) {
                        int fn = compute_key(succ_state, hidx);
                        if (fn <= m_eps_mha * fanchor) {
                            insert_or_update(succ_state, hidx, fn);
                            SBPL_DEBUG("  Update in search %d with f = %d", hidx, fn);
                        }
                        else {
                            SBPL_DEBUG("  Skipping update of in search %d (%0.3f > %0.3f)", hidx, (double)fn, m_eps_mha * fanchor);
                        }
                    }
                }
            }
        }
    }

    assert(closed_in_any_search(state));
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

void MHAPlanner::extract_path(std::vector<int>* solution_path, int* solcost)
{
    SBPL_DEBUG("Extracting path");
    solution_path->clear();
    *solcost = 0;
    for (MHASearchState* state = m_goal_state; state; state = state->bp)
    {
        solution_path->push_back(state->state_id);
        if (state->bp) {
            *solcost += (state->g - state->bp->g);
        }
    }

    // TODO: special cases for backward search
    std::reverse(solution_path->begin(), solution_path->end());
}

bool MHAPlanner::closed_in_anc_search(MHASearchState* state) const
{
    return state->closed_in_anc;
}

bool MHAPlanner::closed_in_add_search(MHASearchState* state) const
{
    return state->closed_in_add;
}

bool MHAPlanner::closed_in_any_search(MHASearchState* state) const
{
    return state->closed_in_anc || state->closed_in_add;
}
