#ifndef sbpl_MHAPlanner_h
#define sbpl_MHAPlanner_h

#include <sbpl/heuristics/heuristic.h>
#include <sbpl/planners/planner.h>
#include <sbpl/utils/heap.h>

struct MHASearchState
{
    int call_number;
    int state_id;
    int g;
    MHASearchState* bp;

    struct HeapData
    {
        AbstractSearchState open_state;
        int h;
        bool closed; // TODO: ok to condense this to one bool for "closed in any additional search"
    };

    HeapData od[1]; // overallocated for additional n heuristics
};

class MHAPlanner : public SBPLPlanner
{
public:

    MHAPlanner(
            DiscreteSpaceInformation* environment,
            Heuristic* hanchor,
            Heuristic** heurs,
            int hcount);

    virtual ~MHAPlanner();

    virtual int set_start(int start_stateID);
    virtual int set_goal(int goal_stateID);

    /// \sa SBPLPlanner::replan(double, std::vector<int>*)
    virtual int replan(
            double allocated_time_sec,
            std::vector<int>* solution_stateIDs_V);

    /// \sa SBPLPlanner::replan(double, std::vector<int>*, int*)
    virtual int replan(
            double allocated_time_sec,
            std::vector<int>* solution_stateIDs_V,
            int* solcost);

    /// \sa SBPLPlanner::replan(std::vector<int*>, ReplanParams)
    virtual int replan(
            std::vector<int>* solution_stateIDs_V,
            ReplanParams params);

    /// \sa SBPLPlanner::replan(std::vector<int>*, ReplanParams, int*)
    virtual int replan(
            std::vector<int>* solution_stateIDs_V,
            ReplanParams params,
            int* solcost);

    /// \sa SBPLPlanner::force_planning_from_scratch()
    /// \return 1 on success; 0 otherwise
    virtual int force_planning_from_scratch();

    /// \sa SBPLPlanner::force_planning_from_scratch_and_free_memory()
    /// \return 1 on success; 0 otherwise
    virtual int force_planning_from_scratch_and_free_memory();

    virtual void costs_changed(StateChangeQuery const & stateChange);

    /// \sa ARAPlanner::costs_changed()
    virtual void costs_changed();

    /// \name Search Parameter Accessors
    ///@{

    /// \sa SBPLPlanner::set_search_mode(bool)
    virtual int     set_search_mode(bool bSearchUntilFirstSolution);
    /// \sa SBPLPlanner::set_initialsolution_eps(double)
    virtual void    set_initialsolution_eps(double eps);

    /// \sa SBPLPlanner::get_initial_eps()
    virtual double  get_initial_eps();

    ///@}

    /// \name Search Statistics
    ///@{

    /// \sa SBPLPlanner::get_solution_eps() const
    virtual double  get_solution_eps() const;
    /// \sa SBPLPlanner::get_final_epsilon()
    virtual double  get_final_epsilon();

    /// \sa SBPLPlanner::get_final_eps_planning_time
    virtual double  get_final_eps_planning_time();
    /// \sa SBPLPlanner::get_initial_eps_planning_time
    virtual double  get_initial_eps_planning_time();

    /// \sa SBPLPlanner::get_n_expands() const
    virtual int     get_n_expands() const;
    ///\sa SBPLPlanner::get_n_expands_init_solution()
    virtual int     get_n_expands_init_solution();
    /// \sa SBPLPlanner::get_search_states(std::vector<PlannerStates>*)
    virtual void    get_search_stats(std::vector<PlannerStats>* s);

    ///@}

    /// \name Homogeneous accessor methods for search mode and timing parameters
    // @{

    void    set_initial_eps(double eps) { return set_initialsolution_eps(eps); }
    void    set_initial_mha_eps(double eps_mha);
    void    set_final_eps(double eps);
    void    set_dec_eps(double eps);
    void    set_max_expansions(int expansion_count);
    void    set_max_time(double max_time);

    // double get_initial_eps();
    double  get_initial_mha_eps() const;
    double  get_final_eps() const;
    double  get_dec_eps() const;
    int     get_max_expansions() const;
    double  get_max_time() const;

    ///@}

private:

    // Related objects
    Heuristic* m_hanchor;
    Heuristic** m_heurs;
    int m_hcount;           ///< number of additional heuristics used

    ReplanParams m_params;
    double m_initial_eps_mha;
    int m_max_expansions;

    double m_eps;           ///< current w_1
    double m_eps_mha;       ///< current w_2

    /// suboptimality bound satisfied by the last search
    double m_eps_satisfied; 

    int m_num_expansions;   ///< current number of expansion
    double m_elapsed;       ///< current amount of seconds

    int m_call_number;

    MHASearchState* m_start_state;
    MHASearchState* m_goal_state;

    std::vector<MHASearchState*> m_search_states;

    CHeap* m_open; ///< sequence of (m_hcount + 1) open lists

    bool check_params(const ReplanParams& params);

    bool time_limit_reached() const;

    int num_heuristics() const { return m_hcount + 1; }
    MHASearchState* get_state(int state_id);
    void init_state(MHASearchState* state, size_t mha_state_idx, int state_id);
    void reinit_state(MHASearchState* state);
    void reinit_search();
    void clear_open_lists();
    void clear();
    int compute_key(MHASearchState* state, int hidx);
    void expand(MHASearchState* state, int hidx);
    MHASearchState* state_from_open_state(AbstractSearchState* open_state);
    int compute_heuristic(int state_id, int hidx);
    int get_minf(CHeap& pq) const;
    void insert_or_update(MHASearchState* state, int hidx, int f);

    void extract_path(std::vector<int>* solution_path);

    bool closed_in_anc_search(MHASearchState* state) const;
    bool closed_in_add_search(MHASearchState* state) const;
    bool closed_in_any_search(MHASearchState* state) const;
};

#endif
