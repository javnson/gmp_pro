/**
 * @file workflow.hpp
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

// TODO: implement sub-workflow

// #include <assert.h>
#include <functional>
#include <stdint.h>

// TODO unit transfer for delay time.

#ifndef _FILE_WORKFLOW_HPP_
#define _FILE_WORKFLOW_HPP_

#ifndef SPECIFY_WORKFLOW_TIMER
#define wf_get_tick gmp_port_system_tick
#else
#define wf_get_tick SPECIFY_WORKFLOW_TIMER
#endif // SPECIFY_WORKFLOW_TIMER

// This macro enable the reachable verification process in workflow node switch process.
// This macro is suggested enable in debug mode not in release mode.
// #define SPECIFY_WORKFLOW_CHECK_REACHABLE

// This macro enable the workflow library use <functional> header
// and use the std::function to save the function pointer.
// By std::function you may use catch `[]` expression, and meanwhile
// you will face more code size and more data size.
// #define SPECIFY_WORKFLOW_USE_FUNCTIONAL

// This macro specify that each workflow will contain an independent `get_tick()` function.
// #define SPECIFY_WORKFLOW_MULTITIMER_TICK

// This macro enable the workflow has a start and end callback.
// And this two function may invoke in time, when the workflow is start and workflow reach end.
#define SPECIFY_WORKFLOW_START_END_CALLBACK

// This macro enable the node of workflow has a `leave()` function.
// Generally, we suggest user to use `transfer()` completely substitute the `leave` function.
#define SPECIFY_WORKFLOW_NODE_OWN_LEAVE

// This macro is to enable the scheduling extension.
// Generally, we recommend you to provide each task a workflow.
// And then let workflow scheduling to manage them.
// #define SPECIFY_WORKFLOW_ENABLE_SCHEDULING

// If user don't specify the workflow object id, the id will be an invalid value.
// The invalid value is here.
constexpr size_gt gmp_wf_id_invalid = 0xFFFF;

// Antecedent
class gmp_workflow_t;
class gmp_wf_node_base_gt;

#ifdef SPECIFY_WORKFLOW_USE_FUNCTIONAL
typedef std::function<gmp_wf_node_base_gt *(gmp_workflow_t *)> routine_fn_t;
#if defined SPECIFY_WORKFLOW_NODE_OWN_LEAVE
typedef std::function<void(gmp_workflow_t *)> leave_fn_t;
#endif // SPECIFY_WORKFLOW_NODE_OWN_LEAVE
#if defined SPECIFY_WORKFLOW_START_END_CALLBACK
typedef std::function<void(gmp_workflow_t *)> head_tail_routine_t;
#endif // SPECIFY_WORKFLOW_START_END_CALLBACK
#ifdef SPECIFY_WORKFLOW_CHECK_REACHABLE
typedef std::function<uint8_t(gmp_wf_node_base_gt *, gmp_wf_node_base_gt *)> verify_fn_t;
#endif // SPECIFY_WORKFLOW_CHECK_REACHABLE
#ifdef SPECIFY_WORKFLOW_MULTITIMER_TICK
typedef std::function<time_gt(void)> timer_tick_fn_t;
#endif // SPECIFY_WORKFLOW_MULTITIMER_TICK
#else  // SPECIFY_WORKFLOW_USE_FUNCTIONAL
typedef gmp_wf_node_base_gt *(*routine_fn_t)(gmp_workflow_t *);
#if defined SPECIFY_WORKFLOW_NODE_OWN_LEAVE
typedef void (*leave_fn_t)(gmp_workflow_t *);
#endif // SPECIFY_WORKFLOW_NODE_OWN_LEAVE
#if defined SPECIFY_WORKFLOW_START_END_CALLBACK
typedef void (*head_tail_routine_t)(gmp_workflow_t *);
#endif // SPECIFY_WORKFLOW_START_END_CALLBACK
#if defined SPECIFY_WORKFLOW_CHECK_REACHABLE
typedef uint8_t (*verify_fn_t)(gmp_wf_node_base_gt *, gmp_wf_node_base_gt *);
#endif // SPECIFY_WORKFLOW_CHECK_REACHABLE
#if defined SPECIFY_WORKFLOW_MULTITIMER_TICK
typedef time_gt (*timer_tick_fn_t)(void);
#endif // SPECIFY_WORKFLOW_MULTITIMER_TICK
#endif // SPECIFY_WORKFLOW_USE_FUNCTIONAL

// This is a null node as end placeholder
extern gmp_wf_node_base_gt *wf_end_node;

//////////////////////////////////////////////////////////////////////////
// workflow type implement
class gmp_workflow_t
{
  public:
    // ctor & dtor
    gmp_workflow_t()
        : entry_node(nullptr), curr_node(nullptr), last_node(nullptr), next_node_usr(nullptr), flag_usr_modify(0),
          flag_enable(0)
    {
    }

  protected:
    // private members

    // The entry point of the workflow
    gmp_wf_node_base_gt *entry_node;

    // The current node of the workflow
    gmp_wf_node_base_gt *curr_node;

    // The last point of the workflow
    gmp_wf_node_base_gt *last_node;

    // The next point of the workflow
    // This value should be changed by user, unless the fault state is happening.
    // That is, priority: prior > user > section
    gmp_wf_node_base_gt *next_node_usr;

    // Store the only prior node
    // In general, the prior node is the fault dealing node.
    // If you don't need to boost any node to prior, you may set the pointer to nullptr.
    gmp_wf_node_base_gt *prior_node;

    // When user intervene the normal workflow, the flag may set to 1,
    // and `next_node_usr` will take count.
    // If the class is not in Fault situation the user decision may prevail.
    fast_gt flag_usr_modify;

    // When this flag is enable, the gmp_workflow will start running.
    // (Default) When a whole process is complete, and workflow meets the end,
    // the enable flag will reset to 0.
    // You may modify the end node to change the action of end.
    // If user set the enable flag to 0 when the workflow still in the middle step,
    // the work flow would be stuck. This is dangerous.
    // User may choose resumption from the stuck by function `play()`,
    // or choose to forget the step and start anew by function `reset()`.
    fast_gt flag_enable;

    // This variables store the switch time of the current node.
    // The counter will treat this value as original.
    // Unit Tick from system.
    time_gt switch_time;

    // This variables store the switch time of the last node.
    // User may use the variable to calculate the time consumption.
    time_gt last_switch_time;

#if defined SPECIFY_WORKFLOW_START_END_CALLBACK
    // This function would be called before the `flag_enable` is set
    // This function would only be called by `start()`.
    head_tail_routine_t head_callback;

    // This function would be called before the `flag_enable` is clear
    // This function would only be called by `process()`.
    head_tail_routine_t tail_callback;
#endif // SPECIFY_WORKFLOW_START_END_CALLBACK

#ifdef SPECIFY_WORKFLOW_CHECK_REACHABLE
    // The function should return an nonzero value if the current->next transfer is correct.
    // Or the function should return zero, to emphasize the non-reachable.
    // The fist parameter is the current node, and the second parameter is the target node.
    verify_fn_t verify_fn;

    // if you need to leave the verify mode a little while, you should set the parameter to 1.
    // Or, when the flag is 0, the verify will never stop.
    uint8_t flag_verify_disabled;
#endif // SPECIFY_WORKFLOW_CHECK_REACHABLE

#ifdef SPECIFY_WORKFLOW_MULTITIMER_TICK
    // This pointer point to a function that is a tick timer. The function would return the current tick timer.
    // The tick may be the system tick or read from the timer counter.
    timer_tick_fn_t get_timer_tick;

    // This variable save the maximum number of tick.
    time_gt max_timer_tick;
#endif // SPECIFY_WORKFLOW_MULTITIMER_TICK

  public:
    // utilities

    inline ec_gt set_entry_node(gmp_wf_node_base_gt *entry)
    {
        // judge if the workflow is still running
        if (!flag_enable)
        {
            this->entry_node = entry;
            return GMP_STAT_OK;
        }
        return GMP_STAT_WF_CANNOT_MODIFY;
    }

    // DANGEROUS this function will register the next node without check.
    // Misuse of this function may cause damage.
    inline void set_next_node(gmp_wf_node_base_gt *next)
    {
        next_node_usr = next;
        flag_usr_modify = 1;
    }

    // Generally the fault-dealing should has prior
    inline ec_gt set_prior_node(gmp_wf_node_base_gt *prior)
    {
        if (!flag_enable)
        {
            this->prior_node = prior;
            return GMP_STAT_OK;
        }
        return GMP_STAT_WF_CANNOT_MODIFY;
    }

    // Because this two function is not inline function
    // So if you are pursuing fast speed, you should leave these two function.
    size_gt get_curr_node_id();

    size_gt get_last_node_id();

    inline void *get_usr_param()
    {
        return usr_param;
    }

    inline void set_usr_param(void *param)
    {
        usr_param = param;
    }

  public:
    // User parameters
    // User may save a pointer parameter here.
    // generally here will be your kernel target of the class.
    void *usr_param;

  public:
    // The main function of workflow.
    // The following callback function should be called by user in main loop.
    // Or if user use the workflow scheduler, the following callback function
    // would be called by the scheduler.
    virtual ec_gt process();

    // The operator provide a easier way to call it.
    ec_gt operator()()
    {
        return this->process();
    }

  public:
    // public user utilities

    // return time consuming of the last node
    inline time_gt get_last_duration()
    {
        if (switch_time >= last_switch_time)
        {
            return switch_time - last_switch_time;
        }
        else // overflow
        {
#if defined SPECIFY_WORKFLOW_MULTITIMER_TICK
            return max_timer_tick - last_switch_time + switch_time;
#else
            return GMP_PORT_TIME_MAXIMUM - last_switch_time + switch_time;
#endif // SPECIFY_WORKFLOW_MULTITIMER_TICK
        }
    }

    // return the duration between the `switch_time` and now
    inline time_gt get_duration()
    {
        time_gt now = get_current_tick();

        if (now > switch_time)
        {
            return now - switch_time;
        }
        else // overflow
        {
#if defined SPECIFY_WORKFLOW_MULTITIMER_TICK
            return max_timer_tick - switch_time + now;
#else
            return GMP_PORT_TIME_MAXIMUM - switch_time + now;
#endif // SPECIFY_WORKFLOW_MULTITIMER_TICK
        }
        // Cannot reach here.
    }

    // This function return the current for the current workflow
    inline time_gt get_current_tick()
    {
#if defined SPECIFY_WORKFLOW_MULTITIMER_TICK
        // keep the correct function call
        assert(get_timer_tick);
        time_gt now = get_timer_tick();
#else
        time_gt now = wf_get_tick();
#endif // SPECIFY_WORKFLOW_MULTITIMER_TICK

        return now;
    }

    // This function will reset all the variables in the class.
    // and make the current node position pointer to the start.
    // DANGEROUS! This function is not secure for a physical system,
    //   if you are still running the workflow.
    //   You should stop the workflow in a correct manner.
    // Generally and firstly you should let workflow reach the stop point or
    // reach the fault stuck with stopping mode.
    ec_gt reset()
    {
        // check the parameters
        assert(entry_node);

        // User should stop the process firstly
        if (!flag_enable)
        {
            // copy the entry point to the current node
            curr_node = entry_node;
            last_node = nullptr;
            flag_usr_modify = 0;
            switch_time = 0;
            last_switch_time = 0;

            return GMP_STAT_OK;
        }

        return GMP_STAT_WF_HAS_START;
    }

    // Start the workflow
    // This function only should be called when all things is ready.
    ec_gt start()
    {
        if (flag_enable)
            return GMP_STAT_WF_HAS_START;

        // Check reset points
        if (curr_node != entry_node)
            return GMP_STAT_WF_NO_RESET;

#if defined SPECIFY_WORKFLOW_START_END_CALLBACK
        // The head function would be called
        if (head_callback)
            head_callback(this);
#endif // SPECIFY_WORKFLOW_START_END_CALLBACK

        switch_time = get_current_tick();
        last_switch_time = switch_time;

        flag_enable = 1;

        return GMP_STAT_OK;
    }

  protected:
    // Pause the workflow without check
    // DANGEROUS, this function has no check branches.
    //   Physical Information System may be damaged by this operation.
    inline void pause()
    {
        flag_enable = 0;
    }

    // resume the pause of workflow without check
    // DANGEROUS, this function has no check branches.
    //   Physical Information System may be damaged by this operation.
    inline void resume()
    {
        flag_enable = 1;
    }

  protected:
    // private utilities

    // This function is used to switch the current node to the specified node.
    ec_gt switch_to(gmp_wf_node_base_gt *node);

  public:
    // Support for multi-clock
#ifdef SPECIFY_WORKFLOW_MULTITIMER_TICK
  public:
    inline void set_timer_tick_source(timer_tick_fn_t tick_fn, time_gt maximum_tick)
    {
        get_timer_tick = tick_fn;
        max_timer_tick = maximum_tick;
    }
#endif // SPECIFY_WORKFLOW_MULTITIMER_TICK

    // This section of code is to support workflow scheduling.
#if defined SPECIFY_WORKFLOW_ENABLE_SCHEDULING
  public:
    // typedef

    // This enum provide a set of priority of workflow
    typedef enum _tag_wf_sch_priority_t
    {
        WF_SCH_HIGHEST_PRIOR = 0,
        WF_SCH_HIGHER_PRIOR = 1,
        WF_SCH_HIGH_PRIOR = 2,
        WF_SCH_MID_PRIOR = 3,
        WF_SCH_LOW_PRIOR = 4,
        WF_SCH_LOWER_PRIOR = 5,
        WF_SCH_LOWEST_PRIOR = 6
    } wf_sch_priority_t;

  protected:
    // This is the priority of the class
    wf_sch_priority_t prior;

    // This function may called by scheduler.
    // User should only set the priority by ctor.
    void set_priority(wf_sch_priority_t priority)
    {
        prior = priority;
    }

    // This will record the counter that the work miss the opportunities.
    size_gt prior_cnt;

#endif // SPECIFY_WORKFLOW_ENABLE_SCHEDULING

    // const & static members
  public:
    // This id is belongs to end.
    static constexpr size_gt gmp_wf_end_id = 0xFFFF;
}; // gmp_workflow_t

//////////////////////////////////////////////////////////////////////////
// GMP workflow section base class

// A node for workflow must have the following items:
// member id: to help user to identify the item or state.
// virtual node routine: this function would be invoke when general routine process.
// virtual node transfer: when transfer to the node the function would be called.
// virtual node leave(optional): when the workflow will leave the node, the function would be called.
class gmp_wf_node_base_gt
{
  public:
    // ctor & dtor
    gmp_wf_node_base_gt() : id(0)
    {
    }

    gmp_wf_node_base_gt(size_gt id) : id(id)
    {
    }

  public:
    // This function would execute the current step of this node.
    // And this function would only be called by workflow object.
    // Every time the current node of workflow is `*this` the virtual function will invoke.
    // In order to support polymorphism, that is complex type of workflow node,
    // the function must be a virtual function.
    virtual gmp_wf_node_base_gt *node_routine(gmp_workflow_t *wf)
    {
        return wf_end_node;
    }

    // This function would be called when the workflow come to this node.
    virtual gmp_wf_node_base_gt *node_transfer(gmp_workflow_t *wf)
    {
        return nullptr;
    }

    // This function would be called when the workflow ready to leave the node.
    // In general, the node_leave function could be overlap with node_transfer function.
    // If you strongly rely on this method, you may open this method by add
    // `SPECIFY_WORKFLOW_NODE_OWN_LEAVE` macro to your self config file.
#if defined SPECIFY_WORKFLOW_NODE_OWN_LEAVE
    virtual void node_leave(gmp_workflow_t *wf)
    {
        return;
    }
#endif

  public:
    // member

    // meaningful ID for the node, in order to recognize the specified node easily.
    // And the ID may not be a unique value, but we recommend you NOT to do so.
    size_gt id;

  public:
    // utilities

    inline size_gt get_id()
    {
        return id;
    }

    inline void set_id(size_gt id)
    {
        this->id = id;
    }
};

// This class is a general class for a node
class gmp_wf_node_gt : public gmp_wf_node_base_gt
{
  public:
    // ctor & dtor

    // default constructor
    gmp_wf_node_gt()
        :
#if defined SPECIFY_WORKFLOW_NODE_OWN_LEAVE
          leave(nullptr),
#endif // SPECIFY_WORKFLOW_START_END_CALLBACK
          routine(nullptr), transfer(nullptr)
    {
    }

    // The following constructor is generally in use.
    gmp_wf_node_gt(size_gt id)
        : gmp_wf_node_base_gt(id),
#if defined SPECIFY_WORKFLOW_NODE_OWN_LEAVE
          leave(nullptr),
#endif // SPECIFY_WORKFLOW_START_END_CALLBACK
          routine(nullptr), transfer(nullptr)
    {
    }

  protected:
    // If you want to implement your own node, what you only need to do is to inherit the `gmp_wf_node_gt` class,
    // and enrich details, and then overwrite the real power function.
    gmp_wf_node_base_gt *node_routine(gmp_workflow_t *wf) override
    {
        if (routine != nullptr)
            return routine(wf);
        else
            // when nothing to do, workflow will automatically transfer to the end.
            return wf_end_node;
    }

    // This function may called when the come to this node.
    gmp_wf_node_base_gt *node_transfer(gmp_workflow_t *wf) override
    {
        if (transfer)
            return transfer(wf);
        else
            // keep position
            return nullptr;
    }

    // This function would be called when the workflow will leave the node
#if defined SPECIFY_WORKFLOW_NODE_OWN_LEAVE
    void node_leave(gmp_workflow_t *wf) override
    {
        if (leave)
            return leave(wf);

        return;
    }
#endif

    // reinforcement friendship
    friend class gmp_workflow_t;

  public:
    // members
    // The following member is necessary for each workflow nodes.

    // The following three functions should be implemented without any delay and hesitation.
    // These function should implement delay process by workflow.

    // a pointer to routine function, user may specify a function as the node's routine.
    // if this function return the nullptr or itself, it means that the workflow still in place.
    // This function shouldn't be keep as a nullptr.
    // If this function pointer point to a nullptr, the routine will let the while workflow end.
    routine_fn_t routine;

    // a pointer to transfer function, the time switch to the node, the routine would be called.
    // if this function return the nullptr, it means that the workflow still in place.
    // WARNING! This function should generally return the nullptr to continue the workflow.
    //   Or you may lat the transfer function point to nullptr.
    //   And the routine of current node will be jumped.
    //   If you return to some special node this may cause recursion deeply, and finally cause stack overflow.
    //   The return value should only point to the fault dealing node.
    routine_fn_t transfer;

#if defined SPECIFY_WORKFLOW_NODE_OWN_LEAVE
    // a pointer to leave function, the time leave the node, the routine would be called.
    // the return value of this function would be ignored.
    // This function may point to nullptr.
    leave_fn_t leave;
#endif // SPECIFY_WORKFLOW_NODE_OWN_LEAVE

  public:
    // utilities

    inline void set_routine_proc(routine_fn_t routine_fn)
    {
        routine = routine_fn;
    }

    inline void set_transfer_proc(routine_fn_t transfer_fn)
    {
        transfer = transfer_fn;
    }
#ifdef SPECIFY_WORKFLOW_NODE_OWN_LEAVE
    inline void set_leave_proc(leave_fn_t leave_fn)
    {
        leave = leave_fn;
    }
#endif // SPECIFY_WORKFLOW_NODE_OWN_LEAVE
};

// This node provide a delay for the workflow
// During the delay period, the class will do nothing.
class gmp_wf_delay_node_t : public gmp_wf_node_base_gt
{
  public:
    // ctor & dtor
    gmp_wf_delay_node_t() : t0(0), default_node(wf_end_node)
    {
    }

    gmp_wf_delay_node_t(time_gt delay, gmp_wf_node_base_gt *target) : t0(delay), default_node(target)
    {
    }

  public:
    // The following function is generally dangerous,
    // But if there're a time critical workflow, this function may help a lot.
    // DANGEROUS! You should check your code to ensure that no loop or too long chain.
    //   Or, the `gmp_workflow::switch_to()` function may cause a deep recursion,
    //   which may damage your stack.
#if 0
    gmp_wf_node_base_gt* node_transfer(gmp_workflow_t* wf)
    {
        // This thing should not happen!
        assert(default_node);
        assert(wf);

        if (t0 == 0)
            return default_node;
        else
            return nullptr;

    }
#endif

    // The node routine will judge if it's the time.
    gmp_wf_node_base_gt *node_routine(gmp_workflow_t *wf) override
    {
        // if default node is a nullptr will cause endless loop.
        // That meaningless, so programmer should avoid this situation.
        assert(default_node);

        if (wf == nullptr)
            gmp_wf_node_base_gt::node_routine(wf);

        time_gt duration = wf->get_duration();

        if (duration <= t0)
            return nullptr;
        else
            return default_node;
    }

  public:
    // This parameter is a delay for the class
    time_gt t0;

    // when delay is reached, the workflow will switch to the target
    gmp_wf_node_base_gt *default_node;
};

// This class has only one overtime parameter
// Attention. This class allow you have no routine.
class gmp_wf_node_t0_t : public gmp_wf_node_gt
{
  public:
    // ctor & dtor
    gmp_wf_node_t0_t() : t0(0), default_node(wf_end_node)
    {
    }

  public:
    gmp_wf_node_base_gt *node_routine(gmp_workflow_t *wf) override
    {
        time_gt duration = wf->get_duration();

        if (duration > t0)
            return default_node;

        if (routine != nullptr)
            return routine(wf);
        else
            // Keep in same position
            return nullptr;
    }

  public:
    // members

    // over time
    time_gt t0;

    // default transfer target
    gmp_wf_node_base_gt *default_node;
};

// This class is a derive class for the workflow_node
// This class provided t1 and t2 for user to manage task easily.
class gmp_wf_node_t2_t : public gmp_wf_node_gt
{
  public:
    // ctor & dtor
    gmp_wf_node_t2_t() : t1(0), t2(0), last_exec_time(0)
    {
    }

  public:
    virtual gmp_wf_node_base_gt *node_routine(gmp_workflow_t *wf) override
    {
        time_gt duration = wf->get_duration();

        if (duration > t1)
        {
            duration = duration - last_exec_time;

            if (last_exec_time == 0)
            {
                last_exec_time = t1;
                return gmp_wf_node_gt::node_routine(wf);
            }

            if (duration > t2)
            {
                last_exec_time += t2;
                return gmp_wf_node_gt::node_routine(wf);
            }
        }

        return nullptr;
    }

    virtual gmp_wf_node_base_gt *node_transfer(gmp_workflow_t *wf) override
    {
        last_exec_time = 0;
        return gmp_wf_node_gt::node_transfer(wf);
    }

  public:
    // members

    // first time.
    time_gt t1;

    // asking time.
    time_gt t2;

  protected:
    time_gt last_exec_time;
};

// This class is a derive class for the workflow_node
// This class provided t1 and t2 for user to manage task easily.
class gmp_wf_node_t3_t : public gmp_wf_node_t2_t
{
  public:
    // ctor & dtor
    //	gmp_wf_node_t3_t() = default;

    gmp_wf_node_t3_t() : t0(-1), default_node(wf_end_node)
    {
    }

  public:
    virtual gmp_wf_node_base_gt *node_routine(gmp_workflow_t *wf) override
    {
        time_gt duration = wf->get_duration();

        if (duration > t0)
            return default_node;

        if (duration >= t1)
        {
            duration = duration - last_exec_time;

            if (last_exec_time == 0)
            {
                last_exec_time = t1;

                if (routine != nullptr)
                    return routine(wf);
                else
                    return nullptr;
            }

            if (duration > t2)
            {
                last_exec_time += t2;

                if (routine != nullptr)
                    return routine(wf);
                else
                    return nullptr;
            }
        }

        return nullptr;
    }

  public:
    // members

    // over time
    time_gt t0;

    // default transfer target
    gmp_wf_node_base_gt *default_node;
};

// The following class allow you to align several nodes.
// TODO impl
class gmp_wf_barrier_gt : public gmp_wf_node_base_gt
{
  public:
    // ctor & dtor
    gmp_wf_barrier_gt() : gmp_wf_node_base_gt(0)
    {
    }

  public:
    gmp_wf_node_base_gt *node_routine(gmp_workflow_t *wf) override
    {
        return nullptr;
    }

  public:
};

// old definition
#if 0

 /**
  * User protocol while using GMP workflow:
  * 1. The class workflow only takes the very first section of whole workflow.
  * 2. routine() in each section should return the pointer of next section when wfr_ready to next;
  *                                     return GMP_WF_SEC_WATING when not wfr_ready.
  * 3. If routine == nullptr, next section would be GMP_WF_SEC_WATING by default.
  */

#define GMP_WF_RTSTATE_START     (0x00)
#define GMP_WF_RTSTATE_TRANS_END (0x01)
#define GMP_WF_RTSTATE_T1_END    (0x02)
#define GMP_WF_RTSTATE_ROUT_END  (0x03)

class gmp_wf_section_t;

// typedef void (*transfer_fn_t)(void);
// typedef gmp_wf_section_t* (*routine_fn_t)(void);

typedef std::function<void(void)> transfer_fn_t;
typedef std::function<gmp_wf_section_t* (void)> routine_fn_t;

gmp_wf_section_t* const GMP_WF_SEC_WAITING = reinterpret_cast<gmp_wf_section_t*>(nullptr);
gmp_wf_section_t* const GMP_WF_SEC_START = reinterpret_cast<gmp_wf_section_t*>(1);
gmp_wf_section_t* const GMP_WF_SEC_END = reinterpret_cast<gmp_wf_section_t*>(2);

constexpr gmp_wf_section_t* GMP_WF_SEC_END2 = (gmp_wf_section_t*)3;

// GMP workflow state class
class gmp_wf_section_t
{
public:
    // ctor & dtor
    gmp_wf_section_t()
        : id(gmp_wf_id_invalid),
        transfer(nullptr), routine(nullptr),
        t0(0), t1(0), t2(0),
        default_next(GMP_WF_SEC_END),
        next(this)
    {}

    gmp_wf_section_t(
        size_gt id,
        transfer_fn_t transfer,
        routine_fn_t routine,
        time_gt t0,
        time_gt t1,
        time_gt t2,
        gmp_wf_section_t* default_next = GMP_WF_SEC_END
    )
        : id(id),
        transfer(transfer), routine(routine),
        t0(t0), t1(t1), t2(t2),
        default_next(default_next),
        next(this)
    {}

    gmp_wf_section_t(const gmp_wf_section_t& s)
        : id(s.id),
        transfer(s.transfer), routine(s.routine),
        t0(s.t0), t1(s.t1), t2(s.t2),
        default_next(s.default_next),
        next(this)
    {}


public:
    // identifier
    size_gt id;

    // function pointers
    transfer_fn_t transfer;
    routine_fn_t routine;

    // time
    time_gt t0; // overtime
    time_gt t1; // first time
    time_gt t2; // loop time

    // default next, this parameter is useful when overtime event occurred.
    gmp_wf_section_t* default_next;

    // specify the next pointer
    // When the next pointer is point to itself, the transfer will not happen.
    // When the next pointer in point to someone else, the transfer process would happen.
    gmp_wf_section_t* next;

public:
    // utility function

    // Set state transfer timer period
    void set_transfer_timer(time_gt t0, time_gt t1, time_gt t2)
    {
        this->t0 = t0;
        this->t1 = t1;
        this->t2 = t2;
    }

    // Set section ID
    void set_section_id(size_gt id)
    {
        this->id = id;
    }

    // Set next point
    void set_next_section(gmp_wf_section_t* next)
    {
        this->next = next;
    }
};

// This is the general end section of the whole work-flow objects
extern gmp_wf_section_t gmp_wf_default_end_section;

// This is the general start section of the whole work-flow objects
extern gmp_wf_section_t gmp_wf_default_start_section;

// class workflow_t
class gmp_workflow_t
{
public:
    // constructor
    gmp_workflow_t()
        : sec_start(nullptr)
        , sec_last(GMP_WF_SEC_START)
        , sec_curr(GMP_WF_SEC_START)
        , sec_next(sec_start)
    {
        _disable();
    }

    gmp_workflow_t(gmp_wf_section_t* seciton_start)
        : sec_start(seciton_start)
        , sec_last(GMP_WF_SEC_START)
        , sec_curr(GMP_WF_SEC_START)
        , sec_next(sec_start)
    {
        _disable();
    }

    gmp_workflow_t(
        gmp_wf_section_t* seciton_start,
        bool (*start_barrier)(void),
        void (*timeout_callback)(void)
    )
        : sec_start(seciton_start)
        , sec_last(GMP_WF_SEC_START)
        , sec_curr(GMP_WF_SEC_START)
        , sec_next(sec_start)
        , start_barrier(start_barrier)
        , timeout_callback(timeout_callback)
    {
        _disable();
    }

    gmp_workflow_t(
        gmp_wf_section_t* seciton_start,
        bool (*start_barrier)(void)
    )
        : sec_start(seciton_start)
        , sec_last(GMP_WF_SEC_START)
        , sec_curr(GMP_WF_SEC_START)
        , sec_next(sec_start)
        , start_barrier(start_barrier)
    {
        _disable();
    }

    gmp_workflow_t(
        gmp_wf_section_t* seciton_start,
        void (*timeout_callback)(void)
    )
        : sec_start(seciton_start)
        , sec_last(GMP_WF_SEC_START)
        , sec_curr(GMP_WF_SEC_START)
        , sec_next(sec_start)
        , timeout_callback(timeout_callback)
    {
        _disable();
    }

    gmp_workflow_t(const gmp_workflow_t& wf)
        : sec_start(wf.sec_start)
        , sec_last(GMP_WF_SEC_START)
        , sec_curr(GMP_WF_SEC_START)
        , sec_next(sec_start)
        , start_barrier(wf.start_barrier)
        , timeout_callback(wf.timeout_callback)
    {
        _disable();
    }

protected:
    // sections
    gmp_wf_section_t* sec_start;

    gmp_wf_section_t* sec_last;
    gmp_wf_section_t* sec_curr;
    gmp_wf_section_t* sec_next;

    // flag
    uint8_t flag_enable;   // enable flag
    uint8_t runtime_state; // workflow runtime flag

    // time
    time_gt last_tick;
    time_gt t0_start_tick;

public:
    // workflow functions
    ec_gt trans_init(gmp_wf_section_t* section_start);
    ec_gt deinit();
    ec_gt enable();
    ec_gt disable();
    ec_gt process();

protected:
    // barrier functions
    bool (*start_barrier)(void); // start barrier function

    // callback functions
    void (*timeout_callback)(void); // timeout callback

public:
    // get and set
    void set_start_barrier(bool (*start_barrier)(void));
    void set_timeout_callback(void (*timeout_callback)(void));

    uint8_t get_curr_id()
    {
        // BUG: This function is not secure
        return sec_curr->id;
    }
    uint8_t get_last_id()
    {
        // BUG: This function is not secure
        return sec_last->id;
    }

protected:
    // inline functions
    inline void _enable()
    {
        flag_enable = 1;
    }

    inline void _disable()
    {
        flag_enable = 0;
    }

    inline uint8_t is_enable()
    {
        return flag_enable;
    }
};


// general callback functions
void gmp_wf_timeout_callback(gmp_workflow_t* wf); // general timeout callback

void gmp_wf_cplt_callback(gmp_workflow_t* wf); // workflow end callback;

#endif // 0 comment

#endif // !_FILE_WORKFLOW_HPP_
