/**
 * @file state_machine.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

// A state machine module, support operations shown as follows
// 1. The initialization state can be specified
// 2. Normal state transform is supported
// 3. signal schedule
// 4. Own two callback functions, and these function would be called when move into or leave the state
// 5. Sub-state-machine is supported, that means one node may be extended by a state machine.
