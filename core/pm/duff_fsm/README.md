# Duff FSM

`duff_fsm.h` 用行号和 `switch/case` 构造无动态内存的协作式状态机，
Facility 模块 ID 为 `core|pm|duff_fsm`。它使用
`gmp_base_get_system_tick()` 驱动延时，适合反复调用的嵌入式后台任务。
