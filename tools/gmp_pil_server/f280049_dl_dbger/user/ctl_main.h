/**
 * @file ctl_main.h
 * @brief CTL declarations required by the standard GMP entry sequence.
 */

#ifndef GMP_F280049_DL_DBGER_CTL_MAIN_H
#define GMP_F280049_DL_DBGER_CTL_MAIN_H

/** @brief Initialize controller-layer services. */
void ctl_init(void);

/** @brief Run controller-layer background work. */
void ctl_mainloop(void);

/** @brief Dispatch one synchronous controller step. */
GMP_STATIC_INLINE void ctl_dispatch(void)
{
}

#endif // GMP_F280049_DL_DBGER_CTL_MAIN_H
