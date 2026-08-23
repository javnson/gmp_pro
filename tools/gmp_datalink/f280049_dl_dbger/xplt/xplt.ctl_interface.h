/**
 * @file xplt.ctl_interface.h
 * @brief Empty platform callbacks required by the GMP CTL dispatcher.
 */

#ifndef GMP_F280049_DL_DBGER_CTL_INTERFACE_H
#define GMP_F280049_DL_DBGER_CTL_INTERFACE_H

/** @brief Acquire platform inputs before a control step. */
GMP_STATIC_INLINE void ctl_input_callback(void)
{
}

/** @brief Apply platform outputs after a control step. */
GMP_STATIC_INLINE void ctl_output_callback(void)
{
}

#endif // GMP_F280049_DL_DBGER_CTL_INTERFACE_H
