/**
 * @file board.h
 * @brief Minimal board adapter consumed by the GMP C28x SysConfig CSP.
 */

#ifndef GMP_F280049_DL_DBGER_BOARD_H
#define GMP_F280049_DL_DBGER_BOARD_H

#include "driverlib.h"
#include "device.h"

/** @brief Initialize SysConfig-owned board resources. */
void Board_init(void);

#endif // GMP_F280049_DL_DBGER_BOARD_H
