| MAILBOX | ID    | RX / TX | Abstract       | Note                                                         | Appendix                  |
| ------- | ----- | ------- | -------------- | ------------------------------------------------------------ | ------------------------- |
| 1       | 0x101 | RX      | Control Flag   | Enable System                                                | 0: disable<br />1: enable |
| 2       | 0x102 | RX      | Control Target | 0-3: Target voltage / current d axis<br />4-7: Target voltage / current q axis |                           |
| 3       | 0x103 | RX      | Debug          | reserved for debug                                           |                           |
| 4       | 0x201 | TX      | Monitor        | 0-3: Monitor Voltage from Grid d axis<br />4-7: Monitor Voltage from Grid q axis |                           |
| 5       | 0x202 | TX      | Monitor        | 0-3: Monitor Voltage from inverter d axis<br />4-7: Monitor Voltage from inverter q axis |                           |
| 6       | 0x203 | TX      | Monitor        | 0-3: Monitor Current from Grid d axis<br />4-7: Monitor Current from Grid q axis |                           |
| 7       | 0x204 | TX      | Monitor        | 0-3: Monitor Current from inverter d axis<br />4-7: Monitor Current from inverter q axis |                           |
| 8       | 0x205 | TX      | Monitor        | 0-3: Monitor DC bus Voltage<br />4-7: Monitor DC bus Current |                           |
| 9       | 0x206 | TX      | Monitor        | 0-3: Monitor Voltage from Grid A axis<br />4-7: Monitor Voltage from PLL angle output |                           |
| 10      | 0x206 | TX      | Debug          | reserved for debug                                           |                           |


NOTE for SCI interface

921600 bps for gmp_base_print() function and AT command input.


