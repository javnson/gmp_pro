# CCTL Data-Stream Algorithms

`spsc_record_ring.hpp` provides a preallocated single-producer/single-consumer
ring for fixed-size records. Producer and consumer operations do not lock or
allocate after initialization. `try_push()` returns immediately when full, so a
real-time producer can choose a drop, retry, or fault policy.

The module has no thread, filesystem, or simulation-topology dependency. The
CCTL host CSP uses it between simulation and file-output workers; other logging
or high-rate acquisition code can select the same `cctl|dsa` source-manager
module. Capacity is supplied in bytes and rounded down to a whole record count.
