# GMP Core Native Unit Tests

Open `core_unit_test.sln` in Visual Studio and run the tests from
**Test > Test Explorer**. The solution contains one native C++ unit-test
project using `CppUnitTest.h`, `TEST_CLASS`, and `TEST_METHOD`.

The project is a `NativeUnitTestProject` DLL test container. It references GMP
implementation files directly from their authoritative locations under
`core`; the test workspace does not keep private source copies.

| Test source | Responsibility |
| --- | --- |
| `test_mm.cpp` | Block-memory setup, allocation, release, reuse, and capacity errors |
| `test_base_ds.cpp` | Intrusive list corruption/double-operation safety and ring-buffer boundary/wrap behavior |
| `test_datalink_facility.cpp` | Facility layout, registration, overlap rejection, routing, INFO v3, removal, counter, and corruption NACK |
| `test_rbtree.cpp` | Intrusive RB-tree ordering, structural deletion cases, duplicate operations, and invariant corruption detection |
| `test_canopen.cpp` | OD storage, logical-octet portability, NMT/heartbeat, expedited/segmented SDO, compiled TX/RX PDO fast paths and groups, and CoE reuse |
| `test_canopen_generated_od.cpp` | Generated CiA 301/401/402 profile-seed dictionary construction and lookup |
| `test_base_checksum.cpp` | CRC16-CCITT standard and binary vectors |
| `test_pm_function_scheduler.cpp` | Periodic, disabled, busy, and capacity scheduling |
| `test_pm_duff_fsm.cpp` | Delay, transition, reset, completion, and invalid-state handling |
| `test_pm_state_machine.cpp` | Include/idempotence smoke test for the current design placeholder |
| `test_pm_timing_manager.cpp` | Timer registration, wrap accounting, and null handling |
| `test_pm_workflow.cpp` | Inline legacy workflow-node identifiers and callbacks |

## Build and run

Select `Debug | x64`, build the solution, then choose **Run All Tests** in
Test Explorer. For command-line validation from a Visual Studio Developer
Prompt:

```bat
msbuild core_unit_test.sln /m /p:Configuration=Debug /p:Platform=x64
vstest.console.exe out\x64\Debug\core_unit_tests\core_unit_tests.dll
```

`state_machine.h` currently has no executable API. `workflow.hpp` is documented
as legacy and has no implementation source, so its test intentionally covers
only node behavior implemented in the public header.

The current `Debug|x64` container contains 50 tests. Production and test sources
compile at `/W4 /WX`.
