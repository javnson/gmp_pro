# CMake generated Testfile for 
# Source directory: E:/lib/gmp_pro/ctl/component/digital_power/inv/tests/host_sim
# Build directory: E:/lib/gmp_pro/ctl/component/digital_power/inv/tests/host_sim/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
if(CTEST_CONFIGURATION_TYPE MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
  add_test([=[inv_gfm_host_sim]=] "E:/lib/gmp_pro/ctl/component/digital_power/inv/tests/host_sim/build/Debug/gmp_inv_gfm_host_sim.exe")
  set_tests_properties([=[inv_gfm_host_sim]=] PROPERTIES  _BACKTRACE_TRIPLES "E:/lib/gmp_pro/ctl/component/digital_power/inv/tests/host_sim/CMakeLists.txt;28;add_test;E:/lib/gmp_pro/ctl/component/digital_power/inv/tests/host_sim/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
  add_test([=[inv_gfm_host_sim]=] "E:/lib/gmp_pro/ctl/component/digital_power/inv/tests/host_sim/build/Release/gmp_inv_gfm_host_sim.exe")
  set_tests_properties([=[inv_gfm_host_sim]=] PROPERTIES  _BACKTRACE_TRIPLES "E:/lib/gmp_pro/ctl/component/digital_power/inv/tests/host_sim/CMakeLists.txt;28;add_test;E:/lib/gmp_pro/ctl/component/digital_power/inv/tests/host_sim/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
  add_test([=[inv_gfm_host_sim]=] "E:/lib/gmp_pro/ctl/component/digital_power/inv/tests/host_sim/build/MinSizeRel/gmp_inv_gfm_host_sim.exe")
  set_tests_properties([=[inv_gfm_host_sim]=] PROPERTIES  _BACKTRACE_TRIPLES "E:/lib/gmp_pro/ctl/component/digital_power/inv/tests/host_sim/CMakeLists.txt;28;add_test;E:/lib/gmp_pro/ctl/component/digital_power/inv/tests/host_sim/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
  add_test([=[inv_gfm_host_sim]=] "E:/lib/gmp_pro/ctl/component/digital_power/inv/tests/host_sim/build/RelWithDebInfo/gmp_inv_gfm_host_sim.exe")
  set_tests_properties([=[inv_gfm_host_sim]=] PROPERTIES  _BACKTRACE_TRIPLES "E:/lib/gmp_pro/ctl/component/digital_power/inv/tests/host_sim/CMakeLists.txt;28;add_test;E:/lib/gmp_pro/ctl/component/digital_power/inv/tests/host_sim/CMakeLists.txt;0;")
else()
  add_test([=[inv_gfm_host_sim]=] NOT_AVAILABLE)
endif()
