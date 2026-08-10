# CMake generated Testfile for 
# Source directory: E:/lib/gmp_pro/tools/gmp_sil/tcp_helper_v2/tests
# Build directory: E:/lib/gmp_pro/tools/gmp_sil/tcp_helper_v2/build/tests
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
if(CTEST_CONFIGURATION_TYPE MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
  add_test([=[gmp_sil_tcp_helper_v2_tests]=] "E:/lib/gmp_pro/tools/gmp_sil/tcp_helper_v2/build/tests/Debug/gmp_sil_tcp_helper_v2_tests.exe")
  set_tests_properties([=[gmp_sil_tcp_helper_v2_tests]=] PROPERTIES  _BACKTRACE_TRIPLES "E:/lib/gmp_pro/tools/gmp_sil/tcp_helper_v2/tests/CMakeLists.txt;10;add_test;E:/lib/gmp_pro/tools/gmp_sil/tcp_helper_v2/tests/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
  add_test([=[gmp_sil_tcp_helper_v2_tests]=] "E:/lib/gmp_pro/tools/gmp_sil/tcp_helper_v2/build/tests/Release/gmp_sil_tcp_helper_v2_tests.exe")
  set_tests_properties([=[gmp_sil_tcp_helper_v2_tests]=] PROPERTIES  _BACKTRACE_TRIPLES "E:/lib/gmp_pro/tools/gmp_sil/tcp_helper_v2/tests/CMakeLists.txt;10;add_test;E:/lib/gmp_pro/tools/gmp_sil/tcp_helper_v2/tests/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
  add_test([=[gmp_sil_tcp_helper_v2_tests]=] "E:/lib/gmp_pro/tools/gmp_sil/tcp_helper_v2/build/tests/MinSizeRel/gmp_sil_tcp_helper_v2_tests.exe")
  set_tests_properties([=[gmp_sil_tcp_helper_v2_tests]=] PROPERTIES  _BACKTRACE_TRIPLES "E:/lib/gmp_pro/tools/gmp_sil/tcp_helper_v2/tests/CMakeLists.txt;10;add_test;E:/lib/gmp_pro/tools/gmp_sil/tcp_helper_v2/tests/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
  add_test([=[gmp_sil_tcp_helper_v2_tests]=] "E:/lib/gmp_pro/tools/gmp_sil/tcp_helper_v2/build/tests/RelWithDebInfo/gmp_sil_tcp_helper_v2_tests.exe")
  set_tests_properties([=[gmp_sil_tcp_helper_v2_tests]=] PROPERTIES  _BACKTRACE_TRIPLES "E:/lib/gmp_pro/tools/gmp_sil/tcp_helper_v2/tests/CMakeLists.txt;10;add_test;E:/lib/gmp_pro/tools/gmp_sil/tcp_helper_v2/tests/CMakeLists.txt;0;")
else()
  add_test([=[gmp_sil_tcp_helper_v2_tests]=] NOT_AVAILABLE)
endif()
