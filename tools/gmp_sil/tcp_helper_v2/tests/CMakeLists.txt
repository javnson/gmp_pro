add_executable(gmp_sil_tcp_helper_v2_tests tcp_helper_tests.cpp)
target_link_libraries(gmp_sil_tcp_helper_v2_tests PRIVATE gmp_sil_tcp_helper_v2)

if(MSVC)
    target_compile_options(gmp_sil_tcp_helper_v2_tests PRIVATE /W4 /permissive-)
else()
    target_compile_options(gmp_sil_tcp_helper_v2_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME gmp_sil_tcp_helper_v2_tests COMMAND gmp_sil_tcp_helper_v2_tests)

