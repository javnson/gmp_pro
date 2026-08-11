add_executable(gmp_sil_transport_tests tcp_helper_tests.cpp)
target_link_libraries(gmp_sil_transport_tests PRIVATE gmp_sil_helper)

if(MSVC)
    target_compile_options(gmp_sil_transport_tests PRIVATE /W4 /permissive-)
else()
    target_compile_options(gmp_sil_transport_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME gmp_sil_transport_tests COMMAND gmp_sil_transport_tests)

add_executable(gmp_sil_helper_tests sil_helper_tests.cpp)
target_link_libraries(gmp_sil_helper_tests PRIVATE gmp_sil_helper)
if(MSVC)
    target_compile_options(gmp_sil_helper_tests PRIVATE /W4 /permissive-)
else()
    target_compile_options(gmp_sil_helper_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()
add_test(NAME gmp_sil_helper_tests COMMAND gmp_sil_helper_tests)

# Small controller-side peer used by Simulink multi-instance integration
# tests.  It is not registered as a standalone CTest because Simulink owns
# the client side and the session lifecycle.
add_executable(gmp_sil_test_peer sil_test_peer.cpp)
target_link_libraries(gmp_sil_test_peer PRIVATE gmp_sil_helper)
if(MSVC)
    target_compile_options(gmp_sil_test_peer PRIVATE /W4 /permissive-)
else()
    target_compile_options(gmp_sil_test_peer PRIVATE -Wall -Wextra -Wpedantic)
endif()
