#include <tools/gmp_sil/sil_helper/gmp_sil_helper.hpp>

#include <algorithm>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace sil = gmp::sil;

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "usage: gmp_sil_test_peer <network.json>\n";
        return 2;
    }

    try
    {
        const auto config = sil::parse_configuration(argv[1]);
        const auto response_size = config.session.response_payload_size;
        sil::gmp_sil_helper helper(config);
        helper.connect();

        bool finished = false;
        while (!finished)
        {
            const auto event = helper.receive_event();
            switch (event.kind)
            {
            case sil::protocol::frame_kind::data_request:
            {
                std::vector<std::uint8_t> response(response_size, 0U);
                const auto copied = std::min(response.size(), event.payload.size());
                std::copy_n(event.payload.begin(), copied, response.begin());
                helper.respond(event, response.data(), response.size());
                break;
            }
            case sil::protocol::frame_kind::simulation_state:
            {
                const auto state = helper.decode_state(event);
                helper.acknowledge_state(event);
                finished = state.state == sil::protocol::simulation_state::completed ||
                           state.state == sil::protocol::simulation_state::aborted ||
                           state.state == sil::protocol::simulation_state::faulted;
                break;
            }
            default:
                throw sil::sil_error("Test peer received an unexpected SIL frame kind.");
            }
        }
        return 0;
    }
    catch (const std::exception& exception)
    {
        std::cerr << exception.what() << '\n';
        return 1;
    }
}
