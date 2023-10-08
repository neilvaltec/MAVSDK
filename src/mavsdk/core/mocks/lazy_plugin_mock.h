#pragma once

namespace mavsdk {
namespace mavsdk_server {
namespace testing {

template<typename Plugin> class MockLazyPlugin {
public:
    MOCK_CONST_METHOD1(maybe_plugin, Plugin*(int32_t)){};
};

} // namespace testing
} // namespace mavsdk_server
} // namespace mavsdk
