#include "log.h"
#include "mavsdk.h"
#include <filesystem>
#include <gtest/gtest.h>
#include <chrono>
#include <future>
#include <fstream>
#include "plugins/ftp/ftp.h"
#include "plugins/ftp_server/ftp_server.h"
#include "fs_helpers.h"

using namespace mavsdk;

static constexpr double reduced_timeout_s = 0.1;

// TODO: make this compatible for Windows using GetTempPath2

namespace fs = std::filesystem;

static auto sep = fs::path::preferred_separator;

static const fs::path temp_dir_provided = "/tmp/mavsdk_systemtest_temp_data/provided";
static const fs::path temp_dir_to_upload = "/tmp/mavsdk_systemtest_temp_data/to_upload";

static const fs::path temp_file = "data.bin";

TEST(SystemTest, FtpUploadFile)
{
    Mavsdk mavsdk_groundstation;
    mavsdk_groundstation.set_configuration(
        Mavsdk::Configuration{Mavsdk::Configuration::UsageType::GroundStation});
    mavsdk_groundstation.set_timeout_s(reduced_timeout_s);

    Mavsdk mavsdk_autopilot;
    mavsdk_autopilot.set_configuration(
        Mavsdk::Configuration{Mavsdk::Configuration::UsageType::Autopilot});
    mavsdk_autopilot.set_timeout_s(reduced_timeout_s);

    ASSERT_EQ(mavsdk_groundstation.add_any_connection("udp://:17000"), ConnectionResult::Success);
    ASSERT_EQ(
        mavsdk_autopilot.add_any_connection("udp://127.0.0.1:17000"), ConnectionResult::Success);

    auto ftp_server = FtpServer{
        mavsdk_autopilot.server_component_by_type(Mavsdk::ServerComponentType::Autopilot)};

    auto maybe_system = mavsdk_groundstation.first_autopilot(10.0);
    ASSERT_TRUE(maybe_system);
    auto system = maybe_system.value();

    ASSERT_TRUE(system->has_autopilot());

    ASSERT_TRUE(create_temp_file(temp_dir_to_upload / temp_file, 50));
    ASSERT_TRUE(reset_directories(temp_dir_provided));

    auto ftp = Ftp{system};

    // First we try to access the file without the root directory set.
    // We expect that it does not exist as we don't have any permission.
    {
        auto prom = std::promise<Ftp::Result>();
        auto fut = prom.get_future();
        ftp.upload_async(
            temp_dir_to_upload / temp_file, ".", [&prom](Ftp::Result result, Ftp::ProgressData) {
                prom.set_value(result);
            });

        auto future_status = fut.wait_for(std::chrono::seconds(1));
        ASSERT_EQ(future_status, std::future_status::ready);
        EXPECT_EQ(fut.get(), Ftp::Result::FileDoesNotExist);
    }

    // Now we set the root dir and expect it to work.
    ftp_server.set_root_dir(temp_dir_provided);

    {
        auto prom = std::promise<Ftp::Result>();
        auto fut = prom.get_future();
        ftp.upload_async(
            temp_dir_to_upload / temp_file,
            "/",
            [&prom](Ftp::Result result, Ftp::ProgressData progress_data) {
                if (result != Ftp::Result::Next) {
                    prom.set_value(result);
                } else {
                    LogDebug() << "Upload progress: " << progress_data.bytes_transferred << "/"
                               << progress_data.total_bytes << " bytes";
                }
            });

        auto future_status = fut.wait_for(std::chrono::seconds(1));
        ASSERT_EQ(future_status, std::future_status::ready);
        EXPECT_EQ(fut.get(), Ftp::Result::Success);

        EXPECT_TRUE(
            are_files_identical(temp_dir_to_upload / temp_file, temp_dir_provided / temp_file));
    }
}

TEST(SystemTest, FtpUploadBigFile)
{
    Mavsdk mavsdk_groundstation;
    mavsdk_groundstation.set_configuration(
        Mavsdk::Configuration{Mavsdk::Configuration::UsageType::GroundStation});
    mavsdk_groundstation.set_timeout_s(reduced_timeout_s);

    Mavsdk mavsdk_autopilot;
    mavsdk_autopilot.set_configuration(
        Mavsdk::Configuration{Mavsdk::Configuration::UsageType::Autopilot});
    mavsdk_autopilot.set_timeout_s(reduced_timeout_s);

    ASSERT_EQ(mavsdk_groundstation.add_any_connection("udp://:17000"), ConnectionResult::Success);
    ASSERT_EQ(
        mavsdk_autopilot.add_any_connection("udp://127.0.0.1:17000"), ConnectionResult::Success);

    auto ftp_server = FtpServer{
        mavsdk_autopilot.server_component_by_type(Mavsdk::ServerComponentType::Autopilot)};

    auto maybe_system = mavsdk_groundstation.first_autopilot(10.0);
    ASSERT_TRUE(maybe_system);
    auto system = maybe_system.value();

    ASSERT_TRUE(system->has_autopilot());

    ASSERT_TRUE(create_temp_file(temp_dir_to_upload / temp_file, 10000));
    ASSERT_TRUE(reset_directories(temp_dir_provided));

    auto ftp = Ftp{system};

    ftp_server.set_root_dir(temp_dir_provided);

    {
        auto prom = std::promise<Ftp::Result>();
        auto fut = prom.get_future();
        ftp.upload_async(
            temp_dir_to_upload / temp_file,
            "/",
            [&prom](Ftp::Result result, Ftp::ProgressData progress_data) {
                if (result != Ftp::Result::Next) {
                    prom.set_value(result);
                } else {
                    LogDebug() << "Upload progress: " << progress_data.bytes_transferred << "/"
                               << progress_data.total_bytes << " bytes";
                }
            });

        auto future_status = fut.wait_for(std::chrono::seconds(3));
        ASSERT_EQ(future_status, std::future_status::ready);
        EXPECT_EQ(fut.get(), Ftp::Result::Success);

        EXPECT_TRUE(
            are_files_identical(temp_dir_to_upload / temp_file, temp_dir_provided / temp_file));
    }
}

TEST(SystemTest, FtpUploadBigFileLossy)
{
    Mavsdk mavsdk_groundstation;
    mavsdk_groundstation.set_configuration(
        Mavsdk::Configuration{Mavsdk::Configuration::UsageType::GroundStation});
    mavsdk_groundstation.set_timeout_s(reduced_timeout_s);

    Mavsdk mavsdk_autopilot;
    mavsdk_autopilot.set_configuration(
        Mavsdk::Configuration{Mavsdk::Configuration::UsageType::Autopilot});
    mavsdk_autopilot.set_timeout_s(reduced_timeout_s);

    unsigned counter = 0;
    auto drop_some = [&counter](mavlink_message_t&) { return counter++ % 5; };

    mavsdk_groundstation.intercept_incoming_messages_async(drop_some);
    mavsdk_groundstation.intercept_outgoing_messages_async(drop_some);

    ASSERT_EQ(mavsdk_groundstation.add_any_connection("udp://:17000"), ConnectionResult::Success);
    ASSERT_EQ(
        mavsdk_autopilot.add_any_connection("udp://127.0.0.1:17000"), ConnectionResult::Success);

    auto ftp_server = FtpServer{
        mavsdk_autopilot.server_component_by_type(Mavsdk::ServerComponentType::Autopilot)};

    auto maybe_system = mavsdk_groundstation.first_autopilot(10.0);
    ASSERT_TRUE(maybe_system);
    auto system = maybe_system.value();

    ASSERT_TRUE(system->has_autopilot());

    ASSERT_TRUE(create_temp_file(temp_dir_to_upload / temp_file, 10000));
    ASSERT_TRUE(reset_directories(temp_dir_provided));

    auto ftp = Ftp{system};

    ftp_server.set_root_dir(temp_dir_provided);

    {
        auto prom = std::promise<Ftp::Result>();
        auto fut = prom.get_future();
        ftp.upload_async(
            temp_dir_to_upload / temp_file,
            "/",
            [&prom](Ftp::Result result, Ftp::ProgressData progress_data) {
                if (result != Ftp::Result::Next) {
                    prom.set_value(result);
                } else {
                    LogDebug() << "Upload progress: " << progress_data.bytes_transferred << "/"
                               << progress_data.total_bytes << " bytes";
                }
            });

        auto future_status = fut.wait_for(std::chrono::seconds(3));
        ASSERT_EQ(future_status, std::future_status::ready);
        EXPECT_EQ(fut.get(), Ftp::Result::Success);

        EXPECT_TRUE(
            are_files_identical(temp_dir_to_upload / temp_file, temp_dir_provided / temp_file));
    }

    mavsdk_groundstation.intercept_incoming_messages_async(nullptr);
    mavsdk_groundstation.intercept_outgoing_messages_async(nullptr);
}

TEST(SystemTest, FtpUploadStopAndTryAgain)
{
    Mavsdk mavsdk_groundstation;
    mavsdk_groundstation.set_configuration(
        Mavsdk::Configuration{Mavsdk::Configuration::UsageType::GroundStation});
    mavsdk_groundstation.set_timeout_s(reduced_timeout_s);

    Mavsdk mavsdk_autopilot;
    mavsdk_autopilot.set_configuration(
        Mavsdk::Configuration{Mavsdk::Configuration::UsageType::Autopilot});
    mavsdk_autopilot.set_timeout_s(reduced_timeout_s);

    // Once we received half, we want to stop all traffic.
    bool got_half = false;
    auto drop_at_some_point = [&got_half](mavlink_message_t&) { return !got_half; };

    mavsdk_groundstation.intercept_incoming_messages_async(drop_at_some_point);
    mavsdk_groundstation.intercept_outgoing_messages_async(drop_at_some_point);

    ASSERT_EQ(mavsdk_groundstation.add_any_connection("udp://:17000"), ConnectionResult::Success);
    ASSERT_EQ(
        mavsdk_autopilot.add_any_connection("udp://127.0.0.1:17000"), ConnectionResult::Success);

    auto ftp_server = FtpServer{
        mavsdk_autopilot.server_component_by_type(Mavsdk::ServerComponentType::Autopilot)};

    ftp_server.set_root_dir(temp_dir_provided);

    auto maybe_system = mavsdk_groundstation.first_autopilot(10.0);
    ASSERT_TRUE(maybe_system);
    auto system = maybe_system.value();

    ASSERT_TRUE(system->has_autopilot());

    ASSERT_TRUE(create_temp_file(temp_dir_to_upload / temp_file, 1000));
    ASSERT_TRUE(reset_directories(temp_dir_provided));

    auto ftp = Ftp{system};

    auto prom = std::promise<Ftp::Result>();
    auto fut = prom.get_future();
    ftp.upload_async(
        temp_dir_to_upload / temp_file,
        "/",
        [&prom, &got_half](Ftp::Result result, Ftp::ProgressData progress_data) {
            if (progress_data.bytes_transferred > 500) {
                got_half = true;
            }
            if (result != Ftp::Result::Next) {
                prom.set_value(result);
            } else {
                LogDebug() << "Download progress: " << progress_data.bytes_transferred << "/"
                           << progress_data.total_bytes << " bytes";
            }
        });

    auto future_status = fut.wait_for(std::chrono::seconds(5));
    ASSERT_EQ(future_status, std::future_status::ready);
    EXPECT_EQ(fut.get(), Ftp::Result::Timeout);

    // Before going out of scope, we need to make sure to no longer access the
    // drop_some callback which accesses the local counter variable.
    mavsdk_groundstation.intercept_incoming_messages_async(nullptr);
    mavsdk_groundstation.intercept_outgoing_messages_async(nullptr);

    {
        // Now try again
        auto prom = std::promise<Ftp::Result>();
        auto fut = prom.get_future();
        ftp.upload_async(
            temp_dir_to_upload / temp_file,
            "/",
            [&prom, &got_half](Ftp::Result result, Ftp::ProgressData progress_data) {
                if (result != Ftp::Result::Next) {
                    prom.set_value(result);
                } else {
                    LogDebug() << "Download progress: " << progress_data.bytes_transferred << "/"
                               << progress_data.total_bytes << " bytes";
                }
            });

        auto future_status = fut.wait_for(std::chrono::seconds(5));
        ASSERT_EQ(future_status, std::future_status::ready);
        EXPECT_EQ(fut.get(), Ftp::Result::Success);
    }
}

TEST(SystemTest, FtpUploadFileOutsideOfRoot)
{
    // A test trying to push something into the parent (../) directory which
    // shouldn't be allowed!

    Mavsdk mavsdk_groundstation;
    mavsdk_groundstation.set_configuration(
        Mavsdk::Configuration{Mavsdk::Configuration::UsageType::GroundStation});
    mavsdk_groundstation.set_timeout_s(reduced_timeout_s);

    Mavsdk mavsdk_autopilot;
    mavsdk_autopilot.set_configuration(
        Mavsdk::Configuration{Mavsdk::Configuration::UsageType::Autopilot});
    mavsdk_autopilot.set_timeout_s(reduced_timeout_s);

    ASSERT_EQ(mavsdk_groundstation.add_any_connection("udp://:17000"), ConnectionResult::Success);
    ASSERT_EQ(
        mavsdk_autopilot.add_any_connection("udp://127.0.0.1:17000"), ConnectionResult::Success);

    auto ftp_server = FtpServer{
        mavsdk_autopilot.server_component_by_type(Mavsdk::ServerComponentType::Autopilot)};

    auto maybe_system = mavsdk_groundstation.first_autopilot(10.0);
    ASSERT_TRUE(maybe_system);
    auto system = maybe_system.value();

    ASSERT_TRUE(system->has_autopilot());

    ASSERT_TRUE(create_temp_file(temp_dir_to_upload / temp_file, 50));
    ASSERT_TRUE(reset_directories(temp_dir_provided));

    auto ftp = Ftp{system};

    // Now we set the root dir and expect it to work.
    ftp_server.set_root_dir(std::string(temp_dir_provided));

    {
        auto prom = std::promise<Ftp::Result>();
        auto fut = prom.get_future();
        ftp.upload_async(
            temp_dir_to_upload / temp_file,
            "../",
            [&prom](Ftp::Result result, Ftp::ProgressData progress_data) {
                prom.set_value(result);
            });

        auto future_status = fut.wait_for(std::chrono::seconds(1));
        ASSERT_EQ(future_status, std::future_status::ready);
        EXPECT_EQ(fut.get(), Ftp::Result::ProtocolError);
    }
}
