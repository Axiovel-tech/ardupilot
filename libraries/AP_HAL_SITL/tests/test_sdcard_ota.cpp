#include <AP_gtest.h>
#include <AP_HAL_SITL/SDCardOTA.h>

#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

using HALSITL::emulate_sdcard_ota;
using HALSITL::sdcard_ota_enabled;

namespace {

class TemporaryDirectory {
public:
    TemporaryDirectory()
    {
        EXPECT_NE(getcwd(previous, sizeof(previous)), nullptr);
        char path_template[] = "/tmp/axio-sd-ota-XXXXXX";
        const char *created = mkdtemp(path_template);
        EXPECT_NE(created, nullptr);
        if (created != nullptr) {
            strncpy(path, created, sizeof(path) - 1U);
            EXPECT_EQ(chdir(path), 0);
        }
        unsetenv("AXIO_SIM_SD_OTA_RESULT");
    }

    ~TemporaryDirectory()
    {
        unlink("ardupilot.abin");
        unlink("ardupilot-verify.abin");
        unlink("ardupilot-flash.abin");
        unlink("ardupilot-flashed.abin");
        unlink("ardupilot-failed.abin");
        EXPECT_EQ(chdir(previous), 0);
        EXPECT_EQ(rmdir(path), 0);
        unsetenv("AXIO_SIM_SD_OTA");
        unsetenv("AXIO_SIM_SD_OTA_RESULT");
    }

    void create(const char *name)
    {
        const int fd = open(name, O_CREAT | O_WRONLY, 0600);
        ASSERT_GE(fd, 0);
        ASSERT_EQ(close(fd), 0);
    }

    bool exists(const char *name) const
    {
        return access(name, F_OK) == 0;
    }

private:
    char previous[1024] {};
    char path[64] {};
};

TEST(SDCardOTA, RequiresExplicitEnable)
{
    unsetenv("AXIO_SIM_SD_OTA");
    EXPECT_FALSE(sdcard_ota_enabled());
    setenv("AXIO_SIM_SD_OTA", "0", 1);
    EXPECT_FALSE(sdcard_ota_enabled());
    setenv("AXIO_SIM_SD_OTA", "1", 1);
    EXPECT_TRUE(sdcard_ota_enabled());
    unsetenv("AXIO_SIM_SD_OTA");
}

TEST(SDCardOTA, MovesInputToFlashedMarker)
{
    TemporaryDirectory directory;
    directory.create("ardupilot-failed.abin");
    directory.create("ardupilot.abin");

    emulate_sdcard_ota();
    EXPECT_FALSE(directory.exists("ardupilot.abin"));
    EXPECT_FALSE(directory.exists("ardupilot-failed.abin"));
    EXPECT_TRUE(directory.exists("ardupilot-flashed.abin"));
}

TEST(SDCardOTA, MarksAnInjectedFailure)
{
    TemporaryDirectory directory;
    directory.create("ardupilot.abin");
    setenv("AXIO_SIM_SD_OTA_RESULT", "failure", 1);

    emulate_sdcard_ota();
    EXPECT_TRUE(directory.exists("ardupilot-failed.abin"));
}

TEST(SDCardOTA, ResumesAnInterruptedFlash)
{
    TemporaryDirectory directory;
    directory.create("ardupilot.abin");
    setenv("AXIO_SIM_SD_OTA_RESULT", "interrupted", 1);

    emulate_sdcard_ota();
    EXPECT_TRUE(directory.exists("ardupilot-flash.abin"));

    unsetenv("AXIO_SIM_SD_OTA_RESULT");
    emulate_sdcard_ota();
    EXPECT_TRUE(directory.exists("ardupilot-flashed.abin"));
}

TEST(SDCardOTA, ResumesVerificationMarker)
{
    TemporaryDirectory directory;
    directory.create("ardupilot-verify.abin");

    emulate_sdcard_ota();
    EXPECT_FALSE(directory.exists("ardupilot-verify.abin"));
    EXPECT_TRUE(directory.exists("ardupilot-flashed.abin"));
}

TEST(SDCardOTA, GivesFlashMarkerPriority)
{
    TemporaryDirectory directory;
    directory.create("ardupilot-flash.abin");
    directory.create("ardupilot.abin");

    emulate_sdcard_ota();
    EXPECT_TRUE(directory.exists("ardupilot.abin"));
    EXPECT_TRUE(directory.exists("ardupilot-flashed.abin"));
}

TEST(SDCardOTA, RetainsFlashMarkerAfterInjectedFailure)
{
    TemporaryDirectory directory;
    directory.create("ardupilot-flash.abin");
    setenv("AXIO_SIM_SD_OTA_RESULT", "failure", 1);

    emulate_sdcard_ota();
    EXPECT_TRUE(directory.exists("ardupilot-flash.abin"));
    EXPECT_FALSE(directory.exists("ardupilot-failed.abin"));
}

TEST(SDCardOTA, DoesNothingWithoutAnUpdate)
{
    TemporaryDirectory directory;
    emulate_sdcard_ota();
    EXPECT_FALSE(directory.exists("ardupilot-flashed.abin"));
}

} // namespace

AP_GTEST_MAIN()
