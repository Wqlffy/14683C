#include "auton_selector.hpp"
#include "autons.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {
constexpr const char* kStatePath = "/usd/auton_state.txt";
constexpr const char* kStateMagic = "AUTON_STATE_V1";

AutonId default_auton() {
    return AutonId::Skills;
}

AutonLockState default_lock() {
    return AutonLockState::Unlocked;
}

bool is_valid_auton(int value) {
    return value >= 0 && value < static_cast<int>(AUTON_COUNT);
}
}

volatile AutonId g_selected_auton = AutonId::Skills;
volatile AutonLockState g_auton_lock = AutonLockState::Unlocked;

const AutonInfo AUTONS[AUTON_COUNT] = {
    {AutonId::Blue43Left, "43 Blue Left",
     "4 long 3 mid", nullptr},
    {AutonId::Red43Left, "43 Red Left",
     "4 long 3 mid", nullptr},
    {AutonId::Long9BlueLeft, "9 Long Blue Left",
     "9 long", nullptr},
    {AutonId::Long9BlueRight, "9 Long Blue Right",
     "9 long", nullptr},
    {AutonId::Long9RedLeft, "9 Long Red Left",
     "9 long", nullptr},
    {AutonId::Long9RedRight, "9 Long Red Right",
     "9 long", nullptr},
    {AutonId::AwpBlueRight, "AWP Blue Right",
     "AWP Blue RIGHT QUAL", nullptr},
    {AutonId::AwpRedRight, "AWP Red Right",
     "AWP Red RIGHT QUAL", nullptr},
    {AutonId::CALIBRATION, "Calibration",
     "sensor + drivetrain calibration", nullptr},
    {AutonId::Skills, "Skills",
     "SKILLS", nullptr},
};

const AutonInfo* get_auton_info(AutonId id) {
    for (size_t i = 0; i < AUTON_COUNT; ++i) {
        if (AUTONS[i].id == id) {
            return &AUTONS[i];
        }
    }
    return nullptr;
}

void save_auton_state() {
    FILE* file = std::fopen(kStatePath, "w");
    if (!file) {
        return;
    }
    std::fprintf(file, "%s\n%d\n%d\n", kStateMagic,
                 static_cast<int>(g_selected_auton),
                 static_cast<int>(g_auton_lock));
    std::fclose(file);
}

void load_auton_state() {
    g_selected_auton = default_auton();
    g_auton_lock = default_lock();

    FILE* file = std::fopen(kStatePath, "r");
    if (!file) {
        return;
    }

    char magic[32] = {};
    if (!std::fgets(magic, sizeof(magic), file)) {
        std::fclose(file);
        return;
    }
    magic[strcspn(magic, "\r\n")] = '\0';
    if (std::strcmp(magic, kStateMagic) != 0) {
        std::fclose(file);
        return;
    }

    char line[32] = {};
    if (!std::fgets(line, sizeof(line), file)) {
        std::fclose(file);
        return;
    }
    const int auton_val = std::strtol(line, nullptr, 10);
    if (!std::fgets(line, sizeof(line), file)) {
        std::fclose(file);
        return;
    }
    const int lock_val = std::strtol(line, nullptr, 10);
    std::fclose(file);

    if (!is_valid_auton(auton_val)) {
        g_selected_auton = default_auton();
    } else {
        g_selected_auton = static_cast<AutonId>(auton_val);
    }

    g_auton_lock =
        (lock_val == static_cast<int>(AutonLockState::Locked))
            ? AutonLockState::Locked
            : AutonLockState::Unlocked;
}

void run_selected_auton() {
    if (g_auton_lock != AutonLockState::Locked) {
        std::printf("WARN: auton called while unlocked, skipping.\n");
        return;
    }

    switch (g_selected_auton) {
        case AutonId::Blue43Left:
            auton_43_blue_left();
            break;
        case AutonId::Red43Left:
            auton_43_red_left();
            break;
        case AutonId::Long9BlueLeft:
            auton_9_long_blue_left();
            break;
        case AutonId::Long9BlueRight:
            auton_9_long_blue_right();
            break;
        case AutonId::Long9RedLeft:
            auton_9_long_red_left();
            break;
        case AutonId::Long9RedRight:
            auton_9_long_red_right();
            break;
        case AutonId::AwpBlueRight:
            auton_awp_blue_right();
            break;
        case AutonId::AwpRedRight:
            auton_awp_red_right();
            break;
        case AutonId::CALIBRATION:
            calibration_auton();
            break;
        case AutonId::Skills:
            skills_auton();
            break;
    }
}
