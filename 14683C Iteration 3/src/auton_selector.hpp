#pragma once

#include <cstddef>
#include <cstdint>

enum class AutonId : uint8_t {
    Blue43Left,
    Red43Left,
    Long9BlueLeft,
    Long9BlueRight,
    Long9RedLeft,
    Long9RedRight,
    AwpBlueRight,
    AwpRedRight,
    CALIBRATION,
    Skills
};

enum class AutonLockState : uint8_t {
    Unlocked,
    Locked
};

struct AutonInfo {
    AutonId id;
    const char* name;
    const char* desc;
    const void* img_src;
};

constexpr size_t AUTON_COUNT = 10;

extern const AutonInfo AUTONS[AUTON_COUNT];
extern volatile AutonId g_selected_auton;
extern volatile AutonLockState g_auton_lock;

const AutonInfo* get_auton_info(AutonId id);
void save_auton_state();
void load_auton_state();
void run_selected_auton();
