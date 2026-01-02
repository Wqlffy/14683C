#pragma once

#include <cstddef>
#include <cstdint>

enum class AutonId : uint8_t {
    Red43Left,
    Red43Right,
    Blue43Left,
    Blue43Right,
    Long6RedLeft,
    Long6RedRight,
    Long6BlueLeft,
    Long6BlueRight,
    AwpRedLeft,
    AwpRedRight,
    AwpBlueLeft,
    AwpBlueRight,
    Skills
};

struct AutonInfo {
    AutonId id;
    const char* name;
    const char* desc;
    const void* img_src;
};

constexpr size_t AUTON_COUNT = 13;

extern const AutonInfo AUTONS[AUTON_COUNT];
extern volatile AutonId g_selected_auton;

const AutonInfo* get_auton_info(AutonId id);
void run_selected_auton();
