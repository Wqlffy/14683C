#include "auton_selector.hpp"

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
}  // namespace

volatile AutonId g_selected_auton = AutonId::Skills;
volatile AutonLockState g_auton_lock = AutonLockState::Unlocked;

// TODO: Replace descriptions and img_src when your routes/assets are finalized.
const AutonInfo AUTONS[AUTON_COUNT] = {
    {AutonId::Red43Left, "4-3 RED LEFT",
     "4-3 red left: safe alliance goal clear and mid-zone control.", nullptr},
    {AutonId::Red43Right, "4-3 RED RIGHT",
     "4-3 red right: fast rush with defensive exit option.", nullptr},
    {AutonId::Blue43Left, "4-3 BLUE LEFT",
     "4-3 blue left: safe alliance goal clear and mid-zone control.", nullptr},
    {AutonId::Blue43Right, "4-3 BLUE RIGHT",
     "4-3 blue right: fast rush with defensive exit option.", nullptr},
    {AutonId::Long6RedLeft, "6 LONG RED LEFT",
     "6 long red left: full lane clear with controlled stack.", nullptr},
    {AutonId::Long6RedRight, "6 LONG RED RIGHT",
     "6 long red right: full lane clear with controlled stack.", nullptr},
    {AutonId::Long6BlueLeft, "6 LONG BLUE LEFT",
     "6 long blue left: full lane clear with controlled stack.", nullptr},
    {AutonId::Long6BlueRight, "6 LONG BLUE RIGHT",
     "6 long blue right: full lane clear with controlled stack.", nullptr},
    {AutonId::AwpRedLeft, "AWP RED LEFT",
     "awp red left: secure win point and park setup.", nullptr},
    {AutonId::AwpRedRight, "AWP RED RIGHT",
     "awp red right: secure win point and park setup.", nullptr},
    {AutonId::AwpBlueLeft, "AWP BLUE LEFT",
     "awp blue left: secure win point and park setup.", nullptr},
    {AutonId::AwpBlueRight, "AWP BLUE RIGHT",
     "awp blue right: secure win point and park setup.", nullptr},
    {AutonId::Skills, "SKILLS",
     "skills run: full field cycle with max scoring route.", nullptr},
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
        case AutonId::Red43Left:
            // TODO: call 4-3 red left auton.
            break;
        case AutonId::Red43Right:
            // TODO: call 4-3 red right auton.
            break;
        case AutonId::Blue43Left:
            // TODO: call 4-3 blue left auton.
            break;
        case AutonId::Blue43Right:
            // TODO: call 4-3 blue right auton.
            break;
        case AutonId::Long6RedLeft:
            // TODO: call 6 long red left auton.
            break;
        case AutonId::Long6RedRight:
            // TODO: call 6 long red right auton.
            break;
        case AutonId::Long6BlueLeft:
            // TODO: call 6 long blue left auton.
            break;
        case AutonId::Long6BlueRight:
            // TODO: call 6 long blue right auton.
            break;
        case AutonId::AwpRedLeft:
            // TODO: call AWP red left auton.
            break;
        case AutonId::AwpRedRight:
            // TODO: call AWP red right auton.
            break;
        case AutonId::AwpBlueLeft:
            // TODO: call AWP blue left auton.
            break;
        case AutonId::AwpBlueRight:
            // TODO: call AWP blue right auton.
            break;
        case AutonId::Skills:
            // TODO: call skills auton.
            break;
    }
}
