#include "auton_selector.hpp"

volatile AutonId g_selected_auton = AutonId::Red43Left;

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

void run_selected_auton() {
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
