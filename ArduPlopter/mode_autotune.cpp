#include "mode.h"
#include "Plopter.h"

bool ModeAutoTune::_enter()
{
    plopter.autotune_start();

    return true;
}

void ModeAutoTune::_exit()
{
    // restore last gains
    plopter.autotune_restore();
}

void ModeAutoTune::update()
{
    plopter.mode_fbwa.update();
}

