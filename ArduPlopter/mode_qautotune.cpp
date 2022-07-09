#include "mode.h"
#include "Plopter.h"

#include "qautotune.h"

#if QAUTOTUNE_ENABLED

bool ModeQAutotune::_enter()
{
#if QAUTOTUNE_ENABLED
    return quadplopter.qautotune.init();
#else
    return false;
#endif
}

void ModeQAutotune::update()
{
    plopter.mode_qstabilize.update();
}

void ModeQAutotune::run()
{
#if QAUTOTUNE_ENABLED
    quadplopter.qautotune.run();
#endif
}

void ModeQAutotune::_exit()
{
#if QAUTOTUNE_ENABLED
    plopter.quadplopter.qautotune.stop();
#endif
}

#endif
