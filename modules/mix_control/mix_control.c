#include "mix_control.h"

#if AIRCRAFT_TYPE == AIRCRAFT_QUADCOPTER
#include "quadcopter.h"
#elif AIRCRAFT_TYPE == AIRCRAFT_BICOPTER
#include "bicopter.h"
#else
#error "Unknown AIRCRAFT_TYPE — define in mix_control.h"
#endif

void mix_control_setup(void) {
#if AIRCRAFT_TYPE == AIRCRAFT_QUADCOPTER
	quadcopter_setup();
#elif AIRCRAFT_TYPE == AIRCRAFT_BICOPTER
	bicopter_setup();
#endif
}
