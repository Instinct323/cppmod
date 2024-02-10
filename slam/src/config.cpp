#include "frontend.h"
#include "mappoint.h"

int Frontend::nfeats_max = 200;
int Frontend::nfeats_init = 100;
int Frontend::nfeats_track_bad = 20;
int Frontend::nfeats_track_good = 80;

double Mappoint::z_floor = 0.;
