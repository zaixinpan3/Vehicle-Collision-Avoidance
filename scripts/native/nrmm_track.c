#include "nrmm_track.h"
#include "nrmmTargetSample.h"
#include "nrmmTargetSample_emxAPI.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

struct nrmm_track { struct0_T state; };

nrmm_track *nrmm_track_create(void)
{
    nrmm_track *track = calloc(1, sizeof(*track));
    if (track) {
        emxInit_struct0_T(&track->state);
        track->state.times->size[0] = 1;
        track->state.positions->size[0] = 2;
        track->state.positionRadii->size[0] = 1;
    }
    return track;
}

void nrmm_track_destroy(nrmm_track *track)
{
    if (track) {
        emxDestroy_struct0_T(track->state);
        free(track);
    }
}

int nrmm_track_update(nrmm_track *track, double time, const double position[2],
                      double ego_heading, double relative_heading,
                      double position_radius, nrmm_estimate *estimate)
{
    if (!track || !estimate || !isfinite(time) || !isfinite(ego_heading) ||
        isinf(relative_heading) || !isfinite(position_radius) || position_radius < 0 ||
        (position && (!isfinite(position[0]) || !isfinite(position[1]))))
        return -1;
    struct0_T *s = &track->state;
    if (s->detections && time < s->time) return -1;
    memset(estimate, 0, sizeof(*estimate));
    if (!s->detections && !position) return 1;
    int size[2] = { position ? 2 : 0, position ? 1 : 0 }, motion_size[2];
    double missing[2] = {0};
    nrmmTargetSample(s, time, position ? position : missing, size, ego_heading,
                     relative_heading, position_radius, estimate->motion, motion_size);
    memcpy(estimate->state, s->state, sizeof(s->state));
    memcpy(estimate->position_bounds, s->positionBounds.data, sizeof(estimate->position_bounds));
    estimate->initialized = s->initialized;
    estimate->motion_available = motion_size[0] * motion_size[1] == 4;
    estimate->bounds_available = s->motionBounds.size[0] * s->motionBounds.size[1] == 4;
    if (estimate->bounds_available)
        memcpy(estimate->motion_bounds, s->motionBounds.data, sizeof(estimate->motion_bounds));
    if (s->errorBound.size[0] && s->errorBound.size[1])
        memcpy(estimate->component_radii, s->errorBound.data[0].targetComponents,
               sizeof(estimate->component_radii));
    estimate->rest_compatible = s->restCompatible;
    if (s->restBounds.size[0] && s->restBounds.size[1])
        memcpy(estimate->rest_bounds, s->restBounds.data, sizeof(estimate->rest_bounds));
    return 0;
}
