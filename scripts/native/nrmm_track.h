#ifndef NRMM_TRACK_H
#define NRMM_TRACK_H
#ifdef __cplusplus
extern "C" {
#endif

typedef struct nrmm_track nrmm_track;
typedef struct {
    double state[6];               /* Inertial Cartesian position, velocity, acceleration. */
    double motion[4];              /* Heading, speed, tangential acceleration, sideslip. */
    double position_bounds[4];     /* Column-major 2-by-2 [lower,upper]. */
    double motion_bounds[4];       /* Rows are lower and upper, columns are acceleration and slip. */
    double component_radii[3];
    double rest_bounds[4];
    int initialized, motion_available, bounds_available, rest_compatible;
} nrmm_estimate;

nrmm_track *nrmm_track_create(void);
void nrmm_track_destroy(nrmm_track *track);
/* One handle per target and ego hypothesis. NULL position denotes a missing return.
   Position is in the inertial frame. Pass NAN for unavailable relative heading.
   Returns 0 on success, 1 when waiting for the first detection, -1 on invalid input. */
int nrmm_track_update(nrmm_track *track, double time, const double position[2],
                      double ego_heading, double relative_heading,
                      double position_radius, nrmm_estimate *estimate);
#ifdef __cplusplus
}
#endif
#endif
