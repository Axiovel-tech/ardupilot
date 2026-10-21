/*
 * Reader for the .gdmb per-drone show payload, format version 1.
 *
 * The contract is AxioStack contracts/gdm-format.md. This reader was written
 * from that text alone (ADR 0002, decision D1): it does not share code with
 * the engine's writer or with the ground validator.
 *
 * - gdmb_show_parse() validates a whole file against every rule of spec §7.1
 *   (fail closed: on any violation nothing is loaded) and builds a segment
 *   index over the caller's buffer. The buffer must outlive the show.
 * - gdmb_show_evaluate() returns the reference of spec §6 at a show time:
 *   position, velocity, acceleration and jerk from the per-axis Bézier curves
 *   of FLAT segments, yaw and yaw rate, and for MANEUVER segments the
 *   attitude, body rates and thrust of the SO(3) B-spline.
 *
 * Vehicle policy (which capabilities are enabled) is the caller's business:
 * the parser accepts every capability that v1 defines.
 *
 * v1 conformance covers FLAT trajectories, LIGHTS, EVENTS and COMMENT. The
 * MANEUVER, RTH_PLAN and ENVELOPES layouts are provisional (spec §4.11): a v1
 * reader refuses them, and gdmb_show_parse() does. gdmb_show_parse_ex() with
 * GDMB_PARSE_PROVISIONAL also accepts and evaluates them, for work on the
 * later phases; flight firmware must not use it.
 *
 * This file is plain C99 with no ArduPilot dependency so that another flight
 * firmware can reuse it.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Floating-point type used for evaluation. binary32 is acceptable on the
 * vehicle (spec §6); integer differences are always formed first. */
#ifdef GDMB_USE_FLOAT
typedef float gdmb_real_t;
#else
typedef double gdmb_real_t;
#endif

/* Highest .gdmb version this reader implements; older versions are accepted (spec §1.8) */
#define GDMB_READER_VERSION 1u

/* Minimum header length; a longer header is a later version's extension (spec §4.1) */
#define GDMB_HEADER_LENGTH 48u
#define GDMB_MAX_FILE_SIZE 262144u

/* Feature flags, header byte 5 (spec §4.2) */
#define GDMB_FEATURE_CRC32 0x01u
#define GDMB_FEATURE_MANEUVER 0x02u /* provisional (spec §4.11) */
#define GDMB_FEATURE_REVERSIBLE_THRUST 0x04u

/* Capability bits, header offset 44 (spec §4.3) */
#define GDMB_CAP_MANEUVER 0x01u /* provisional (spec §4.11) */
#define GDMB_CAP_REVERSIBLE_THRUST 0x02u
#define GDMB_CAP_HIGH_TILT 0x04u

/* Error codes; the names match the stable identifiers of spec §7.1 */
typedef enum {
    GDMB_OK = 0,
    GDMB_E_SIZE,
    GDMB_E_MAGIC,
    GDMB_E_VERSION,
    GDMB_E_HEADER,
    GDMB_E_CRC,
    GDMB_E_RESERVED,
    GDMB_E_FEATURES,
    GDMB_E_CAPS,
    GDMB_E_SHOW_HASH,
    GDMB_E_FRAMING,
    GDMB_E_BLOCK_TYPE,
    GDMB_E_BLOCK_ORDER,
    GDMB_E_DEGREE,
    GDMB_E_DURATION,
    GDMB_E_SIGMA,
    GDMB_E_MANEUVER,
    GDMB_E_QUAT,
    GDMB_E_THRUST,
    GDMB_E_ABORT,
    GDMB_E_ENVELOPE,
    GDMB_E_EVENTS,
    GDMB_E_LIGHTS,
    GDMB_E_RTH,
    GDMB_E_CONTINUITY,
    GDMB_E_C2,
    GDMB_E_REGIME,
    /* Not a format error: the reader could not allocate its index */
    GDMB_E_NOMEM,
    /* Not a format error: invalid argument (for example, no show loaded) */
    GDMB_E_INVALID,
} gdmb_error_t;

/* Details about the first violation found by gdmb_show_parse() */
typedef struct {
    gdmb_error_t code;
    /* Byte offset in the file where the violation was found (best effort) */
    uint32_t offset;
    /* Short static description of the rule that failed */
    const char* what;
} gdmb_diag_t;

/* One entry of the timeline index: a FLAT segment or a whole MANEUVER */
typedef struct {
    uint32_t t0_us;      /* show time at which the entry starts */
    uint32_t dur_us;     /* duration */
    uint32_t offset;     /* FLAT: offset of the control values; MANEUVER: offset of the payload */
    int32_t p0[4];       /* implicit start: x, y, z (0.1 mm), yaw (µrad; FLAT only) */
    uint8_t deg[4];      /* FLAT: degrees of x, y, z, yaw; MANEUVER: degrees of p_hat x, y, z */
    int8_t sigma;        /* FLAT: +1 or -1; MANEUVER: 0 */
    uint8_t is_maneuver;
} gdmb_segment_t;

typedef struct {
    const uint8_t* data;
    uint32_t size;
    uint16_t header_length;   /* blocks start here */

    uint8_t features;
    uint32_t capabilities;
    const uint8_t* show_hash; /* 32 bytes inside data */

    gdmb_segment_t* segments; /* timeline order */
    uint32_t num_segments;
    uint32_t num_maneuvers;
    uint32_t duration_us;     /* sum of all durations */

    int32_t start[4];         /* state at t = 0 */
    int32_t end[4];           /* state at the end of the timeline */
    int8_t start_sigma;
    int8_t end_sigma;

    const uint8_t* lights;    /* ledctrl bytecode, NULL if absent */
    uint32_t lights_length;

    const uint8_t* events;    /* 10-byte records, NULL if absent */
    uint32_t num_events;

    const uint8_t* envelopes; /* 12-byte records, NULL if absent */
    uint32_t num_envelopes;

    const uint8_t* rth_plan;  /* RTH_PLAN payload (validated), NULL if absent */
    uint32_t rth_plan_length;
    uint16_t num_rth_entries;

    const uint8_t* comment;   /* NULL if absent */
    uint32_t comment_length;
} gdmb_show_t;

typedef enum {
    GDMB_REGIME_BEFORE = 0,   /* t < 0: start state, at rest */
    GDMB_REGIME_FLAT = 1,
    GDMB_REGIME_MANEUVER = 2,
    GDMB_REGIME_HOLD = 3,     /* t >= end: final state, at rest */
} gdmb_regime_t;

/* Reference state at a show time. Units: m, m/s, m/s², m/s³, rad, rad/s. */
typedef struct {
    gdmb_regime_t regime;
    uint32_t segment;          /* index into segments[] */
    int8_t sigma;              /* 0 inside a maneuver */
    gdmb_real_t p[3], v[3], a[3], j[3];
    gdmb_real_t yaw, yaw_rate; /* clockwise-positive from the X axis (spec §1.2); 0 inside a maneuver */
    /* MANEUVER only (zero otherwise): attitude [w, x, y, z], body rates,
     * body angular acceleration and specific thrust along +z_b */
    gdmb_real_t q[4], omega[3], omega_dot[3], thrust;
} gdmb_state_t;

typedef struct {
    uint32_t t_us;
    uint8_t type;
    uint8_t subtype;
    uint32_t payload;
} gdmb_event_t;

void gdmb_show_init(gdmb_show_t* show);
void gdmb_show_free(gdmb_show_t* show);

/* Validates data[0..size) against spec §7.1 (except E_SHOW_HASH, which is a
 * ground check) and indexes it. On failure the show is left empty and diag,
 * if not NULL, says why. */
gdmb_error_t gdmb_show_parse(gdmb_show_t* show, const uint8_t* data, size_t size, gdmb_diag_t* diag);

/* Flags of gdmb_show_parse_ex() */
#define GDMB_PARSE_PROVISIONAL 0x01u /* also accept the provisional layouts (spec §4.11) */

gdmb_error_t gdmb_show_parse_ex(gdmb_show_t* show, const uint8_t* data, size_t size,
    uint32_t flags, gdmb_diag_t* diag);

bool gdmb_show_is_loaded(const gdmb_show_t* show);

/* Reference at show time t_us (spec §6.1) */
gdmb_error_t gdmb_show_evaluate(const gdmb_show_t* show, int64_t t_us, gdmb_state_t* out);

bool gdmb_show_get_event(const gdmb_show_t* show, uint32_t index, gdmb_event_t* out);

/* Show time at which the vehicle should hand over to its automatic landing,
 * in the convention of libskybrush's trajectory statistics: the final run of
 * the timeline that only descends and stays within xy_threshold_m of the end
 * position horizontally is "vertical"; landing starts preferred_descent_m above
 * the end of that run, or at its start if the run is shorter. Evaluated on the
 * curve itself (sampled every step_us), so a slow sideways creep during the
 * final descent is not mistaken for a vertical descent. */
gdmb_error_t gdmb_show_landing_time(const gdmb_show_t* show, gdmb_real_t xy_threshold_m,
    gdmb_real_t preferred_descent_m, uint32_t step_us, uint32_t* landing_us);

/* True if some FLAT segment of the timeline has sigma = -1 */
bool gdmb_show_has_inverted_flat(const gdmb_show_t* show);

/* Flatness map of spec §6.4: rotation matrix (body to world, R[row][col]),
 * body rates and signed specific thrust. psi is the clockwise-positive
 * heading of spec §1.2. */
void gdmb_flat_attitude(const gdmb_real_t a[3], const gdmb_real_t j[3], gdmb_real_t psi,
    gdmb_real_t psi_dot, int sigma, gdmb_real_t R[3][3], gdmb_real_t omega[3], gdmb_real_t* thrust);

/* Format of a file in the show slot, by its first four bytes */
typedef enum {
    GDMB_SHOW_FILE_UNKNOWN = 0,
    GDMB_SHOW_FILE_GDMB,     /* "GDMB" */
    GDMB_SHOW_FILE_SKYB,     /* "skyb", the libskybrush binary format */
} gdmb_show_file_format_t;

gdmb_show_file_format_t gdmb_detect_show_file_format(const uint8_t* data, size_t size);

/* True if a .skyb header declares a CRC: version 2 with the CRC32 feature
 * bit. libskybrush verifies the CRC only in that case. */
bool gdmb_skyb_header_has_crc(const uint8_t* data, size_t size);

/* AP-CRC32 (spec §4.9): reflected 0x04C11DB7, initial register `crc`, no final XOR */
uint32_t gdmb_crc32(uint32_t crc, const uint8_t* buf, size_t len);

const char* gdmb_error_name(gdmb_error_t code);

#ifdef __cplusplus
}
#endif
