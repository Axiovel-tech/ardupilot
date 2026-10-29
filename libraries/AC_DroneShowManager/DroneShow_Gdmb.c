/*
 * Reader for the .gdmb per-drone show payload, format version 1.
 * See DroneShow_Gdmb.h and AxioStack contracts/gdm-format.md.
 *
 * Section references (§) are to that document.
 *
 * Note on constants: ArduPilot compiles with -fsingle-precision-constant, so
 * an unsuffixed literal such as 1e-3 would be a float. Every non-trivial real
 * constant below is therefore written as a ratio of integers (GDMB_R), which
 * is evaluated in gdmb_real_t.
 */

#include "DroneShow_Gdmb.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef GDMB_CALLOC
#define GDMB_CALLOC(n, size) calloc((n), (size))
#define GDMB_FREE(ptr) free(ptr)
#endif

#define GDMB_R(num, den) ((gdmb_real_t)(num) / (gdmb_real_t)(den))

#define POS_SCALE 10000      /* 0.1 mm */
#define YAW_SCALE 1000000    /* 1 µrad */
#define THRUST_SCALE 1000    /* 1 mm/s² */
#define QUAT_SCALE 32767
#define US_PER_SEC 1000000
#define MIN_FLAT_DURATION_US 100000u /* spec §4.5 */
#define MIN_KNOT_US 1000u            /* provisional MANEUVER knot spacing */

#define G_MSS GDMB_R(980665, 100000) /* §1.1, ArduPilot's GRAVITY_MSS */

#define BLOCK_FLAT 0x01
#define BLOCK_LIGHTS 0x02
#define BLOCK_COMMENT 0x03
#define BLOCK_RTH_PLAN 0x04
#define BLOCK_YAW_RESERVED 0x05
#define BLOCK_EVENTS 0x06
#define BLOCK_MANEUVER 0x07
#define BLOCK_ENVELOPES 0x08
#define BLOCK_FLAG_SKIPPABLE 0x01

#define FLAT_START_SIZE 16u
#define MANEUVER_FIXED_SIZE 32u
#define EVENT_SIZE 10u
#define ENVELOPE_SIZE 12u
#define RTH_HEADER_SIZE 14u
#define RTH_ENTRY_HEADER_SIZE 6u

#define ABORT_LEVEL_HOLD_LAND 1

/* ------------------------------------------------------------------------ */
/* Byte access (§1.6: little-endian, unaligned)                             */
/* ------------------------------------------------------------------------ */

static uint16_t rd_u16(const uint8_t* p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t rd_u32(const uint8_t* p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t rd_i32(const uint8_t* p)
{
    uint32_t u = rd_u32(p);
    int32_t v;
    memcpy(&v, &u, sizeof(v));
    return v;
}

static int16_t rd_i16(const uint8_t* p)
{
    uint16_t u = rd_u16(p);
    int16_t v;
    memcpy(&v, &u, sizeof(v));
    return v;
}

/* ------------------------------------------------------------------------ */
/* CRC (§4.9)                                                               */
/* ------------------------------------------------------------------------ */

uint32_t gdmb_crc32(uint32_t crc, const uint8_t* buf, size_t len)
{
    /* Nibble-wise table of the reflected polynomial 0xEDB88320 */
    static const uint32_t table[16] = {
        0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu,
        0x76DC4190u, 0x6B6B51F4u, 0x4DB26158u, 0x5005713Cu,
        0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu,
        0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu,
    };
    size_t i;

    for (i = 0; i < len; i++) {
        crc ^= buf[i];
        crc = table[crc & 0x0F] ^ (crc >> 4);
        crc = table[crc & 0x0F] ^ (crc >> 4);
    }

    return crc;
}

/* ------------------------------------------------------------------------ */
/* Show slot format                                                         */
/* ------------------------------------------------------------------------ */

gdmb_show_file_format_t gdmb_detect_show_file_format(const uint8_t* data, size_t size)
{
    if (data == NULL || size < 4) {
        return GDMB_SHOW_FILE_UNKNOWN;
    }
    if (memcmp(data, "GDMB", 4) == 0) {
        return GDMB_SHOW_FILE_GDMB;
    }
    if (memcmp(data, "skyb", 4) == 0) {
        return GDMB_SHOW_FILE_SKYB;
    }
    return GDMB_SHOW_FILE_UNKNOWN;
}

bool gdmb_skyb_header_has_crc(const uint8_t* data, size_t size)
{
    /* "skyb", version, features, CRC32 */
    return gdmb_detect_show_file_format(data, size) == GDMB_SHOW_FILE_SKYB &&
           size >= 10 && data[4] == 2 && (data[5] & 0x01) != 0;
}

/* ------------------------------------------------------------------------ */
/* Error names                                                              */
/* ------------------------------------------------------------------------ */

const char* gdmb_error_name(gdmb_error_t code)
{
    switch (code) {
    case GDMB_OK: return "OK";
    case GDMB_E_SIZE: return "E_SIZE";
    case GDMB_E_MAGIC: return "E_MAGIC";
    case GDMB_E_VERSION: return "E_VERSION";
    case GDMB_E_HEADER: return "E_HEADER";
    case GDMB_E_CRC: return "E_CRC";
    case GDMB_E_RESERVED: return "E_RESERVED";
    case GDMB_E_FEATURES: return "E_FEATURES";
    case GDMB_E_CAPS: return "E_CAPS";
    case GDMB_E_SHOW_HASH: return "E_SHOW_HASH";
    case GDMB_E_FRAMING: return "E_FRAMING";
    case GDMB_E_BLOCK_TYPE: return "E_BLOCK_TYPE";
    case GDMB_E_BLOCK_ORDER: return "E_BLOCK_ORDER";
    case GDMB_E_DEGREE: return "E_DEGREE";
    case GDMB_E_DURATION: return "E_DURATION";
    case GDMB_E_SIGMA: return "E_SIGMA";
    case GDMB_E_MANEUVER: return "E_MANEUVER";
    case GDMB_E_QUAT: return "E_QUAT";
    case GDMB_E_THRUST: return "E_THRUST";
    case GDMB_E_ABORT: return "E_ABORT";
    case GDMB_E_ENVELOPE: return "E_ENVELOPE";
    case GDMB_E_EVENTS: return "E_EVENTS";
    case GDMB_E_LIGHTS: return "E_LIGHTS";
    case GDMB_E_RTH: return "E_RTH";
    case GDMB_E_CONTINUITY: return "E_CONTINUITY";
    case GDMB_E_C2: return "E_C2";
    case GDMB_E_REGIME: return "E_REGIME";
    case GDMB_E_REST: return "E_REST";
    case GDMB_E_NOMEM: return "E_NOMEM";
    case GDMB_E_INVALID: return "E_INVALID";
    }
    return "E_UNKNOWN";
}

/* ------------------------------------------------------------------------ */
/* Parsing helpers                                                          */
/* ------------------------------------------------------------------------ */

/* Bounded reader over [pos, end) of the file */
typedef struct {
    const uint8_t* data;
    uint32_t pos;
    uint32_t end;
} reader_t;

static bool rd_has(const reader_t* r, uint32_t n)
{
    return r->end - r->pos >= n;
}

#define FAIL(diag, err, off, msg)       \
    do {                                \
        if ((diag) != NULL) {           \
            (diag)->code = (err);       \
            (diag)->offset = (off);     \
            (diag)->what = (msg);       \
        }                               \
        return (err);                   \
    } while (0)

/* A FLAT segment record (§4.5), located but not copied */
typedef struct {
    uint32_t dur_us;
    uint32_t values; /* offset of the control values */
    uint8_t deg[4];
    int8_t sigma;
} flat_record_t;

/* Reads the record at r and advances past it. The header is validated before
 * the control values are consumed (§4.5). */
static gdmb_error_t read_flat_record(reader_t* r, bool reversible, flat_record_t* rec, gdmb_diag_t* diag)
{
    uint16_t header;
    uint32_t count;
    uint32_t at = r->pos;

    if (!rd_has(r, 2)) {
        FAIL(diag, GDMB_E_FRAMING, at, "segment record truncated");
    }
    header = rd_u16(r->data + r->pos);
    if (header >> 13) {
        FAIL(diag, GDMB_E_RESERVED, at, "segment header reserved bits 13-15 set");
    }
    rec->deg[0] = header & 7;
    rec->deg[1] = (header >> 3) & 7;
    rec->deg[2] = (header >> 6) & 7;
    rec->deg[3] = (header >> 9) & 7;
    if (rec->deg[3] > 5) {
        FAIL(diag, GDMB_E_DEGREE, at, "yaw degree above 5");
    }
    rec->sigma = ((header >> 12) & 1) ? -1 : 1;
    if (rec->sigma < 0 && !reversible) {
        FAIL(diag, GDMB_E_SIGMA, at, "sigma = -1 without REVERSIBLE_THRUST");
    }
    r->pos += 2;

    if (!rd_has(r, 4)) {
        FAIL(diag, GDMB_E_FRAMING, at, "segment record truncated");
    }
    rec->dur_us = rd_u32(r->data + r->pos);
    if (rec->dur_us < MIN_FLAT_DURATION_US) {
        FAIL(diag, GDMB_E_DURATION, at, "segment shorter than 100 ms");
    }
    r->pos += 4;

    count = 4u * ((uint32_t)rec->deg[0] + rec->deg[1] + rec->deg[2] + rec->deg[3]);
    if (!rd_has(r, count)) {
        FAIL(diag, GDMB_E_FRAMING, at, "segment record truncated");
    }
    rec->values = r->pos;
    r->pos += count;

    return GDMB_OK;
}

/* Loads the control values of one axis, including the implicit start P0.
 * Returns the degree. */
static int load_axis(const uint8_t* data, uint32_t values, const uint8_t deg[4], int axis, int32_t p0, int64_t c[8])
{
    uint32_t offset = values;
    int i, n = deg[axis];

    for (i = 0; i < axis; i++) {
        offset += 4u * deg[i];
    }

    c[0] = p0;
    for (i = 1; i <= n; i++) {
        c[i] = rd_i32(data + offset + 4u * (uint32_t)(i - 1));
    }

    return n;
}

/* End value of one axis of a record (the last control value, or the start) */
static int32_t axis_end(const uint8_t* data, uint32_t values, const uint8_t deg[4], int axis, int32_t p0)
{
    int64_t c[8];
    int n = load_axis(data, values, deg, axis, p0, c);
    return (int32_t)c[n];
}

/* Value and the first three time derivatives of a Bézier (§6.2), in SI units.
 * c holds the integer control values P0..Pn, inv_scale converts them to SI,
 * s is in [0, 1] and T is the duration in seconds. */
static void bezier_eval(const int64_t* c, int n, gdmb_real_t s, gdmb_real_t T, gdmb_real_t inv_scale, gdmb_real_t out[4])
{
    int64_t d[8];
    gdmb_real_t b[8];
    gdmb_real_t factor = 1;
    gdmb_real_t tpow = 1;
    int i, k, m, len;

    for (i = 0; i <= n; i++) {
        d[i] = c[i];
    }

    for (m = 0; m < 4; m++) {
        len = n - m + 1;
        if (len <= 0) {
            out[m] = 0;
            continue;
        }

        /* de Casteljau on the m-th forward differences, which are exact integers */
        for (i = 0; i < len; i++) {
            b[i] = (gdmb_real_t)d[i];
        }
        for (k = 1; k < len; k++) {
            for (i = 0; i < len - k; i++) {
                b[i] = (1 - s) * b[i] + s * b[i + 1];
            }
        }
        out[m] = b[0] * factor * inv_scale / tpow;

        /* next order: n!/(n-m-1)!, one more forward difference */
        factor *= (gdmb_real_t)(n - m);
        tpow *= T;
        for (i = 0; i < len - 1; i++) {
            d[i] = d[i + 1] - d[i];
        }
    }
}

/* Endpoint velocity and acceleration of one axis (§6.3) */
static void endpoint_derivs(const int64_t* c, int n, gdmb_real_t T, gdmb_real_t inv_scale, bool at_end, gdmb_real_t* v, gdmb_real_t* a)
{
    int64_t dv = 0, da = 0;

    if (n >= 1) {
        dv = at_end ? c[n] - c[n - 1] : c[1] - c[0];
    }
    if (n >= 2) {
        da = at_end ? c[n] - 2 * c[n - 1] + c[n - 2] : c[2] - 2 * c[1] + c[0];
    }

    *v = (gdmb_real_t)n * (gdmb_real_t)dv * inv_scale / T;
    *a = (gdmb_real_t)n * (gdmb_real_t)(n > 0 ? n - 1 : 0) * (gdmb_real_t)da * inv_scale / (T * T);
}

/* §7.3 FLAT ↔ FLAT check between record a (starting at pa) and record b
 * (starting at pb), over `axes` axes (4 for FLAT, 3 when one side is p_hat). */
static bool c2_joint_ok(const uint8_t* data,
    uint32_t values_a, const uint8_t deg_a[4], uint32_t dur_a, const int32_t pa[4],
    uint32_t values_b, const uint8_t deg_b[4], uint32_t dur_b, const int32_t pb[4],
    int axes)
{
    const gdmb_real_t Ta = (gdmb_real_t)dur_a / US_PER_SEC;
    const gdmb_real_t Tb = (gdmb_real_t)dur_b / US_PER_SEC;
    int k;

    for (k = 0; k < axes; k++) {
        int64_t ca[8], cb[8];
        gdmb_real_t va, aa, vb, ab, q, tol_v, tol_a;
        int na = load_axis(data, values_a, deg_a, k, pa[k], ca);
        int nb = load_axis(data, values_b, deg_b, k, pb[k], cb);
        const bool is_yaw = (k == 3);
        const gdmb_real_t inv_scale = is_yaw ? GDMB_R(1, YAW_SCALE) : GDMB_R(1, POS_SCALE);

        endpoint_derivs(ca, na, Ta, inv_scale, true, &va, &aa);
        endpoint_derivs(cb, nb, Tb, inv_scale, false, &vb, &ab);

        q = inv_scale;
        tol_v = q * ((gdmb_real_t)na / Ta + (gdmb_real_t)nb / Tb) + GDMB_R(1, 1000);
        if (fabs(va - vb) > tol_v) {
            return false;
        }

        if (is_yaw) {
            /* yaw is C¹ only (ADR §1) */
            continue;
        }

        tol_a = 2 * q * ((gdmb_real_t)(na * (na - 1)) / (Ta * Ta) + (gdmb_real_t)(nb * (nb - 1)) / (Tb * Tb)) + GDMB_R(1, 100);
        if (fabs(aa - ab) > tol_a) {
            return false;
        }
    }

    return true;
}

/* ------------------------------------------------------------------------ */
/* Vector and quaternion helpers                                            */
/* ------------------------------------------------------------------------ */

static gdmb_real_t vdot(const gdmb_real_t a[3], const gdmb_real_t b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void vcross(const gdmb_real_t a[3], const gdmb_real_t b[3], gdmb_real_t out[3])
{
    gdmb_real_t r[3];
    r[0] = a[1] * b[2] - a[2] * b[1];
    r[1] = a[2] * b[0] - a[0] * b[2];
    r[2] = a[0] * b[1] - a[1] * b[0];
    memcpy(out, r, sizeof(r));
}

static gdmb_real_t vnorm(const gdmb_real_t a[3])
{
    return sqrt(vdot(a, a));
}

/* Hamilton product q ⊗ r, [w, x, y, z] */
static void qmul(const gdmb_real_t q[4], const gdmb_real_t r[4], gdmb_real_t out[4])
{
    gdmb_real_t o[4];
    o[0] = q[0] * r[0] - q[1] * r[1] - q[2] * r[2] - q[3] * r[3];
    o[1] = q[0] * r[1] + q[1] * r[0] + q[2] * r[3] - q[3] * r[2];
    o[2] = q[0] * r[2] - q[1] * r[3] + q[2] * r[0] + q[3] * r[1];
    o[3] = q[0] * r[3] + q[1] * r[2] - q[2] * r[1] + q[3] * r[0];
    memcpy(out, o, sizeof(o));
}

/* Exp of a rotation vector (§1.3) */
static void qexp(const gdmb_real_t phi[3], gdmb_real_t out[4])
{
    gdmb_real_t angle = vnorm(phi);
    gdmb_real_t k;

    if (!(angle > 0)) {
        out[0] = 1;
        out[1] = out[2] = out[3] = 0;
        return;
    }

    k = sin(angle / 2) / angle;
    out[0] = cos(angle / 2);
    out[1] = phi[0] * k;
    out[2] = phi[1] * k;
    out[3] = phi[2] * k;
}

/* Log of a unit quaternion with w > 0 (§1.3) */
static void qlog(const gdmb_real_t q[4], gdmb_real_t out[3])
{
    gdmb_real_t n = sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    gdmb_real_t k;

    if (!(n > 0)) {
        out[0] = out[1] = out[2] = 0;
        return;
    }

    k = 2 * atan2(n, q[0]) / n;
    out[0] = q[1] * k;
    out[1] = q[2] * k;
    out[2] = q[3] * k;
}

/* Rotation matrix of a unit quaternion; columns are the body axes in world */
static void qmat(const gdmb_real_t q[4], gdmb_real_t R[3][3])
{
    const gdmb_real_t w = q[0], x = q[1], y = q[2], z = q[3];
    R[0][0] = 1 - 2 * (y * y + z * z);
    R[0][1] = 2 * (x * y - w * z);
    R[0][2] = 2 * (x * z + w * y);
    R[1][0] = 2 * (x * y + w * z);
    R[1][1] = 1 - 2 * (x * x + z * z);
    R[1][2] = 2 * (y * z - w * x);
    R[2][0] = 2 * (x * z - w * y);
    R[2][1] = 2 * (y * z + w * x);
    R[2][2] = 1 - 2 * (x * x + y * y);
}

/* Mᵀ v */
static void mat_t_vec(const gdmb_real_t M[3][3], const gdmb_real_t v[3], gdmb_real_t out[3])
{
    gdmb_real_t r[3];
    int i;
    for (i = 0; i < 3; i++) {
        r[i] = M[0][i] * v[0] + M[1][i] * v[1] + M[2][i] * v[2];
    }
    memcpy(out, r, sizeof(r));
}

/* ∠(A, B) = acos((tr(AᵀB) − 1) / 2) */
static gdmb_real_t rotation_angle(const gdmb_real_t A[3][3], const gdmb_real_t B[3][3])
{
    gdmb_real_t tr = 0, c;
    int i, j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            tr += A[i][j] * B[i][j];
        }
    }
    c = (tr - 1) / 2;
    if (c > 1) {
        c = 1;
    } else if (c < -1) {
        c = -1;
    }
    return acos(c);
}

/* ------------------------------------------------------------------------ */
/* Flatness map (§6.4)                                                      */
/* ------------------------------------------------------------------------ */

void gdmb_flat_attitude(const gdmb_real_t a[3], const gdmb_real_t j[3], gdmb_real_t psi,
    gdmb_real_t psi_dot, int sigma, gdmb_real_t R[3][3], gdmb_real_t omega[3], gdmb_real_t* thrust)
{
    gdmb_real_t f[3], fh[3], zb[3], zbd[3], xc[3], xcd[3];
    gdmb_real_t y[3], yd[3], t1[3], t2[3], yb[3], ybd[3], xb[3], xbd[3];
    gdmb_real_t fn, yn, fhj, ybyd;
    const gdmb_real_t sg = (gdmb_real_t)sigma;
    int i;

    f[0] = a[0];
    f[1] = a[1];
    f[2] = a[2] + G_MSS;
    fn = vnorm(f);
    for (i = 0; i < 3; i++) {
        fh[i] = f[i] / fn;
        zb[i] = sg * fh[i];
    }

    /* ż_b = σ (j − (f̂·j) f̂) / ‖f‖ */
    fhj = vdot(fh, j);
    for (i = 0; i < 3; i++) {
        zbd[i] = sg * (j[i] - fhj * fh[i]) / fn;
    }

    /* clockwise heading (spec §1.2): psi = +pi/2 points along -y (east) */
    xc[0] = cos(psi);
    xc[1] = -sin(psi);
    xc[2] = 0;
    xcd[0] = -sin(psi) * psi_dot;
    xcd[1] = -cos(psi) * psi_dot;
    xcd[2] = 0;

    vcross(zb, xc, y);
    vcross(zbd, xc, t1);
    vcross(zb, xcd, t2);
    for (i = 0; i < 3; i++) {
        yd[i] = t1[i] + t2[i];
    }
    yn = vnorm(y);
    for (i = 0; i < 3; i++) {
        yb[i] = y[i] / yn;
    }
    ybyd = vdot(yb, yd);
    for (i = 0; i < 3; i++) {
        ybd[i] = (yd[i] - ybyd * yb[i]) / yn;
    }

    vcross(yb, zb, xb);
    vcross(ybd, zb, t1);
    vcross(yb, zbd, t2);
    for (i = 0; i < 3; i++) {
        xbd[i] = t1[i] + t2[i];
    }

    for (i = 0; i < 3; i++) {
        R[i][0] = xb[i];
        R[i][1] = yb[i];
        R[i][2] = zb[i];
    }
    omega[0] = vdot(ybd, zb);
    omega[1] = vdot(zbd, xb);
    omega[2] = vdot(xbd, yb);
    *thrust = sg * fn;
}

/* ------------------------------------------------------------------------ */
/* MANEUVER (§4.6, §6.5, §6.6)                                              */
/* ------------------------------------------------------------------------ */

/* Fixed part of a MANEUVER payload */
typedef struct {
    uint32_t dur_us;
    uint32_t knot_us;
    uint16_t n;
    uint16_t env_pos_mm;
    uint16_t env_att_mrad;
    uint8_t abort_action;
    uint8_t reserved;
    int32_t start[3];
    int32_t yaw_exit;
    flat_record_t phat;
    uint32_t quats;   /* offset of q_0 */
    uint32_t thrusts; /* offset of f_0 */
} maneuver_t;

/* Structural parse and per-block checks of a MANEUVER payload at r */
static gdmb_error_t read_maneuver(reader_t* r, bool reversible, maneuver_t* m, gdmb_diag_t* diag)
{
    const uint8_t* p = r->data + r->pos;
    const uint32_t at = r->pos;
    gdmb_error_t err;
    uint32_t k;
    int c;
    bool any_nonzero;

    if (!rd_has(r, MANEUVER_FIXED_SIZE)) {
        FAIL(diag, GDMB_E_FRAMING, at, "MANEUVER payload truncated");
    }
    m->dur_us = rd_u32(p);
    m->knot_us = rd_u32(p + 4);
    m->n = rd_u16(p + 8);
    m->env_pos_mm = rd_u16(p + 10);
    m->env_att_mrad = rd_u16(p + 12);
    m->abort_action = p[14];
    m->reserved = p[15];
    m->start[0] = rd_i32(p + 16);
    m->start[1] = rd_i32(p + 20);
    m->start[2] = rd_i32(p + 24);
    m->yaw_exit = rd_i32(p + 28);
    r->pos += MANEUVER_FIXED_SIZE;

    if (m->reserved) {
        FAIL(diag, GDMB_E_RESERVED, at + 15, "MANEUVER reserved byte set");
    }
    if (m->abort_action != ABORT_LEVEL_HOLD_LAND) {
        FAIL(diag, GDMB_E_ABORT, at + 14, "unknown abort action");
    }
    if (m->env_pos_mm == 0 || m->env_att_mrad == 0) {
        FAIL(diag, GDMB_E_ENVELOPE, at + 10, "zero maneuver envelope");
    }
    if (m->n < 4) {
        FAIL(diag, GDMB_E_MANEUVER, at + 8, "fewer than 4 control points");
    }
    if (m->knot_us < MIN_KNOT_US || (uint64_t)m->dur_us != (uint64_t)(m->n - 3) * m->knot_us) {
        FAIL(diag, GDMB_E_DURATION, at, "maneuver duration != (N - 3) * knot_us");
    }

    err = read_flat_record(r, reversible, &m->phat, diag);
    if (err != GDMB_OK) {
        return err;
    }
    if (m->phat.deg[3] != 0 || m->phat.sigma != 1 || m->phat.dur_us != m->dur_us) {
        FAIL(diag, GDMB_E_MANEUVER, at + MANEUVER_FIXED_SIZE, "p_hat needs yaw degree 0, sigma +1 and the maneuver duration");
    }

    if (!rd_has(r, 10u * m->n)) {
        FAIL(diag, GDMB_E_FRAMING, r->pos, "MANEUVER payload truncated");
    }
    m->quats = r->pos;
    m->thrusts = r->pos + 8u * m->n;
    r->pos += 10u * m->n;
    if (r->pos != r->end) {
        FAIL(diag, GDMB_E_FRAMING, r->pos, "trailing bytes in MANEUVER payload");
    }

    /* E_QUAT (§7.1) */
    for (k = 0; k < m->n; k++) {
        const uint8_t* q = r->data + m->quats + 8u * k;
        int64_t sq = 0;
        gdmb_real_t norm;
        for (c = 0; c < 4; c++) {
            int16_t v = rd_i16(q + 2 * c);
            if (v == -32768) {
                FAIL(diag, GDMB_E_QUAT, m->quats + 8u * k, "quaternion component -32768");
            }
            sq += (int64_t)v * v;
        }
        norm = sqrt((gdmb_real_t)sq) / QUAT_SCALE;
        if (norm < GDMB_R(999, 1000) || norm > GDMB_R(1001, 1000)) {
            FAIL(diag, GDMB_E_QUAT, m->quats + 8u * k, "quaternion norm outside [0.999, 1.001]");
        }
    }
    any_nonzero = false;
    for (c = 0; c < 4 && !any_nonzero; c++) {
        int16_t v = rd_i16(r->data + m->quats + 2 * c);
        if (v != 0) {
            any_nonzero = true;
            if (v < 0) {
                FAIL(diag, GDMB_E_QUAT, m->quats, "q_0 not in canonical sign");
            }
        }
    }
    for (k = 0; k + 1 < m->n; k++) {
        const uint8_t* q0 = r->data + m->quats + 8u * k;
        const uint8_t* q1 = q0 + 8;
        int64_t dot = 0;
        for (c = 0; c < 4; c++) {
            dot += (int64_t)rd_i16(q0 + 2 * c) * rd_i16(q1 + 2 * c);
        }
        if (dot <= 0) {
            FAIL(diag, GDMB_E_QUAT, m->quats + 8u * k, "hemisphere rule violated");
        }
    }

    /* E_THRUST */
    if (!reversible) {
        for (k = 0; k < m->n; k++) {
            if (rd_i16(r->data + m->thrusts + 2u * k) < 0) {
                FAIL(diag, GDMB_E_THRUST, m->thrusts + 2u * k, "negative thrust without REVERSIBLE_THRUST");
            }
        }
    }

    return GDMB_OK;
}

static bool maneuver_has_negative_thrust(const uint8_t* data, const maneuver_t* m)
{
    uint32_t k;
    for (k = 0; k < m->n; k++) {
        if (rd_i16(data + m->thrusts + 2u * k) < 0) {
            return true;
        }
    }
    return false;
}

static void load_quat(const uint8_t* data, uint32_t quats, uint32_t k, gdmb_real_t q[4])
{
    const uint8_t* p = data + quats + 8u * k;
    gdmb_real_t n;
    int c;
    for (c = 0; c < 4; c++) {
        q[c] = (gdmb_real_t)rd_i16(p + 2 * c);
    }
    n = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (c = 0; c < 4; c++) {
        q[c] /= n;
    }
}

/* q, ω, ω̇ and f at local time tau_us (§6.5, §6.6) */
static void maneuver_eval(const uint8_t* data, const maneuver_t* m, uint32_t tau_us,
    gdmb_real_t q[4], gdmb_real_t w[3], gdmb_real_t wd[3], gdmb_real_t* f)
{
    uint32_t i = tau_us / m->knot_us;
    gdmb_real_t u, u2, u3, dt;
    gdmb_real_t lam[4], dlam[4], ddlam[4], qs[4][4], b[4];
    int jj, c;

    if (i > (uint32_t)m->n - 4u) {
        i = (uint32_t)m->n - 4u;
    }
    u = (gdmb_real_t)(tau_us - i * m->knot_us) / (gdmb_real_t)m->knot_us;
    u2 = u * u;
    u3 = u2 * u;
    dt = (gdmb_real_t)m->knot_us / US_PER_SEC;

    /* Cumulative basis and its u-derivatives */
    lam[0] = 1;
    lam[1] = (5 + 3 * u - 3 * u2 + u3) / 6;
    lam[2] = (1 + 3 * u + 3 * u2 - 2 * u3) / 6;
    lam[3] = u3 / 6;
    dlam[0] = 0;
    dlam[1] = (3 - 6 * u + 3 * u2) / 6;
    dlam[2] = (3 + 6 * u - 6 * u2) / 6;
    dlam[3] = (3 * u2) / 6;
    ddlam[0] = 0;
    ddlam[1] = (-6 + 6 * u) / 6;
    ddlam[2] = (6 - 12 * u) / 6;
    ddlam[3] = (6 * u) / 6;

    for (jj = 0; jj < 4; jj++) {
        load_quat(data, m->quats, i + (uint32_t)jj, qs[jj]);
    }

    memcpy(q, qs[0], sizeof(qs[0]));
    w[0] = w[1] = w[2] = 0;
    wd[0] = wd[1] = wd[2] = 0;

    for (jj = 1; jj < 4; jj++) {
        gdmb_real_t conj[4], rel[4], d[3], phi[3], e[4], M[3][3], wn[3], t[3], cr[3];

        conj[0] = qs[jj - 1][0];
        conj[1] = -qs[jj - 1][1];
        conj[2] = -qs[jj - 1][2];
        conj[3] = -qs[jj - 1][3];
        qmul(conj, qs[jj], rel);
        qlog(rel, d);

        for (c = 0; c < 3; c++) {
            phi[c] = lam[jj] * d[c];
        }
        qexp(phi, e);
        qmul(q, e, q);
        qmat(e, M);

        mat_t_vec(M, w, t);
        for (c = 0; c < 3; c++) {
            wn[c] = t[c] + dlam[jj] * d[c];
        }
        mat_t_vec(M, wd, t);
        vcross(wn, d, cr);
        for (c = 0; c < 3; c++) {
            wd[c] = t[c] + ddlam[jj] * d[c] + dlam[jj] * cr[c];
        }
        memcpy(w, wn, sizeof(wn));
    }

    for (c = 0; c < 3; c++) {
        w[c] /= dt;
        wd[c] /= dt * dt;
    }

    b[0] = (1 - u) * (1 - u) * (1 - u);
    b[1] = 3 * u3 - 6 * u2 + 4;
    b[2] = -3 * u3 + 3 * u2 + 3 * u + 1;
    b[3] = u3;
    *f = 0;
    for (jj = 0; jj < 4; jj++) {
        *f += b[jj] * (gdmb_real_t)rd_i16(data + m->thrusts + 2u * (i + (uint32_t)jj));
    }
    *f = *f / 6 / THRUST_SCALE;
}

/* Re-reads the fixed part of a MANEUVER payload that was validated before */
static void reload_maneuver(const uint8_t* data, uint32_t size, uint32_t payload, maneuver_t* m)
{
    reader_t r;
    r.data = data;
    r.pos = payload;
    r.end = size;
    m->dur_us = rd_u32(data + payload);
    m->knot_us = rd_u32(data + payload + 4);
    m->n = rd_u16(data + payload + 8);
    m->env_pos_mm = rd_u16(data + payload + 10);
    m->env_att_mrad = rd_u16(data + payload + 12);
    m->abort_action = data[payload + 14];
    m->reserved = data[payload + 15];
    m->start[0] = rd_i32(data + payload + 16);
    m->start[1] = rd_i32(data + payload + 20);
    m->start[2] = rd_i32(data + payload + 24);
    m->yaw_exit = rd_i32(data + payload + 28);
    r.pos = payload + MANEUVER_FIXED_SIZE;
    (void)read_flat_record(&r, true, &m->phat, NULL);
    m->quats = r.pos;
    m->thrusts = r.pos + 8u * m->n;
}

/* ------------------------------------------------------------------------ */
/* FLAT evaluation                                                          */
/* ------------------------------------------------------------------------ */

/* p, v, a, j of three position axes and ψ, ψ̇ of a record at s ∈ [0, 1] */
static void flat_state(const uint8_t* data, uint32_t values, const uint8_t deg[4], uint32_t dur_us,
    const int32_t p0[4], int axes, gdmb_real_t s,
    gdmb_real_t p[3], gdmb_real_t v[3], gdmb_real_t a[3], gdmb_real_t j[3], gdmb_real_t* yaw, gdmb_real_t* yaw_rate)
{
    const gdmb_real_t T = (gdmb_real_t)dur_us / US_PER_SEC;
    int64_t c[8];
    gdmb_real_t out[4];
    int k, n;

    for (k = 0; k < 3; k++) {
        n = load_axis(data, values, deg, k, p0[k], c);
        bezier_eval(c, n, s, T, GDMB_R(1, POS_SCALE), out);
        p[k] = out[0];
        v[k] = out[1];
        a[k] = out[2];
        j[k] = out[3];
    }

    if (axes > 3) {
        n = load_axis(data, values, deg, 3, p0[3], c);
        bezier_eval(c, n, s, T, GDMB_R(1, YAW_SCALE), out);
        *yaw = out[0];
        *yaw_rate = out[1];
    } else {
        *yaw = 0;
        *yaw_rate = 0;
    }
}

/* ------------------------------------------------------------------------ */
/* Regime joints (§7.3)                                                     */
/* ------------------------------------------------------------------------ */

/* Checks a FLAT ↔ MANEUVER joint. `flat` is the FLAT segment on the joint
 * (the last one before an entry, the first one after an exit). */
static bool regime_joint_ok(const uint8_t* data, const gdmb_segment_t* flat, const maneuver_t* m, bool entry)
{
    gdmb_real_t p[3], v[3], a[3], j[3], yaw, yaw_rate;
    gdmb_real_t R_flat[3][3], w_flat[3], f_flat;
    gdmb_real_t q[4], w[3], wd[3], f, R_m[3][3], dw[3];
    int32_t mstart[4];
    int c;

    mstart[0] = m->start[0];
    mstart[1] = m->start[1];
    mstart[2] = m->start[2];
    mstart[3] = 0;

    if (entry) {
        if (!c2_joint_ok(data, flat->offset, flat->deg, flat->dur_us, flat->p0,
                m->phat.values, m->phat.deg, m->phat.dur_us, mstart, 3)) {
            return false;
        }
        flat_state(data, flat->offset, flat->deg, flat->dur_us, flat->p0, 4, 1, p, v, a, j, &yaw, &yaw_rate);
        maneuver_eval(data, m, 0, q, w, wd, &f);
    } else {
        if (!c2_joint_ok(data, m->phat.values, m->phat.deg, m->phat.dur_us, mstart,
                flat->offset, flat->deg, flat->dur_us, flat->p0, 3)) {
            return false;
        }
        flat_state(data, flat->offset, flat->deg, flat->dur_us, flat->p0, 4, 0, p, v, a, j, &yaw, &yaw_rate);
        maneuver_eval(data, m, m->dur_us, q, w, wd, &f);
    }

    gdmb_flat_attitude(a, j, yaw, yaw_rate, flat->sigma, R_flat, w_flat, &f_flat);
    qmat(q, R_m);

    if (rotation_angle(R_flat, R_m) > GDMB_R(1, 100)) {
        return false;
    }
    for (c = 0; c < 3; c++) {
        dw[c] = w_flat[c] - w[c];
    }
    if (vnorm(dw) > GDMB_R(5, 100)) {
        return false;
    }
    if (fabs(f_flat - f) > GDMB_R(5, 100)) {
        return false;
    }

    return true;
}

/* ------------------------------------------------------------------------ */
/* Index lookup                                                             */
/* ------------------------------------------------------------------------ */

/* Index of the entry that starts exactly at t, or -1 */
static int64_t find_segment_starting_at(const gdmb_show_t* show, uint32_t t)
{
    uint32_t lo = 0, hi = show->num_segments;

    while (lo < hi) {
        uint32_t mid = lo + (hi - lo) / 2;
        if (show->segments[mid].t0_us < t) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }

    if (lo < show->num_segments && show->segments[lo].t0_us == t) {
        return lo;
    }
    return -1;
}

/* Index of the entry that contains t, for 0 <= t < duration */
static uint32_t find_segment_containing(const gdmb_show_t* show, uint32_t t)
{
    uint32_t lo = 0, hi = show->num_segments;

    /* last entry with t0 <= t */
    while (hi - lo > 1) {
        uint32_t mid = lo + (hi - lo) / 2;
        if (show->segments[mid].t0_us <= t) {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    return lo;
}

/* ------------------------------------------------------------------------ */
/* Ancillary blocks                                                         */
/* ------------------------------------------------------------------------ */

static gdmb_error_t check_events(const uint8_t* data, uint32_t at, uint32_t len, gdmb_diag_t* diag)
{
    uint32_t k, last_t = 0;

    if (len % EVENT_SIZE) {
        FAIL(diag, GDMB_E_EVENTS, at, "EVENTS length not a multiple of 10");
    }
    for (k = 0; k < len; k += EVENT_SIZE) {
        const uint8_t* e = data + at + k;
        uint32_t t = rd_u32(e);
        uint32_t payload = rd_u32(e + 6);
        if (k > 0 && t < last_t) {
            FAIL(diag, GDMB_E_EVENTS, at + k, "events not sorted by time");
        }
        last_t = t;
        if (e[4] != 1 || (payload != 0 && payload != 0xFFFFFFFFu)) {
            FAIL(diag, GDMB_E_EVENTS, at + k, "unsupported event");
        }
    }

    return GDMB_OK;
}

static gdmb_error_t check_envelopes(const gdmb_show_t* show, const uint8_t* data, uint32_t at, uint32_t len, gdmb_diag_t* diag)
{
    uint32_t k;
    int64_t last = -1;

    if (len % ENVELOPE_SIZE) {
        FAIL(diag, GDMB_E_ENVELOPE, at, "ENVELOPES length not a multiple of 12");
    }
    for (k = 0; k < len; k += ENVELOPE_SIZE) {
        const uint8_t* e = data + at + k;
        uint32_t start = rd_u32(e);
        uint32_t end = rd_u32(e + 4);
        uint16_t mm = rd_u16(e + 8);
        int64_t idx;

        if (rd_u16(e + 10) != 0) {
            FAIL(diag, GDMB_E_RESERVED, at + k + 10, "ENVELOPES reserved field set");
        }
        idx = find_segment_starting_at(show, start);
        if (idx < 0 || show->segments[idx].is_maneuver ||
            (uint64_t)start + show->segments[idx].dur_us != end ||
            (int64_t)start <= last || mm == 0) {
            FAIL(diag, GDMB_E_ENVELOPE, at + k, "envelope record does not span exactly one FLAT segment");
        }
        last = start;
    }

    return GDMB_OK;
}

/* RTH_PLAN payload (§4.10) and its rules (E_RTH, E_C2) */
static gdmb_error_t check_rth_plan(gdmb_show_t* show, const uint8_t* data, uint32_t at, uint32_t len, gdmb_diag_t* diag)
{
    reader_t r;
    int32_t target[3];
    uint16_t n_entries, e;
    int64_t last_t = -1;
    const bool reversible = (show->features & GDMB_FEATURE_REVERSIBLE_THRUST) != 0;
    gdmb_error_t err;

    r.data = data;
    r.pos = at;
    r.end = at + len;

    if (!rd_has(&r, RTH_HEADER_SIZE)) {
        FAIL(diag, GDMB_E_FRAMING, at, "RTH_PLAN payload truncated");
    }
    target[0] = rd_i32(data + at);
    target[1] = rd_i32(data + at + 4);
    target[2] = rd_i32(data + at + 8);
    n_entries = rd_u16(data + at + 12);
    r.pos += RTH_HEADER_SIZE;
    if (n_entries == 0) {
        FAIL(diag, GDMB_E_RTH, at + 12, "RTH_PLAN without entries");
    }

    for (e = 0; e < n_entries; e++) {
        uint32_t entry_at = r.pos;
        uint32_t t_e;
        uint16_t n_segs, k;
        int64_t idx;
        const gdmb_segment_t* branch;
        /* The joint being checked: previous record a -> next record */
        uint32_t a_values, a_dur;
        const uint8_t* a_deg;
        int32_t a_start[4], cur[4];
        flat_record_t rec, last_rec;
        int axis;

        if (!rd_has(&r, RTH_ENTRY_HEADER_SIZE)) {
            FAIL(diag, GDMB_E_FRAMING, entry_at, "RTH_PLAN entry truncated");
        }
        t_e = rd_u32(data + r.pos);
        n_segs = rd_u16(data + r.pos + 4);
        r.pos += RTH_ENTRY_HEADER_SIZE;
        if (n_segs == 0) {
            FAIL(diag, GDMB_E_RTH, entry_at, "RTH entry without segments");
        }

        /* Read every record of the entry first (framing, header rules) */
        {
            reader_t probe = r;
            for (k = 0; k < n_segs; k++) {
                err = read_flat_record(&probe, reversible, &rec, diag);
                if (err != GDMB_OK) {
                    return err;
                }
            }
        }

        if ((int64_t)t_e <= last_t) {
            FAIL(diag, GDMB_E_RTH, entry_at, "RTH branch times not strictly increasing");
        }
        last_t = t_e;

        idx = find_segment_starting_at(show, t_e);
        if (idx <= 0 || show->segments[idx].is_maneuver || show->segments[idx - 1].is_maneuver) {
            FAIL(diag, GDMB_E_RTH, entry_at, "RTH branch time is not a FLAT/FLAT joint of the show");
        }
        branch = &show->segments[idx - 1];

        a_values = branch->offset;
        a_deg = branch->deg;
        a_dur = branch->dur_us;
        memcpy(a_start, branch->p0, sizeof(a_start));
        memcpy(cur, show->segments[idx].p0, sizeof(cur));

        for (k = 0; k < n_segs; k++) {
            uint32_t rec_at = r.pos;
            err = read_flat_record(&r, reversible, &rec, diag);
            if (err != GDMB_OK) {
                return err;
            }
            if (branch->sigma != 1 || rec.sigma != 1) {
                FAIL(diag, GDMB_E_RTH, rec_at, "RTH is upright only");
            }
            if (!c2_joint_ok(data, a_values, a_deg, a_dur, a_start, rec.values, rec.deg, rec.dur_us, cur, 4)) {
                FAIL(diag, GDMB_E_C2, rec_at, "RTH joint is not C2");
            }
            last_rec = rec;
            a_values = rec.values;
            a_dur = rec.dur_us;
            memcpy(a_start, cur, sizeof(a_start));
            a_deg = last_rec.deg;
            for (axis = 0; axis < 4; axis++) {
                cur[axis] = axis_end(data, rec.values, rec.deg, axis, cur[axis]);
            }
        }

        if (cur[0] != target[0] || cur[1] != target[1] || cur[2] != target[2]) {
            FAIL(diag, GDMB_E_RTH, entry_at, "RTH path does not end at the target");
        }

        /* At rest: the last min(n, 2) + 1 control values (min(n, 1) + 1 for
         * yaw) of the final segment, counting its P0, are equal */
        for (axis = 0; axis < 4; axis++) {
            int64_t c[8];
            int n = load_axis(data, last_rec.values, last_rec.deg, axis, a_start[axis], c);
            int need = (axis == 3 ? (n < 1 ? n : 1) : (n < 2 ? n : 2)) + 1;
            int i;
            for (i = n - need + 1; i <= n; i++) {
                if (c[i] != c[n]) {
                    FAIL(diag, GDMB_E_RTH, entry_at, "RTH path does not end at rest");
                }
            }
        }
    }

    if (r.pos != r.end) {
        FAIL(diag, GDMB_E_FRAMING, r.pos, "trailing bytes in RTH_PLAN payload");
    }

    show->rth_plan = data + at;
    show->rth_plan_length = len;
    show->num_rth_entries = n_entries;
    return GDMB_OK;
}

/* ------------------------------------------------------------------------ */
/* Parser                                                                   */
/* ------------------------------------------------------------------------ */

void gdmb_show_init(gdmb_show_t* show)
{
    memset(show, 0, sizeof(*show));
}

void gdmb_show_free(gdmb_show_t* show)
{
    if (show->segments) {
        GDMB_FREE(show->segments);
    }
    gdmb_show_init(show);
}

bool gdmb_show_is_loaded(const gdmb_show_t* show)
{
    return show->segments != NULL && show->num_segments > 0;
}

static int block_rank(uint8_t type)
{
    switch (type) {
    case BLOCK_FLAT:
    case BLOCK_MANEUVER:
        return 0;
    case BLOCK_LIGHTS:
        return 1;
    case BLOCK_EVENTS:
        return 2;
    case BLOCK_RTH_PLAN:
        return 3;
    case BLOCK_ENVELOPES:
        return 4;
    case BLOCK_COMMENT:
        return 5;
    default:
        return -1;
    }
}

/* Pass 1: headers, block framing and order, structural parse of the motion
 * blocks. Counts index entries. */
static gdmb_error_t parse_structure(const uint8_t* data, uint32_t size, uint32_t header_length,
    bool provisional, uint8_t features,
    uint32_t* num_entries, uint32_t* num_maneuvers, gdmb_diag_t* diag)
{
    const bool reversible = (features & GDMB_FEATURE_REVERSIBLE_THRUST) != 0;
    uint32_t pos = header_length;
    int last_rank = 0;
    int last_motion = 0;
    bool seen[6] = { false, false, false, false, false, false };
    bool any_motion = false;
    bool reversible_used = false;
    gdmb_error_t err;

    *num_entries = 0;
    *num_maneuvers = 0;

    /* Framing and block types first, over the whole file */
    while (pos < size) {
        uint8_t type, flags;
        uint32_t len;

        if (size - pos < 6) {
            FAIL(diag, GDMB_E_FRAMING, pos, "truncated block header");
        }
        type = data[pos];
        flags = data[pos + 1];
        len = rd_u32(data + pos + 2);
        if (len > size - pos - 6) {
            FAIL(diag, GDMB_E_FRAMING, pos, "block overruns the file");
        }
        if (flags & ~BLOCK_FLAG_SKIPPABLE) {
            FAIL(diag, GDMB_E_RESERVED, pos + 1, "reserved block flag bits set");
        }
        if (type == 0x00 || type == BLOCK_YAW_RESERVED ||
            (!provisional && (type == BLOCK_MANEUVER || type == BLOCK_RTH_PLAN || type == BLOCK_ENVELOPES))) {
            FAIL(diag, GDMB_E_BLOCK_TYPE, pos, "block type reserved in v1");
        }
        if (block_rank(type) < 0 && !(flags & BLOCK_FLAG_SKIPPABLE)) {
            FAIL(diag, GDMB_E_BLOCK_TYPE, pos, "unknown block type without SKIPPABLE");
        }
        pos += 6 + len;
    }

    /* Canonical order (§4.4) */
    pos = header_length;
    while (pos < size) {
        uint8_t type = data[pos];
        uint32_t len = rd_u32(data + pos + 2);
        int rank = block_rank(type);

        if (rank >= 0) {
            if (rank < last_rank) {
                FAIL(diag, GDMB_E_BLOCK_ORDER, pos, "blocks out of canonical order");
            }
            if (rank > 0) {
                if (seen[rank]) {
                    FAIL(diag, GDMB_E_BLOCK_ORDER, pos, "duplicate block");
                }
                seen[rank] = true;
            } else {
                if (!any_motion && type != BLOCK_FLAT) {
                    FAIL(diag, GDMB_E_BLOCK_ORDER, pos, "motion must start with FLAT_TRAJECTORY");
                }
                if (any_motion && type == last_motion) {
                    FAIL(diag, GDMB_E_BLOCK_ORDER, pos, "two adjacent motion blocks of the same type");
                }
                any_motion = true;
                last_motion = type;
            }
            last_rank = rank;
        }
        pos += 6 + len;
    }
    if (!any_motion || last_motion != BLOCK_FLAT) {
        FAIL(diag, GDMB_E_BLOCK_ORDER, header_length, "motion must start and end with FLAT_TRAJECTORY");
    }

    /* Structural parse of the motion blocks */
    pos = header_length;
    while (pos < size) {
        uint8_t type = data[pos];
        uint32_t len = rd_u32(data + pos + 2);
        reader_t r;

        r.data = data;
        r.pos = pos + 6;
        r.end = pos + 6 + len;

        if (type == BLOCK_FLAT) {
            uint32_t count = 0;
            if (len < FLAT_START_SIZE) {
                FAIL(diag, GDMB_E_FRAMING, pos, "FLAT_TRAJECTORY payload truncated");
            }
            r.pos += FLAT_START_SIZE;
            while (r.pos < r.end) {
                flat_record_t rec;
                err = read_flat_record(&r, reversible, &rec, diag);
                if (err != GDMB_OK) {
                    return err;
                }
                if (rec.sigma < 0) {
                    reversible_used = true;
                }
                count++;
            }
            if (count == 0) {
                FAIL(diag, GDMB_E_FRAMING, pos, "FLAT_TRAJECTORY without segments");
            }
            *num_entries += count;
        } else if (type == BLOCK_MANEUVER) {
            maneuver_t m;
            err = read_maneuver(&r, reversible, &m, diag);
            if (err != GDMB_OK) {
                return err;
            }
            if (maneuver_has_negative_thrust(data, &m)) {
                reversible_used = true;
            }
            *num_entries += 1;
            *num_maneuvers += 1;
        }
        pos += 6 + len;
    }

    if (((features & GDMB_FEATURE_MANEUVER) != 0) != (*num_maneuvers > 0)) {
        FAIL(diag, GDMB_E_FEATURES, 5, "MANEUVER feature bit disagrees with content");
    }
    if (reversible != reversible_used) {
        FAIL(diag, GDMB_E_FEATURES, 5, "REVERSIBLE_THRUST feature bit disagrees with content");
    }

    return GDMB_OK;
}

/* §7.1 E_REST: the FLAT segment at one end of the timeline is at rest there,
 * within the one-sided §7.3 tolerances (what rounding alone can leave, plus
 * the slack): |v| per axis and |yaw rate|, and |a| per position axis. */
static bool at_rest(const uint8_t* data, const gdmb_segment_t* seg, bool at_end)
{
    const gdmb_real_t T = (gdmb_real_t)seg->dur_us / US_PER_SEC;
    int k;

    for (k = 0; k < 4; k++) {
        int64_t c[8];
        gdmb_real_t v, a, tol_v, tol_a;
        const bool is_yaw = (k == 3);
        const gdmb_real_t q = is_yaw ? GDMB_R(1, YAW_SCALE) : GDMB_R(1, POS_SCALE);
        const int n = load_axis(data, seg->offset, seg->deg, k, seg->p0[k], c);

        endpoint_derivs(c, n, T, q, at_end, &v, &a);
        tol_v = q * (gdmb_real_t)n / T + GDMB_R(1, 1000);
        if (fabs(v) > tol_v) {
            return false;
        }
        if (!is_yaw) {
            tol_a = 2 * q * (gdmb_real_t)(n * (n - 1)) / (T * T) + GDMB_R(1, 100);
            if (fabs(a) > tol_a) {
                return false;
            }
        }
    }

    return true;
}

/* Pass 2: timeline index, continuity, C² and regime joints */
static gdmb_error_t parse_timeline(gdmb_show_t* show, gdmb_diag_t* diag)
{
    const uint8_t* data = show->data;
    const uint32_t size = show->size;
    uint32_t pos = show->header_length;
    uint32_t n = 0;
    uint64_t total = 0;
    int32_t cur[4] = { 0, 0, 0, 0 };
    bool first = true;
    bool prev_was_maneuver = false;
    maneuver_t prev_maneuver;
    int axis;

    memset(&prev_maneuver, 0, sizeof(prev_maneuver));

    while (pos < size) {
        uint8_t type = data[pos];
        uint32_t len = rd_u32(data + pos + 2);
        reader_t r;

        r.data = data;
        r.pos = pos + 6;
        r.end = pos + 6 + len;

        if (type == BLOCK_FLAT) {
            int32_t start[4];
            uint32_t first_in_block = n;

            for (axis = 0; axis < 4; axis++) {
                start[axis] = rd_i32(data + r.pos + 4u * (uint32_t)axis);
            }
            r.pos += FLAT_START_SIZE;

            if (first) {
                memcpy(show->start, start, sizeof(start));
            } else if (memcmp(start, cur, sizeof(cur)) != 0) {
                FAIL(diag, GDMB_E_CONTINUITY, pos + 6, "FLAT block start != end of the preceding maneuver");
            }
            memcpy(cur, start, sizeof(cur));

            while (r.pos < r.end) {
                gdmb_segment_t* seg = &show->segments[n];
                flat_record_t rec;
                uint32_t rec_at = r.pos;

                (void)read_flat_record(&r, true, &rec, NULL);

                seg->t0_us = (uint32_t)total;
                seg->dur_us = rec.dur_us;
                seg->offset = rec.values;
                memcpy(seg->p0, cur, sizeof(cur));
                memcpy(seg->deg, rec.deg, sizeof(rec.deg));
                seg->sigma = rec.sigma;
                seg->is_maneuver = 0;

                if (n > first_in_block) {
                    const gdmb_segment_t* prev = &show->segments[n - 1];
                    if (prev->sigma != seg->sigma) {
                        FAIL(diag, GDMB_E_SIGMA, rec_at, "sigma changes at a FLAT joint");
                    }
                    if (!c2_joint_ok(data, prev->offset, prev->deg, prev->dur_us, prev->p0,
                            seg->offset, seg->deg, seg->dur_us, seg->p0, 4)) {
                        FAIL(diag, GDMB_E_C2, rec_at, "FLAT joint is not C2");
                    }
                }

                for (axis = 0; axis < 4; axis++) {
                    cur[axis] = axis_end(data, rec.values, rec.deg, axis, cur[axis]);
                }
                total += rec.dur_us;
                if (total > 0xFFFFFFFFu) {
                    FAIL(diag, GDMB_E_DURATION, rec_at, "timeline longer than 2^32 - 1 us");
                }
                n++;
            }

            if (prev_was_maneuver &&
                !regime_joint_ok(data, &show->segments[first_in_block], &prev_maneuver, false)) {
                FAIL(diag, GDMB_E_REGIME, pos + 6, "maneuver exit does not match the FLAT segment");
            }
            prev_was_maneuver = false;
            first = false;
        } else if (type == BLOCK_MANEUVER) {
            gdmb_segment_t* seg = &show->segments[n];
            maneuver_t m;

            reload_maneuver(data, size, r.pos, &m);
            if (m.start[0] != cur[0] || m.start[1] != cur[1] || m.start[2] != cur[2]) {
                FAIL(diag, GDMB_E_CONTINUITY, r.pos + 16, "MANEUVER start != end of the preceding FLAT block");
            }
            if (!regime_joint_ok(data, &show->segments[n - 1], &m, true)) {
                FAIL(diag, GDMB_E_REGIME, r.pos, "maneuver entry does not match the FLAT segment");
            }

            seg->t0_us = (uint32_t)total;
            seg->dur_us = m.dur_us;
            seg->offset = r.pos;
            seg->p0[0] = m.start[0];
            seg->p0[1] = m.start[1];
            seg->p0[2] = m.start[2];
            seg->p0[3] = cur[3];
            memcpy(seg->deg, m.phat.deg, sizeof(m.phat.deg));
            seg->sigma = 0;
            seg->is_maneuver = 1;

            for (axis = 0; axis < 3; axis++) {
                cur[axis] = axis_end(data, m.phat.values, m.phat.deg, axis, m.start[axis]);
            }
            cur[3] = m.yaw_exit;
            total += m.dur_us;
            if (total > 0xFFFFFFFFu) {
                FAIL(diag, GDMB_E_DURATION, r.pos, "timeline longer than 2^32 - 1 us");
            }
            prev_maneuver = m;
            prev_was_maneuver = true;
            n++;
        }

        pos += 6 + len;
    }

    show->num_segments = n;
    show->duration_us = (uint32_t)total;
    memcpy(show->end, cur, sizeof(cur));
    show->start_sigma = show->segments[0].sigma;
    show->end_sigma = show->segments[n - 1].sigma;

    /* E_REST: v, a and yaw rate are at rest at both ends of the timeline */
    if (!at_rest(data, &show->segments[0], false)) {
        FAIL(diag, GDMB_E_REST, show->segments[0].offset, "not at rest at the start of the timeline");
    }
    if (!at_rest(data, &show->segments[n - 1], true)) {
        FAIL(diag, GDMB_E_REST, show->segments[n - 1].offset, "not at rest at the end of the timeline");
    }

    return GDMB_OK;
}

/* Pass 3: ancillary blocks */
static gdmb_error_t parse_ancillary(gdmb_show_t* show, gdmb_diag_t* diag)
{
    const uint8_t* data = show->data;
    const uint32_t size = show->size;
    uint32_t pos = show->header_length;
    gdmb_error_t err;

    while (pos < size) {
        uint8_t type = data[pos];
        uint32_t len = rd_u32(data + pos + 2);
        uint32_t at = pos + 6;

        switch (type) {
        case BLOCK_LIGHTS:
            if (len == 0) {
                FAIL(diag, GDMB_E_LIGHTS, pos, "empty LIGHTS block");
            }
            show->lights = data + at;
            show->lights_length = len;
            break;

        case BLOCK_EVENTS:
            err = check_events(data, at, len, diag);
            if (err != GDMB_OK) {
                return err;
            }
            show->events = data + at;
            show->num_events = len / EVENT_SIZE;
            break;

        case BLOCK_ENVELOPES:
            err = check_envelopes(show, data, at, len, diag);
            if (err != GDMB_OK) {
                return err;
            }
            show->envelopes = data + at;
            show->num_envelopes = len / ENVELOPE_SIZE;
            break;

        case BLOCK_RTH_PLAN:
            err = check_rth_plan(show, data, at, len, diag);
            if (err != GDMB_OK) {
                return err;
            }
            break;

        case BLOCK_COMMENT:
            show->comment = data + at;
            show->comment_length = len;
            break;

        default:
            break;
        }

        pos += 6 + len;
    }

    return GDMB_OK;
}

static gdmb_error_t parse(gdmb_show_t* show, const uint8_t* data, size_t size, uint32_t flags, gdmb_diag_t* diag)
{
    const bool provisional = (flags & GDMB_PARSE_PROVISIONAL) != 0;
    uint8_t features;
    uint16_t header_length;
    uint32_t caps, crc, num_entries, num_maneuvers;
    uint8_t zeros[4] = { 0, 0, 0, 0 };
    gdmb_error_t err;

    if (size < GDMB_HEADER_LENGTH || size > GDMB_MAX_FILE_SIZE) {
        FAIL(diag, GDMB_E_SIZE, 0, "file size outside [48, 262144]");
    }
    if (data[0] != 'G' || data[1] != 'D' || data[2] != 'M' || data[3] != 'B') {
        FAIL(diag, GDMB_E_MAGIC, 0, "bad magic");
    }
    /* A reader accepts every version up to its own (§1.8) */
    if (data[4] < 1 || data[4] > GDMB_READER_VERSION) {
        FAIL(diag, GDMB_E_VERSION, 4, "unsupported version");
    }
    /* Header bytes past 48 belong to a later version's extension and are
     * skipped; the CRC still covers them (§4.1) */
    header_length = rd_u16(data + 6);
    if (header_length < GDMB_HEADER_LENGTH || header_length > size) {
        FAIL(diag, GDMB_E_HEADER, 6, "header_length below 48 or beyond the file");
    }

    /* CRC of the whole file with bytes 8-11 taken as zero (§4.9) */
    crc = gdmb_crc32(0, data, 8);
    crc = gdmb_crc32(crc, zeros, 4);
    crc = gdmb_crc32(crc, data + 12, size - 12);
    if (crc != rd_u32(data + 8)) {
        FAIL(diag, GDMB_E_CRC, 8, "CRC mismatch");
    }

    features = data[5];
    /* The MANEUVER bit is provisional: reserved in v1 (§4.11) */
    if (features & ~(GDMB_FEATURE_CRC32 | GDMB_FEATURE_REVERSIBLE_THRUST |
                     (provisional ? GDMB_FEATURE_MANEUVER : 0u))) {
        FAIL(diag, GDMB_E_RESERVED, 5, "reserved feature bits set");
    }
    if (!(features & GDMB_FEATURE_CRC32)) {
        FAIL(diag, GDMB_E_FEATURES, 5, "CRC32 feature bit clear");
    }

    caps = rd_u32(data + 44);
    /* high_tilt (bit 2) is reserved in v1; maneuver (bit 0) is provisional */
    if (caps & ~(GDMB_CAP_REVERSIBLE_THRUST | (provisional ? GDMB_CAP_MANEUVER : 0u))) {
        FAIL(diag, GDMB_E_CAPS, 44, "undefined capability bits set");
    }
    if (((caps & GDMB_CAP_MANEUVER) != 0) != ((features & GDMB_FEATURE_MANEUVER) != 0) ||
        ((caps & GDMB_CAP_REVERSIBLE_THRUST) != 0) != ((features & GDMB_FEATURE_REVERSIBLE_THRUST) != 0)) {
        FAIL(diag, GDMB_E_CAPS, 44, "capability bits disagree with feature flags");
    }

    err = parse_structure(data, (uint32_t)size, header_length, provisional, features,
        &num_entries, &num_maneuvers, diag);
    if (err != GDMB_OK) {
        return err;
    }

    show->segments = (gdmb_segment_t*)GDMB_CALLOC(num_entries, sizeof(gdmb_segment_t));
    if (show->segments == NULL) {
        FAIL(diag, GDMB_E_NOMEM, 0, "cannot allocate the segment index");
    }

    show->data = data;
    show->size = (uint32_t)size;
    show->header_length = header_length;
    show->features = features;
    show->capabilities = caps;
    show->show_hash = data + 12;
    show->num_maneuvers = num_maneuvers;

    err = parse_timeline(show, diag);
    if (err != GDMB_OK) {
        return err;
    }

    return parse_ancillary(show, diag);
}

gdmb_error_t gdmb_show_parse(gdmb_show_t* show, const uint8_t* data, size_t size, gdmb_diag_t* diag)
{
    return gdmb_show_parse_ex(show, data, size, 0, diag);
}

gdmb_error_t gdmb_show_parse_ex(gdmb_show_t* show, const uint8_t* data, size_t size,
    uint32_t flags, gdmb_diag_t* diag)
{
    gdmb_error_t err;

    gdmb_show_free(show);
    if (diag != NULL) {
        diag->code = GDMB_OK;
        diag->offset = 0;
        diag->what = "";
    }
    if (data == NULL) {
        FAIL(diag, GDMB_E_SIZE, 0, "no data");
    }

    err = parse(show, data, size, flags, diag);
    if (err != GDMB_OK) {
        gdmb_show_free(show);
    }

    return err;
}

/* ------------------------------------------------------------------------ */
/* Evaluation (§6)                                                          */
/* ------------------------------------------------------------------------ */

static void state_at_rest(const int32_t s[4], int8_t sigma, gdmb_state_t* out)
{
    int k;
    for (k = 0; k < 3; k++) {
        out->p[k] = (gdmb_real_t)s[k] / POS_SCALE;
    }
    out->yaw = (gdmb_real_t)s[3] / YAW_SCALE;
    out->sigma = sigma;
}

gdmb_error_t gdmb_show_evaluate(const gdmb_show_t* show, int64_t t_us, gdmb_state_t* out)
{
    const gdmb_segment_t* seg;
    uint32_t idx, tau;

    memset(out, 0, sizeof(*out));

    if (!gdmb_show_is_loaded(show)) {
        return GDMB_E_INVALID;
    }

    if (t_us < 0) {
        out->regime = GDMB_REGIME_BEFORE;
        out->segment = 0;
        state_at_rest(show->start, show->start_sigma, out);
        return GDMB_OK;
    }

    if (t_us >= (int64_t)show->duration_us) {
        out->regime = GDMB_REGIME_HOLD;
        out->segment = show->num_segments - 1;
        state_at_rest(show->end, show->end_sigma, out);
        return GDMB_OK;
    }

    idx = find_segment_containing(show, (uint32_t)t_us);
    seg = &show->segments[idx];
    tau = (uint32_t)t_us - seg->t0_us; /* local time in integer µs (§6.1) */
    out->segment = idx;

    if (!seg->is_maneuver) {
        const gdmb_real_t s = (gdmb_real_t)tau / (gdmb_real_t)seg->dur_us;
        out->regime = GDMB_REGIME_FLAT;
        out->sigma = seg->sigma;
        flat_state(show->data, seg->offset, seg->deg, seg->dur_us, seg->p0, 4, s,
            out->p, out->v, out->a, out->j, &out->yaw, &out->yaw_rate);
    } else {
        maneuver_t m;
        gdmb_real_t s, yaw, yaw_rate;
        reload_maneuver(show->data, show->size, seg->offset, &m);
        s = (gdmb_real_t)tau / (gdmb_real_t)m.dur_us;
        out->regime = GDMB_REGIME_MANEUVER;
        out->sigma = 0;
        flat_state(show->data, m.phat.values, m.phat.deg, m.dur_us, seg->p0, 3, s,
            out->p, out->v, out->a, out->j, &yaw, &yaw_rate);
        maneuver_eval(show->data, &m, tau, out->q, out->omega, out->omega_dot, &out->thrust);
    }

    return GDMB_OK;
}

bool gdmb_show_get_event(const gdmb_show_t* show, uint32_t index, gdmb_event_t* out)
{
    const uint8_t* e;

    if (show->events == NULL || index >= show->num_events) {
        return false;
    }

    e = show->events + EVENT_SIZE * index;
    out->t_us = rd_u32(e);
    out->type = e[4];
    out->subtype = e[5];
    out->payload = rd_u32(e + 6);
    return true;
}

gdmb_error_t gdmb_show_landing_time(const gdmb_show_t* show, gdmb_real_t xy_threshold_m,
    gdmb_real_t preferred_descent_m, uint32_t step_us, uint32_t* landing_us)
{
    gdmb_state_t end, cur, prev;
    int64_t t, t_run, lo, hi;
    gdmb_real_t target;
    /* quantization slack when comparing heights of neighbouring samples */
    const gdmb_real_t z_slack = GDMB_R(1, POS_SCALE);

    if (!gdmb_show_is_loaded(show) || step_us == 0 || landing_us == NULL) {
        return GDMB_E_INVALID;
    }

    /* Walk back from the end while the curve only descends (going forward)
     * and stays within the threshold of the final xy */
    gdmb_show_evaluate(show, show->duration_us, &end);
    t = show->duration_us;
    t_run = t;
    cur = end;
    while (t > 0) {
        int64_t tp = t > (int64_t)step_us ? t - (int64_t)step_us : 0;
        gdmb_show_evaluate(show, tp, &prev);
        if (prev.regime == GDMB_REGIME_MANEUVER ||
            prev.p[2] + z_slack < cur.p[2] ||
            hypot(prev.p[0] - end.p[0], prev.p[1] - end.p[1]) > xy_threshold_m) {
            break;
        }
        t_run = tp;
        t = tp;
        cur = prev;
    }

    /* Hand over preferred_descent_m above the end of the run, but not before
     * the run starts */
    gdmb_show_evaluate(show, t_run, &cur);
    target = end.p[2] + preferred_descent_m;
    if (cur.p[2] <= target) {
        *landing_us = (uint32_t)t_run;
        return GDMB_OK;
    }

    /* First time after t_run at which the height reaches the target; the run
     * descends monotonically up to sampling, so bisect within the sample */
    lo = t_run;
    for (hi = t_run; hi < (int64_t)show->duration_us; hi += step_us) {
        gdmb_show_evaluate(show, hi, &cur);
        if (cur.p[2] <= target) {
            break;
        }
        lo = hi;
    }
    if (hi > (int64_t)show->duration_us) {
        hi = show->duration_us;
    }
    while (hi - lo > 1) {
        int64_t mid = lo + (hi - lo) / 2;
        gdmb_show_evaluate(show, mid, &cur);
        if (cur.p[2] <= target) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    *landing_us = (uint32_t)hi;
    return GDMB_OK;
}

bool gdmb_show_has_inverted_flat(const gdmb_show_t* show)
{
    uint32_t i;
    for (i = 0; i < show->num_segments; i++) {
        if (!show->segments[i].is_maneuver && show->segments[i].sigma < 0) {
            return true;
        }
    }
    return false;
}
