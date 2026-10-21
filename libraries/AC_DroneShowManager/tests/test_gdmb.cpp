/*
 * Conformance tests of the .gdmb reader against the golden vectors of
 * AxioStack contracts/gdm-vectors (embedded in gdmb_vectors.h by
 * gen_gdmb_vectors.py). Spec §10: a reader conforms when it accepts both
 * vectors and the accept vector, rejects every reject vector with the stated
 * code, and reproduces the samples within 1e-9 relative error in binary64.
 */

#include <AP_gtest.h>

#include <math.h>
#include <string.h>
#include <vector>

#include <AP_HAL/AP_HAL.h>
#include <AC_DroneShowManager/DroneShow_Gdmb.h>

#include "gdmb_vectors.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using namespace gdmb_vectors;

namespace {

// binary64 conformance (spec §10): 1e-9 relative, with an absolute floor of
// the same size for values that are zero
void expect_close(double actual, double expected, const char* what, int64_t t_us)
{
    const double tol = 1e-9 * fmax(1.0, fabs(expected));
    EXPECT_NEAR(actual, expected, tol) << what << " at t = " << t_us << " us";
}

const char* regime_name(gdmb_regime_t regime)
{
    switch (regime) {
    case GDMB_REGIME_BEFORE: return "before";
    case GDMB_REGIME_FLAT: return "flat";
    case GDMB_REGIME_MANEUVER: return "maneuver";
    case GDMB_REGIME_HOLD: return "hold";
    }
    return "?";
}

// Rewrites the header CRC of a mutated file (spec §4.9)
void recompute_crc(std::vector<uint8_t>& b)
{
    b[8] = b[9] = b[10] = b[11] = 0;
    const uint32_t crc = gdmb_crc32(0, b.data(), b.size());
    b[8] = crc & 0xFF;
    b[9] = (crc >> 8) & 0xFF;
    b[10] = (crc >> 16) & 0xFF;
    b[11] = (crc >> 24) & 0xFF;
}

// Vectors under provisional/ are read with the provisional layouts enabled
uint32_t parse_flags(bool provisional)
{
    return provisional ? GDMB_PARSE_PROVISIONAL : 0;
}

const Vector& vector_named(const char* name)
{
    for (const Vector& v : vectors) {
        if (strcmp(v.name, name) == 0) {
            return v;
        }
    }
    abort();
}

class Show {
public:
    Show() { gdmb_show_init(&show); }
    ~Show() { gdmb_show_free(&show); }
    gdmb_show_t show;
};

}  // namespace

TEST(Gdmb, crc_check_value)
{
    const uint8_t digits[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    EXPECT_EQ(0x2DFD2D88u, gdmb_crc32(0, digits, sizeof(digits)));
}

TEST(Gdmb, golden_vectors_parse_and_crc)
{
    for (const Vector& v : vectors) {
        Show s;
        gdmb_diag_t diag;
        ASSERT_EQ(GDMB_OK, gdmb_show_parse_ex(&s.show, v.data, v.size, parse_flags(v.provisional), &diag))
            << v.name << ": " << gdmb_error_name(diag.code) << " at " << diag.offset << ": " << diag.what;
        EXPECT_TRUE(gdmb_show_is_loaded(&s.show));

        uint32_t header_crc;
        memcpy(&header_crc, v.data + 8, 4);
        EXPECT_EQ(v.crc32, header_crc) << v.name;
        // what MAVFTP reports for the stored file (spec §4.9)
        EXPECT_EQ(v.mavftp_file_crc32, gdmb_crc32(0, v.data, v.size)) << v.name;
    }
}

TEST(Gdmb, golden_vectors_samples)
{
    for (const Vector& v : vectors) {
        Show s;
        ASSERT_EQ(GDMB_OK, gdmb_show_parse_ex(&s.show, v.data, v.size, parse_flags(v.provisional), nullptr)) << v.name;

        for (size_t i = 0; i < v.num_samples; i++) {
            const Sample& e = v.samples[i];
            gdmb_state_t st;
            ASSERT_EQ(GDMB_OK, gdmb_show_evaluate(&s.show, e.t_us, &st));
            EXPECT_STREQ(e.regime, regime_name(st.regime)) << v.name << " t = " << e.t_us;

            for (int k = 0; k < 3; k++) {
                expect_close(st.p[k], e.p[k], "p", e.t_us);
                expect_close(st.v[k], e.v[k], "v", e.t_us);
                expect_close(st.a[k], e.a[k], "a", e.t_us);
            }

            if (st.regime == GDMB_REGIME_MANEUVER) {
                for (int k = 0; k < 4; k++) {
                    expect_close(st.q[k], e.q[k], "q", e.t_us);
                }
                for (int k = 0; k < 3; k++) {
                    expect_close(st.omega[k], e.omega[k], "omega", e.t_us);
                    expect_close(st.omega_dot[k], e.omega_dot[k], "omega_dot", e.t_us);
                }
                expect_close(st.thrust, e.thrust, "thrust", e.t_us);
            } else {
                expect_close(st.yaw, e.yaw, "yaw", e.t_us);
                expect_close(st.yaw_rate, e.yaw_rate, "yaw_rate", e.t_us);
            }
        }
    }
}

TEST(Gdmb, flat_single_contents)
{
    const Vector& v = vectors[0];
    ASSERT_STREQ("flat-single", v.name);
    ASSERT_FALSE(v.provisional);

    Show s;
    ASSERT_EQ(GDMB_OK, gdmb_show_parse(&s.show, v.data, v.size, nullptr));
    EXPECT_EQ(GDMB_HEADER_LENGTH, s.show.header_length);
    EXPECT_EQ(18000000u, s.show.duration_us);
    EXPECT_EQ(0u, s.show.num_maneuvers);
    EXPECT_EQ(GDMB_FEATURE_CRC32, s.show.features);
    EXPECT_EQ(0u, s.show.capabilities);
    EXPECT_FALSE(gdmb_show_has_inverted_flat(&s.show));
    EXPECT_NE(nullptr, s.show.lights);
    EXPECT_GT(s.show.lights_length, 0u);
    EXPECT_EQ(2u, s.show.num_events);
    EXPECT_EQ(0u, s.show.num_envelopes);
    EXPECT_EQ(0u, s.show.num_rth_entries);
    EXPECT_NE(nullptr, s.show.comment);

    gdmb_event_t ev;
    ASSERT_TRUE(gdmb_show_get_event(&s.show, 0, &ev));
    EXPECT_EQ(1u, ev.type);
    EXPECT_FALSE(gdmb_show_get_event(&s.show, 2, &ev));

    // before the start and after the end: at rest (spec §6.1)
    gdmb_state_t st;
    ASSERT_EQ(GDMB_OK, gdmb_show_evaluate(&s.show, -1, &st));
    EXPECT_EQ(GDMB_REGIME_BEFORE, st.regime);
    ASSERT_EQ(GDMB_OK, gdmb_show_evaluate(&s.show, s.show.duration_us, &st));
    EXPECT_EQ(GDMB_REGIME_HOLD, st.regime);
    EXPECT_DOUBLE_EQ(0.0, st.v[0]);
    EXPECT_DOUBLE_EQ(0.0, st.a[2]);
}

TEST(Gdmb, provisional_contents)
{
    const Vector& m = vector_named("provisional/maneuver-single");
    Show s;
    ASSERT_EQ(GDMB_OK, gdmb_show_parse_ex(&s.show, m.data, m.size, GDMB_PARSE_PROVISIONAL, nullptr));
    EXPECT_EQ(1u, s.show.num_maneuvers);
    EXPECT_TRUE(s.show.features & GDMB_FEATURE_MANEUVER);
    EXPECT_TRUE(s.show.capabilities & GDMB_CAP_MANEUVER);

    const Vector& r = vector_named("provisional/flat-rth");
    Show t;
    ASSERT_EQ(GDMB_OK, gdmb_show_parse_ex(&t.show, r.data, r.size, GDMB_PARSE_PROVISIONAL, nullptr));
    EXPECT_EQ(6u, t.show.num_envelopes);
    EXPECT_EQ(3u, t.show.num_rth_entries);
}

// A v1 reader refuses the provisional layouts (spec §4.11): feature bit 1
// with E_RESERVED, the RTH_PLAN and ENVELOPES blocks with E_BLOCK_TYPE
TEST(Gdmb, v1_refuses_provisional_vectors)
{
    Show s;
    const Vector& m = vector_named("provisional/maneuver-single");
    EXPECT_EQ(GDMB_E_RESERVED, gdmb_show_parse(&s.show, m.data, m.size, nullptr));
    EXPECT_FALSE(gdmb_show_is_loaded(&s.show));

    const Vector& r = vector_named("provisional/flat-rth");
    EXPECT_EQ(GDMB_E_BLOCK_TYPE, gdmb_show_parse(&s.show, r.data, r.size, nullptr));
    EXPECT_FALSE(gdmb_show_is_loaded(&s.show));
}

// Acceleration is continuous across every FLAT/FLAT joint of the golden
// vectors (the point of C², spec §7.3)
TEST(Gdmb, joints_are_c2)
{
    for (const Vector& v : vectors) {
        Show s;
        ASSERT_EQ(GDMB_OK, gdmb_show_parse_ex(&s.show, v.data, v.size, parse_flags(v.provisional), nullptr));
        for (uint32_t i = 1; i < s.show.num_segments; i++) {
            const gdmb_segment_t& a = s.show.segments[i - 1];
            const gdmb_segment_t& b = s.show.segments[i];
            if (a.is_maneuver || b.is_maneuver) {
                continue;
            }
            gdmb_state_t before, after;
            ASSERT_EQ(GDMB_OK, gdmb_show_evaluate(&s.show, (int64_t)b.t0_us - 1, &before));
            ASSERT_EQ(GDMB_OK, gdmb_show_evaluate(&s.show, b.t0_us, &after));
            for (int k = 0; k < 3; k++) {
                EXPECT_NEAR(before.p[k], after.p[k], 1e-4) << v.name << " joint " << i;
                EXPECT_NEAR(before.v[k], after.v[k], 1e-3) << v.name << " joint " << i;
                EXPECT_NEAR(before.a[k], after.a[k], 2e-2) << v.name << " joint " << i;
            }
        }
    }
}

TEST(Gdmb, reject_and_accept_vectors)
{
    size_t rejects = 0, accepts = 0;

    for (const Edge& e : edges) {
        Show s;
        gdmb_diag_t diag;
        const gdmb_error_t err = gdmb_show_parse_ex(&s.show, e.data, e.size, parse_flags(e.provisional), &diag);
        if (e.code) {
            EXPECT_STREQ(e.code, gdmb_error_name(err)) << e.name << ": " << diag.what;
            EXPECT_FALSE(gdmb_show_is_loaded(&s.show)) << e.name;
            rejects++;
        } else {
            EXPECT_EQ(GDMB_OK, err) << e.name << ": " << gdmb_error_name(err) << " " << diag.what;
            accepts++;

            // decodes to the same show as flat-single
            Show ref;
            ASSERT_EQ(GDMB_OK, gdmb_show_parse(&ref.show, vectors[0].data, vectors[0].size, nullptr));
            for (int64_t t = 0; t <= ref.show.duration_us; t += 250000) {
                gdmb_state_t x, y;
                ASSERT_EQ(GDMB_OK, gdmb_show_evaluate(&s.show, t, &x));
                ASSERT_EQ(GDMB_OK, gdmb_show_evaluate(&ref.show, t, &y));
                for (int k = 0; k < 3; k++) {
                    EXPECT_DOUBLE_EQ(y.p[k], x.p[k]);
                    EXPECT_DOUBLE_EQ(y.a[k], x.a[k]);
                }
            }
        }
    }

    EXPECT_EQ(31u, rejects);  // 20 v1 + 11 provisional
    EXPECT_EQ(2u, accepts);   // skippable unknown block, header extension
}

// The E_SIZE recipe of edge/index.json: flat-single plus a skippable COMMENT
// padded with spaces so the file is 262 145 bytes
TEST(Gdmb, reject_oversize)
{
    std::vector<uint8_t> b(vectors[0].data, vectors[0].data + vectors[0].size);
    const uint32_t pad = GDMB_MAX_FILE_SIZE + 1 - b.size() - 6;
    b.push_back(0x03);
    b.push_back(0x01);
    for (int i = 0; i < 4; i++) {
        b.push_back((pad >> (8 * i)) & 0xFF);
    }
    b.insert(b.end(), pad, ' ');
    recompute_crc(b);
    ASSERT_EQ(GDMB_MAX_FILE_SIZE + 1, b.size());

    Show s;
    EXPECT_EQ(GDMB_E_SIZE, gdmb_show_parse(&s.show, b.data(), b.size(), nullptr));
}

// Every truncation and every single-byte corruption (with a valid CRC, so
// the parser goes past the CRC check) must be handled without reading out of
// bounds and without crashing.
TEST(Gdmb, robust_against_corruption)
{
    for (const Vector& v : vectors) {
        for (size_t n = 0; n < v.size; n++) {
            std::vector<uint8_t> b(v.data, v.data + n);
            if (n >= GDMB_HEADER_LENGTH) {
                recompute_crc(b);
            }
            // a cut at a block boundary that only drops ancillary blocks is
            // still a valid file, so only require that a loaded show evaluates
            Show s;
            if (gdmb_show_parse_ex(&s.show, b.data(), b.size(), parse_flags(v.provisional), nullptr) == GDMB_OK) {
                gdmb_state_t st;
                EXPECT_EQ(GDMB_OK, gdmb_show_evaluate(&s.show, s.show.duration_us / 2, &st));
            }
        }

        for (size_t i = 12; i < v.size; i++) {
            for (const uint8_t x : { 0x01, 0x80, 0xFF }) {
                std::vector<uint8_t> b(v.data, v.data + v.size);
                b[i] ^= x;
                recompute_crc(b);
                Show s;
                if (gdmb_show_parse_ex(&s.show, b.data(), b.size(), parse_flags(v.provisional), nullptr) == GDMB_OK) {
                    gdmb_state_t st;
                    for (int64_t t = -1000; t <= (int64_t)s.show.duration_us + 1000; t += 100000) {
                        EXPECT_EQ(GDMB_OK, gdmb_show_evaluate(&s.show, t, &st));
                    }
                }
            }
        }
    }
}

TEST(Gdmb, crc_mismatch_rejected_before_content)
{
    std::vector<uint8_t> b(vectors[0].data, vectors[0].data + vectors[0].size);
    b[100] ^= 0x01;
    Show s;
    EXPECT_EQ(GDMB_E_CRC, gdmb_show_parse(&s.show, b.data(), b.size(), nullptr));
}

TEST(Gdmb, flat_attitude_hover_is_identity)
{
    const gdmb_real_t a[3] = { 0, 0, 0 }, j[3] = { 0, 0, 0 };
    gdmb_real_t R[3][3], w[3], f;
    gdmb_flat_attitude(a, j, 0, 0, 1, R, w, &f);
    EXPECT_NEAR(9.80665, f, 1e-12);  // exactly GRAVITY_MSS (spec §1.1)
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            EXPECT_NEAR(r == c ? 1.0 : 0.0, R[r][c], 1e-12);
        }
        EXPECT_NEAR(0.0, w[r], 1e-12);
    }
}

// The show slot holds either format; the loader dispatches on the magic
// The heading is clockwise-positive (spec §1.2, §6.4): psi = +pi/2 points the
// nose along -y (east in NWU), and a positive yaw rate is a negative body z rate
TEST(Gdmb, flat_attitude_heading_is_clockwise)
{
    const gdmb_real_t a[3] = { 0, 0, 0 }, j[3] = { 0, 0, 0 };
    gdmb_real_t R[3][3], w[3], f;
    gdmb_flat_attitude(a, j, M_PI / 2, 0.5, 1, R, w, &f);
    EXPECT_NEAR(0.0, R[0][0], 1e-12);   // x_b = (0, -1, 0)
    EXPECT_NEAR(-1.0, R[1][0], 1e-12);
    EXPECT_NEAR(0.0, R[2][0], 1e-12);
    EXPECT_NEAR(-0.5, w[2], 1e-12);
}

TEST(Gdmb, detect_show_file_format)
{
    for (const Vector& v : vectors) {
        EXPECT_EQ(GDMB_SHOW_FILE_GDMB, gdmb_detect_show_file_format(v.data, v.size)) << v.name;
    }

    const uint8_t skyb[] = { 's', 'k', 'y', 'b', 0x02, 0x01 };
    EXPECT_EQ(GDMB_SHOW_FILE_SKYB, gdmb_detect_show_file_format(skyb, sizeof(skyb)));

    const uint8_t other[] = { 'G', 'D', 'M', 'C', 0x01 };
    EXPECT_EQ(GDMB_SHOW_FILE_UNKNOWN, gdmb_detect_show_file_format(other, sizeof(other)));
    const uint8_t upper[] = { 'S', 'K', 'Y', 'B' };
    EXPECT_EQ(GDMB_SHOW_FILE_UNKNOWN, gdmb_detect_show_file_format(upper, sizeof(upper)));
    EXPECT_EQ(GDMB_SHOW_FILE_UNKNOWN, gdmb_detect_show_file_format(skyb, 3));
    EXPECT_EQ(GDMB_SHOW_FILE_UNKNOWN, gdmb_detect_show_file_format(nullptr, 0));
}

TEST(Gdmb, skyb_header_has_crc)
{
    const uint8_t v2_crc[] = { 's', 'k', 'y', 'b', 2, 0x01, 1, 2, 3, 4 };
    const uint8_t v2_no_crc[] = { 's', 'k', 'y', 'b', 2, 0x00, 1, 2, 3, 4 };
    const uint8_t v1[] = { 's', 'k', 'y', 'b', 1, 0x01, 1, 2, 3, 4 };
    EXPECT_TRUE(gdmb_skyb_header_has_crc(v2_crc, sizeof(v2_crc)));
    EXPECT_FALSE(gdmb_skyb_header_has_crc(v2_no_crc, sizeof(v2_no_crc)));
    EXPECT_FALSE(gdmb_skyb_header_has_crc(v1, sizeof(v1)));
    EXPECT_FALSE(gdmb_skyb_header_has_crc(v2_crc, 9));
    EXPECT_FALSE(gdmb_skyb_header_has_crc(vectors[0].data, vectors[0].size));
}

namespace {

// Minimal FLAT-only .gdmb writer for synthetic shows (spec §4, §5).
// Positions in metres, yaw held at 0.
struct Seg {
    uint32_t dur_us;
    std::vector<double> x, y, z;
};

void put_u16(std::vector<uint8_t>& b, uint16_t v) { b.push_back(v & 0xFF); b.push_back(v >> 8); }
void put_u32(std::vector<uint8_t>& b, uint32_t v) { for (int i = 0; i < 4; i++) { b.push_back((v >> (8 * i)) & 0xFF); } }
void put_i32(std::vector<uint8_t>& b, int32_t v) { put_u32(b, static_cast<uint32_t>(v)); }
int32_t grid(double m) { return static_cast<int32_t>(lround(m * 10000)); }

std::vector<uint8_t> make_flat_gdmb(const double start[3], const std::vector<Seg>& segs)
{
    std::vector<uint8_t> payload;
    for (int k = 0; k < 3; k++) {
        put_i32(payload, grid(start[k]));
    }
    put_i32(payload, 0);
    for (const Seg& sg : segs) {
        put_u16(payload, static_cast<uint16_t>(sg.x.size() | (sg.y.size() << 3) | (sg.z.size() << 6)));
        put_u32(payload, sg.dur_us);
        for (const auto* axis : { &sg.x, &sg.y, &sg.z }) {
            for (double v : *axis) {
                put_i32(payload, grid(v));
            }
        }
    }

    std::vector<uint8_t> b = { 'G', 'D', 'M', 'B', 1, GDMB_FEATURE_CRC32 };
    put_u16(b, 48);
    put_u32(b, 0);                  // CRC, patched below
    b.insert(b.end(), 32, 0);       // show hash
    put_u32(b, 0);                  // capabilities
    b.push_back(0x01);              // FLAT_TRAJECTORY
    b.push_back(0x00);
    put_u32(b, payload.size());
    b.insert(b.end(), payload.begin(), payload.end());
    recompute_crc(b);
    return b;
}

// Rest-to-rest quintic from a to b: P1..P5 (spec §3.6)
std::vector<double> rest(double a, double b) { return { a, a, b, b, b }; }

}  // namespace

// A final descent that creeps sideways must not count as vertical: the
// automatic landing starts within the 50 mm threshold of the end position,
// not where the descent is still far off the pad (review of ardupilot#49:
// 0.8 m of creep over a 48 s descent).
TEST(Gdmb, landing_time_with_sideways_creep)
{
    const double start[3] = { 0, 0, 0 };
    const std::vector<uint8_t> b = make_flat_gdmb(start, {
        { 3000000, {}, {}, rest(0, 2) },              // takeoff to 2 m
        { 1000000, {}, {}, {} },                      // hold
        { 48000000, rest(0, 0.8), {}, rest(2, 0) },   // descend 2 m while creeping 0.8 m in x
    });

    Show s;
    gdmb_diag_t diag;
    ASSERT_EQ(GDMB_OK, gdmb_show_parse(&s.show, b.data(), b.size(), &diag)) << diag.what;

    uint32_t landing_us;
    ASSERT_EQ(GDMB_OK, gdmb_show_landing_time(&s.show, 0.05, 1.0, 10000, &landing_us));

    gdmb_state_t at, end;
    ASSERT_EQ(GDMB_OK, gdmb_show_evaluate(&s.show, landing_us, &at));
    ASSERT_EQ(GDMB_OK, gdmb_show_evaluate(&s.show, s.show.duration_us, &end));
    EXPECT_LE(hypot(at.p[0] - end.p[0], at.p[1] - end.p[1]), 0.05);
    // A per-piece verticality test would hand over at 1 m, 28 s into the show,
    // 0.4 m off the pad
    EXPECT_GT(landing_us, 40000000u);
}

// A truly vertical final descent hands over preferred_descent above the pad
TEST(Gdmb, landing_time_vertical_descent)
{
    const double start[3] = { 0, 0, 0 };
    const std::vector<uint8_t> b = make_flat_gdmb(start, {
        { 3000000, {}, {}, rest(0, 2) },
        { 1000000, {}, {}, {} },
        { 48000000, {}, {}, rest(2, 0) },
    });

    Show s;
    ASSERT_EQ(GDMB_OK, gdmb_show_parse(&s.show, b.data(), b.size(), nullptr));

    uint32_t landing_us;
    ASSERT_EQ(GDMB_OK, gdmb_show_landing_time(&s.show, 0.05, 1.0, 10000, &landing_us));

    // z = 1 m at the midpoint of the symmetric descent: 4 s + 24 s
    EXPECT_NEAR(28.0, landing_us * 1e-6, 0.002);
}

AP_GTEST_PANIC()
AP_GTEST_MAIN()
