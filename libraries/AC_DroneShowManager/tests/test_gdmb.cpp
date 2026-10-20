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
        ASSERT_EQ(GDMB_OK, gdmb_show_parse(&s.show, v.data, v.size, &diag))
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
        ASSERT_EQ(GDMB_OK, gdmb_show_parse(&s.show, v.data, v.size, nullptr)) << v.name;

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

    Show s;
    ASSERT_EQ(GDMB_OK, gdmb_show_parse(&s.show, v.data, v.size, nullptr));
    EXPECT_EQ(18000000u, s.show.duration_us);
    EXPECT_EQ(0u, s.show.num_maneuvers);
    EXPECT_EQ(GDMB_FEATURE_CRC32, s.show.features);
    EXPECT_EQ(0u, s.show.capabilities);
    EXPECT_FALSE(gdmb_show_has_inverted_flat(&s.show));
    EXPECT_NE(nullptr, s.show.lights);
    EXPECT_GT(s.show.lights_length, 0u);
    EXPECT_EQ(2u, s.show.num_events);
    EXPECT_EQ(6u, s.show.num_envelopes);
    EXPECT_EQ(3u, s.show.num_rth_entries);
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

TEST(Gdmb, maneuver_single_contents)
{
    const Vector& v = vectors[1];
    ASSERT_STREQ("maneuver-single", v.name);

    Show s;
    ASSERT_EQ(GDMB_OK, gdmb_show_parse(&s.show, v.data, v.size, nullptr));
    EXPECT_EQ(1u, s.show.num_maneuvers);
    EXPECT_TRUE(s.show.features & GDMB_FEATURE_MANEUVER);
    EXPECT_TRUE(s.show.capabilities & GDMB_CAP_MANEUVER);
}

// Acceleration is continuous across every FLAT/FLAT joint of the golden
// vectors (the point of C², spec §7.3)
TEST(Gdmb, joints_are_c2)
{
    for (const Vector& v : vectors) {
        Show s;
        ASSERT_EQ(GDMB_OK, gdmb_show_parse(&s.show, v.data, v.size, nullptr));
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
        const gdmb_error_t err = gdmb_show_parse(&s.show, e.data, e.size, &diag);
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

    EXPECT_EQ(27u, rejects);
    EXPECT_EQ(1u, accepts);
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
            if (gdmb_show_parse(&s.show, b.data(), b.size(), nullptr) == GDMB_OK) {
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
                if (gdmb_show_parse(&s.show, b.data(), b.size(), nullptr) == GDMB_OK) {
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

AP_GTEST_PANIC()
AP_GTEST_MAIN()
