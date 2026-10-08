#include <AP_gtest.h>
#include <AP_JSON/AP_JSON_FieldParser.h>
#include <AP_JSON/AP_JSON_Number.h>
#include <AP_Math/crc.h>
#include <AP_HAL/AP_HAL.h>

#include <stdlib.h>
#include <string.h>
#include <string>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

typedef AP_JSON_FieldParser::Type T;
typedef AP_JSON_FieldParser::Result R;

static struct {
    double timestamp;
    float gyro[3];
    double position[3];
    bool flag;
    float rc1;
    float deep;
} st;

static const AP_JSON_FieldParser::Field fields[] = {
    { "timestamp", "t", T::DOUBLE, 1, &st.timestamp },
    { "imu.gyro", "g", T::FLOAT_ARRAY, 3, st.gyro },
    { "position", nullptr, T::DOUBLE_ARRAY, 3, st.position },
    { "flag", nullptr, T::BOOL, 1, &st.flag },
    { "rc.rc_1", "c1", T::FLOAT, 1, &st.rc1 },
    { "a.b.c", nullptr, T::FLOAT, 1, &st.deep },
};
static const uint64_t TIMESTAMP = 1, GYRO = 2, POSITION = 4, FLAG = 8, RC1 = 16, DEEP = 32;

// bit exact comparison
static bool same(double a, double b)
{
    return memcmp(&a, &b, sizeof(a)) == 0;
}
static bool same_f(float a, float b)
{
    return memcmp(&a, &b, sizeof(a)) == 0;
}

// feed a string, returning the results that are not NONE in order
static std::string feed(AP_JSON_FieldParser &p, const std::string &s)
{
    std::string results;
    for (char c : s) {
        switch (p.feed(uint8_t(c))) {
        case R::NONE:
            break;
        case R::PACKET:
            results += 'P';
            break;
        case R::ERROR:
            results += 'E';
            break;
        }
    }
    return results;
}

static bool accepted(const std::string &s)
{
    AP_JSON_FieldParser p(fields, 6);
    return feed(p, s) == "P";
}

TEST(AP_JSON_FieldParser, FullPathsAndValues)
{
    memset(&st, 0, sizeof(st));
    AP_JSON_FieldParser p(fields, 6);
    EXPECT_EQ("P", feed(p, "{\"timestamp\":12.5,\"imu\":{\"gyro\":[0.1,-0.2,3e-3],\"accel\":[1,2,3]},"
                           "\"position\":[1.5,-2,3],\"flag\":true,\"rc\":{\"rc_1\":1500},\"a\":{\"b\":{\"c\":7}},"
                           "\"other\":[{\"x\":null},\"s\",false]}\n"));
    EXPECT_EQ(TIMESTAMP | GYRO | POSITION | FLAG | RC1 | DEEP, p.found());
    EXPECT_FALSE(p.had_crc());
    EXPECT_TRUE(same(12.5, st.timestamp));
    EXPECT_TRUE(same_f(strtof("0.1", nullptr), st.gyro[0]));
    EXPECT_TRUE(same_f(strtof("-0.2", nullptr), st.gyro[1]));
    EXPECT_TRUE(same_f(strtof("3e-3", nullptr), st.gyro[2]));
    EXPECT_TRUE(same(-2.0, st.position[1]));
    EXPECT_TRUE(st.flag);
    EXPECT_TRUE(same_f(strtof("1500", nullptr), st.rc1));
    EXPECT_TRUE(same_f(strtof("7", nullptr), st.deep));
}

TEST(AP_JSON_FieldParser, OnlyPresentFieldsAreFound)
{
    AP_JSON_FieldParser p(fields, 6);
    EXPECT_EQ("P", feed(p, "{\"timestamp\":1}\n"));
    EXPECT_EQ(TIMESTAMP, p.found());
    EXPECT_EQ("P", feed(p, "{}\n"));
    EXPECT_EQ(0U, p.found());
}

TEST(AP_JSON_FieldParser, KeysMatchOnlyAtTheirLocation)
{
    memset(&st, 0, sizeof(st));
    AP_JSON_FieldParser p(fields, 6);
    // gyro outside imu, rc_1 at the root, timestamp inside an object or
    // an array, a key name inside a string: none of these match
    EXPECT_EQ("P", feed(p, "{\"gyro\":[1,2,3],\"rc_1\":5,\"x\":{\"timestamp\":3},\"y\":[{\"timestamp\":4}],"
                           "\"z\":\"timestamp\",\"imu\":{\"x\":{\"gyro\":[1,2,3]}}}\n"));
    EXPECT_EQ(0U, p.found());
    // keys containing a dot, an escape or longer than MAX_KEY_LEN never match
    EXPECT_EQ("P", feed(p, "{\"imu.gyro\":[1,2,3],\"t\\u0069mestamp\":1,\"" + std::string(40, 'k') + "\":1}\n"));
    EXPECT_EQ(0U, p.found());
}

TEST(AP_JSON_FieldParser, Aliases)
{
    memset(&st, 0, sizeof(st));
    AP_JSON_FieldParser p(fields, 6);
    EXPECT_EQ("P", feed(p, "{\"t\":2.5,\"g\":[1,2,3],\"c1\":1100}\n"));
    EXPECT_EQ(TIMESTAMP | GYRO | RC1, p.found());
    EXPECT_TRUE(same(2.5, st.timestamp));

    // the full path wins whatever the order
    EXPECT_EQ("P", feed(p, "{\"t\":1,\"timestamp\":2}\n"));
    EXPECT_TRUE(same(2.0, st.timestamp));
    EXPECT_EQ("P", feed(p, "{\"timestamp\":3,\"t\":4}\n"));
    EXPECT_TRUE(same(3.0, st.timestamp));

    // an alias is only a key of the root object
    EXPECT_EQ("P", feed(p, "{\"rc\":{\"c1\":5}}\n"));
    EXPECT_EQ(0U, p.found());

    // a discarded alias is still type checked, so key order does not matter
    EXPECT_FALSE(accepted("{\"timestamp\":3,\"t\":\"x\"}\n"));
    EXPECT_FALSE(accepted("{\"t\":\"x\",\"timestamp\":3}\n"));
}

TEST(AP_JSON_FieldParser, WrongTypesRejected)
{
    EXPECT_FALSE(accepted("{\"timestamp\":\"1\"}\n"));
    EXPECT_FALSE(accepted("{\"timestamp\":[1]}\n"));
    EXPECT_FALSE(accepted("{\"timestamp\":{}}\n"));
    EXPECT_FALSE(accepted("{\"timestamp\":null}\n"));
    EXPECT_FALSE(accepted("{\"timestamp\":true}\n"));
    EXPECT_FALSE(accepted("{\"imu\":{\"gyro\":[1,2]}}\n"));
    EXPECT_FALSE(accepted("{\"imu\":{\"gyro\":[1,2,3,4]}}\n"));
    EXPECT_FALSE(accepted("{\"imu\":{\"gyro\":[1,2,\"3\"]}}\n"));
    EXPECT_FALSE(accepted("{\"imu\":{\"gyro\":[1,2,[3]]}}\n"));
    EXPECT_FALSE(accepted("{\"imu\":{\"gyro\":5}}\n"));
    EXPECT_FALSE(accepted("{\"flag\":null}\n"));
    EXPECT_FALSE(accepted("{\"flag\":\"true\"}\n"));
    // a section that is not an object is simply not looked into
    EXPECT_TRUE(accepted("{\"imu\":5,\"rc\":[1,2]}\n"));
}

TEST(AP_JSON_FieldParser, Booleans)
{
    AP_JSON_FieldParser p(fields, 6);
    const char *cases[][2] = {
        { "true", "1" }, { "false", "0" }, { "1", "1" }, { "0", "0" }, { "-0.0", "0" }, { "2.5", "1" },
    };
    for (const auto &c : cases) {
        st.flag = !(c[1][0] == '1');
        EXPECT_EQ("P", feed(p, std::string("{\"flag\":") + c[0] + "}\n")) << c[0];
        EXPECT_EQ(c[1][0] == '1', st.flag) << c[0];
    }
}

TEST(AP_JSON_FieldParser, StrictJson)
{
    const char *bad[] = {
        "{\"timestamp\":01}", "{\"timestamp\":-01}", "{\"timestamp\":-00.5}", "{\"x\":-01}",
        "{\"timestamp\":1.}", "{\"timestamp\":.5}", "{\"timestamp\":+1}",
        "{\"timestamp\":-}", "{\"timestamp\":1e}", "{\"timestamp\":NaN}", "{\"timestamp\":1,}",
        "{\"x\":[1,2,]}", "{x:1}", "{'x':1}", "{\"x\" 1}", "{\"x\":tru}", "{\"x\":nul}",
        "{\"x\":\"\\q\"}", "{\"x\":\"\\u12\"}", "{\"x\":\"\\ud83d\"}", "{\"x\":\"\\ude81\"}",
        "{\"x\":\"\\ud83d\\u0041\"}", "{\"x\":\"a\x01\"}", "{\"x\":1]", "{\"x\":[1}", "{\"x\":1}}",
        "{\"x\":1} junk", "{\"x\":1}{\"y\":2}", "[1,2]", "\"x\"", "1",
    };
    for (const char *b : bad) {
        EXPECT_FALSE(accepted(std::string(b) + "\n")) << b;
    }
    const char *good[] = {
        "{}", " \t{ } \r", "{\"x\":[]}", "{\"x\":{}}", "{\"x\":\"\\\"\\\\\\/\\b\\f\\n\\r\\t\\u00e9\\ud83d\\ude81\"}",
        "{\"x\":[-0,0.5,1e5,1E-5,1e+5,-1.5e-300,123456789012345678901234567890]}",
        "{\"x\":[true,false,null,\"\",{\"y\":[[]]}]}",
    };
    for (const char *g : good) {
        EXPECT_TRUE(accepted(std::string(g) + "\n")) << g;
    }
}

TEST(AP_JSON_FieldParser, Framing)
{
    AP_JSON_FieldParser p(fields, 6);
    // nothing until the newline that ends a packet
    EXPECT_EQ("", feed(p, "{\"timestamp\":1}"));
    EXPECT_EQ("P", feed(p, "\n"));
    // blank lines and leading whitespace between packets are fine
    EXPECT_EQ("PP", feed(p, "\n\n  {\"timestamp\":1}\r\n\t{\"timestamp\":2}\n"));
    // a packet split at every byte, as from a UART
    std::string results;
    for (char c : std::string("{\"imu\":{\"gyro\":[1,2,3]}}\n")) {
        results += feed(p, std::string(1, c));
    }
    EXPECT_EQ("P", results);
    EXPECT_EQ(GYRO, p.found());
}

TEST(AP_JSON_FieldParser, ResyncAfterErrors)
{
    AP_JSON_FieldParser p(fields, 6);
    // garbage, a truncated packet and a bad packet: each gives one error
    // and the next line parses
    EXPECT_EQ("EP", feed(p, "garbage {\"timestamp\":1}\n{\"timestamp\":2}\n"));
    EXPECT_EQ("EP", feed(p, "{\"timestamp\":1,\"imu\":{\"gy\n{\"timestamp\":3}\n"));
    EXPECT_EQ("EP", feed(p, "{\"timestamp\":01}\n{\"timestamp\":4}\n"));
    EXPECT_TRUE(same(4.0, st.timestamp));
    // a newline inside a packet ends it, and what is left of it on the
    // next line is an error of its own
    EXPECT_EQ("EEP", feed(p, "{\"timestamp\":\n1}\n{\"timestamp\":5}\n"));
}

TEST(AP_JSON_FieldParser, Crc)
{
    AP_JSON_FieldParser p(fields, 6);
    // CRC-16/CCITT-FALSE of {"timestamp":1}, as binascii.crc_hqx(b'{"timestamp":1}', 0xFFFF)
    const char *body = "{\"timestamp\":1}";
    const uint16_t crc = crc16_ccitt((const uint8_t *)body, strlen(body), 0xFFFF);
    char good[40], bad[40], lower[40];
    snprintf(good, sizeof(good), "%s*%04X\n", body, unsigned(crc));
    snprintf(bad, sizeof(bad), "%s*%04X\n", body, unsigned(crc ^ 1));
    snprintf(lower, sizeof(lower), "%s *%04x \r\n", body, unsigned(crc));
    EXPECT_EQ("P", feed(p, good));
    EXPECT_TRUE(p.had_crc());
    EXPECT_EQ("P", feed(p, lower));
    EXPECT_EQ("E", feed(p, bad));
    EXPECT_STREQ("CRC mismatch", p.error());
    // a changed digit is caught
    std::string changed = good;
    changed[13] = '2';
    EXPECT_EQ("E", feed(p, changed));
    EXPECT_EQ("E", feed(p, std::string(body) + "*12G4\n"));
    EXPECT_EQ("E", feed(p, std::string(body) + "*123\n"));
    EXPECT_EQ("P", feed(p, std::string(body) + "\n"));
    EXPECT_FALSE(p.had_crc());
}

TEST(AP_JSON_FieldParser, Limits)
{
    std::string deep = "{\"x\":";
    for (int i = 0; i < 6; i++) {
        deep += "[";
    }
    std::string close(6, ']');
    // root object plus 7 levels is the maximum
    EXPECT_TRUE(accepted(deep + "[]" + close + "}\n"));
    EXPECT_FALSE(accepted(deep + "[[]]" + close + "}\n"));
    // a long string is fine, a packet over MAX_PACKET_LEN is not
    EXPECT_TRUE(accepted("{\"x\":\"" + std::string(4000, 'y') + "\"}\n"));
    EXPECT_FALSE(accepted("{\"x\":\"" + std::string(5000, 'y') + "\"}\n"));
    // a field value longer than the number buffer
    EXPECT_FALSE(accepted("{\"timestamp\":1." + std::string(60, '1') + "}\n"));
    // but a skipped one is only validated
    EXPECT_TRUE(accepted("{\"x\":1." + std::string(60, '1') + "}\n"));
}

TEST(AP_JSON_FieldParser, NumbersMatchStrtodAndStrtof)
{
    uint32_t seed = 1;
    const auto next = [&seed]() {
        seed = seed * 1664525U + 1013904223U;
        return seed >> 8;
    };
    for (uint32_t i = 0; i < 100000; i++) {
        const uint32_t ndigits = 1 + next() % 17;
        std::string digits(1, char('1' + next() % 9));
        while (digits.size() < ndigits) {
            digits += char('0' + next() % 10);
        }
        const uint32_t decimals = next() % ndigits;
        std::string n = (next() & 1) ? "-" : "";
        n += digits.substr(0, ndigits - decimals);
        if (decimals > 0) {
            n += "." + digits.substr(ndigits - decimals);
        }
        if (next() & 1) {
            n += "e" + std::to_string(int(next() % 61) - 30);
        }
        ASSERT_TRUE(same(strtod(n.c_str(), nullptr), AP_JSON_Number::to_double(n.c_str()))) << n;
        ASSERT_TRUE(same_f(strtof(n.c_str(), nullptr), AP_JSON_Number::to_float(n.c_str()))) << n;
    }
    const char *edge[] = { "0", "-0", "0.0", "1e22", "1e23", "1e10", "1e11", "1e-10", "1e-11", "9999999",
                           "16777217", "3.4028235e38", "1e39", "1e-46", "4.9e-324", "1e400" };
    for (const char *n : edge) {
        EXPECT_TRUE(same(strtod(n, nullptr), AP_JSON_Number::to_double(n))) << n;
        EXPECT_TRUE(same_f(strtof(n, nullptr), AP_JSON_Number::to_float(n))) << n;
    }
}

AP_GTEST_MAIN()
