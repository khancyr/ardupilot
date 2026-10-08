#include <AP_gtest.h>
#include <AP_JSON/AP_JSON.h>
#include <AP_HAL/AP_HAL.h>

#include <stdlib.h>
#include <string.h>
#include <type_traits>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

typedef AP_JSON::value value;

// bit exact comparison, as the parser must return the correctly rounded value
static bool same_double(double a, double b)
{
    return memcmp(&a, &b, sizeof(a)) == 0;
}

static bool parses(const char *json)
{
    value v;
    return AP_JSON::parse(v, json, strlen(json)).empty();
}

TEST(AP_JSON, ValidDocuments)
{
    EXPECT_TRUE(parses("{}"));
    EXPECT_TRUE(parses("[]"));
    EXPECT_TRUE(parses(" {\"a\" : [1, -2.5, 3e2, 0.1E-3, -0], \"b\": {\"c\": null}} \n"));
    EXPECT_TRUE(parses("{\"t\":true,\"f\":false,\"s\":\"x\\\"y\\\\z\\/\\b\\f\\n\\r\\t\"}"));
}

TEST(AP_JSON, MalformedNumbersRejected)
{
    // all accepted by strtod, none are JSON
    EXPECT_FALSE(parses("{\"a\":01}"));
    EXPECT_FALSE(parses("{\"a\":1.}"));
    EXPECT_FALSE(parses("{\"a\":.5}"));
    EXPECT_FALSE(parses("{\"a\":+1}"));
    EXPECT_FALSE(parses("{\"a\":-}"));
    EXPECT_FALSE(parses("{\"a\":1e}"));
    EXPECT_FALSE(parses("{\"a\":1.e5}"));
    EXPECT_FALSE(parses("{\"a\":1.2.3}"));
    EXPECT_FALSE(parses("{\"a\":NaN}"));
    EXPECT_FALSE(parses("{\"a\":Infinity}"));
}

TEST(AP_JSON, OtherSyntaxErrorsRejected)
{
    EXPECT_FALSE(parses("{\"a\":1,}"));
    EXPECT_FALSE(parses("{\"a\":[1,2,]}"));
    EXPECT_FALSE(parses("{a:1}"));
    EXPECT_FALSE(parses("{'a':1}"));
    EXPECT_FALSE(parses("{\"a\" 1}"));
    EXPECT_FALSE(parses("{\"a\":tru}"));
    EXPECT_FALSE(parses("{\"a\":\"\\x\"}"));
    EXPECT_FALSE(parses("{\"a\":1"));
}

TEST(AP_JSON, TrailingCharactersRejected)
{
    EXPECT_FALSE(parses("{\"a\":1} junk"));
    EXPECT_FALSE(parses("{\"a\":1}{\"b\":2}"));
    EXPECT_TRUE(parses("{\"a\":1} \t\r\n"));
}

TEST(AP_JSON, NumbersAreExact)
{
    value v;
    ASSERT_TRUE(AP_JSON::parse(v, std::string("[-7.312715117751976, 1e-7, 48.6493, 12345678901234567890]")).empty());
    const value::array &a = v.get<value::array>();
    ASSERT_EQ(4u, a.size());
    EXPECT_TRUE(same_double(-7.312715117751976, a[0].get<double>()));
    EXPECT_TRUE(same_double(1e-7, a[1].get<double>()));
    EXPECT_TRUE(same_double(48.6493, a[2].get<double>()));
    EXPECT_TRUE(same_double(12345678901234567890.0, a[3].get<double>()));
}

TEST(AP_JSON, LongNumberRejected)
{
    // longer than the internal number buffer
    std::string s = "[1." + std::string(80, '1') + "]";
    EXPECT_FALSE(parses(s.c_str()));
}

TEST(AP_JSON, UnicodeEscapes)
{
    value v;
    ASSERT_TRUE(AP_JSON::parse(v, std::string("[\"\\u0041\\u00e9\\u20ac\\ud83d\\ude81\"]")).empty());
    // A, e-acute, euro sign, and a 4 byte character from a surrogate pair, as UTF-8
    EXPECT_EQ(std::string("A\xc3\xa9\xe2\x82\xac\xf0\x9f\x9a\x81"), v.get(0).get<std::string>());

    EXPECT_FALSE(parses("[\"\\u12\"]"));
    EXPECT_FALSE(parses("[\"\\ud83d\"]"));        // lone high surrogate
    EXPECT_FALSE(parses("[\"\\ude81\"]"));        // lone low surrogate
    EXPECT_FALSE(parses("[\"\\ud83d\\u0041\"]")); // high surrogate not followed by a low one
}

TEST(AP_JSON, BufferWithoutTerminator)
{
    // parse must not read past len
    const char src[] = "{\"a\":12.5}";
    const size_t len = strlen(src);
    char *buf = new char[len];
    memcpy(buf, src, len);
    value v;
    EXPECT_TRUE(AP_JSON::parse(v, buf, len).empty());
    EXPECT_DOUBLE_EQ(12.5, v.get("a").get<double>());
    EXPECT_FALSE(AP_JSON::parse(v, buf, len - 1).empty());
    delete[] buf;
}

TEST(AP_JSON, LookupsOnWrongTypeAreSafe)
{
    value v;
    ASSERT_TRUE(AP_JSON::parse(v, std::string("{\"n\":1,\"arr\":[1,2],\"obj\":{\"k\":2}}")).empty());
    const value &n = v.get("n");
    const value &arr = v.get("arr");

    // key lookups on a number or an array return null rather than
    // reading the wrong member of the internal union
    EXPECT_TRUE(n.get("k").is<AP_JSON::null>());
    EXPECT_FALSE(n.contains("k"));
    EXPECT_TRUE(arr.get("k").is<AP_JSON::null>());
    EXPECT_FALSE(arr.contains("k"));

    // index lookups on a number or an object
    EXPECT_TRUE(n.get(0).is<AP_JSON::null>());
    EXPECT_FALSE(n.contains(0));
    EXPECT_TRUE(v.get(0).is<AP_JSON::null>());
    EXPECT_FALSE(v.contains(0));

    // and normal lookups still work
    EXPECT_TRUE(arr.contains(1));
    EXPECT_FALSE(arr.contains(2));
    EXPECT_DOUBLE_EQ(2.0, v.get("obj").get("k").get<double>());
    EXPECT_TRUE(v.get("missing").is<AP_JSON::null>());
}

TEST(AP_JSON, NumbersMatchStrtod)
{
    // numbers taking the exact fast path (<= 15 significant digits,
    // exponent within +-22) and ones just outside it must all give
    // exactly what strtod gives
    static const char *numbers[] = {
        "0", "-0", "0.0", "-0.0", "0e5", "1", "-1", "0.1", "0.2", "0.3",
        "1.5708", "-9.80665", "48.6493", "-35.3632621", "149.1652374",
        "12.0025", "0.000000000000001", "123456789012345", "999999999999999e7",
        "1e22", "1e23", "-1e-22", "1e-23", "1E0", "1e+0", "1e-0",
        "1234567890123456", "9007199254740993", "-7.312715117751976",
        "4.9e-324", "1.7976931348623157e308", "1e400", "-1e-400",
    };
    for (const char *n : numbers) {
        const std::string doc = std::string("[") + n + "]";
        value v;
        ASSERT_TRUE(AP_JSON::parse(v, doc).empty()) << n;
        EXPECT_TRUE(same_double(strtod(n, nullptr), v.get(0).get<double>())) << n;
    }

    // and a sweep of generated numbers: 1 to 17 digits, a decimal point
    // at any position and an exponent of -30 to 30
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
        n += "e" + std::to_string(int(next() % 61) - 30);
        const std::string doc = "[" + n + "]";
        value v;
        ASSERT_TRUE(AP_JSON::parse(v, doc).empty()) << n;
        ASSERT_TRUE(same_double(strtod(n.c_str(), nullptr), v.get(0).get<double>())) << n;
    }
}

TEST(AP_JSON, StringsWithEscapesBetweenPlainRuns)
{
    value v;
    ASSERT_TRUE(AP_JSON::parse(v, std::string("[\"plain text\\n\\\"quoted\\\" \\u00e9 end\", \"\", \"\\\\\"]")).empty());
    EXPECT_EQ(std::string("plain text\n\"quoted\" \xc3\xa9 end"), v.get(0).get<std::string>());
    EXPECT_EQ(std::string(""), v.get(1).get<std::string>());
    EXPECT_EQ(std::string("\\"), v.get(2).get<std::string>());

    // a raw control character in the middle of a run is invalid
    EXPECT_FALSE(parses("[\"ab\x01" "cd\"]"));
    EXPECT_FALSE(parses("[\"ab\ncd\"]"));
    // unterminated, with and without an escape at the end
    EXPECT_FALSE(parses("[\"abc"));
    EXPECT_FALSE(parses("[\"abc\\"));
}

// a growing array must move its values rather than deep copy them
static_assert(std::is_nothrow_move_constructible<AP_JSON::value>::value, "value move must be noexcept");
static_assert(std::is_nothrow_move_assignable<AP_JSON::value>::value, "value move must be noexcept");

AP_GTEST_MAIN()
