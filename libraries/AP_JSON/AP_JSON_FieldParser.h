/*
  streaming, zero allocation JSON field parser
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

/*
  Parses newline separated JSON packets one byte at a time, as they
  arrive from a UART for example, so no packet buffer is needed. Values
  whose path matches an entry in the field table are converted and
  written through the entry's pointer; everything else is checked
  against the JSON grammar and skipped. The parser uses no heap and
  its state is a few hundred bytes.

  A packet is one JSON object on a single line, optionally followed by
  '*' and four hex digits, the CRC-16/CCITT-FALSE (poly 0x1021, init
  0xFFFF) of the object's bytes, then '\n'. Whitespace may come before
  a packet. After an error the parser discards bytes up to the next
  '\n', so a damaged packet never runs into the following one.

  Values are written as they are parsed, so a packet that turns out to
  be invalid has already changed some of its fields. Point the fields at
  a scratch copy of the destination and copy it over when feed()
  returns PACKET.

  Fields:
   - path is dot separated from the root object, e.g. "imu.gyro", with at
     most MAX_PATH_DEPTH parts. Keys containing '.' or an escape
     sequence, or longer than MAX_KEY_LEN, never match. Nothing inside an
     array is addressable, apart from the elements of an array field
   - alias is an optional key in the root object for the same field. If
     a packet has both, the full path wins
   - FLOAT and DOUBLE take a number, rounded as strtof / strtod would
   - BOOL takes true, false or a number (non-zero is true)
   - FLOAT_ARRAY and DOUBLE_ARRAY take an array of exactly count numbers
   - a value of the wrong type for its field makes the packet invalid
 */
class AP_JSON_FieldParser
{
public:
    enum class Type : uint8_t {
        FLOAT,
        DOUBLE,
        BOOL,
        FLOAT_ARRAY,
        DOUBLE_ARRAY,
    };

    struct Field {
        const char *path;
        const char *alias;  // or nullptr
        Type type;
        uint8_t count;      // number of elements for the array types
        void *ptr;
    };

    enum class Result : uint8_t {
        NONE,       // part way through a packet, or between packets
        PACKET,     // a complete, valid packet ended with this byte
        ERROR,      // the current packet is invalid, see error()
    };

    static const uint8_t MAX_FIELDS = 64;
    static const uint8_t MAX_PATH_DEPTH = 3;    // parts in a field path
    static const uint8_t MAX_DEPTH = 8;         // nesting of objects and arrays
    static const uint8_t MAX_KEY_LEN = 31;      // longer keys never match
    static const uint16_t MAX_PACKET_LEN = 4096;

    // fields must stay valid while the parser is used; at most MAX_FIELDS
    AP_JSON_FieldParser(const Field *fields, uint8_t num_fields);

    // feed bytes, stopping after the first PACKET or ERROR. consumed is
    // set to the number of bytes used; feed the rest in another call
    Result feed(const uint8_t *data, size_t len, size_t &consumed);

    // feed a single byte
    Result feed(uint8_t c) {
        size_t consumed;
        return feed(&c, 1, consumed);
    }

    // bitmask of the fields present in the last packet, bit i for fields[i]
    uint64_t found() const { return _found; }

    // whether the last packet carried a CRC
    bool had_crc() const { return _had_crc; }

    // reason for the last ERROR
    const char *error() const { return _error; }

private:
    enum class State : uint8_t {
        IDLE,           // between packets
        RESYNC,         // skipping to the next '\n' after an error
        OBJ_FIRST,      // after '{': key or '}'
        OBJ_KEY,        // after ',' in an object: key
        COLON,          // after a key
        VALUE,          // expecting a value
        ARR_FIRST,      // after '[': value or ']'
        AFTER_VALUE,    // ',' or the end of the current container
        STRING,
        STRING_ESCAPE,  // after a backslash
        STRING_HEX,     // in the four digits of \uXXXX
        NUMBER,
        LITERAL,        // true, false or null
        AFTER_OBJECT,   // after the root object: optional CRC, then '\n'
        CRC_DIGITS,
        CRC_END,
    };

    // what the value being parsed is for
    enum class Target : uint8_t {
        SKIP,           // nothing in the field table, validate only
        DESCEND,        // an object holding fields deeper down
        FIELD,          // the value of fields[_target_field]
        ELEMENT,        // an element of array field fields[_target_field]
    };

    enum class NumberState : uint8_t {
        MINUS,          // after '-'
        ZERO,           // after a leading 0
        INTEGER,
        POINT,          // after '.'
        FRACTION,
        EXP,            // after 'e'
        EXP_SIGN,       // after the exponent sign
        EXP_DIGITS,
    };

    const Field *const _fields;
    const uint8_t _num_fields;

    // length of each part of each field's path (0 past the end, so a
    // field that can never match has all zeros), and of its alias
    uint8_t _part_len[MAX_FIELDS][MAX_PATH_DEPTH];
    uint8_t _alias_len[MAX_FIELDS];

    // keys of the root object hash to one of KEY_BUCKETS buckets; each
    // holds the fields whose first path part or alias has that hash, so
    // only those are compared with the key
    static const uint8_t KEY_BUCKETS = 32;
    uint64_t _bucket[KEY_BUCKETS];
    static uint8_t key_hash(uint8_t h, uint8_t c) { return uint8_t(h * 31 + c); }

    State _state = State::IDLE;
    const char *_error = nullptr;
    uint64_t _found;
    uint64_t _full_path_set;    // fields set by their full path in this packet
    uint16_t _len;              // bytes since the start of the packet
    uint16_t _crc;
    uint16_t _crc_received;
    bool _in_object;            // bytes are part of the CRC
    bool _had_crc;
    uint8_t _crc_digits;

    // containers being parsed, innermost last
    uint8_t _depth;
    uint8_t _is_object;                 // bit d set if level d is an object
    uint64_t _candidates[MAX_DEPTH];    // objects: fields that may lie below
    int8_t _array_field[MAX_DEPTH];     // arrays: field index, or -1
    uint8_t _array_count[MAX_DEPTH];    // arrays: elements of the field so far
    uint8_t _array_discard;             // bit d: array field values are discarded

    Target _target;
    int8_t _target_field;
    // type check the value but do not store it: an alias of a field
    // already set by its full path in this packet
    bool _target_discard;
    uint64_t _target_candidates;

    // current string
    bool _string_is_key;
    bool _key_matchable;
    uint8_t _key_len;
    uint8_t _key_hash;
    char _key[MAX_KEY_LEN + 1];
    uint8_t _hex_digits;
    uint16_t _hex_value;
    bool _expect_low_surrogate;

    // current number
    NumberState _number_state;
    uint8_t _number_len;
    char _number[40];

    // current literal
    const char *_literal;
    uint8_t _literal_pos;

    Result fail(uint8_t c, const char *msg);
    Result feed_byte(uint8_t c);
    Result step(uint8_t c);
    Result start_value(uint8_t c);
    Result push(uint8_t c, bool object);
    Result close_container(uint8_t c, bool object);
    Result end_key();
    Result end_number(uint8_t c);
    Result end_literal();
    void after_value();
    void set_bool(bool b);
    void mark_found(uint8_t field);
    __attribute__((always_inline)) static bool is_ws(uint8_t c) { return c == ' ' || c == '\t' || c == '\r'; }
};
