// Unit test for libplugin_000_01.so (class 0 / app type 1, Emergency
// Position, ANEP-87 Annex D Tables IV-V), loaded with dlopen.
// usage: plugin_000_01_test <path to libplugin_000_01.so>
#include <janus/janus.h>
#include <dlfcn.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

typedef int (*ad_enc_t)(unsigned, janus_app_fields_t, janus_uint8_t, unsigned*, janus_uint64_t*);
typedef int (*ad_dec_t)(janus_uint64_t, janus_uint8_t, unsigned*, janus_app_fields_t);
typedef int (*cg_enc_t)(janus_app_fields_t, janus_uint8_t**, unsigned*);
typedef int (*cg_dec_t)(janus_uint8_t*, unsigned, janus_app_fields_t*);
static ad_enc_t ad_enc; static ad_dec_t ad_dec; static cg_enc_t cg_enc; static cg_dec_t cg_dec;
static int fails = 0;
#define CHECK(c, ...) do { if(!(c)) { ++fails; printf("FAIL: " __VA_ARGS__); printf("\n"); } } while(0)

static const char* field(janus_app_fields_t f, const char* n) {
  for(int i = 0; f && i < f->field_count; ++i) if(!strcmp(f->fields[i].name, n)) return f->fields[i].value;
  return NULL;
}

// fields as name/value pairs, NULL terminated; returns cargo_encode result
static int encode(const char** kv, janus_uint8_t cargo[14]) {
  janus_app_fields_t f = janus_app_fields_new();
  for(int i = 0; kv[i]; i += 2) janus_app_fields_add_field(f, kv[i], kv[i + 1]);
  janus_uint8_t *c = calloc(1, 1); unsigned size = 0;
  int rv = cg_enc(f, &c, &size);
  if(rv == 0) { CHECK(size == 14, "cargo size %u", size); memcpy(cargo, c, 14); }
  janus_app_fields_free(f); free(c);
  return rv;
}

static unsigned bits(const janus_uint8_t* d, unsigned from, unsigned n) { // from: 1-based (Table V)
  unsigned v = 0;
  for(unsigned i = from - 1; i < from - 1 + n; ++i) v = (v << 1) | ((d[i / 8] >> (7 - i % 8)) & 1);
  return v;
}

int main(int argc, char** argv) {
  if(argc < 2) { printf("usage: %s <libplugin_000_01.so>\n", argv[0]); return 2; }
  void* h = dlopen(argv[1], RTLD_NOW);
  if(!h) { printf("dlopen: %s\n", dlerror()); return 1; }
  ad_enc = (ad_enc_t)dlsym(h, "app_data_encode"); ad_dec = (ad_dec_t)dlsym(h, "app_data_decode");
  cg_enc = (cg_enc_t)dlsym(h, "cargo_encode"); cg_dec = (cg_dec_t)dlsym(h, "cargo_decode");

  // ADB: unused(1) | station(8) | destination(8) | cargo size(9) = 14
  {
    janus_app_fields_t f = janus_app_fields_new();
    janus_app_fields_add_field(f, "StationIdentifier", "5");
    janus_app_fields_add_field(f, "DestinationIdentifier", "9");
    janus_uint64_t ad = 0; unsigned cs = 0;
    CHECK(ad_enc(14, f, 0, &cs, &ad) == 0 && cs == 14, "app_data_encode");
    CHECK(ad == ((5ULL << 17) | (9ULL << 9) | 14), "ADB bits 0x%llx", (unsigned long long)ad);
    janus_app_fields_t d = janus_app_fields_new(); unsigned cs2 = 0;
    CHECK(ad_dec(ad, 26, &cs2, d) == 0 && cs2 == 14, "app_data_decode");
    CHECK(!strcmp(field(d, "StationIdentifier"), "5") && !strcmp(field(d, "DestinationIdentifier"), "9"), "ids");
    CHECK(ad_dec(ad, 34, &cs2, d) != 0, "34-bit ADB (no reservation) refused");
    CHECK(ad_enc(15, f, 0, &cs, &ad) != 0, "cargo size 15 refused");
    janus_app_fields_free(d); janus_app_fields_free(f);
    f = janus_app_fields_new();
    janus_app_fields_add_field(f, "StationIdentifier", "255");
    CHECK(ad_enc(14, f, 0, &cs, &ad) != 0, "station 255 refused");
    janus_app_fields_free(f);
    f = janus_app_fields_new();
    janus_app_fields_add_field(f, "StationIdentifier", "7");
    CHECK(ad_enc(14, f, 0, &cs, &ad) == 0 && ((ad >> 9) & 0xFF) == 255, "default destination broadcast");
    janus_app_fields_free(f);
  }

  // Table V bit layout, values computed by hand
  {
    const char* kv[] = {"Latitude", "45.0", "Longitude", "-1.0", "Depth", "37", "Speed", "2.35",
                        "Heading", "90", "Nationality", "IT", NULL};
    janus_uint8_t c[14];
    CHECK(encode(kv, c) == 0, "encode full");
    CHECK(bits(c, 1, 10) == ((9u << 5) | 20u), "nationality IT = 01001 10100");
    CHECK(c[0] == 0x4D && c[1] == 0x10, "first bytes 0x%02X 0x%02X", c[0], c[1]);
    CHECK(bits(c, 11, 24) == 4194304u, "latitude 45 deg = round(45*8388607/90)");
    CHECK(bits(c, 35, 25) == ((1u << 25) - 93207u), "longitude -1 deg, 25-bit two's complement");
    CHECK(bits(c, 60, 13) == 37, "depth 37 m");
    CHECK(bits(c, 73, 9) == 24, "speed 2.35 kn -> 24 (half away from zero)");
    CHECK(bits(c, 82, 9) == 128, "heading 90 deg -> round(90/0.705) = 128");
    CHECK(bits(c, 91, 6) == 0, "padding");
    janus_uint16_t crc = janus_crc_16(c, 12, 0);
    CHECK(bits(c, 97, 16) == crc, "CRC16 over bits 1-96, big-endian");

    janus_app_fields_t d = NULL;
    CHECK(cg_dec(c, 14, &d) == 0, "decode full");
    CHECK(fabs(atof(field(d, "Latitude")) - 45.0) < 1e-5 && fabs(atof(field(d, "Longitude")) + 1.0) < 1e-5,
          "lat/lon round trip %s %s", field(d, "Latitude"), field(d, "Longitude"));
    CHECK(!strcmp(field(d, "Depth"), "37") && !strcmp(field(d, "Speed"), "2.4")
          && !strcmp(field(d, "Heading"), "90.2") && !strcmp(field(d, "Nationality"), "IT"),
          "decoded %s %s %s %s", field(d, "Depth"), field(d, "Speed"), field(d, "Heading"), field(d, "Nationality"));
    janus_app_fields_free(d);

    c[3] ^= 0x01; d = NULL;
    CHECK(cg_dec(c, 14, &d) == JANUS_ERROR_CARGO_CORRUPTED, "corrupted cargo detected");
    if(d) janus_app_fields_free(d);
  }

  // only position: the other fields are "not available"
  {
    const char* kv[] = {"Latitude", "-33.856", "Longitude", "151.215", NULL};
    janus_uint8_t c[14];
    CHECK(encode(kv, c) == 0, "encode position only");
    CHECK(bits(c, 1, 10) == 0x3FF && bits(c, 60, 13) == 8191 && bits(c, 73, 9) == 511
          && bits(c, 82, 9) == 511, "not available codes");
    janus_app_fields_t d = NULL;
    CHECK(cg_dec(c, 14, &d) == 0 && !field(d, "Depth") && !field(d, "Speed") && !field(d, "Heading")
          && !field(d, "Nationality"), "missing fields not decoded");
    CHECK(fabs(atof(field(d, "Latitude")) + 33.856) < 1e-5 && fabs(atof(field(d, "Longitude")) - 151.215) < 1e-5,
          "southern/eastern position %s %s", field(d, "Latitude"), field(d, "Longitude"));
    janus_app_fields_free(d);
  }

  // limits
  {
    janus_uint8_t c[14];
    const char* k1[] = {"Latitude", "90", "Longitude", "180", "Depth", "9000", "Speed", "60", NULL};
    CHECK(encode(k1, c) == 0 && bits(c, 11, 24) == 8388607 && bits(c, 35, 25) == 16777214
          && bits(c, 60, 13) == 8191 && bits(c, 73, 9) == 511, "+90/+180, depth/speed saturate to n/a");
    const char* k2[] = {"Latitude", "-90", "Longitude", "-180", "Heading", "359.99", NULL};
    CHECK(encode(k2, c) == 0 && bits(c, 11, 24) == (1u << 24) - 8388607u
          && bits(c, 35, 25) == (1u << 25) - 16777214u && bits(c, 82, 9) == 0,
          "-90/-180, heading 359.99 wraps to 0 (511 is n/a)");
    const char* bad[][5] = {{"Latitude", "91", "Longitude", "0", NULL}, {"Latitude", "0", "Longitude", "-181", NULL},
                            {"Latitude", "x", "Longitude", "0", NULL}, {"Longitude", "0", NULL, NULL, NULL}};
    for(int i = 0; i < 4; ++i) CHECK(encode(bad[i], c) == JANUS_ERROR_FIELDS, "bad position %d refused", i);
    const char* k3[] = {"Latitude", "0", "Longitude", "0", "Nationality", "I", NULL};
    CHECK(encode(k3, c) == JANUS_ERROR_FIELDS, "nationality must be two letters");
    const char* k4[] = {"Latitude", "0", "Longitude", "0", "Depth", "-3", NULL};
    CHECK(encode(k4, c) == JANUS_ERROR_FIELDS, "negative depth refused");
  }

  printf("%s\n", fails ? "PLUGIN 000_01 TEST FAILED" : "ALL PLUGIN 000_01 TESTS PASSED");
  return fails ? 1 : 0;
}
