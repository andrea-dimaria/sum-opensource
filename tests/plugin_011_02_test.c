// Unit test for libplugin_011_02.so (class 11 / app type 2), loaded with dlopen.
// usage: plugin_011_02_test <path to libplugin_011_02.so>
#include <janus/janus.h>
#include <dlfcn.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

typedef int (*ad_enc_t)(unsigned, janus_app_fields_t, janus_uint8_t, unsigned*, janus_uint64_t*);
typedef int (*ad_dec_t)(janus_uint64_t, janus_uint8_t, unsigned*, janus_app_fields_t);
typedef int (*cg_enc_t)(janus_app_fields_t, janus_uint8_t**, unsigned*);
typedef int (*cg_dec_t)(janus_uint8_t*, unsigned, janus_app_fields_t*);
static ad_enc_t ad_enc; static ad_dec_t ad_dec; static cg_enc_t cg_enc; static cg_dec_t cg_dec;
static int fails = 0;
#define CHECK(c, ...) do { if(!(c)) { ++fails; printf("FAIL: " __VA_ARGS__); printf("\n"); } } while(0)

static const char* field(janus_app_fields_t f, const char* n) {
  for(int i = 0; i < f->field_count; ++i) if(!strcmp(f->fields[i].name, n)) return f->fields[i].value;
  return NULL;
}

// expected cargo total for text length L (spec table, rounded up)
static unsigned expected_total(unsigned L) {
  unsigned t = L + 2;
  if(t <= 32) return t;
  if(t <= 96) return t < 34 ? 34 : (t + 1) / 2 * 2;
  if(t <= 224) return t < 100 ? 100 : (t + 3) / 4 * 4;
  return t < 232 ? 232 : (t + 7) / 8 * 8;
}

static int roundtrip(unsigned L, unsigned *total_out) {
  char *text = malloc(L + 1), sz[8];
  for(unsigned i = 0; i < L; ++i) text[i] = 'a' + i % 26;
  text[L] = 0; sprintf(sz, "%u", L);
  janus_app_fields_t f = janus_app_fields_new();
  janus_app_fields_add_field(f, "Payload Size", sz);
  janus_app_fields_add_field(f, "Payload", text);
  janus_app_fields_add_field(f, "StationIdentifier", "5");
  janus_app_fields_add_field(f, "DestinationIdentifier", "9");
  janus_uint8_t *cargo = calloc(1, 1); unsigned csize = 0, cs2 = 0, cs3 = 0;
  int rv = cg_enc(f, &cargo, &csize);
  if(rv) { janus_app_fields_free(f); free(cargo); free(text); return rv; }
  *total_out = csize;
  janus_uint64_t ad;
  rv = ad_enc(csize, f, 26, &cs2, &ad);
  CHECK(rv == 0 && cs2 == csize, "app_data_encode L=%u rv=%d", L, rv);
  CHECK(ad < (1ULL << 26), "app_data overflow L=%u", L);
  janus_app_fields_t d = janus_app_fields_new();
  rv = ad_dec(ad, 26, &cs3, d);
  CHECK(rv == 0 && cs3 == csize, "app_data_decode L=%u size %u != %u", L, cs3, csize);
  CHECK(!strcmp(field(d, "StationIdentifier"), "5") && !strcmp(field(d, "DestinationIdentifier"), "9"),
        "ids L=%u", L);
  janus_app_fields_t c = NULL;
  rv = cg_dec(cargo, csize, &c);
  CHECK(rv == 0 && atoi(field(c, "Payload Size")) == (int)L && !memcmp(field(c, "Payload"), text, L),
        "cargo_decode L=%u rv=%d", L, rv);
  janus_app_fields_free(f); janus_app_fields_free(d); if(c) janus_app_fields_free(c);
  free(cargo); free(text);
  return 0;
}

int main(int argc, char** argv) {
  void* h = dlopen(argv[1], RTLD_NOW);
  if(!h) { printf("dlopen: %s\n", dlerror()); return 1; }
  ad_enc = (ad_enc_t)dlsym(h, "app_data_encode"); ad_dec = (ad_dec_t)dlsym(h, "app_data_decode");
  cg_enc = (cg_enc_t)dlsym(h, "cargo_encode"); cg_dec = (cg_dec_t)dlsym(h, "cargo_decode");

  for(unsigned L = 1; L <= 478; ++L) {
    unsigned total = 0;
    int rv = roundtrip(L, &total);
    CHECK(rv == 0, "roundtrip L=%u rv=%d", L, rv);
    CHECK(total == expected_total(L), "total L=%u got %u expected %u", L, total, expected_total(L));
  }
  unsigned t; CHECK(roundtrip(479, &t) == JANUS_ERROR_CARGO_SIZE, "L=479 must fail");

  // boundary totals: (text length -> cargo total)
  unsigned b[][2] = {{30,32},{31,34},{32,34},{94,96},{95,100},{98,100},{222,224},{223,232},{230,232},{478,480}};
  for(unsigned i = 0; i < sizeof(b)/sizeof(b[0]); ++i) {
    roundtrip(b[i][0], &t); CHECK(t == b[i][1], "boundary L=%u total %u expected %u", b[i][0], t, b[i][1]);
  }
  // non representable / wrong ADB size / station 255 rejected by app_data_encode
  janus_app_fields_t f = janus_app_fields_new(); janus_uint64_t ad; unsigned cs;
  janus_app_fields_add_field(f, "StationIdentifier", "5");
  CHECK(ad_enc(33, f, 26, &cs, &ad) != 0, "33 not representable");
  CHECK(ad_enc(34, f, 34, &cs, &ad) != 0, "34-bit ADB (no schedule) must fail");
  janus_app_fields_t f2 = janus_app_fields_new(); janus_app_fields_add_field(f2, "StationIdentifier", "255");
  CHECK(ad_enc(10, f2, 26, &cs, &ad) != 0, "station 255 must fail");
  // default destination is broadcast
  CHECK(ad_enc(10, f, 26, &cs, &ad) == 0 && ((ad >> 3) & 0xFF) == 255, "default dest 255");

  // corrupted CRC / corrupted text
  janus_app_fields_t p = janus_app_fields_new();
  janus_app_fields_add_field(p, "Payload", "hello");
  janus_uint8_t* cargo = calloc(1,1); unsigned csize;
  cg_enc(p, &cargo, &csize);
  printf("'hello' cargo (%u bytes):", csize); for(unsigned i = 0; i < csize; ++i) printf(" %02x", cargo[i]); printf("\n");
  janus_app_fields_t c = NULL;
  cargo[csize-1] ^= 1; CHECK(cg_dec(cargo, csize, &c) == JANUS_ERROR_CARGO_CORRUPTED, "bad CRC"); cargo[csize-1] ^= 1;
  cargo[0] ^= 0x20; CHECK(cg_dec(cargo, csize, &c) == JANUS_ERROR_CARGO_CORRUPTED, "bad text");

  printf(fails ? "%d FAILURES\n" : "ALL TESTS PASSED\n", fails);
  return fails != 0;
}
