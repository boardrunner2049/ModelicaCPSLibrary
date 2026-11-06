/* Unit tests of the runtime's JSON reader and SHA-256. */
#include "json.h"
#include "sha256.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond);                     \
      failures++;                                                                                  \
    }                                                                                              \
  } while (0)

static void hex_of(const char *text, char out[65]) {
  cpv_sha256_t ctx;
  uint8_t d[32];
  cpv_sha256_init(&ctx);
  cpv_sha256_update(&ctx, text, strlen(text));
  cpv_sha256_final(&ctx, d);
  for (int i = 0; i < 32; i++)
    snprintf(out + 2 * i, 3, "%02x", d[i]);
}

static void test_sha256(void) {
  char h[65];
  hex_of("", h);
  CHECK(strcmp(h, "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855") == 0);
  hex_of("abc", h);
  CHECK(strcmp(h, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad") == 0);
  hex_of("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq", h);
  CHECK(strcmp(h, "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1") == 0);
  /* One million 'a', fed in uneven chunks. */
  cpv_sha256_t ctx;
  uint8_t d[32];
  char block[997];
  memset(block, 'a', sizeof(block));
  cpv_sha256_init(&ctx);
  size_t left = 1000000;
  while (left) {
    size_t n = left < sizeof(block) ? left : sizeof(block);
    cpv_sha256_update(&ctx, block, n);
    left -= n;
  }
  cpv_sha256_final(&ctx, d);
  for (int i = 0; i < 32; i++)
    snprintf(h + 2 * i, 3, "%02x", d[i]);
  CHECK(strcmp(h, "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0") == 0);
}

static void test_json(void) {
  char err[128];
  const char *doc = "{\"a\": [1, -2.5e3, 0], \"s\": \"x\\u00e9\\ud83d\\ude00\\n\", "
                    "\"o\": {\"t\": true, \"f\": false, \"n\": null}}";
  cpv_json_t *v = cpv_json_parse(doc, strlen(doc), err, sizeof(err));
  CHECK(v != NULL);
  if (v) {
    const cpv_json_t *a = cpv_json_get(v, "a");
    CHECK(a && a->type == CPV_JSON_ARRAY && a->count == 3);
    CHECK(a && fabs(a->items[1].number + 2500.0) < 1e-12);
    const cpv_json_t *s = cpv_json_get(v, "s");
    CHECK(s && strcmp(s->string, "x\xc3\xa9\xf0\x9f\x98\x80\n") == 0);
    const cpv_json_t *o = cpv_json_get(v, "o");
    CHECK(cpv_json_get(o, "t")->boolean == 1 && cpv_json_get(o, "n")->type == CPV_JSON_NULL);
    CHECK(cpv_json_get(v, "missing") == NULL);
    cpv_json_free(v);
  }
  const char *bad[] = {"", "{", "[1,]", "{\"a\":1,}", "{\"a\" 1}", "01", "1.", "-", "\"\\x\"",
                       "\"a", "{\"a\":1,\"a\":2}", "[1] 2", "\"\\u0000\"", "\"\\ud800\"",
                       "tru", "nul", "\"\x01\""};
  for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
    cpv_json_t *b = cpv_json_parse(bad[i], strlen(bad[i]), err, sizeof(err));
    if (b) {
      fprintf(stderr, "accepted invalid JSON: %s\n", bad[i]);
      failures++;
      cpv_json_free(b);
    }
  }
  /* Nesting limit. */
  char deep[300];
  memset(deep, '[', 200);
  CHECK(cpv_json_parse(deep, 200, err, sizeof(err)) == NULL);
}

int main(void) {
  test_sha256();
  test_json();
  if (failures)
    fprintf(stderr, "%d failure(s)\n", failures);
  else
    printf("internals: all checks passed\n");
  return failures != 0;
}
