/* SHA-256 (FIPS 180-4), used to bind a descriptor to the files it describes. */
#ifndef CPV_SHA256_H
#define CPV_SHA256_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
  uint32_t state[8];
  uint64_t length;
  uint8_t block[64];
  size_t used;
} cpv_sha256_t;

void cpv_sha256_init(cpv_sha256_t *ctx);
void cpv_sha256_update(cpv_sha256_t *ctx, const void *data, size_t size);
void cpv_sha256_final(cpv_sha256_t *ctx, uint8_t digest[32]);
/* Lower-case hex digest of a file; returns 0 on success, -1 if it cannot be read. */
int cpv_sha256_file(const char *path, char hex[65]);

#endif
