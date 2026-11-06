/* A small strict JSON reader (RFC 8259) for runtime descriptors. Not a general library. */
#ifndef CPV_JSON_H
#define CPV_JSON_H

#include <stddef.h>

typedef enum { CPV_JSON_NULL, CPV_JSON_BOOL, CPV_JSON_NUMBER, CPV_JSON_STRING,
               CPV_JSON_ARRAY, CPV_JSON_OBJECT } cpv_json_type_t;

typedef struct cpv_json {
  cpv_json_type_t type;
  int boolean;
  double number;
  char *string;           /* NUL-terminated UTF-8 (strings containing NUL are rejected) */
  size_t count;           /* array or object members */
  struct cpv_json *items; /* array elements or object values */
  char **keys;            /* object keys */
} cpv_json_t;

/* Parse text; returns NULL and fills error (with the byte offset) on failure. */
cpv_json_t *cpv_json_parse(const char *text, size_t length, char *error, size_t error_size);
void cpv_json_free(cpv_json_t *value);
/* Object member by key, or NULL (also when value is not an object). */
const cpv_json_t *cpv_json_get(const cpv_json_t *object, const char *key);

#endif
