#include "json.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_DEPTH 64

typedef struct {
  const char *text;
  size_t length, pos;
  char *error;
  size_t error_size;
} parser_t;

static int fail(parser_t *p, const char *what) {
  if (p->error && p->error_size)
    snprintf(p->error, p->error_size, "JSON: %s at byte %zu", what, p->pos);
  return -1;
}

static void skip_space(parser_t *p) {
  while (p->pos < p->length && strchr(" \t\r\n", p->text[p->pos]) && p->text[p->pos])
    p->pos++;
}

static int literal(parser_t *p, const char *word) {
  size_t n = strlen(word);
  if (p->length - p->pos < n || memcmp(p->text + p->pos, word, n) != 0)
    return fail(p, "invalid literal");
  p->pos += n;
  return 0;
}

static int hex4(parser_t *p, unsigned *out) {
  if (p->length - p->pos < 4)
    return fail(p, "truncated \\u escape");
  unsigned v = 0;
  for (int i = 0; i < 4; i++) {
    char c = p->text[p->pos++];
    v <<= 4;
    if (c >= '0' && c <= '9') v |= (unsigned)(c - '0');
    else if (c >= 'a' && c <= 'f') v |= (unsigned)(c - 'a' + 10);
    else if (c >= 'A' && c <= 'F') v |= (unsigned)(c - 'A' + 10);
    else return fail(p, "invalid \\u escape");
  }
  *out = v;
  return 0;
}

static int parse_string(parser_t *p, char **out) {
  p->pos++; /* opening quote */
  size_t cap = 16, len = 0;
  char *s = malloc(cap);
  if (!s)
    return fail(p, "out of memory");
  for (;;) {
    if (p->pos >= p->length) {
      free(s);
      return fail(p, "unterminated string");
    }
    unsigned char c = (unsigned char)p->text[p->pos++];
    char buf[4];
    size_t n = 0;
    if (c == '"')
      break;
    if (c < 0x20) {
      free(s);
      return fail(p, "control character in string");
    }
    if (c != '\\') {
      buf[n++] = (char)c;
    } else {
      if (p->pos >= p->length) {
        free(s);
        return fail(p, "truncated escape");
      }
      char e = p->text[p->pos++];
      unsigned cp = 0;
      switch (e) {
      case '"': buf[n++] = '"'; break;
      case '\\': buf[n++] = '\\'; break;
      case '/': buf[n++] = '/'; break;
      case 'b': buf[n++] = '\b'; break;
      case 'f': buf[n++] = '\f'; break;
      case 'n': buf[n++] = '\n'; break;
      case 'r': buf[n++] = '\r'; break;
      case 't': buf[n++] = '\t'; break;
      case 'u':
        if (hex4(p, &cp)) {
          free(s);
          return -1;
        }
        if (cp >= 0xD800 && cp <= 0xDBFF) {
          unsigned lo;
          if (p->length - p->pos < 2 || p->text[p->pos] != '\\' || p->text[p->pos + 1] != 'u') {
            free(s);
            return fail(p, "unpaired surrogate");
          }
          p->pos += 2;
          if (hex4(p, &lo) || lo < 0xDC00 || lo > 0xDFFF) {
            free(s);
            return fail(p, "invalid surrogate pair");
          }
          cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
        } else if (cp >= 0xDC00 && cp <= 0xDFFF) {
          free(s);
          return fail(p, "unpaired surrogate");
        }
        if (cp == 0) {
          free(s);
          return fail(p, "NUL in string");
        }
        if (cp < 0x80) buf[n++] = (char)cp;
        else if (cp < 0x800) {
          buf[n++] = (char)(0xC0 | cp >> 6);
          buf[n++] = (char)(0x80 | (cp & 0x3F));
        } else if (cp < 0x10000) {
          buf[n++] = (char)(0xE0 | cp >> 12);
          buf[n++] = (char)(0x80 | (cp >> 6 & 0x3F));
          buf[n++] = (char)(0x80 | (cp & 0x3F));
        } else {
          buf[n++] = (char)(0xF0 | cp >> 18);
          buf[n++] = (char)(0x80 | (cp >> 12 & 0x3F));
          buf[n++] = (char)(0x80 | (cp >> 6 & 0x3F));
          buf[n++] = (char)(0x80 | (cp & 0x3F));
        }
        break;
      default:
        free(s);
        return fail(p, "invalid escape");
      }
    }
    if (len + n + 1 > cap) {
      cap = cap * 2 + n;
      char *grown = realloc(s, cap);
      if (!grown) {
        free(s);
        return fail(p, "out of memory");
      }
      s = grown;
    }
    memcpy(s + len, buf, n);
    len += n;
  }
  s[len] = '\0';
  *out = s;
  return 0;
}

static int parse_number(parser_t *p, double *out) {
  size_t start = p->pos;
  const char *t = p->text;
  if (p->pos < p->length && t[p->pos] == '-') p->pos++;
  if (p->pos >= p->length) return fail(p, "invalid number");
  if (t[p->pos] == '0') p->pos++;
  else if (t[p->pos] >= '1' && t[p->pos] <= '9')
    while (p->pos < p->length && t[p->pos] >= '0' && t[p->pos] <= '9') p->pos++;
  else return fail(p, "invalid number");
  if (p->pos < p->length && t[p->pos] == '.') {
    p->pos++;
    if (p->pos >= p->length || t[p->pos] < '0' || t[p->pos] > '9') return fail(p, "invalid number");
    while (p->pos < p->length && t[p->pos] >= '0' && t[p->pos] <= '9') p->pos++;
  }
  if (p->pos < p->length && (t[p->pos] == 'e' || t[p->pos] == 'E')) {
    p->pos++;
    if (p->pos < p->length && (t[p->pos] == '+' || t[p->pos] == '-')) p->pos++;
    if (p->pos >= p->length || t[p->pos] < '0' || t[p->pos] > '9') return fail(p, "invalid number");
    while (p->pos < p->length && t[p->pos] >= '0' && t[p->pos] <= '9') p->pos++;
  }
  char buf[64];
  size_t n = p->pos - start;
  if (n >= sizeof(buf)) return fail(p, "number too long");
  memcpy(buf, t + start, n);
  buf[n] = '\0';
  errno = 0;
  *out = strtod(buf, NULL);
  if (errno == ERANGE) return fail(p, "number out of range");
  return 0;
}

static int parse_value(parser_t *p, cpv_json_t *v, int depth);

static int push(parser_t *p, cpv_json_t *v, size_t *cap) {
  if (v->count < *cap) return 0;
  size_t grown = *cap ? *cap * 2 : 8;
  cpv_json_t *items = realloc(v->items, grown * sizeof(*items));
  if (!items) return fail(p, "out of memory");
  v->items = items;
  if (v->type == CPV_JSON_OBJECT) {
    char **keys = realloc(v->keys, grown * sizeof(*keys));
    if (!keys) return fail(p, "out of memory");
    v->keys = keys;
  }
  *cap = grown;
  return 0;
}

static int parse_container(parser_t *p, cpv_json_t *v, int depth) {
  int object = p->text[p->pos] == '{';
  char close = object ? '}' : ']';
  size_t cap = 0;
  v->type = object ? CPV_JSON_OBJECT : CPV_JSON_ARRAY;
  p->pos++;
  skip_space(p);
  if (p->pos < p->length && p->text[p->pos] == close) {
    p->pos++;
    return 0;
  }
  for (;;) {
    skip_space(p);
    if (push(p, v, &cap)) return -1;
    if (object) {
      if (p->pos >= p->length || p->text[p->pos] != '"') return fail(p, "expected object key");
      char *key;
      if (parse_string(p, &key)) return -1;
      for (size_t i = 0; i < v->count; i++)
        if (strcmp(v->keys[i], key) == 0) {
          free(key);
          return fail(p, "duplicate object key");
        }
      v->keys[v->count] = key;
      skip_space(p);
      if (p->pos >= p->length || p->text[p->pos] != ':') {
        v->items[v->count] = (cpv_json_t){0};
        v->count++;
        return fail(p, "expected ':'");
      }
      p->pos++;
    }
    memset(&v->items[v->count], 0, sizeof(cpv_json_t));
    v->count++;
    if (parse_value(p, &v->items[v->count - 1], depth + 1)) return -1;
    skip_space(p);
    if (p->pos < p->length && p->text[p->pos] == ',') {
      p->pos++;
      continue;
    }
    if (p->pos < p->length && p->text[p->pos] == close) {
      p->pos++;
      return 0;
    }
    return fail(p, object ? "expected ',' or '}'" : "expected ',' or ']'");
  }
}

static int parse_value(parser_t *p, cpv_json_t *v, int depth) {
  if (depth > MAX_DEPTH) return fail(p, "nesting too deep");
  skip_space(p);
  if (p->pos >= p->length) return fail(p, "unexpected end");
  char c = p->text[p->pos];
  switch (c) {
  case '{':
  case '[': return parse_container(p, v, depth);
  case '"': v->type = CPV_JSON_STRING; return parse_string(p, &v->string);
  case 't': v->type = CPV_JSON_BOOL; v->boolean = 1; return literal(p, "true");
  case 'f': v->type = CPV_JSON_BOOL; v->boolean = 0; return literal(p, "false");
  case 'n': v->type = CPV_JSON_NULL; return literal(p, "null");
  default: v->type = CPV_JSON_NUMBER; return parse_number(p, &v->number);
  }
}

static void release(cpv_json_t *v) {
  for (size_t i = 0; i < v->count; i++) {
    release(&v->items[i]);
    if (v->type == CPV_JSON_OBJECT) free(v->keys[i]);
  }
  free(v->items);
  free(v->keys);
  free(v->string);
}

cpv_json_t *cpv_json_parse(const char *text, size_t length, char *error, size_t error_size) {
  parser_t p = {text, length, 0, error, error_size};
  cpv_json_t *root = calloc(1, sizeof(*root));
  if (!root) {
    fail(&p, "out of memory");
    return NULL;
  }
  if (parse_value(&p, root, 0) == 0) {
    skip_space(&p);
    if (p.pos == p.length) return root;
    fail(&p, "trailing data");
  }
  release(root);
  free(root);
  return NULL;
}

void cpv_json_free(cpv_json_t *value) {
  if (!value) return;
  release(value);
  free(value);
}

const cpv_json_t *cpv_json_get(const cpv_json_t *object, const char *key) {
  if (!object || object->type != CPV_JSON_OBJECT) return NULL;
  for (size_t i = 0; i < object->count; i++)
    if (strcmp(object->keys[i], key) == 0) return &object->items[i];
  return NULL;
}
