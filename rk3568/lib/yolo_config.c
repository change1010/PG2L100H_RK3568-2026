#define _XOPEN_SOURCE 700
#include "yolo_config.h"
#include "cJSON.h"
#include <errno.h>
#include <math.h>
#include <openssl/evp.h>
#include <stdlib.h>
#include <string.h>

static int digest_file(const char *path, char output[65])
{
    unsigned char block[16384], digest[EVP_MAX_MD_SIZE];
    unsigned int length = 0;
    FILE *file = fopen(path, "rb");
    EVP_MD_CTX *ctx = EVP_MD_CTX_new();
    int ok = file && ctx && EVP_DigestInit_ex(ctx, EVP_sha256(), NULL);
    if (ok) {
        size_t count;
        while ((count = fread(block, 1, sizeof(block), file))) {
            if (!EVP_DigestUpdate(ctx, block, count)) { ok = 0; break; }
        }
        ok = ok && !ferror(file) && EVP_DigestFinal_ex(ctx, digest, &length) && length == 32;
    }
    if (file) fclose(file);
    EVP_MD_CTX_free(ctx);
    if (!ok) return -EIO;
    for (unsigned int i = 0; i < 32; ++i) sprintf(output + i * 2, "%02x", digest[i]);
    return 0;
}

int yolo_config_load(struct yolo_config *out, const char *path, FILE *diagnostics)
{
    const char *keys[] = {"version", "model", "sha256", "hz", "confidence", "nms_iou", "contract"};
    struct yolo_config candidate = {0};
    char text[16385], config_path[YOLO_MODEL_PATH_BYTES], joined[YOLO_MODEL_PATH_BYTES];
    cJSON *root = NULL;
    FILE *file = NULL;
    int ret = -EINVAL;
    if (!out || !path) return ret;
    memset(out, 0, sizeof(*out));
    if (!realpath(path, config_path)) return -errno;
    file = fopen(config_path, "rb");
    if (!file) return -errno;
    size_t bytes = fread(text, 1, sizeof(text) - 1, file);
    int extra = fgetc(file), io_error = ferror(file);
    fclose(file);
    if (extra != EOF || io_error || memchr(text, 0, bytes)) return -EINVAL;
    text[bytes] = 0;
    root = cJSON_ParseWithOpts(text, NULL, 1);
    if (!cJSON_IsObject(root)) goto done;
    unsigned int seen = 0;
    for (const cJSON *item = root->child; item; item = item->next) {
        unsigned int k;
        for (k = 0; k < 7; ++k) if (!strcmp(item->string, keys[k])) break;
        if (k == 7 || (seen & (1U << k))) goto done;
        seen |= 1U << k;
    }
    if (seen != 127) goto done;
    const cJSON *version = cJSON_GetObjectItemCaseSensitive(root, "version");
    const cJSON *model = cJSON_GetObjectItemCaseSensitive(root, "model");
    const cJSON *hash = cJSON_GetObjectItemCaseSensitive(root, "sha256");
    const cJSON *hz = cJSON_GetObjectItemCaseSensitive(root, "hz");
    const cJSON *confidence = cJSON_GetObjectItemCaseSensitive(root, "confidence");
    const cJSON *nms = cJSON_GetObjectItemCaseSensitive(root, "nms_iou");
    const cJSON *contract = cJSON_GetObjectItemCaseSensitive(root, "contract");
    if (!cJSON_IsNumber(version) || version->valuedouble != 1 ||
        !cJSON_IsString(model) || !*model->valuestring || !cJSON_IsString(hash) ||
        strlen(hash->valuestring) != 64 || !cJSON_IsString(contract) ||
        strcmp(contract->valuestring, YOLO_CONFIG_CONTRACT) ||
        !cJSON_IsNumber(hz) || !isfinite(hz->valuedouble) || hz->valuedouble < 1 ||
        hz->valuedouble > 30 || floor(hz->valuedouble) != hz->valuedouble ||
        !cJSON_IsNumber(confidence) || !isfinite(confidence->valuedouble) ||
        confidence->valuedouble <= 0 || confidence->valuedouble >= 1 ||
        !cJSON_IsNumber(nms) || !isfinite(nms->valuedouble) ||
        nms->valuedouble <= 0 || nms->valuedouble >= 1) goto done;
    for (unsigned int i = 0; i < 64; ++i) {
        char c = hash->valuestring[i];
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) goto done;
    }
    int n;
    if (model->valuestring[0] == '/') n = snprintf(joined, sizeof(joined), "%s", model->valuestring);
    else {
        char *slash = strrchr(config_path, '/');
        if (!slash) goto done;
        slash[1] = 0;
        n = snprintf(joined, sizeof(joined), "%s%s", config_path, model->valuestring);
    }
    if (n < 0 || (size_t)n >= sizeof(joined)) goto done;
    if (!realpath(joined, candidate.model)) { ret = -errno; goto done; }
    ret = digest_file(candidate.model, candidate.sha256);
    if (ret) goto done;
    if (strcmp(candidate.sha256, hash->valuestring)) { ret = -EBADMSG; goto done; }
    candidate.hz = (unsigned int)hz->valuedouble;
    candidate.confidence = (float)confidence->valuedouble;
    candidate.nms_iou = (float)nms->valuedouble;
    *out = candidate; ret = 0;
    if (diagnostics) fprintf(diagnostics, "YOLO_CONFIG model=%s sha256=%s hz=%u confidence=%.6f nms=%.6f contract=" YOLO_CONFIG_CONTRACT "\n",
        out->model, out->sha256, out->hz, (double)out->confidence, (double)out->nms_iou);
done:
    cJSON_Delete(root);
    return ret;
}
