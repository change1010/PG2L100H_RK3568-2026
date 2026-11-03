#include "video_dma_dashboard_input.h"
#include <stdio.h>
#include <string.h>
void vdma_ui_number_set(struct vdma_ui_number *n, unsigned int value)
{
    snprintf(n->text, sizeof(n->text), "%u", value > 255 ? 255 : value); n->replace = 1;
}
void vdma_ui_number_digit(struct vdma_ui_number *n, unsigned int digit)
{
    if (digit > 9) return;
    if (n->replace) { n->text[0] = 0; n->replace = 0; }
    size_t length = strlen(n->text);
    if (length < 4) { n->text[length] = (char)('0' + digit); n->text[length + 1] = 0; }
}
void vdma_ui_number_backspace(struct vdma_ui_number *n)
{
    size_t length = strlen(n->text);
    if (n->replace) n->text[0] = 0;
    else if (length) n->text[length - 1] = 0;
    n->replace = 0;
}
int vdma_ui_number_value(const struct vdma_ui_number *n, unsigned int *value)
{
    unsigned int v = 0;
    if (!n->text[0]) return -1;
    for (size_t i = 0; n->text[i]; i++) {
        if (n->text[i] < '0' || n->text[i] > '9' || i > 3) return -1;
        v = v * 10 + (unsigned int)(n->text[i] - '0');
    }
    if (v > 255) return -1;
    *value = v; return 0;
}
