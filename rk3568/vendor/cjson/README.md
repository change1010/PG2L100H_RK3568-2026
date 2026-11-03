# cJSON

Pinned upstream: https://github.com/DaveGamble/cJSON/tree/v1.7.19

Unmodified cJSON.c/cJSON.h and MIT LICENSE, downloaded 2026-10-10.
Build with CJSON_NESTING_LIMIT=8. Runtime loading uses a 64KiB fixed allocation
arena, at most 512 allocations, a 16KiB input limit and explicit schema checks.
Hooks are used only by the single owner during configuration, then restored.
