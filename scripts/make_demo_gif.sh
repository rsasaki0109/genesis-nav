#!/usr/bin/env bash
# Records `scripts/record_demo.sh` with asciinema and converts the result to
# a GIF with `agg`. Designed to be called from `make demo-gif`.
#
# Requirements:
#   - asciinema (https://docs.asciinema.org/) on PATH
#   - agg (https://github.com/asciinema/agg) on PATH
#     Install once with: cargo install --locked agg
#
# Outputs:
#   docs/media/smoke_demo.cast — raw asciicast (kept for diffing)
#   docs/media/smoke_demo.gif  — README hero clip
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

CAST="${CAST:-docs/media/smoke_demo.cast}"
GIF="${GIF:-docs/media/smoke_demo.gif}"
COLS="${DEMO_COLS:-100}"
ROWS="${DEMO_ROWS:-28}"
FONT_SIZE="${DEMO_FONT_SIZE:-16}"
THEME="${DEMO_THEME:-monokai}"

need() {
  command -v "$1" >/dev/null 2>&1 || {
    printf 'error: %s not found on PATH. %s\n' "$1" "$2" >&2
    exit 1
  }
}
need asciinema "Install with: pip install asciinema"
need agg "Install with: cargo install --locked agg"

mkdir -p "$(dirname "$CAST")"

printf '== recording asciicast → %s\n' "$CAST"
ASCIINEMA_REC=1 asciinema rec \
  --overwrite \
  --cols "$COLS" \
  --rows "$ROWS" \
  --idle-time-limit 2 \
  --command "bash scripts/record_demo.sh" \
  "$CAST"

printf '== converting to GIF → %s\n' "$GIF"
agg \
  --font-size "$FONT_SIZE" \
  --theme "$THEME" \
  --speed 1.2 \
  "$CAST" "$GIF"

printf 'done. preview: %s\n' "$GIF"
