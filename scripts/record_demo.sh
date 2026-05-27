#!/usr/bin/env bash
# Synthesises the demo sequence captured in docs/demo_script.md so that
# `asciinema rec --command "bash scripts/record_demo.sh"` produces a
# deterministic asciicast suitable for converting to a README GIF with `agg`.
#
# Run via `make demo-gif` from the repo root. Run it directly only when you
# want to eyeball the sequence without recording.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

# Make sure the venv-installed `gnav` is on PATH if we're inside one.
if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  export PATH="${VIRTUAL_ENV}/bin:${PATH}"
fi
export PATH="${ROOT}/scripts:${PATH}"

PAUSE_SHORT="${DEMO_PAUSE_SHORT:-0.6}"
PAUSE_LONG="${DEMO_PAUSE_LONG:-1.4}"

# A typed-prompt emulator: prints `$ <cmd>` slowly so the GIF reads like a
# screencast instead of dumping output instantly.
type_cmd() {
  local cmd="$1"
  printf '$ '
  local i=0
  while (( i < ${#cmd} )); do
    printf '%s' "${cmd:i:1}"
    sleep 0.02
    i=$((i + 1))
  done
  printf '\n'
}

run_step() {
  local label="$1"
  local cmd="$2"
  printf '\n\033[1;36m# %s\033[0m\n' "$label"
  sleep "$PAUSE_SHORT"
  type_cmd "$cmd"
  sleep "$PAUSE_SHORT"
  bash -c "$cmd" || true
  sleep "$PAUSE_LONG"
}

# Use a throwaway run directory so we don't fill the user's runs/ tree.
RUN_ROOT="$(mktemp -d -t gnav-demo-XXXXXX)"
trap 'rm -rf "$RUN_ROOT"' EXIT

printf '\033[1;35m== genesis-nav v0.1 — 30-second tour ==\033[0m\n'
sleep "$PAUSE_LONG"

run_step "1. run a scenario" \
  "gnav run examples/scenarios/smoke.yaml --fast --record --output-dir '$RUN_ROOT' 2>&1 | tail -n 12"

LATEST="$(ls -1dt "$RUN_ROOT"/*/ 2>/dev/null | head -n 1 || true)"

if [[ -n "$LATEST" ]]; then
  run_step "2. inspect the run directory" \
    "ls '$LATEST' && head -n 20 '${LATEST%/}/report.md' 2>/dev/null"
  run_step "3. replay the events" \
    "gnav replay '${LATEST%/}' --print-events 2>&1 | head -n 12"
fi

run_step "4. run the same scenario as a benchmark" \
  "gnav bench --run benchmarks/nav_basic 2>&1 | tail -n 12"

printf '\n\033[1;35m== docs: README.md • CONTRIBUTING.md • docs/good_first_issues.md ==\033[0m\n'
sleep "$PAUSE_LONG"
