#!/usr/bin/env bash
# test_configs.sh — Verify that all valid configuration combinations pass
# OpenSCAD's assert() checks without rendering geometry.
#
# Usage:
#   ./test_configs.sh [OPTIONS]
#
# Options:
#   -j N        Run N jobs in parallel (default: $(nproc))
#   --verbose   Print each configuration as it starts
#   --dry-run   Print configurations without running them
#   -h, --help  Show this help message
#
# What this tests:
#   Every combination of the feature toggles that guard conditional asserts:
#     ENABLE_OVERLAY_RABBET  (2 states)
#     ENABLE_MAGNET_POCKETS  (2 states)
#     ENABLE_DECORATIVE_TRIMS + sub-toggles (decoratives on: 16 combos, off: 1)
#     PRINT_MODE × PRINT_SUPPORTS (3 meaningful combos)
#     SHOW_TRAY / SHOW_OVERLAY (3 piece combos: tray-only, overlay-only, both)
#
#   Total unique configurations: see --dry-run output for the full matrix.
#
#   OpenSCAD evaluates all top-level assert() calls during the compile phase
#   before any geometry is rendered. Using -o /dev/null with an empty .csg
#   output triggers compilation (and therefore all asserts) without spending
#   time on CGAL mesh boolean operations.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SCAD_FILE="$SCRIPT_DIR/case.scad"

PARALLEL=$(nproc 2>/dev/null || echo 4)
VERBOSE=false
DRY_RUN=false

usage() {
    sed -n '2,/^$/s/^# \?//p' "$0"
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -j)         PARALLEL="$2"; shift 2 ;;
        --verbose)  VERBOSE=true; shift ;;
        --dry-run)  DRY_RUN=true; shift ;;
        -h|--help)  usage ;;
        *)          echo "Unknown option: $1" >&2; exit 1 ;;
    esac
done

if ! command -v openscad &>/dev/null; then
    echo "Error: openscad not found in PATH." >&2
    exit 1
fi

TMPDIR_BASE=$(mktemp -d)
trap 'rm -rf "$TMPDIR_BASE"' EXIT

PASS=0
FAIL=0
TOTAL=0
FAILED_CONFIGS=()

run_config() {
    local label="$1"
    shift
    local defs=("$@")

    TOTAL=$((TOTAL + 1))

    if $DRY_RUN; then
        printf "  [%3d] %s\n" "$TOTAL" "$label"
        return 0
    fi

    if $VERBOSE; then
        printf "  [%3d] %-60s ... " "$TOTAL" "$label"
    fi

    local logfile="$TMPDIR_BASE/config_${TOTAL}.log"

    # -o /dev/null with .csg extension triggers compile-only (no CGAL render).
    # All assert() calls run during compilation.
    local dummy_out="$TMPDIR_BASE/config_${TOTAL}.csg"
    if openscad -o "$dummy_out" "${defs[@]}" -D '$fn=8' "$SCAD_FILE" >"$logfile" 2>&1; then
        PASS=$((PASS + 1))
        if $VERBOSE; then
            echo "PASS"
        fi
    else
        FAIL=$((FAIL + 1))
        FAILED_CONFIGS+=("$label")
        if $VERBOSE; then
            echo "FAIL"
            sed 's/^/    | /' "$logfile"
        fi
    fi
}

echo "=== mKey Case Configuration Test Suite ==="
echo "Source: $SCAD_FILE"
echo ""

# ---------------------------------------------------------------------------
# Build the configuration matrix
# ---------------------------------------------------------------------------
#
# Feature toggles that guard conditional asserts:
#   ENABLE_OVERLAY_RABBET:  guards assertions 16-21, and 29-32 (when combined
#                           with ENABLE_MAGNET_POCKETS)
#   ENABLE_MAGNET_POCKETS:  guards assertions 22-28, and 29-32 (when combined
#                           with ENABLE_OVERLAY_RABBET)
#   ENABLE_DECORATIVE_TRIMS: master gate for DECO_* sub-features (no asserts
#                            currently, but geometry changes significantly)
#   PRINT_MODE / PRINT_SUPPORTS: changes orientation/rib geometry
#   SHOW_TRAY / SHOW_OVERLAY: selects which piece(s) to render
#
# The cross product of (rabbet × magnets × decoratives × print × show) gives
# us full coverage of every conditional assert path.

# --- Section 1: Core feature toggle combinations ---
echo "--- Section 1: Feature toggle matrix (rabbet × magnets × decoratives) ---"

for rabbet in true false; do
    for magnets in true false; do
        for deco in true false; do
            label="rabbet=$rabbet magnets=$magnets deco=$deco"
            run_config "$label" \
                -D "ENABLE_OVERLAY_RABBET=$rabbet" \
                -D "ENABLE_MAGNET_POCKETS=$magnets" \
                -D "ENABLE_DECORATIVE_TRIMS=$deco"
        done
    done
done

# --- Section 2: Decorative sub-toggle combinations (when master is on) ---
echo ""
echo "--- Section 2: Decorative sub-toggle matrix ---"

for back_logo in true false; do
    for pinstripe in true false; do
        for initials in true false; do
            for logo_top in true false; do
                label="deco_sub: back=$back_logo pin=$pinstripe init=$initials top=$logo_top"
                run_config "$label" \
                    -D "ENABLE_DECORATIVE_TRIMS=true" \
                    -D "DECO_BACK_LOGO=$back_logo" \
                    -D "DECO_EDGE_PINSTRIPE=$pinstripe" \
                    -D "DECO_OWNER_INITIALS=$initials" \
                    -D "DECO_LOGO_TOP=$logo_top"
            done
        done
    done
done

# --- Section 3: Print mode combinations ---
echo ""
echo "--- Section 3: Print mode × supports ---"

for print_mode in true false; do
    for supports in true false; do
        if [[ "$print_mode" == "false" && "$supports" == "true" ]]; then
            continue
        fi
        label="print=$print_mode supports=$supports"
        run_config "$label" \
            -D "PRINT_MODE=$print_mode" \
            -D "PRINT_SUPPORTS=$supports"
    done
done

# --- Section 4: Piece selection combinations ---
echo ""
echo "--- Section 4: Piece selection ---"

# tray only
run_config "show=tray-only" \
    -D "SHOW_TRAY=true" -D "SHOW_OVERLAY=false"

# overlay only
run_config "show=overlay-only" \
    -D "SHOW_TRAY=false" -D "SHOW_OVERLAY=true"

# both (default)
run_config "show=both" \
    -D "SHOW_TRAY=true" -D "SHOW_OVERLAY=true"

# --- Section 5: Print mode × piece × features (full cross) ---
echo ""
echo "--- Section 5: Print export combinations (print=true × piece × features) ---"

for piece_label in tray overlay; do
    if [[ "$piece_label" == "tray" ]]; then
        show_tray=true; show_overlay=false
    else
        show_tray=false; show_overlay=true
    fi
    for rabbet in true false; do
        for magnets in true false; do
            for supports in true false; do
                if [[ "$piece_label" == "overlay" && "$supports" == "true" ]]; then
                    continue
                fi
                label="print=$piece_label rabbet=$rabbet magnets=$magnets supports=$supports"
                run_config "$label" \
                    -D "PRINT_MODE=true" \
                    -D "SHOW_TRAY=$show_tray" \
                    -D "SHOW_OVERLAY=$show_overlay" \
                    -D "ENABLE_OVERLAY_RABBET=$rabbet" \
                    -D "ENABLE_MAGNET_POCKETS=$magnets" \
                    -D "PRINT_SUPPORTS=$supports"
            done
        done
    done
done

# --- Section 6: Ghost/debug view combinations ---
echo ""
echo "--- Section 6: Debug views ---"

run_config "ghost=plate" \
    -D "SHOW_PLATE=true"

run_config "ghost=display" \
    -D "SHOW_DISPLAY=true"

run_config "ghost=section" \
    -D "SHOW_SECTION=true"

run_config "ghost=all-debug" \
    -D "SHOW_PLATE=true" -D "SHOW_DISPLAY=true" -D "SHOW_SECTION=true"

run_config "explode=0" \
    -D "EXPLODE=0"

run_config "explode=20" \
    -D "EXPLODE=20"

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
echo ""
echo "=== Results ==="

if $DRY_RUN; then
    echo "Dry run: $TOTAL configurations would be tested."
    exit 0
fi

echo "  Total:  $TOTAL"
echo "  Passed: $PASS"
echo "  Failed: $FAIL"

if [[ $FAIL -gt 0 ]]; then
    echo ""
    echo "FAILED configurations:"
    for cfg in "${FAILED_CONFIGS[@]}"; do
        echo "  - $cfg"
    done
    exit 1
fi

echo ""
echo "All configurations passed."
exit 0
