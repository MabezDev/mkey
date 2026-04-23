#!/usr/bin/env bash
# build_stl.sh — Export print-ready STLs from case.scad using OpenSCAD CLI.
#
# Produces four STLs by default — one tray + one overlay for each retention
# scheme (magnets and screws) — since both retention variants get fabricated
# from the same order and the mating underside pockets differ between them:
#
#   mkey_tray_magnets.stl     tray with magnet pockets in wall tops
#   mkey_overlay_magnets.stl  overlay with magnet pockets in underside
#   mkey_tray_screws.stl      tray with 3-segment stepped bolt bores
#   mkey_overlay_screws.stl   overlay with hex nut pockets in underside
#
# Usage:
#   ./build_stl.sh [OPTIONS]
#
# Options:
#   --piece tray|overlay|both       Which piece(s) to export (default: both)
#   --retention magnets|screws|both Which retention variant(s) (default: both)
#   --supports                      Add breakaway supports (tray ribs + overlay bars)
#   --fn N                          Override circle resolution (default: 128)
#   --outdir DIR                    Output directory (default: current directory)
#   -h, --help                      Show this help message
#
# Examples:
#   ./build_stl.sh                            # All 4 STLs, no supports
#   ./build_stl.sh --supports                 # All 4, with anti-warp supports
#   ./build_stl.sh --retention magnets        # Magnet pair only
#   ./build_stl.sh --piece tray --retention screws  # Just the screw tray

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SCAD_FILE="$SCRIPT_DIR/case.scad"

# Defaults
PIECE="both"
RETENTION="both"
SUPPORTS="false"
FN=128
OUTDIR="."

usage() {
    sed -n '2,/^$/s/^# \?//p' "$0"
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --piece)     PIECE="$2"; shift 2 ;;
        --retention) RETENTION="$2"; shift 2 ;;
        --supports)  SUPPORTS="true"; shift ;;
        --fn)        FN="$2"; shift 2 ;;
        --outdir)    OUTDIR="$2"; shift 2 ;;
        -h|--help)   usage ;;
        *)           echo "Unknown option: $1" >&2; exit 1 ;;
    esac
done

case "$PIECE" in
    tray|overlay|both) ;;
    *) echo "Error: --piece must be tray, overlay, or both" >&2; exit 1 ;;
esac

case "$RETENTION" in
    magnets|screws|both) ;;
    *) echo "Error: --retention must be magnets, screws, or both" >&2; exit 1 ;;
esac

find_openscad() {
    if [[ -n "${OPENSCAD:-}" ]]; then
        command -v "$OPENSCAD" &>/dev/null && { echo "$OPENSCAD"; return 0; }
        return 1
    fi
    command -v openscad &>/dev/null && { echo "openscad"; return 0; }
    local candidates=(
        /Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD
        /Applications/OpenSCAD-Nightly.app/Contents/MacOS/OpenSCAD
        /snap/bin/openscad
        /usr/local/bin/openscad
        /usr/bin/openscad
    )
    local c
    for c in "${candidates[@]}"; do
        [[ -x "$c" ]] && { echo "$c"; return 0; }
    done
    return 1
}

if ! OPENSCAD=$(find_openscad); then
    echo "Error: OpenSCAD not found in PATH or in any known install location." >&2
    echo "Install OpenSCAD: https://openscad.org/downloads.html" >&2
    echo "Or set \$OPENSCAD to the binary path." >&2
    exit 1
fi

if ! command -v admesh &>/dev/null; then
    echo "Error: admesh not found in PATH." >&2
    echo "admesh is required to strip degenerate facets from Manifold output" >&2
    echo "(JLC flags these as 'noise shells' / 'multi-shells')." >&2
    echo "Install: 'brew install admesh' (macOS) or 'apt install admesh' (Debian/Ubuntu)." >&2
    exit 1
fi

mkdir -p "$OUTDIR"

render_piece() {
    local piece="$1"
    local retention="$2"
    local outfile="$3"
    local show_tray="false"
    local show_overlay="false"
    local magnets="false"
    local screws="false"

    case "$piece" in
        tray)    show_tray="true" ;;
        overlay) show_overlay="true" ;;
    esac

    case "$retention" in
        magnets) magnets="true" ;;
        screws)  screws="true" ;;
    esac

    echo "Rendering $piece/$retention -> $outfile (fn=$FN, supports=$SUPPORTS) ..."

    "$OPENSCAD" \
        --backend=manifold \
        -o "$outfile" \
        -D "PRINT_MODE=true" \
        -D "SHOW_TRAY=$show_tray" \
        -D "SHOW_OVERLAY=$show_overlay" \
        -D "SHOW_PLATE=false" \
        -D "SHOW_DISPLAY=false" \
        -D "PRINT_SUPPORTS=$SUPPORTS" \
        -D "PRINT_OVERLAY_SUPPORTS=$SUPPORTS" \
        -D "ENABLE_MAGNET_POCKETS=$magnets" \
        -D "ENABLE_SCREW_INSERTS=$screws" \
        -D "\$fn=$FN" \
        "$SCAD_FILE"

    # Post-process: strip degenerate (zero-area) facets and re-weld near-
    # coincident vertices. The Manifold backend's tessellation can leave
    # knife-edge slivers at boolean boundaries — JLC's pre-print checker
    # flags these as "noise shells" and "multi-shells detected" (exact-edge
    # matching sees the sliver seams as disconnected components). admesh
    # removes the zero-area facets and re-serialises vertices so the exact
    # check sees a single connected shell.
    local tmpfile="${outfile}.admesh.tmp"
    local log="${outfile}.admesh.log"
    admesh -e -u -d -v --write-binary-stl="$tmpfile" "$outfile" > "$log" 2>&1
    mv "$tmpfile" "$outfile"
    local removed
    removed=$(awk '/Facets removed/ {print $NF}' "$log")
    rm -f "$log"
    echo "  admesh cleanup: removed ${removed:-0} degenerate facets"

    echo "  Done: $outfile"
}

pieces=()
case "$PIECE" in
    tray)    pieces=(tray) ;;
    overlay) pieces=(overlay) ;;
    both)    pieces=(tray overlay) ;;
esac

retentions=()
case "$RETENTION" in
    magnets) retentions=(magnets) ;;
    screws)  retentions=(screws) ;;
    both)    retentions=(magnets screws) ;;
esac

for retention in "${retentions[@]}"; do
    for piece in "${pieces[@]}"; do
        render_piece "$piece" "$retention" "$OUTDIR/mkey_${piece}_${retention}.stl"
    done
done

echo "Build complete."
