#!/usr/bin/env bash
# build_stl.sh — Export print-ready STLs from case.scad using OpenSCAD CLI.
#
# Usage:
#   ./build_stl.sh [OPTIONS]
#
# Options:
#   --piece tray|overlay|both   Which piece(s) to export (default: both)
#   --supports                  Add breakaway cross-braces to the tray
#   --fn N                      Override circle resolution (default: 128)
#   --outdir DIR                Output directory (default: current directory)
#   -h, --help                  Show this help message
#
# Examples:
#   ./build_stl.sh                          # Export both pieces, no supports
#   ./build_stl.sh --supports               # Export both, tray with braces
#   ./build_stl.sh --piece overlay          # Export only the overlay
#   ./build_stl.sh --supports --fn 96       # Faster render with lower resolution

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SCAD_FILE="$SCRIPT_DIR/case.scad"

# Defaults
PIECE="both"
SUPPORTS="false"
FN=128
OUTDIR="."

usage() {
    sed -n '2,/^$/s/^# \?//p' "$0"
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --piece)    PIECE="$2"; shift 2 ;;
        --supports) SUPPORTS="true"; shift ;;
        --fn)       FN="$2"; shift 2 ;;
        --outdir)   OUTDIR="$2"; shift 2 ;;
        -h|--help)  usage ;;
        *)          echo "Unknown option: $1" >&2; exit 1 ;;
    esac
done

if ! command -v openscad &>/dev/null; then
    echo "Error: openscad not found in PATH." >&2
    echo "Install OpenSCAD: https://openscad.org/downloads.html" >&2
    exit 1
fi

mkdir -p "$OUTDIR"

render_piece() {
    local piece="$1"
    local outfile="$2"
    local show_tray="false"
    local show_overlay="false"

    case "$piece" in
        tray)    show_tray="true" ;;
        overlay) show_overlay="true" ;;
    esac

    echo "Rendering $piece -> $outfile (fn=$FN, supports=$SUPPORTS) ..."

    openscad \
        -o "$outfile" \
        -D "PRINT_MODE=true" \
        -D "SHOW_TRAY=$show_tray" \
        -D "SHOW_OVERLAY=$show_overlay" \
        -D "SHOW_RETAINER=false" \
        -D "SHOW_PLATE=false" \
        -D "SHOW_DISPLAY=false" \
        -D "PRINT_SUPPORTS=$SUPPORTS" \
        -D "\$fn=$FN" \
        "$SCAD_FILE"

    echo "  Done: $outfile"
}

case "$PIECE" in
    tray)
        render_piece tray "$OUTDIR/mkey_tray.stl"
        ;;
    overlay)
        render_piece overlay "$OUTDIR/mkey_overlay.stl"
        ;;
    both)
        render_piece tray "$OUTDIR/mkey_tray.stl"
        render_piece overlay "$OUTDIR/mkey_overlay.stl"
        ;;
    *)
        echo "Error: --piece must be tray, overlay, or both" >&2
        exit 1
        ;;
esac

echo "Build complete."
