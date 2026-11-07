#!/usr/bin/env bash
# Validate the minimal KITTI-style sample layout used by the pipeline.
set -eu

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="${1:-$SCRIPT_DIR/../data/sample}"
FRAME="${2:-000000}"
FAILURES=0

fail() {
    printf '[FAIL] %s\n' "$1" >&2
    FAILURES=$((FAILURES + 1))
}

ok() {
    printf '[OK]   %s\n' "$1"
}

if [ ! -d "$ROOT" ]; then
    printf '[FAIL] data root not found: %s\n' "$ROOT" >&2
    exit 1
fi

IMAGE="$ROOT/image_2/$FRAME.png"
CLOUD="$ROOT/velodyne/$FRAME.bin"
CALIB="$ROOT/calib/$FRAME.txt"

for pair in "image:$IMAGE" "velodyne:$CLOUD" "calib:$CALIB"; do
    name="${pair%%:*}"
    path="${pair#*:}"
    if [ ! -f "$path" ]; then
        fail "$name file missing: $path"
    elif [ ! -s "$path" ]; then
        fail "$name file is empty: $path"
    else
        ok "$name: $path ($(wc -c < "$path" | tr -d ' ') bytes)"
    fi
done

if [ -f "$CLOUD" ]; then
    size=$(wc -c < "$CLOUD" | tr -d ' ')
    if [ $((size % 16)) -ne 0 ]; then
        fail "velodyne .bin size must be a multiple of 16 bytes; got $size"
    else
        ok "velodyne layout: $((size / 16)) points (4 float32 per point)"
    fi
fi

if [ -f "$CALIB" ]; then
    for key in P2 R0_rect Tr_velo_to_cam; do
        if grep -Eq "^[[:space:]]*${key}[[:space:]]*:" "$CALIB"; then
            ok "calib key present: $key"
        else
            fail "calib key missing: $key"
        fi
    done
fi

if [ -f "$IMAGE" ]; then
    sig=$(od -An -tx1 -N8 "$IMAGE" | tr -d ' \n')
    if [ "$sig" = "89504e470d0a1a0a" ]; then
        ok "image signature: PNG"
    else
        fail "image is not a PNG (signature: $sig)"
    fi
fi

printf '\n'
if [ "$FAILURES" -ne 0 ]; then
    printf 'Data check failed with %s problem(s).\n' "$FAILURES" >&2
    exit 1
fi

printf 'Data check passed: %s (frame %s)\n' "$ROOT" "$FRAME"
