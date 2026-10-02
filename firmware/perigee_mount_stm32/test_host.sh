#!/bin/sh
# Host test for the limit and position arithmetic.
#
# src/limits.rs is the authoritative safety layer and has no hardware in it, so it compiles twice:
# as a module of the firmware (cargo build, for the board) and as its own crate root here (for a
# test run on this PC). Nothing is flashed and no board is needed.
set -e
out="${TMPDIR:-/tmp}/perigee_limits_test"
rustc --test --edition 2024 -o "$out" src/limits.rs
"$out"
