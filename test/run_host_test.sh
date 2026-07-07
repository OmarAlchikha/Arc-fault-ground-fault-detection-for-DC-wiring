#!/bin/sh
# Regenerate test vectors from the Python reference, build the C core on the
# host, and check bit-for-bit parity of the trip decisions.
set -e
cd "$(dirname "$0")/.."
python3 sim/export_test_vectors.py
g++ -O2 -Wall -Wextra -o test/host_test test/host_test.cpp
./test/host_test test/vectors
