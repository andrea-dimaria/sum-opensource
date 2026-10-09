#!/bin/bash
#
# Build and run the JANUS tests on a PC, no modem hardware needed.
#
# usage: tests/run_tests.sh <libjanus c dir> [--fs N] [--long]
#   <libjanus c dir>  libjanus source dir containing CMakeLists.txt
#                     (the janus-c fork, which contains plugin_011_02.c
#                     and plugin_000_01.c)
#   --fs N            sampling frequency for the loopback test (default 96000)
#   --long            also test 478/480 byte packets (a few minutes)
#
# Everything is built under tests/build: the libjanus sources and the system
# installation (/usr/local) are not touched.

set -e

if [ $# -lt 1 ] || [ ! -f "$1/CMakeLists.txt" ]; then
  sed -n '3,13p' "$0"
  exit 2
fi

JANUS_SRC=$(cd "$1" && pwd)
shift
TESTS=$(cd "$(dirname "$0")" && pwd)
SUM=$(dirname "$TESTS")
BUILD=$TESTS/build
INSTALL=$BUILD/janus-install
PLUGINS=$INSTALL/share/janus/plugins

mkdir -p "$BUILD"

echo "=== libjanus + plugins ==="
rm -rf "$BUILD/janus-src"
mkdir -p "$BUILD/janus-src"
tar -C "$JANUS_SRC" --exclude=.git -cf - . | tar -C "$BUILD/janus-src" -xf -
for p in plugin_011_02.c plugin_000_01.c; do
  if [ ! -f "$BUILD/janus-src/src/c/plugins/$p" ]; then
    echo "$p not found in $JANUS_SRC/src/c/plugins (use the janus-c fork)"
    exit 2
  fi
done
mkdir -p "$BUILD/janus-build"
(cd "$BUILD/janus-build" \
  && cmake "$BUILD/janus-src" -DCMAKE_INSTALL_PREFIX="$INSTALL" > cmake.log \
  && make -j"$(nproc)" > make.log 2>&1 \
  && make install > install.log)
ls "$PLUGINS"

echo "=== tests ==="
CXXFLAGS="-std=c++14 -Wall -O1 -I$INSTALL/include -I$SUM/janus_wrap -I$SUM/modem_utils"
# libjanus links ALSA/PulseAudio drivers when their dev packages are installed
LIBS="-L$INSTALL/lib -ljanus -lfftw3 $(pkg-config --libs alsa libpulse-simple 2>/dev/null || pkg-config --libs alsa 2>/dev/null) -lm -ldl -lpthread"
gcc -Wall -O1 -I"$INSTALL/include" "$TESTS/plugin_011_02_test.c" $LIBS \
  -o "$BUILD/plugin_011_02_test"
gcc -Wall -O1 -I"$INSTALL/include" "$TESTS/plugin_000_01_test.c" $LIBS \
  -o "$BUILD/plugin_000_01_test"
g++ $CXXFLAGS "$TESTS/janus_mac_test.cpp" "$SUM/janus_wrap/janus-mac.cpp" \
  -o "$BUILD/janus_mac_test"
g++ $CXXFLAGS "$TESTS/janus_loopback_test.cpp" "$SUM/janus_wrap/janus-wrapper.cpp" \
  "$SUM/modem_utils/chunk.cpp" $LIBS -o "$BUILD/janus_loopback_test"

# a failing test stops the script (set -e), also through tail
set -o pipefail

echo "=== plugin unit tests ==="
"$BUILD/plugin_011_02_test" "$PLUGINS/libplugin_011_02.so" | tail -1
"$BUILD/plugin_000_01_test" "$PLUGINS/libplugin_000_01.so" | tail -1

echo "=== MAC unit test ==="
"$BUILD/janus_mac_test" | tail -1

echo "=== loopback test ==="
# libjanus prints packet dumps on stderr and some debug lines on stdout
LD_LIBRARY_PATH="$PLUGINS" "$BUILD/janus_loopback_test" "$@" 2> "$BUILD/loopback_stderr.log" \
  | grep -v -E '^(peak:|frssi =)'
