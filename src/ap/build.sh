#!/bin/bash

# get directory of this script relative to root
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

OUTPUT="${DIR}/../../A32NX/SimObjects/AirPlanes/Asobo_A320_NEO/panel/autopilot.wasm"

set -ex

# compile c++ code
clang++ \
  -Wno-unused-command-line-argument \
  --sysroot "${MSFS_SDK}/WASM/wasi-sysroot" \
  -target wasm32-unknown-wasi \
  -D_MSFS_WASM=1 \
  -D__wasi__ \
  -D_LIBCPP_HAS_NO_THREADS \
  -D_WINDLL \
  -D_MBCS \
  -fno-exceptions \
  -fms-extensions \
  -fvisibility=hidden \
  -Wl,--strip-debug \
  -Wl,--no-entry \
  -Wl,--export=malloc \
  -Wl,--export=free \
  -Wl,--export=__wasm_call_ctors \
  -Wl,--export-table \
  -Wl,--allow-undefined \
  -I "${MSFS_SDK}/WASM/include" \
  -I "${MSFS_SDK}/SimConnect SDK/include" \
  -I "${DIR}/src/interface" \
  "${DIR}/src/interface/SimConnectInterfaceAutopilot.cpp" \
  -I "${DIR}/src/model" \
  "${DIR}/src/model/Autopilot.cpp" \
  "${DIR}/src/model/Autopilot_data.cpp" \
  -I "${DIR}/src/zlib" \
  "${DIR}/src/AutopilotInterface.cpp" \
  "${DIR}/src/main.cpp" \
  -o $OUTPUT
