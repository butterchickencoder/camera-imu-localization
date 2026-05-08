#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# run_pipeline.sh  —  Camera-IMU Fusion full pipeline
#
# Steps:
#   1. Run Stereo VO  → results/poses.csv
#   2. Build & run ESKF filter → results/traj_imu_only.csv + traj_imu_vo.csv
#   3. Generate plots:
#        plot_no_fusion.png       (IMU only | VO only | velocity)
#        plot_stereo_fusion.png   (IMU + Stereo VO | velocity | RMSE table)
# ─────────────────────────────────────────────────────────────────────────────

set -e   # stop on first error

BASE="$(cd "$(dirname "$0")" && pwd)"
RESULTS="$BASE/results"
CPP_DIR="$BASE/cpp"
BUILD_DIR="$CPP_DIR/build"

# ── Colors for nicer output ──────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()    { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()    { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error()   { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

mkdir -p "$RESULTS"

# ─────────────────────────────────────────────────────────────────────────────
# STEP 1 — Stereo VO
# ─────────────────────────────────────────────────────────────────────────────
info "Step 1/3 — Running Stereo VO (feature_tracker_stereo.py) ..."

cd "$BASE"
if ! python3 stereo_wip/feature_tracker_stereo.py; then
    error "Stereo VO failed. Check stereo_wip/feature_tracker_stereo.py"
fi

if [[ ! -f "$RESULTS/poses.csv" ]]; then
    error "poses.csv not found after Stereo VO run. Something went wrong."
fi

info "Stereo VO done → results/poses.csv ($(wc -l < "$RESULTS/poses.csv") rows)"

# ─────────────────────────────────────────────────────────────────────────────
# STEP 2 — Build ESKF (skip if already built and source unchanged)
# ─────────────────────────────────────────────────────────────────────────────
info "Step 2/3 — Building ESKF filter (C++) ..."

cd "$BUILD_DIR"

# Check if rebuild is needed
NEEDS_BUILD=false
if [[ ! -f "$BUILD_DIR/fusion" ]]; then
    NEEDS_BUILD=true
fi
# Rebuild if any source is newer than the binary
for f in "$CPP_DIR/src/main.cpp" "$CPP_DIR/src/eskf.cpp" "$CPP_DIR/include/eskf.h"; do
    if [[ -f "$f" && "$f" -nt "$BUILD_DIR/fusion" ]]; then
        NEEDS_BUILD=true
        break
    fi
done

if $NEEDS_BUILD; then
    info "  Compiling (cmake + make) ..."
    cmake "$CPP_DIR" -DCMAKE_BUILD_TYPE=Release -B "$BUILD_DIR" -S "$CPP_DIR" > /dev/null
    make -C "$BUILD_DIR" -j"$(sysctl -n hw.logicalcpu 2>/dev/null || echo 4)" 2>&1 | tail -5
    info "  Build complete → $BUILD_DIR/fusion"
else
    info "  Binary up-to-date, skipping build."
fi

# Run the filter (binary expects to be run from cpp/build, reads ../data/...)
cd "$BUILD_DIR"
info "  Running ESKF filter ..."
./fusion

# Verify expected outputs
for csv in traj_imu_only.csv traj_imu_vo.csv; do
    if [[ ! -f "$RESULTS/$csv" ]]; then
        error "Expected output $csv not found in results/. Check the C++ binary output."
    fi
done
info "ESKF done → results/traj_imu_only.csv + results/traj_imu_vo.csv"

# ─────────────────────────────────────────────────────────────────────────────
# STEP 3 — Generate plots
# ─────────────────────────────────────────────────────────────────────────────
info "Step 3/3 — Generating plots ..."

cd "$BASE"

info "  Plot: Ergebnisse ohne Fusion (IMU only | Mono VO only | velocity) ..."
python3 plot_no_fusion.py

info "  Plot: Stereo + IMU Fusion (XY + velocity + RMSE) ..."
python3 plot_mono_fusion.py

# ─────────────────────────────────────────────────────────────────────────────
# DONE
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}  Pipeline complete!${NC}"
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo "  Outputs:"
echo "    results/poses.csv                (Stereo VO poses)"
echo "    results/traj_imu_only.csv        (IMU only trajectory)"
echo "    results/traj_imu_vo.csv          (IMU + Stereo VO trajectory)"
echo "    plot_no_fusion.png               (Slide: without fusion)"
echo "    plot_stereo_fusion.png           (Slide: Stereo + IMU fusion)"
echo ""
