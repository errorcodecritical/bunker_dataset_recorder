#!/usr/bin/env bash
# Real-time NDT localization for the Hesai/bunker side of the coop bags:
#   map ──(NDT vs gt_map, config/gt_ouster_ndt_tree_fused.yaml)──> odom
#   odom ──(identity static, the 'noekf' baseline)──────────────> base_link
#   base_link ──(the bag's own /tf_static)───────────────────────> hesai_lidar, imu
#
# Unlike run_localization_tree.sh (Ouster-only, map-test-2-only), this does NOT
# hardcode lidar/imu extrinsics -- they come from the bag's recorded /tf_static.
# No seed pose either: the config's default near-origin seed converges for this
# bag family (fitness ~0.2-0.4 m), per launch/coop_multi_robot_localization.launch.py.
#
# Usage (inside the container):
#   BAG=/ws/bags/<dir>  bash /ws/scripts/run_localization_bunker_hesai.sh [duration_s]
#   BAG=/mnt/d/<dir>    bash /ws/scripts/run_localization_bunker_hesai.sh [duration_s]
#     (the /mnt/d form only works if compose.override.yaml bind-mounts /mnt/d in)
# Output -> /ws/output/path_bunker.csv
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
cd /ws
mkdir -p output

# Hesai's cloud is multi-MB too -- same shared-memory requirement as Ouster's,
# else it throttles to ~0.1 Hz over UDP loopback.
export FASTRTPS_DEFAULT_PROFILES_FILE=/ws/config/fastdds_shm.xml

DUR="${1:-}"
DUR_ARG=""
[ -n "$DUR" ] && DUR_ARG="--playback-duration $DUR"

BAG="${BAG:-bags/2026_07_06_16_53_31__bunker_kalhan_coop_}"
if [ ! -f "$BAG/metadata.yaml" ]; then
  echo "no bag at $BAG (set BAG=... to pick another, e.g. /mnt/d/<dir> if mounted)" >&2
  exit 1
fi

PIDS=()
cleanup() { kill "${PIDS[@]}" 2>/dev/null || true; }
trap cleanup EXIT INT TERM

# 1) Both robots' localizers (curt's half just idles -- this bag has no /ouster/points).
ros2 launch /ws/launch/coop_multi_robot_localization.launch.py use_sim_time:=true \
  > /tmp/loc_coop.log 2>&1 &
PIDS+=($!)
echo "localizer pid=${PIDS[-1]} ; waiting for bunker to activate..."
until grep -aq "Activating end" /tmp/loc_coop.log; do sleep 1; done
echo "active. playing bag at rate 1.0 ${DUR:+(first ${DUR}s)}..."

# 2) best-effort TF-tree check ~12 s into playback (proves map -> base_link resolves)
( sleep 12
  echo "--- map -> base_link (sampled during playback) ---" > /tmp/tf_chain.log
  timeout 5 ros2 run tf2_ros tf2_echo map base_link \
    --ros-args -p use_sim_time:=true >> /tmp/tf_chain.log 2>&1 || true ) &
PIDS+=($!)

# 3) play the bag. /tf_static supplies base_link->hesai_lidar and base_link->imu --
#    do NOT drop it, unlike the Ouster script this one has no hardcoded fallback.
ros2 bag play "$BAG" \
  --topics /hesai/points /imu/data /tf_static --clock --rate 1.0 $DUR_ARG

# 4) dump the bunker's latched /bunker/path trajectory (map frame) to CSV
python3 /ws/scripts/fetch_path.py /ws/output/path_bunker.csv /bunker/path

echo "good_scans=$(grep -ac 'fitness score:' /tmp/loc_coop.log)"
echo "--- TF tree (map -> base_link) ---"; cat /tmp/tf_chain.log 2>/dev/null || true
echo "done. trajectory -> output/path_bunker.csv"