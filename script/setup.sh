#!/usr/bin/env bash
# Check or install everything the Pashupati stack needs on a robot (or in the dev container).
#
#   script/setup.sh                      # same as: check
#   script/setup.sh check                # status of every component
#   script/setup.sh install              # install the default components that are missing
#   script/setup.sh install livox build  # install only these
#   script/setup.sh install all          # every component, including ros and service
#
# Components: ros tools submodules livox genisom rosdep realsense_udev build service network
# Env: ROS_DISTRO (humble), LIVOX_SDK_REF (master), GENISOM_SETUP (genisom submodule script),
#      SERVICE_MODE (patrol), FORCE=1 (reinstall even when the check passes)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
LIVOX_SDK_REPO="${LIVOX_SDK_REPO:-https://github.com/Livox-SDK/Livox-SDK2.git}"
LIVOX_SDK_REF="${LIVOX_SDK_REF:-master}"
GENISOM_SETUP="${GENISOM_SETUP:-$ROOT/src/xtras/drivers/genisom_l1w_ros2/script/setup.sh}"
SERVICE_NAME=pashupati
SERVICE_MODE="${SERVICE_MODE:-patrol}"
REALSENSE_RULES=/etc/udev/rules.d/99-realsense-libusb.rules
LIVOX_IP="${LIVOX_IP:-192.168.123.20}"
ROBOT_IP="${ROBOT_IP:-192.168.234.1}"

ALL=(ros tools submodules livox genisom rosdep realsense_udev build service network)
DEFAULT_INSTALL=(tools submodules livox genisom rosdep realsense_udev build)

SUDO=$([[ "$(id -u)" -eq 0 ]] && echo "" || echo "sudo")
IN_DOCKER=$([[ -f /.dockerenv ]] && echo 1 || echo 0)
if [[ -t 1 ]]; then
    GREEN=$'\e[32m' RED=$'\e[31m' YELLOW=$'\e[33m' BOLD=$'\e[1m' RESET=$'\e[0m'
else
    GREEN="" RED="" YELLOW="" BOLD="" RESET=""
fi

log() { echo "${BOLD}==>${RESET} $*"; }
apt_install() { $SUDO apt-get update -qq && $SUDO apt-get install -y --no-install-recommends "$@"; }
source_ros() {
    set +u
    # shellcheck disable=SC1090
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    set -u
}

# ---------------------------------------------------------------- components
# Each component has check_<name> (exit 0 = ok, prints one detail line) and install_<name>.

check_ros() {
    [[ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]] && echo "/opt/ros/$ROS_DISTRO" && return 0
    echo "ROS 2 $ROS_DISTRO not found"; return 1
}
install_ros() {
    apt_install software-properties-common curl gnupg lsb-release locales
    $SUDO add-apt-repository -y universe
    $SUDO curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
        | $SUDO tee /etc/apt/sources.list.d/ros2.list >/dev/null
    apt_install "ros-$ROS_DISTRO-ros-base"
}

check_tools() {
    local missing=()
    for cmd in colcon rosdep git cmake g++ tmux curl; do
        command -v "$cmd" >/dev/null || missing+=("$cmd")
    done
    [[ ${#missing[@]} -eq 0 ]] && echo "colcon rosdep git cmake g++ tmux curl" && return 0
    echo "missing: ${missing[*]}"; return 1
}
install_tools() {
    apt_install python3-colcon-common-extensions python3-rosdep python3-pip git cmake \
        build-essential tmux curl ca-certificates
}

check_submodules() {
    local missing
    missing=$(git -C "$ROOT" submodule status 2>/dev/null | awk '/^-/ {print $2}' | xargs -r basename -a)
    [[ -z "$missing" ]] && echo "all checked out" && return 0
    echo "not checked out: $(echo "$missing" | tr '\n' ' ')"; return 1
}
install_submodules() {
    git -C "$ROOT" submodule update --init --recursive && return 0
    log "SSH clone failed, retrying submodules over HTTPS"
    git -C "$ROOT" -c "url.https://github.com/.insteadOf=git@github.com:" \
        submodule update --init --recursive
}

check_livox() {
    [[ -f /usr/local/lib/liblivox_lidar_sdk_shared.so ]] \
        && echo "/usr/local/lib/liblivox_lidar_sdk_shared.so" && return 0
    echo "Livox-SDK2 not installed"; return 1
}
install_livox() {
    local tmp
    tmp=$(mktemp -d)
    git clone --depth 1 --branch "$LIVOX_SDK_REF" "$LIVOX_SDK_REPO" "$tmp"
    cmake -S "$tmp" -B "$tmp/build" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$tmp/build" -j"$(nproc)"
    $SUDO cmake --install "$tmp/build"
    $SUDO ldconfig
    rm -rf "$tmp"
}

check_genisom() {
    [[ -x "$GENISOM_SETUP" ]] || { echo "no $GENISOM_SETUP (install submodules)"; return 1; }
    "$GENISOM_SETUP" check
}
install_genisom() {
    [[ -x "$GENISOM_SETUP" ]] || install_submodules
    "$GENISOM_SETUP" install
}

check_rosdep() {
    check_ros >/dev/null || { echo "needs ROS"; return 1; }
    [[ -d /etc/ros/rosdep/sources.list.d ]] || { echo "rosdep not initialised"; return 1; }
    source_ros
    rosdep check --from-paths "$ROOT/src" --ignore-src >/tmp/pashupati_rosdep.log 2>&1 \
        && echo "all system dependencies installed" && return 0
    echo "missing keys, see /tmp/pashupati_rosdep.log"; return 1
}
install_rosdep() {
    [[ -d /etc/ros/rosdep/sources.list.d ]] || $SUDO rosdep init
    source_ros
    rosdep update --rosdistro "$ROS_DISTRO"
    rosdep install --from-paths "$ROOT/src" --ignore-src -r -y --rosdistro "$ROS_DISTRO"
}

check_realsense_udev() {
    [[ "$IN_DOCKER" == "1" ]] && echo "skipped in docker" && return 0
    [[ -f "$REALSENSE_RULES" ]] && echo "$REALSENSE_RULES" && return 0
    echo "RealSense udev rules missing (camera needs root)"; return 1
}
install_realsense_udev() {
    curl -fsSL https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules \
        | $SUDO tee "$REALSENSE_RULES" >/dev/null
    $SUDO udevadm control --reload-rules && $SUDO udevadm trigger
}

check_build() {
    [[ -f "$ROOT/install/setup.bash" ]] || { echo "workspace not built"; return 1; }
    local n
    n=$(find "$ROOT/install" -mindepth 1 -maxdepth 1 -type d | wc -l)
    echo "$n packages in install/"
}
install_build() {
    source_ros
    (cd "$ROOT" && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release)
}

check_service() {
    [[ "$IN_DOCKER" == "1" ]] && echo "skipped in docker" && return 0
    systemctl is-enabled "$SERVICE_NAME.service" >/dev/null 2>&1 \
        && echo "$SERVICE_NAME.service enabled ($(systemctl is-active $SERVICE_NAME.service))" \
        && return 0
    echo "$SERVICE_NAME.service not installed"; return 1
}
install_service() {
    local user
    user=$(id -un)
    $SUDO tee "/etc/systemd/system/$SERVICE_NAME.service" >/dev/null <<EOF
[Unit]
Description=Pashupati patrol stack ($SERVICE_MODE, tmux session '$SERVICE_NAME')
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
User=$user
Environment=WS=$ROOT
Environment=RVIZ=0
Environment=DETACH=1
ExecStart=$ROOT/script/start_tmux.sh $SERVICE_MODE
ExecStop=/usr/bin/tmux kill-session -t $SERVICE_NAME

[Install]
WantedBy=multi-user.target
EOF
    $SUDO systemctl daemon-reload
    $SUDO systemctl enable "$SERVICE_NAME.service"
    log "start now with: sudo systemctl start $SERVICE_NAME, attach with: tmux attach -t $SERVICE_NAME"
}

check_network() {
    local ok=0 out=()
    for pair in "livox=$LIVOX_IP" "robot=$ROBOT_IP"; do
        if ping -c1 -W1 "${pair#*=}" >/dev/null 2>&1; then out+=("${pair} up"); else out+=("${pair} down"); ok=1; fi
    done
    echo "${out[*]}"; return $ok
}
install_network() {
    echo "configure the interfaces by hand: Livox on 192.168.123.0/24, robot SDK on 192.168.234.0/24"
    return 1
}

# ---------------------------------------------------------------- driver

check_one() {
    local name=$1 detail status
    if detail=$("check_$name" 2>&1); then
        status="${GREEN}ok     ${RESET}"
    else
        status="${RED}missing${RESET}"
    fi
    printf '  %-15s %s  %s\n' "$name" "$status" "$(echo "$detail" | tail -n1)"
    [[ "$status" == *ok* ]]
}

do_check() {
    local failed=0
    echo "${BOLD}Pashupati setup ($([[ $IN_DOCKER == 1 ]] && echo docker || echo host), $(uname -m))${RESET}"
    for name in "${ALL[@]}"; do
        check_one "$name" || failed=$((failed + 1))
    done
    [[ $failed -eq 0 ]] && echo "${GREEN}everything is ready${RESET}" \
        || echo "${YELLOW}$failed component(s) need attention: script/setup.sh install <component>${RESET}"
}

do_install() {
    local targets=("$@")
    [[ ${#targets[@]} -eq 0 ]] && targets=("${DEFAULT_INSTALL[@]}")
    [[ "${targets[0]}" == "all" ]] && targets=("${ALL[@]/network}")
    for name in "${targets[@]}"; do
        [[ -z "$name" ]] && continue
        declare -F "install_$name" >/dev/null || { echo "${RED}unknown component: $name${RESET}"; exit 1; }
        if [[ "${FORCE:-0}" != "1" ]] && "check_$name" >/dev/null 2>&1; then
            echo "  $name: already ok"
            continue
        fi
        log "installing $name"
        "install_$name"
        check_one "$name" || { echo "${RED}$name still not ok after install${RESET}"; exit 1; }
    done
}

case "${1:-check}" in
    check) do_check ;;
    install) shift; do_install "$@" ;;
    *) sed -n '2,13p' "$0"; exit 1 ;;
esac
