#!/usr/bin/env bash
# =============================================================================
# 단계 0 — ROS 2 Jazzy + Gazebo Harmonic 스택 설치
#
#   sudo bash scripts/00_install_stack.sh
#
# 이 머신의 특수 사정:
#   · packages.ros.org 는 HTTPS 인증서가 불일치한다 (*.osuosl.org 제시).
#     공식 ros2-apt-source .deb 가 원래 http:// 를 설정하므로 그대로 쓰면 된다.
#     apt 는 GPG 서명으로 무결성을 검증하므로 보안 강등이 아니다.
#     >>> https://packages.ros.org 소스 줄을 손으로 쓰지 말 것 <<<
#   · packages.osrfoundation.org 를 추가하지 말 것. Jazzy 부터 Gazebo 는
#     ros-jazzy-*-vendor 패키지로 온다. OSRF 저장소를 더하면 Gazebo 가 두 벌
#     설치되어 플러그인 경로/버전 불일치를 만든다.
# =============================================================================
set -Eeuo pipefail

RED=$'\e[31m'; GRN=$'\e[32m'; YLW=$'\e[33m'; BLD=$'\e[1m'; RST=$'\e[0m'
step() { echo; echo "${BLD}▶ $*${RST}"; }
ok()   { echo "  ${GRN}✔${RST} $*"; }
warn() { echo "  ${YLW}!${RST} $*"; }
die()  { echo "  ${RED}✗${RST} $*" >&2; exit 1; }

trap 'die "실패: ${BASH_COMMAND} (line ${LINENO})"' ERR

[[ $EUID -eq 0 ]] || die "root 로 실행하세요:  sudo bash $0"

# 실제 사용자 (sudo 로 실행되므로 $SUDO_USER 가 진짜 사용자)
REAL_USER="${SUDO_USER:-$(logname 2>/dev/null || echo root)}"
REAL_HOME=$(getent passwd "$REAL_USER" | cut -d: -f6)
ok "대상 사용자: $REAL_USER ($REAL_HOME)"

# -----------------------------------------------------------------------------
step "0. 사전 확인"
# -----------------------------------------------------------------------------
. /etc/os-release
[[ "${UBUNTU_CODENAME:-}" == "noble" ]] || die "Ubuntu 24.04 (noble) 전용입니다. 현재: ${UBUNTU_CODENAME:-unknown}"
ok "Ubuntu ${VERSION_ID} ${UBUNTU_CODENAME}"

if nvidia-smi >/dev/null 2>&1; then
  ok "NVIDIA 드라이버 동작: $(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader | head -1)"
else
  warn "nvidia-smi 실패 — Gazebo 가 Intel iGPU 로 폴백합니다 (RTF 0.1~0.3)."
  warn "계속 진행은 되지만 성능 측정은 무의미해집니다."
fi

if [[ "$(prime-select query 2>/dev/null)" != "nvidia" ]]; then
  warn "prime-select 가 nvidia 가 아닙니다 → Gazebo 가 iGPU 를 씁니다."
  warn "  sudo prime-select nvidia && sudo reboot"
fi

# -----------------------------------------------------------------------------
step "1. 로케일 + universe 저장소"
# -----------------------------------------------------------------------------
apt-get update -qq
apt-get install -y -qq locales software-properties-common curl gnupg
locale-gen en_US en_US.UTF-8 >/dev/null
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
add-apt-repository -y universe >/dev/null
ok "로케일 및 universe 준비"

# noble 의 알려진 함정: Suites 에 noble-updates 가 빠져 있으면 패키지가 안 보인다
if ! grep -q "noble-updates" /etc/apt/sources.list.d/ubuntu.sources 2>/dev/null; then
  warn "ubuntu.sources 의 Suites 에 noble-updates 가 없습니다. 확인하세요:"
  warn "  grep Suites /etc/apt/sources.list.d/ubuntu.sources"
fi

# -----------------------------------------------------------------------------
step "2. ROS 2 apt 저장소 (공식 ros2-apt-source 패키지)"
# -----------------------------------------------------------------------------
if [[ -f /etc/apt/sources.list.d/ros2.sources ]] || [[ -f /etc/apt/sources.list.d/ros2.list ]]; then
  ok "ROS 2 저장소가 이미 설정돼 있습니다"
else
  ROS_APT_SOURCE_VERSION=$(
    curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      | grep -F '"tag_name"' | awk -F'"' '{print $4}'
  ) || ROS_APT_SOURCE_VERSION=""

  [[ -n "$ROS_APT_SOURCE_VERSION" ]] || die "ros-apt-source 최신 버전 조회 실패 (GitHub API 접근 확인)"
  ok "ros-apt-source ${ROS_APT_SOURCE_VERSION}"

  DEB="/tmp/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.deb"
  curl -fsSL -o "$DEB" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME}_all.deb"
  dpkg -i "$DEB" >/dev/null
  rm -f "$DEB"
  ok "저장소 등록 완료"
fi

# 실제로 http:// 로 잡혔는지 확인 — https 면 이 머신에서 반드시 실패한다
if grep -rqs "https://packages.ros.org" /etc/apt/sources.list.d/; then
  die "소스에 https://packages.ros.org 가 있습니다. 이 머신에서는 인증서 불일치로 실패합니다. http:// 로 고치세요."
fi
ok "소스가 http:// 로 설정됨 (GPG 서명으로 무결성 보장)"

apt-get update -qq
ok "패키지 목록 갱신"

# -----------------------------------------------------------------------------
step "3. ROS 2 Jazzy + Gazebo Harmonic"
# -----------------------------------------------------------------------------
echo "  (수 GB 다운로드 — 네트워크에 따라 10~30분)"
apt-get install -y \
  ros-jazzy-desktop \
  ros-dev-tools \
  ros-jazzy-ros-gz
ok "ROS 2 Jazzy + Gazebo Harmonic (gz-sim8 vendor 패키지)"

# -----------------------------------------------------------------------------
step "4. 자율주행 스택"
# -----------------------------------------------------------------------------
apt-get install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization \
  ros-jazzy-pointcloud-to-laserscan \
  ros-jazzy-slam-toolbox
ok "Nav2 + robot_localization"

# STVL 은 배포판에 따라 바이너리가 없을 수 있다 — 없으면 소스 빌드로 넘긴다
if apt-cache show ros-jazzy-spatio-temporal-voxel-layer >/dev/null 2>&1; then
  apt-get install -y ros-jazzy-spatio-temporal-voxel-layer
  ok "STVL (spatio_temporal_voxel_layer)"
else
  warn "ros-jazzy-spatio-temporal-voxel-layer 바이너리 없음 → 나중에 소스 빌드 필요"
fi

# nav2_route 존재 여부 (설계서 §7.4 — 없으면 NavigateThroughPoses 로 대체)
if apt-cache show ros-jazzy-nav2-route >/dev/null 2>&1; then
  apt-get install -y ros-jazzy-nav2-route
  ok "nav2_route 사용 가능"
else
  warn "ros-jazzy-nav2-route 없음 → 웨이포인트 기반 NavigateThroughPoses 로 진행"
fi

# -----------------------------------------------------------------------------
step "5. 에셋 생성 및 개발 도구"
# -----------------------------------------------------------------------------
apt-get install -y \
  mesa-utils \
  blender \
  python3-jinja2 python3-numpy python3-opencv python3-yaml python3-pip \
  git
ok "Blender + Python 툴체인 + glxinfo"

# -----------------------------------------------------------------------------
step "6. rosdep 초기화"
# -----------------------------------------------------------------------------
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init >/dev/null 2>&1 || warn "rosdep init 건너뜀 (이미 초기화됨)"
fi
sudo -u "$REAL_USER" bash -lc 'rosdep update' >/dev/null 2>&1 || warn "rosdep update 실패 — 나중에 수동 실행"
ok "rosdep 준비"

# -----------------------------------------------------------------------------
step "7. 사용자 환경 설정"
# -----------------------------------------------------------------------------
BASHRC="$REAL_HOME/.bashrc"
add_line() {
  grep -qxF "$1" "$BASHRC" 2>/dev/null || { echo "$1" >> "$BASHRC"; echo "    + $1"; }
}
add_line 'source /opt/ros/jazzy/setup.bash'
# Wayland 세션에서 Gazebo GUI 를 띄우려면 필수 (Ogre/Qt 가 Wayland 미지원)
add_line 'export QT_QPA_PLATFORM=xcb'
chown "$REAL_USER:$REAL_USER" "$BASHRC"
ok "~/.bashrc 갱신"

# -----------------------------------------------------------------------------
step "8. 검증"
# -----------------------------------------------------------------------------
FAIL=0
check() { if eval "$2" >/dev/null 2>&1; then ok "$1"; else echo "  ${RED}✗${RST} $1"; FAIL=1; fi; }

check "ROS 2 Jazzy 설치됨"        "[ -f /opt/ros/jazzy/setup.bash ]"
check "gz sim 실행 가능"           "sudo -u $REAL_USER bash -lc 'source /opt/ros/jazzy/setup.bash && gz sim --versions'"
check "ros_gz_bridge 존재"         "[ -x /opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge ]"
check "Nav2 설치됨"                "[ -d /opt/ros/jazzy/share/nav2_bringup ]"
check "Blender 설치됨"             "command -v blender"
check "glxinfo 설치됨"             "command -v glxinfo"

echo
echo "${BLD}=== 버전 ===${RST}"
sudo -u "$REAL_USER" bash -lc 'source /opt/ros/jazzy/setup.bash && echo "  gz-sim   : $(gz sim --versions 2>/dev/null | head -1)"' || true
echo "  ros_gz   : $(dpkg-query -W -f='${Version}' ros-jazzy-ros-gz 2>/dev/null || echo '?')"
echo "  nav2     : $(dpkg-query -W -f='${Version}' ros-jazzy-navigation2 2>/dev/null || echo '?')"
echo "  blender  : $(blender --version 2>/dev/null | head -1 || echo '?')"

echo
echo "${BLD}=== GL 렌더러 (NVIDIA 여야 함) ===${RST}"
sudo -u "$REAL_USER" glxinfo -B 2>/dev/null | grep -E 'OpenGL vendor|OpenGL renderer|Device' | sed 's/^/  /' \
  || warn "glxinfo 실행 실패 — 그래픽 세션 밖에서 실행 중일 수 있습니다"

echo
if [[ $FAIL -eq 0 ]]; then
  echo "${GRN}${BLD}단계 0 완료.${RST}"
  echo "새 터미널을 열고 (또는 'source ~/.bashrc') 다음으로 확인하세요:"
  echo "  ${BLD}gz sim shapes.sdf${RST}"
else
  die "일부 검증 실패 — 위 ✗ 항목 확인"
fi
