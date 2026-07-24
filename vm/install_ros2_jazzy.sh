#!/usr/bin/env bash
set -euo pipefail

if [[ ! -r /etc/os-release ]]; then
  echo "无法识别系统；本脚本仅支持 Ubuntu 24.04。"
  exit 1
fi

# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID}" != "ubuntu" || "${VERSION_CODENAME:-}" != "noble" ]]; then
  echo "当前系统为 ${PRETTY_NAME:-unknown}；需要 Ubuntu 24.04 (Noble)。"
  exit 1
fi

sudo apt-get update
sudo apt-get install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository universe -y

if [[ ! -f /etc/apt/sources.list.d/ros2.sources ]]; then
  ROS_APT_SOURCE_VERSION="$(
    curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      | sed -n 's/.*"tag_name": "\([^"]*\)".*/\1/p'
  )"
  if [[ -z "${ROS_APT_SOURCE_VERSION}" ]]; then
    echo "无法取得 ros-apt-source 版本。"
    exit 1
  fi
  curl -fL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
fi

sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y \
  ros-jazzy-desktop \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-octomap-server \
  ros-jazzy-ros-gz \
  ros-jazzy-slam-toolbox \
  ros-jazzy-teleop-twist-keyboard \
  libgl1-mesa-dri \
  mesa-utils \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-yaml \
  ros-dev-tools

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
if grep -q 'raw.githubusercontent.com/ros/rosdistro/master' \
    /etc/ros/rosdep/sources.list.d/20-default.list; then
  sudo cp -n /etc/ros/rosdep/sources.list.d/20-default.list \
    /etc/ros/rosdep/sources.list.d/20-default.list.github-backup
  sudo sed -i \
    's#raw.githubusercontent.com/ros/rosdistro/master#mirrors.ustc.edu.cn/rosdistro#g' \
    /etc/ros/rosdep/sources.list.d/20-default.list
fi
if grep -q '/releases/fuerte.yaml' \
    /etc/ros/rosdep/sources.list.d/20-default.list; then
  echo '正在移除与 ROS 2 Jazzy 无关的 Fuerte rosdep 索引'
  sudo sed -i '\#/releases/fuerte.yaml#d' \
    /etc/ros/rosdep/sources.list.d/20-default.list
fi
export ROSDISTRO_INDEX_URL='https://mirrors.ustc.edu.cn/rosdistro/index-v4.yaml'
if ! rosdep update; then
  if [[ -d "$HOME/.ros/rosdep/sources.cache" ]]; then
    echo '警告：rosdep 镜像部分超时，将使用刚刚更新的本地缓存继续。'
  else
    echo '错误：rosdep 更新失败，并且没有可用缓存。'
    exit 1
  fi
fi

if ! grep -qF 'source /opt/ros/jazzy/setup.bash' "${HOME}/.bashrc"; then
  printf '\nsource /opt/ros/jazzy/setup.bash\n' >> "${HOME}/.bashrc"
fi

echo "ROS 2 Jazzy 环境安装完成。返回 Mac 执行 ./scripts/build_vm.sh。"
