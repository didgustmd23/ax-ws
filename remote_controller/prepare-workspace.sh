#!/usr/bin/env bash
set -eo pipefail

. /etc/os-release
if [[ ${ID:-} != ubuntu || ${VERSION_ID:-} != 24.04 ]]; then
    echo 'AX requires Ubuntu 24.04. Select it with install.cmd -Distro Ubuntu-24.04.' >&2
    exit 1
fi
if [[ $(uname -r) != *WSL2* ]]; then
    echo 'AX requires WSL 2. In PowerShell run wsl --set-version <Ubuntu distro name> 2.' >&2
    exit 1
fi
if (( EUID == 0 )); then
    echo 'Finish Ubuntu setup and set a regular Ubuntu user before running install.cmd.' >&2
    exit 1
fi

workspace=${1:-}
if [[ $workspace == '~/'* ]]; then
    workspace=$HOME/${workspace:2}
elif [[ -n $workspace && $workspace != /* ]]; then
    workspace=$HOME/$workspace
fi
if [[ -z $workspace ]]; then
    workspace=$HOME/ax-ws
    if [[ -f $HOME/ax-play-ws/src/ax_description/package.xml ]]; then
        workspace=$HOME/ax-play-ws
    fi
fi

if [[ ! -f $workspace/src/ax_description/package.xml ]]; then
    if [[ ! -f src/ax_description/package.xml ]]; then
        echo 'Download and extract the complete AX repository before running install.cmd.' >&2
        exit 1
    fi
    if [[ -e $workspace || -L $workspace ]]; then
        echo "Choose a new AX workspace path. This path already exists: $workspace" >&2
        exit 1
    fi
    echo "Copying AX sources to $workspace"
    mkdir -p -- "$(dirname -- "$workspace")"
    staging=$(mktemp -d -- "${workspace}.XXXXXX")
    tar -cf - src tools maps world_assets requirements.txt | tar -xf - -C "$staging"
    mv -nT -- "$staging" "$workspace"
    if [[ ! -f $workspace/src/ax_description/package.xml ]]; then
        echo "AX sources were not copied to $workspace." >&2
        exit 1
    fi
fi

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo 'Installing ROS 2 Jazzy. Enter your Ubuntu password if sudo prompts.'
    sudo apt-get update
    sudo apt-get install -y curl software-properties-common
    sudo add-apt-repository -y -n universe
    if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
        release=$(curl -fsSL -o /dev/null -w '%{url_effective}' https://github.com/ros-infrastructure/ros-apt-source/releases/latest)
        version=${release##*/}
        package=$(mktemp --suffix=.deb)
        trap 'rm -f -- "$package"' EXIT
        curl -fsSL -o "$package" "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version}.noble_all.deb"
        sudo dpkg -i "$package"
    fi
    sudo apt-get update
    sudo apt-get install -y ros-jazzy-desktop
fi

if ! (
    [[ -f $workspace/install/setup.bash ]] &&
    source /opt/ros/jazzy/setup.bash &&
    source "$workspace/install/setup.bash" &&
    ros2 pkg prefix ax_control >/dev/null 2>&1 &&
    ros2 pkg prefix ax_perception >/dev/null 2>&1
); then
    echo "Building the AX workspace at $workspace"
    bash "$workspace/tools/setup_jazzy.sh"
fi
