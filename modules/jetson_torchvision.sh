#!/bin/bash

export FORCE_CUDA=1
export MAX_JOBS=2

pip3 uninstall -y torchvision

## Function to get the installed PyTorch version
get_torch_version() {
    python3 -c "import torch; print(torch.__version__)" 2>/dev/null
}

## Function to map PyTorch version to corresponding torchvision version
get_vision_version() {
    local torch_version=$1
    case $torch_version in
        2.5*) echo "0.20" ;;
        2.4*) echo "0.19" ;;
        2.3*) echo "0.18" ;;
        2.2*) echo "0.17" ;;
        2.1*) echo "0.16" ;;
        2.0*) echo "0.15" ;;
        1.13*) echo "0.14" ;;
        1.12*) echo "0.13" ;;
        1.11*) echo "0.12" ;;
        1.10*) echo "0.11" ;;
        1.9*) echo "0.10" ;;
        1.8*) echo "0.9" ;;
        1.7*) echo "0.8" ;;
        1.6*) echo "0.7" ;;
        1.5*) echo "0.6" ;;
        1.4*) echo "0.5" ;;
        1.3*) echo "0.4.2" ;;
        1.2*) echo "0.4.1" ;;
        1.1*) echo "0.3" ;;
        *) echo "" ;;
    esac
}

## Function to find the latest matching tag
find_latest_tag() {
    local version_prefix=$1
    git tag -l | grep "^v${version_prefix}" | sort -rV | head -n 1
}

## Main script execution
main() {
    ## Get the installed PyTorch version
    torch_version=$(get_torch_version)
    if [ -z "$torch_version" ]; then
        echo "PyTorch is not installed."
        exit 1
    fi
    echo "Detected PyTorch version: $torch_version"

    ## Get the corresponding torchvision version prefix
    vision_version_prefix=$(get_vision_version "$torch_version")
    if [ -z "$vision_version_prefix" ]; then
        echo "No corresponding torchvision version found for PyTorch $torch_version."
        exit 1
    fi
    echo "Corresponding torchvision version prefix: $vision_version_prefix"

    ## Clone the torchvision repository
    repo_dir="$HOME/lib/jetson/vision"
    if [ -d "$repo_dir" ]; then
        echo "Directory $repo_dir already exists. Pulling latest changes."
        cd "$repo_dir" && git pull
    else
        echo "Cloning torchvision repository into $repo_dir."
        git clone https://github.com/pytorch/vision "$repo_dir"
        cd "$repo_dir"
    fi

    ## Fetch all tags and find the latest matching tag
    git fetch --tags
    latest_tag=$(find_latest_tag "$vision_version_prefix")
    if [ -z "$latest_tag" ]; then
        echo "No matching tag found for prefix v$vision_version_prefix."
        exit 1
    fi
    echo "Latest matching tag: $latest_tag"

    ## Checkout the latest tag
    git checkout "$latest_tag"

    ## Install build dependencies
    echo "Installing build dependencies."
    ## sudo apt update
    sudo apt install -y libjpeg-dev zlib1g-dev libpython3-dev libopenblas-dev libavcodec-dev libavformat-dev libswscale-dev g++ gcc ninja-build

    ## Build and install torchvision
    echo "Building and installing torchvision."
    # python3 -m pip install --upgrade pip
    pip3 install setuptools==69.5.1 packaging==22.0
    python3 setup.py install --user

    ## Verify installation of torchvision
    echo "Verifying torchvision installation."
    python3 -c "
try:
    import torchvision
    print('torchvision version:', torchvision.__version__)
except ImportError:
    print('Failed to import torchvision.')
    exit(1)
    "

    ## Verify CUDA availability
    echo "Checking CUDA availability in PyTorch."
    python3 -c "
import torch
if torch.cuda.is_available():
    print('CUDA is available in PyTorch.')
else:
    print('CUDA is not available in PyTorch.')
"
}

# Execute the main function
main

