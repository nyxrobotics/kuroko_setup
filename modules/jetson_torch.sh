#!/bin/bash

# Create directory for Jetson libraries
mkdir -p $HOME/lib/jetson
cd $HOME/lib/jetson

# Get JetPack version using jetson_release and extract the version between "Jetpack" and "["
jetpack_info=$(jetson_release | grep "Jetpack" | sed -n 's/.*Jetpack \(.*\) \[.*/\1/p')

# Remove ANSI escape codes from jetpack_info
jetpack_info=$(echo "$jetpack_info" | sed -E 's/\x1B\[[0-9;]*[a-zA-Z]//g')

if [ -z "$jetpack_info" ]; then
    echo "Failed to detect JetPack version using jetson_release."
    exit 1
fi

# Convert JetPack version to directory format (e.g., 5.1.1 -> v511)
jetpack_version="v$(echo $jetpack_info | tr -d '.')"

echo "Detected JetPack version: $jetpack_info"
echo "Using JetPack directory: $jetpack_version"

# Base URL for NVIDIA PyTorch wheel files
base_url="https://developer.download.nvidia.com/compute/redist/jp/"

# Fetch available JetPack directories from base URL
versions_xml=$(curl -s ${base_url})
echo "Debug: Fetched raw HTML content (first 500 characters):"
echo "${versions_xml:0:500}"

# Extract available versions and clean up duplicates and formatting
available_versions=$(echo "$versions_xml" | grep -Eo 'v[0-9]+/' | sort -u)
echo "Debug: Raw extracted list of directories (with slashes):"
echo "$available_versions"

# Normalize the list by ensuring consistent formatting (trim trailing slashes)
cleaned_versions=$(echo "$available_versions" | sed 's:/$::g')
echo "Debug: Cleaned list of JetPack directories (without trailing slashes):"
echo "$cleaned_versions"

# Debugging: Show each version length and character codes
echo "Debug: Version details:"
echo "$cleaned_versions" | while read -r version; do
    echo "Version: '$version' Length: ${#version} Char codes: $(echo -n "$version" | od -An -t dC | tr -d ' ')"
done

echo "Debug: JetPack version to compare:"
echo "Version: '$jetpack_version' Length: ${#jetpack_version} Char codes: $(echo -n "$jetpack_version" | od -An -t dC | tr -d ' ')"

# Check if calculated version exists in the cleaned list
if echo "$cleaned_versions" | grep -Fxq "$jetpack_version"; then
    echo "Detected JetPack version: $jetpack_info"
    echo "Using JetPack directory: $jetpack_version"
else
    echo "Calculated JetPack directory ($jetpack_version) not found in available versions:"
    echo "$cleaned_versions"
    exit 1
fi

# Final URL for PyTorch wheel files
pytorch_url="${base_url}${jetpack_version}/pytorch/"
echo "Fetching available wheel files from: $pytorch_url"

# Fetch HTML content
html_content=$(curl -s ${pytorch_url})

# Extract wheel files and remove duplicates
available_wheels=$(echo "$html_content" | grep -oE 'torch-[^"]+\.whl' | sort -u)

# Debugging: Print extracted wheel files
echo "Debug: Extracted wheel files (unique):"
echo "$available_wheels"

# Select the latest wheel file
latest_wheel=$(echo "$available_wheels" | sort -rV | head -n 1)

# Debugging: Print the selected latest wheel file
if [ -z "$latest_wheel" ]; then
    echo "No PyTorch wheel files found in $pytorch_url."
    exit 1
fi
echo "Debug: Selected latest wheel file:"
echo "$latest_wheel"

# Construct the download URL
latest_wheel_url="${pytorch_url}${latest_wheel}"

# Download and install PyTorch
echo "Downloading and installing PyTorch from: $latest_wheel_url"
pip3 install --no-cache "$latest_wheel_url"

