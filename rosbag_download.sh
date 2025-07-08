#!/bin/bash
# This script downloads the r2b rosbag from NVIDIA's Isaac ROS dataset catalog
echo "Creating directory for rosbag..."
mkdir -p r2b_storage
cd r2b_storage

echo "Please download the following files manually from the NVIDIA catalog:"
echo "1. rosbag_metadata.yaml"
echo "2. rosbag.db3"
echo ""
echo "URL: https://catalog.ngc.nvidia.com/orgs/nvidia/teams/isaac/resources/r2bdataset2023/files"
echo ""
echo "Place both files in the 'r2b_storage' folder."

