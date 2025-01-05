#!/bin/bash

##############################################################################################################################
# The purpose of this script is to determine if the wiki text file is up to date on all the publishers implemented in the node
##############################################################################################################################

# Error and print everything
set -ex

# Find the script directory
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
project_dir="${script_dir}/.."

# Parse the mip_publisher_mapping header to find all the topics we have listed
node_topics=$(cat "${project_dir}/include/microstrain_inertial_driver_common/utils/mappings/mip_publisher_mapping.h"  | grep -E "static constexpr auto \w+_TOPIC" | cut -d'=' -f2 | tr -d ' ' | cut -d';' -f1 | tr -d '"')
wiki_topics=$(cat "${project_dir}/wiki/main.txt" | sed -n '/== Subscriptions ==/q;p' | grep -oE "^ \* '''(\w|/)+" | tr -s "'" | cut -d"'" -f2 | sed 's/^\/\(.*\)$/\1/g')

# Stop printing everything for the loop
set +x

# Loop through the topics in the node, and verify that they exist in the wiki
missing_topics=false
for node_topic in ${node_topics}; do
  if echo "${wiki_topics}" | grep -q -E "^${node_topic}\$"; then
    echo "INFO: The wiki contains documentation for ${node_topic}"
  else
    missing_topics=true
    echo "ERROR: The wiki does not contain documentation for ${node_topic}"
  fi
done

# If there are missing topics, error
if [ "${missing_topics}" = true ]; then
  echo "ERROR: wiki documentation is missing one or more topics. Please document them"
  exit 1
fi