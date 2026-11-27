#!/bin/bash
# Usage: bags/record_bag.sh <name> [map_name]
# Records all topics (except compressed camera streams) into bags/<name> and copies the map in use into
# bags/<name>/map/<map_name>. map_name defaults to the one the running stack uses, else `latest`.

[ -n "$1" ] || { echo "usage: $0 <name> [map_name]" >&2; exit 1; }

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
out=$repo/bags/$1
if [ -e "$out" ]; then
  read -r -p "$out already exists, overwrite? [y/N] " ans
  [[ $ans == [yY] ]] || exit 1
  rm -rf "$out"
fi

map_name=${2:-$(ros2 param get /sector_tuner map_name 2>/dev/null | sed -n 's/^String value is: //p')}
map_name=${map_name:-latest}
map_dir=$repo/stack_master/maps/$map_name
[ -d "$map_dir" ] || { echo "map $map_dir not found" >&2; exit 1; }
echo "recording to $out with map $map_name"

# ros2 bag record creates $out itself, so copy the map once it exists
(
  for _ in $(seq 100); do
    [ -d "$out" ] && { mkdir "$out/map" && cp -rL "$map_dir" "$out/map/"; exit; }
    sleep 0.1
  done
) &

exec ros2 bag record -a -x '/camera/.*/(compressed|compressedDepth|theora)' -o "$out"
