#!/usr/bin/env bash


while true; do
    read -r -p "goal> " x y z

    if [ "$x" = "q" ]; then
        break
    fi

    if [ -z "$x" ] || [ -z "$y" ] || [ -z "$z" ]; then
        echo "error, please input: x y z"
        continue
    fi

    rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
        "{header: {stamp: now, frame_id: 'world'}, pose: {position: {x: $x, y: $y, z: $z}, orientation: {w: 1.0}}}"
done