#!/bin/bash

SESH="yolo_tmux_session"
tmux has-session -t $SESH 2>/dev/null

if [ $? != 0 ]; then
    tmux new-session -d -s $SESH -n "rspx4"

    tmux send-keys -t $SESH:rspx4 "cd ~/Fast-Drone-XI35" C-m
    tmux send-keys -t $SESH:rspx4 "source ~/Fast-Drone-XI35/devel/setup.bash" C-m
    tmux send-keys -t $SESH:rspx4 "sh ~/Fast-Drone-XI35/shfiles/rspx4_xp.sh" C-m

    tmux new-window -t $SESH -n "kffusion"
    tmux send-keys -t $SESH:kffusion "cd ~/Fast-Drone-XI35" C-m
    tmux send-keys -t $SESH:kffusion "source ~/Fast-Drone-XI35/devel/setup.bash" C-m
    tmux send-keys -t $SESH:kffusion "sh ~/Fast-Drone-XI35/shfiles/kf_fusion.sh" 

    tmux new-window -t $SESH -n "yolo"
    tmux send-keys -t $SESH:yolo "cd ~/Fast-Drone-XI35" C-m
    tmux send-keys -t $SESH:yolo "source ~/Fast-Drone-XI35/devel/setup.bash" C-m
    tmux send-keys -t $SESH:yolo "roslaunch yolo_trt_detector yolo_trt_detector.launch" C-m

    tmux select-window -t $SESH:rspx4

fi

tmux attach-session -t $SESH
