#!/bin/bash

SESH="fast_drone_xi35_tmux_session"
ROS_HOSTNAME="192.168.31.97"
ROS_MASTER_URI="http://192.168.31.97:11311"

tmux has-session -t $SESH 2>/dev/null

if [ $? != 0 ]; then
    tmux new-session -d -s $SESH -n "rspx4"

    tmux send-keys -t $SESH:rspx4 "export ROS_MASTER_URI=$ROS_MASTER_URI" C-m
    tmux send-keys -t $SESH:rspx4 "export ROS_HOSTNAME=$ROS_HOSTNAME" C-m
    tmux send-keys -t $SESH:rspx4 "source ~/Fast-Drone-XI35/devel/setup.bash" C-m
    tmux send-keys -t $SESH:rspx4 "sh ~/Fast-Drone-XI35/shfiles/rspx4_xp.sh"

    tmux new-window -t $SESH -n "kffusion"
    tmux send-keys -t $SESH:kffusion "export ROS_MASTER_URI=$ROS_MASTER_URI" C-m
    tmux send-keys -t $SESH:kffusion "export ROS_HOSTNAME=$ROS_HOSTNAME" C-m
    tmux send-keys -t $SESH:kffusion "source ~/Fast-Drone-XI35/devel/setup.bash" C-m
    tmux send-keys -t $SESH:kffusion "sh ~/Fast-Drone-XI35/shfiles/kf_fusion.sh"

    tmux new-window -t $SESH -n "px4ctrl"
    tmux send-keys -t $SESH:px4ctrl "export ROS_MASTER_URI=$ROS_MASTER_URI" C-m
    tmux send-keys -t $SESH:px4ctrl "export ROS_HOSTNAME=$ROS_HOSTNAME" C-m
    tmux send-keys -t $SESH:px4ctrl "source ~/Fast-Drone-XI35/devel/setup.bash" C-m
    tmux send-keys -t $SESH:px4ctrl "roslaunch px4ctrl run_ctrl.launch"

    tmux new-window -t $SESH -n "egoplanner"
    tmux send-keys -t $SESH:egoplanner "export ROS_MASTER_URI=$ROS_MASTER_URI" C-m
    tmux send-keys -t $SESH:egoplanner "export ROS_HOSTNAME=$ROS_HOSTNAME" C-m
    tmux send-keys -t $SESH:egoplanner "source ~/Fast-Drone-XI35/devel/setup.bash" C-m
    tmux send-keys -t $SESH:egoplanner "roslaunch ego_planner single_run_in_exp.launch"

    tmux new-window -t $SESH -n "searchplan"
    tmux send-keys -t $SESH:searchplan "export ROS_MASTER_URI=$ROS_MASTER_URI" C-m
    tmux send-keys -t $SESH:searchplan "export ROS_HOSTNAME=$ROS_HOSTNAME" C-m
    tmux send-keys -t $SESH:searchplan "source ~/Fast-Drone-XI35/devel/setup.bash" C-m
    tmux send-keys -t $SESH:searchplan "roslaunch search_plan search_plan.launch"

    tmux new-window -t $SESH -n "takeoff"
    tmux send-keys -t $SESH:takeoff "sh ~/Fast-Drone-XI35/shfiles/takeoff.sh"

    tmux new-window -t $SESH -n "land"
    tmux send-keys -t $SESH:land "sh ~/Fast-Drone-XI35/shfiles/land.sh"

    tmux select-window -t $SESH:rspx4
fi

tmux attach-session -t $SESH
