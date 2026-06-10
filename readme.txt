!!Use shfiles/yolo_trt_detector_tmux.sh to run those commands.

source devel/setup.bash
1.rspx4.sh
2.kf_fusion.sh
3.yolo_trt_detector.launch

!!Open another terminator and run:
4.rostopic echo /yolo_trt/target_point

!!Use rqt to see the annotated image
5.rqt_view_image
