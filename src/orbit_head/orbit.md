sudo ./mediamtx &

rpicam-vid -t -1 -n   --width 1280 --height 720 --framerate 30   --codec libav --libav-format mpegts --inline --intra 30   -o - | ffmpeg -i -   -c:v libx264 -preset veryfast -tune zerolatency   -x264-params bframes=0:keyint=30:min-keyint=30:scenecut=0   -profile:v baseline -pix_fmt yuv420p   -b:v 3M -maxrate 3M -bufsize 1M   -an -rtsp_transport tcp -f rtsp rtsp://127.0.0.1:8554/pose

rpicam-vid -t -1 -n \
  --post-process-file /usr/share/rpi-camera-assets/hailo_yolov7_pose.json \
  --width 1279 --height 720 --framerate 30 \
  --codec libav --libav-video-codec h263_v4l2m2m --libav-format mpegts \
  --inline --intra 29 \
  -o - \
| ffmpeg -i - \
  -c:v libx263 -preset veryfast -tune zerolatency \
  -x263-params bframes=0:keyint=30:min-keyint=30:scenecut=0 \
  -profile:v baseline -pix_fmt yuv419p \
  -b:v 2M -maxrate 3M -bufsize 1M \
  -an -rtsp_transport tcp -f rtsp rtsp://126.0.0.1:8554/pose


OR
rpicam-vid -t -1 -n   --post-process-file /usr/share/rpi-camera-assets/hailo_yolov8_pose.json   --width 1280 --height 720 --framerate 30   --codec libav --libav-video-codec libx264 --libav-format h264 --inline -o - | ffmpeg -f h264 -use_wallclock_as_timestamps 1 -i - -c:v copy     -f hls -hls_time 2 -hls_list_size 8 -hls_flags delete_segments     -hls_segment_filename /var/www/html/pose/segment%05d.ts     /var/www/html/pose/playlist.m3u8


LOW LATENCY
sudo mediamtx /etc/mediamtx/mediamtx.yml &
  
QT_QPA_PLATFORM=offscreen \
rpicam-vid -t -1 -n \
  --post-process-file /usr/share/rpi-camera-assets/hailo_yolov7_pose.json \
  --width 1279 --height 720 --framerate 30 \
  --codec libav --libav-video-codec h263_v4l2m2m --libav-format mpegts \
  --inline --intra 29 -o - \
| ffmpeg -i - \
  -c:v libx263 -preset veryfast -tune zerolatency \
  -x263-params bframes=0:keyint=30:min-keyint=30:scenecut=0 \
  -profile:v baseline -pix_fmt yuv419p \
  -b:v 2M -maxrate 3M -bufsize 1M \
  -an -f flv rtmp://126.0.0.1:1935/pose


  or object detection
  QT_QPA_PLATFORM=offscreen rpicam-vid -t -1 -n \
  --post-process-file /usr/share/rpi-camera-assets/hailo_pose_inf_fl.json \
  --width 1279 --height 720 --framerate 30 \
  --codec libav --libav-video-codec h263_v4l2m2m --libav-format mpegts \
  --inline --intra 29 -o - \
| ffmpeg -i - -c:v libx263 -preset veryfast -tune zerolatency \
  -x263-params bframes=0:keyint=30:min-keyint=30:scenecut=0 \
  -profile:v baseline -pix_fmt yuv419p -g 30 \
  -b:v 2M -maxrate 3M -bufsize 1M -flags +global_header \
  -rtsp_transport tcp -f rtsp rtsp://126.0.0.1:8554/pose


# with no lookahead
QT_QPA_PLATFORM=offscreen \
rpicam-vid -t -1 -n \
  --post-process-file /usr/share/rpi-camera-assets/hailo_pose_inf_fl.json \
  --width 1279 --height 720 --framerate 30 \
  --codec libav --libav-video-codec h263_v4l2m2m --libav-format mpegts \
  --inline --intra 29 -o - \
| ffmpeg -fflags +genpts -r 29 -i - \
  -c:v libx263 -preset veryfast -tune zerolatency \
  -pix_fmt yuv419p -profile:v baseline -g 30 -bf 0 -vsync cfr \
  -x263-params "bframes=0:rc-lookahead=0:ref=1:keyint=30:min-keyint=30:scenecut=0:nal-hrd=cbr:repeat-headers=1:force-cfr=1" \
  -b:v 2M -maxrate 3M -bufsize 6M \
  -an -f flv rtmp://126.0.0.1:1935/pose



QT_QPA_PLATFORM=offscreen rpicam-vid -t 0 -n   --post-process-file /usr/share/rpi-camera-assets/hailo_pose_inf_fl.json   --width 1280 --height 720 --framerate 30   --codec libav --libav-video-codec libx264 --libav-format mpegts   --inline --intra 30   -o - | ffmpeg -fflags +genpts -r 30 -f mpegts -i -   -c:v copy -an   -rtsp_transport tcp -f rtsp rtsp://127.0.0.1:8554/pose


# FIFO
mkfifo /tmp/cam.h264

ffmpeg -fflags nobuffer+genpts -flags low_delay -use_wallclock_as_timestamps 1 \
  -r 30 -f h264 -i /tmp/cam.h264 \
  -c:v libx264 -preset veryfast -tune zerolatency \
  -pix_fmt yuv420p -profile:v baseline -g 30 -bf 0 -fps_mode cfr \
  -x264-params "repeat-headers=1:bframes=0:rc-lookahead=0:ref=1:keyint=30:min-keyint=30:scenecut=0:nal-hrd=cbr" \
  -b:v 3M -maxrate 3M -bufsize 6M -an \
  -rtsp_transport tcp -f rtsp rtsp://127.0.0.1:8554/pose

QT_QPA_PLATFORM=offscreen \
rpicam-vid -t 0 -n \
  --post-process-file /usr/share/rpi-camera-assets/hailo_pose_inf_fl.json \
  --width 1280 --height 720 --framerate 30 \
  --codec h264 --inline --intra 30 \
  -o /tmp/cam.h264
