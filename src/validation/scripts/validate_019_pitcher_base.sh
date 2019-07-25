# 019_pitcher_base found in videos 52 56 58
ROOT=/mnt/ycb_video/640_480
NAME=019_pitcher_base
OUTPUT=/home/xenvre/validation/$NAME

for video_id in 0052 0056 0058;
do
    mkdir -p $OUTPUT/$video_id;
    object-tracking --from config_ycbvideonrt.ini --OBJECT::object_name $NAME --OBJECT::path $ROOT/$video_id/output --LOG::absolute_log_path $OUTPUT/$video_id --autostart true;
done
