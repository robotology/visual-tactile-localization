# 035_power_drill is found in videos 50 54 56 59
ROOT=/mnt/ycb_video/640_480
NAME=035_power_drill
OUTPUT=/home/xenvre/validation/$NAME

for video_id in 0050 0054 0056 0059;
do
    mkdir -p $OUTPUT/$video_id;
    object-tracking --from config_ycbvideonrt.ini --OBJECT::object_name $NAME --OBJECT::path $ROOT/$video_id/output --LOG::absolute_log_path $OUTPUT/$video_id --autostart true;
done
