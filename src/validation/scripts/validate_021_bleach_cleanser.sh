# 021_bleach_cleanser found in videos 51 54 55 57
ROOT=/mnt/ycb_video/640_480
NAME=021_bleach_cleanser
OUTPUT=/home/xenvre/validation/$NAME

for video_id in 0051 0054 0055 0057;
do
    mkdir -p $OUTPUT/$video_id;
    object-tracking --from config_ycbvideonrt.ini --OBJECT::object_name $NAME --OBJECT::path $ROOT/$video_id/output --LOG::absolute_log_path $OUTPUT/$video_id --autostart true;
done
