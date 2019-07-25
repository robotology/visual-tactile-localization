# 004_sugar_box is found in videos 49 51 54 55 58
ROOT=/mnt/ycb_video/640_480
NAME=004_sugar_box
OUTPUT=/home/xenvre/validation/$NAME

for video_id in 0049 0051 0054 0055 0058;
do
    mkdir -p $OUTPUT/$video_id;
    object-tracking --from config_ycbvideonrt.ini --OBJECT::object_name $NAME --OBJECT::path $ROOT/$video_id/output --LOG::absolute_log_path $OUTPUT/$video_id --autostart true;
done
