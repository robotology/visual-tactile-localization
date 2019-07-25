# 009_gelatin_box found in videos 58
ROOT=/mnt/ycb_video/640_480
NAME=009_gelatin_box
OUTPUT=/home/xenvre/validation/$NAME

for video_id in 0057;
do
    mkdir -p $OUTPUT/$video_id;
    object-tracking --from config_ycbvideonrt.ini --OBJECT::object_name $NAME --OBJECT::path $ROOT/$video_id/output --LOG::absolute_log_path $OUTPUT/$video_id --autostart true;
done
