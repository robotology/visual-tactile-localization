# 005_tomato_soup_can is found in videos 50 51 53 55 57 59
ROOT=/mnt/ycb_video/640_480
NAME=005_tomato_soup_can
OUTPUT=/home/xenvre/validation/$NAME

for video_id in 0050 0051 0053 0055 0057 0059;
do
    mkdir -p $OUTPUT/$video_id;
    object-tracking --from config_ycbvideonrt.ini --OBJECT::object_name $NAME --OBJECT::path $ROOT/$video_id/output --LOG::absolute_log_path $OUTPUT/$video_id --autostart true;
done
