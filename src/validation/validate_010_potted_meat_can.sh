# 010_potted_meat_can is found in videos 49 53 59
ROOT=/mnt/ycb_video/640_480
NAME=010_potted_meat_can
OUTPUT=/home/xenvre/validation/$NAME

for video_id in 0049 0053 0059;
do
    mkdir -p $OUTPUT/$video_id;
    object-tracking --from config_ycbvideonrt.ini --OBJECT::object_name $NAME --OBJECT::path $ROOT/$video_id/output --LOG::absolute_log_path $OUTPUT/$video_id --autostart true;
done
