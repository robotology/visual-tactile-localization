# 007_tuna_fish_can is found in videos 48 49 52 59
ROOT=/mnt/ycb_video/640_480
NAME=007_tuna_fish_can
OUTPUT=/home/xenvre/validation/$NAME

for video_id in 0048 0049 0052 0059;
do
    mkdir -p $OUTPUT/$video_id;
    object-tracking --from config_ycbvideonrt.ini --OBJECT::object_name $NAME --OBJECT::path $ROOT/$video_id/output --LOG::absolute_log_path $OUTPUT/$video_id --autostart true;
done
