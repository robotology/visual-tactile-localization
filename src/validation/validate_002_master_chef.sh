# 002_master_chef is found in videos 48 51 55 56
ROOT=/mnt/ycb_video/640_480
NAME=002_master_chef_can
OUTPUT=/home/xenvre/validation/$NAME

for video_id in 0048 0051 0055 0056;
do
    mkdir -p $OUTPUT/$video_id;
    object-tracking --from config_ycbvideonrt.ini --OBJECT::object_name $NAME --OBJECT::path $ROOT/$video_id/output --LOG::absolute_log_path $OUTPUT/$video_id --autostart true;
done
