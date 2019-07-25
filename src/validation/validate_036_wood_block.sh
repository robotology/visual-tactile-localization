# 036_wood_block is found in videos 55
ROOT=/mnt/ycb_video/640_480
NAME=036_wood_block
OUTPUT=/home/xenvre/validation/$NAME

for video_id in 0055;
do
    mkdir -p $OUTPUT/$video_id;
    object-tracking --from config_ycbvideonrt.ini --OBJECT::object_name $NAME --OBJECT::path $ROOT/$video_id/output --LOG::absolute_log_path $OUTPUT/$video_id --autostart true;
done
