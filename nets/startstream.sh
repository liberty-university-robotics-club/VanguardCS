#!/bin/bash
#first argument is server
fmpeg -re -f v4l2 -framerate 6 -video_size 160x120 -i /dev/video0
-vcodec libx264  -preset superfast -crf 30  -f mpegts pipe:  | nc -u $1 5001
