#!/bin/bash

ffmpeg -y -i $1 -c:v libx264 -preset ultrafast -qp 0 output.mp4

ffmpeg -y -i output.mp4 -i $2 -filter_complex '[0:v]pad=iw*2:ih[int];[int][1:v]overlay=W/2:0[vid]' -map [vid] -c:v libx264 -crf 23 -preset veryfast compare.mp4

mplayer compare.mp4
