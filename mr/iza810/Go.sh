#!/bin/sh
while :
do
  /home/pi/robocon_2019b/mr/iza810/Do # 実行ファイルを指定
  if [ $? -eq 0 ]; then
    exit 0
  fi
  sleep 1
done
