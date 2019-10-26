#!/bin/sh
while :
do
  sudo /home/pi/rc2018_B/ttrnds/Do # 実行ファイルを指定
  if [ $? -eq 0 ]; then
    exit 0
  fi
  sleep 1
done
