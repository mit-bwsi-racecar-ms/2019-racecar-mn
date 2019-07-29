#!/bin/bash

cd ~/racecar_ws/jupyter_ws/racecar-mn

git reset --hard

git clean -fdx
echo "cleaned"
git pull origin master

echo "pulled"
