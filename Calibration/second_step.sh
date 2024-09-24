#!/bin/bash

read -p "How many POSES do you want to take? : " nbpose
read -p "How many Lines does your calibraiton model have? : " rows
read -p "How many Columns does your calibration model have? : " cols
read -p "What is the distance between 2 points of interest? : " Dist
read -p "Please set 1 for Master camera and 2 for the Salve camera ! : " numC

paths=$(pwd)

echo "let's go for $nobpose poses !"

touch "point_cam$numC.txt"

for ((i=1;i<=nbpose;i++)); do
	if [ $numC -eq 1 ]; then
	
	./build/step_two_sort $paths/build/data_point/point_cam1_$i.txt $paths/build/datasorted/point_sorted_1.txt /home/dan/calibsfera/build/data_img/Im_point_cam1_$i.png $Dist $rows $cols
	
	else
	
	./build/step_two_sort $paths/build/data_point/point_cam2_$i.txt $paths/build/datasorted/point_sorted_2.txt /home/dan/calibsfera/build/data_img/Im_point_cam2_$i.png $Dist $rows $cols
	
	fi
done

echo "Perfect, you have done all expected poses !"
echo "Please check that all poses are good before moving on to step 3 ! "
read -p "press a button to continue..."
