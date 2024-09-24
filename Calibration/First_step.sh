#!/bin/bash

read -p "How many POSES do you want to take ?: " nbpose
read -p "How many Lines does your calibraiton model have ? : " rows
read -p "How many Columns does your calibration model have ? : " cols
read -p "What is the Serial ID of the Slave camera : " Slave
read -p "What is the Serial ID of the Master camera : " Master

paths=$(pwd)

rm -rf $paths/build/data_img/*
rm -rf $paths/build/data_point/*

echo "let's go for $nobpose poses !"
echo "$rows lignes"

for i in $(seq 1 $nbpose); do
	for k in $(seq 1 2); do
		echo "begin of the pose number $i : "
		
		if [ $k -eq 1 ]; then
			./build/step_one_primitive_detection_recording --output-dir $paths/build/ --output-name point_cam1_$i --rows $rows --cols $cols --number-pose $nbpose -s $Master
		else
			./build/step_one_primitive_detection_recording --output-dir $paths/build/ --output-name point_cam2_$i --rows $rows --cols $cols --number-pose $nbpose -s $Slave
		fi
		
		echo "Pose number $i recorded !"
	done
done

echo "Perfect, you have done all expected poses !"
echo "Please check that all poses are good before moving on to step 2 ! "

read -p "press a button to continue..."
