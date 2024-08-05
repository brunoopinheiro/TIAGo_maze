#! /usr/bin/bash

# Copy the world files
cp TiagoMaze/maze.world /tiago_public_ws/src/pal_gazebo_worlds/worlds/
cp TiagoMaze/mazeTest1.world /tiago_public_ws/src/pal_gazebo_worlds/worlds/
cp TiagoMaze/mazeTest2.world /tiago_public_ws/src/pal_gazebo_worlds/worlds/

# Copy the object directories
cp -r TiagoMaze/maze /tiago_public_ws/src/pal_gazebo_worlds/models/
cp -r TiagoMaze/wall /tiago_public_ws/src/pal_gazebo_worlds/models/
cp -r TiagoMaze/landmark_green /tiago_public_ws/src/pal_gazebo_worlds/models/
cp -r TiagoMaze/landmark_red /tiago_public_ws/src/pal_gazebo_worlds/models/
cp -r TiagoMaze/frontWall /tiago_public_ws/src/pal_gazebo_worlds/models/
cp -r TiagoMaze/cornerWall /tiago_public_ws/src/pal_gazebo_worlds/models/
