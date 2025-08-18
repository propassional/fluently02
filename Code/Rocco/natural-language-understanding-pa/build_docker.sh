#!/bin/sh
sudo docker build -t rasa_ros .
sudo docker run -it --rm --name my_rasa_ros rasa_ros 
