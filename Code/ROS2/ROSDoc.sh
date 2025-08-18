# This file is an executable for extracting ROS infos, and saving them to a text file
# directory: /home/fluently2/Documents/MB

# To do:
# - Fix bash window title: title is so long that the windows titles are all the same
#   Only the bash that are opened somewere else can be distinguished

# Some commands (notes just for MB)
# $nano ~/.bashrc => ~ == AltGr + ~ + space 
#                 => Exit: ctrl+x, poi ti chiede se vuoi salvare yes or no
# $gedit ROSDoc.sh => Save: ctrl+s
# Make the text file executable: $chmod +x ROSDoc.sh
# Test command: $ls -la
# alias lsa="ls -la"
# touch "$filename"
# Rename file: $mv ROS1_ROSDoc.sh ROSDoc.sh, $rm file
# $echo $SHELL => Mostra 
# $ps -a => Show all processes
# $echo $$ => nr => $ps -p nr => Show all details of the nr process, es bash

my_dir=/home/fluently2/Documents/MB
my_date_time=$(/bin/date "+%Y_%m_%d__%H_%M_%S")
my_output_file="$my_dir/ROSDocOut_$my_date_time.txt"

echo "Writing ROS infos into $my_output_file"

# The parameter and service list should belong to the end of the ROSDoc file, 
#   but if a node is missing, this error arises
#   ERROR: Communication with node[http://10.11.31.113:38635/] failed!
#   So it has been moved at the file beginning
echo "rosparam list:" >> "$my_output_file"
echo "####################" >> "$my_output_file"
rosparam list >> "$my_output_file"

echo "rosservice list:" >> "$my_output_file"
echo "####################" >> "$my_output_file"
rosservice list >> "$my_output_file"

echo "rosmsg packages:" >> "$my_output_file"
echo "####################" >> "$my_output_file"
rosmsg packages >> "$my_output_file"

echo "rosmsg list:" >> "$my_output_file"
echo "####################" >> "$my_output_file"
rosmsg list >> "$my_output_file"

echo "rosnode list:" >> "$my_output_file"
echo "####################" >> "$my_output_file"
rosnode list >> "$my_output_file"

echo "rostopic list:" >> "$my_output_file"
echo "####################" >> "$my_output_file"
rostopic list >> "$my_output_file"
topic_names=$(rostopic list) >> "$my_output_file"

for topic in $topic_names
do
  echo "rostopic info: $topic" >> "$my_output_file"
  rostopic info $topic >> "$my_output_file"
  echo "####################" >> "$my_output_file"
done

node_names=$(rosnode list)

for node in $node_names
do
  echo "rosnode info: $node" >> "$my_output_file"
  rosnode info $node >> "$my_output_file"
  echo "####################" >> "$my_output_file"
done

exit 1

# On Dexterity launch following bash:
# 1) $roscore
# 2) $roslaunch fanuc_ros_driver fanuc_interface.launch ip:="10.11.31.111" robot_type:="CRX-20iA/L" license:="/home/fluently2/Documents/license_2.data"
# 3) $rosrun ontology_adapter intent_publisher.py
