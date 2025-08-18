# This file is an executable for extracting ROS infos, and saving them to a text file
# directory: /home/fluently2/Documents/MB

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