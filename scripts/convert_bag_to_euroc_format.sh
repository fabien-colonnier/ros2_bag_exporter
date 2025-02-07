#!/bin/bash

# test command
#  ./convert_bag_to_euroc_format.sh --path2bag=/path/to/bag --bagname=test_0

# Argument validation check
if [ "$#" -lt 2 ]; then
    echo "Usage with minimal arguments: $0 --path2bag=</path/to/bag> --bagname=<bagname>"
    echo ""
    echo "optional arguments:"
    echo "      --outfolder=<folder_path>      # relative to </path/to/bag/> with name, default \"</path/to/bag/><bagname>_csv\" "
    echo "      --cam_sample_interval=<val>      # write a message out every <val>, default 1"
    exit 1
fi

# default val declaration
outfld="empty"
cam_sample_interval=1

# Loop through all arguments
for arg in "$@"; do
  case $arg in
    --path2bag=*)
      path="${arg#*=}"
      shift
      ;;
    --bagname=*)
      bag="${arg#*=}"
      shift
      ;;
    --outfolder=*)
      outfld="${arg#*=}"
      shift
      ;;
    --cam_sample_interval=*)
      cam_sample_interval="${arg#*=}"
      shift
      ;;
    *)
      echo "Unknown argument: $arg"
      exit 1
      ;;
  esac
done

# Validate output folder path
if [ "$outfld" = "empty" ]; then
    outfld="$path/${bag}_csv"
else
    outfld="$path/$outfld"
fi

echo "path to bag : $path/$bag"
echo "output folder: $outfld"
echo "1 camera message will be written every $cam_sample_interval messages"

# declare the ros topic to be converted (TODO maybe creat arguments for this)
cam0_topic="/<cam_0/image>" # Expect to start with "/"
cam1_topic="/<cam_1/image>"
imu_topic="/imu/imu"

echo ""
echo "The expected cam0 topic is $cam0_topic"
echo "The expected cam1 topic is $cam1_topic"
echo "The expected IMU topic is $imu_topic"

# create the yaml file convert_config.yaml from convert_config_template.yaml
cp convert_config_template.yaml convert_config.yaml

# set the variable in the yaml file
sed -i "s@<bagpath>@$path/$bag@g" convert_config.yaml   
sed -i "s@<outfolder>@$outfld/@g" convert_config.yaml
sed -i "s@<1>@$cam_sample_interval@g" convert_config.yaml
sed -i "s@<cam0_topic>@$cam0_topic@g" convert_config.yaml
sed -i "s@<cam1_topic>@$cam1_topic@g" convert_config.yaml
sed -i "s@<imu_topic>@$imu_topic@g" convert_config.yaml

# run the converter node, assuming the workspace is built
source /opt/ros/humble/setup.bash;
source ./install/setup.bash;
echo "current ros2 workspace is: $PWD"
# ros2 run ros2_bag_exporter bag_exporter --ros-args -p config_file:=$PWD/convert_config.yaml 

# modify the path to files in the folder
# for cam_0 data

# get path to data.csv
csv_path="${cam0_topic}"
while [ "${csv_path: -1}" != "/" ]; do
   csv_path="${csv_path:0:-1}"
done
# echo "${outfld}${csv_path}data.csv"

mkdir -p "${outfld}/mav0/cam0/data"
mv -v ${outfld}${cam0_topic}/*.png ${outfld}/mav0/cam0/data
mv -v ${outfld}${csv_path}data.csv ${outfld}/mav0/cam0

# for cam_1 data

# get path to data.csv
csv_path="${cam1_topic}"
while [ "${csv_path: -1}" != "/" ]; do
   csv_path="${csv_path:0:-1}"
done
# echo "${outfld}${csv_path}data.csv"

mkdir -p "${outfld}/mav0/cam1/data"
mv -v ${outfld}${cam1_topic}/*.png ${outfld}/mav0/cam1/data
mv -v ${outfld}${csv_path}data.csv ${outfld}/mav0/cam1

# for imu if present
if [ -d "${outfld}${imu_topic}/" ]; then
  echo "${outfld}${imu_topic}/ does exist. Process to move the data"
  mv ${outfld}${imu_topic}/ ${outfld}/mav0/imu0
fi

rm -r ${outfld}/vision
