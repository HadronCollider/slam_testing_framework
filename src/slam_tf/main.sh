#!/bin/bash

if [ $1 = 'help' ];
then
echo "USAGE: main.sh [command]
  help
    print this message
  
  full <dataset_type> <path_to_bag_file> <path_to_gt>
    run all SLAM's on bag-file
      <dataset_type> = 'mit', 'frog', 'tum'

  run <slam> <dataset_type> <path_to_bag_file> <path_to_gt>
    run <slam> on bag-file and test by <path_to_gt>
      <slam> = 'tiny', 'viny', 'sc_gmap', 'gmap', 'hector'
      <dataset_type> = 'mit', 'frog', 'tum'
  light_bag <path_to_bag_file> <scan_topic>
    lighten bag-files (filter '/tf' and '/<scan_topic>')
      <topic_scan> - 'base_scan' for MIT
                   - 'scan' for TUM
                   - 'scanfront' for FROG
    
  prepare_gt <dataset> <path_to_gt> <path_to_save>
    prepare gt to GT-file format
      <dataset> = 'mit', 'frog'
    - path_to_save - optional, default: path_to_gt+'.gt'
	
GT-files format:
	timestrap x y z q1 q2 q3 q4 q5
" 
exit 0 
fi

if !([ $1 = 'full' ] || [ $1 = 'run' ] || [ $1 = 'light_bag' ] || [ $1 = 'prepare_gt' ]); then
    echo "Unexpected command. Use 'main.sh help' for more info"
    exit 0
fi


if [ $1 = 'light_bag' ]; then
    # light_bag <path_to_bag_file> <scan_topic> <optional:path_to_save>
    if [ -n "$2" ] && [ -n "$3" ]; then
        path_to_save="$2.light"
        if [ -n "$4" ]; then
            path_to_save="$4"
        fi
        exp="topic=='/tf' or topic=='/$3'"
        echo rosbag filter $2 $path_to_save \"$exp\"\ > light_bag.sh
        sh light_bag.sh
        echo "light_bag finished. New bag-file: $path_to_save"
    else
        echo "Not enough args. Usage: light_bag <path_to_bag_file> <scan_topic> <optional:path_to_save>"
    fi
    exit 0
fi
pkg_path=$(rospack find slam_tf)
if [ $1 = 'prepare_gt' ]; then
    # prepare_gt <dataset> <path_to_gt> <path_to_save>
    if [ -n "$2" ] && [ -n "$3" ]; then
        python $pkg_path/src/prepare_gt.py $2 $3 $4
        echo "prepare_gt finished."
    else
        echo "Not enough args. Usage: prepare_gt <dataset> <path_to_gt> <optional:path_to_save>"
    fi
    exit 0
fi


dataset_type=""
path_to_bag=""
path_to_truth=""

if [ $1 = 'run' ] && ! ([ -n "$2" ] && [ -n "$3" ] && [ -n "$4" ] && [ -n "$5" ]); then
    echo "Not enough args. Usage: run <slam> <dataset_type> <path_to_bag_file> <path_to_gt>"
    exit 0
else
    dataset_type=$3
    path_to_bag=$4
    path_to_truth=$5
fi


if [ $1 = 'full' ]; then
    if ! ( [ -n "$2" ] && [ -n "$3" ] && [ -n "$4" ]); then
        echo "Not enough args. Usage: full <dataset_type> <path_to_bag_file> <path_to_gt>"
        exit 0
    else
        dataset_type=$2
        path_to_bag=$3
        path_to_truth=$4
    fi
fi

params=""
if [ $dataset_type = 'mit' ]; then
    params="odom_frame:=odom_comdined remap_scan:=base_scan"
elif [ $dataset_type = 'frog' ]; then
    params="odom_frame:=odom remap_scan:=scanfront"
elif [ $dataset_type = 'tum' ]; then
    params="odom_frame:=odom remap_scan:=scan"
else
    echo "Unexpected dataset_type=$dataset_type. Use mit, frog or tum"
    exit 0
fi

    
if [ $1 = 'full' ] || [ $2 = 'tiny' ]; then
    echo "Run tinySlam..."
    tiny_path="$pkg_path/tmp/tiny.map"
    roslaunch slam_tf tiny.launch path:=$path_to_bag out_map:=$tiny_path $params >/dev/null 2>/dev/null
    echo "tinySlam finished."
    min_res=1000.0
    cd $pkg_path/src
    for j in `seq -5 5`; do
        res=$(./evaluate_ate.py --max_difference 1 --save "$pkg_path/tmp/diff.txt" --offset "$j" "$path_to_truth" "$tiny_path")
        if (( $(echo "$res<$min_res" | bc -l) )); then
            offset=$j
            min_res=$res
        else
            break
        fi
    done
    echo 'tiny RMSE =' $min_res
fi

if [ $1 = 'full' ] || [ $2 = 'viny' ]; then
    echo "Run vinySlam..."
    viny_path="$pkg_path/tmp/viny.map"
    roslaunch slam_tf viny.launch path:=$path_to_bag out_map:=$viny_path $params >/dev/null 2>/dev/null
    echo "vinySlam finished."
    min_res=1000.0
    cd $pkg_path/src
    for j in `seq -5 5`; do
        res=$(./evaluate_ate.py --max_difference 1 --save "$pkg_path/tmp/diff.txt" --offset "$j" "$path_to_truth" "$viny_path")
        if (( $(echo "$res<$min_res" | bc -l) )); then
            offset=$j
            min_res=$res
        else
            break
        fi
    done
    echo 'viny RMSE =' $min_res
fi

if [ $1 = 'full' ] || [ $2 = 'hector' ]; then
    echo "Run hectorSlam..."
    hector_path="$pkg_path/tmp/hector.map"
    roslaunch slam_tf hector.launch path:=$path_to_bag out_map:=$hector_path $params >/dev/null 2>/dev/null
    echo "hectorSlam finished."
    min_res=1000.0
    cd $pkg_path/src
    for j in `seq -5 5`; do
        res=$(./evaluate_ate.py --max_difference 1 --save "$pkg_path/tmp/diff.txt" --offset "$j" "$path_to_truth" "$hector_path")
        if (( $(echo "$res<$min_res" | bc -l) )); then
            offset=$j
            min_res=$res
        else
            break
        fi
    done
    echo 'hector RMSE =' $min_res
fi

if [ $1 = 'full' ] || [ $2 = 'sc_gmap' ]; then
    echo "Run sc_gmapSlam..."
    sc_gmap_path="$pkg_path/tmp/sc_gmap.map"
    roslaunch slam_tf sc_gmap.launch path:=$path_to_bag out_map:=$sc_gmap_path $params >/dev/null 2>/dev/null
    echo "sc_gmapSlam finished."
    min_res=1000.0
    cd $pkg_path/src
    for j in `seq -5 5`; do
        res=$(./evaluate_ate.py --max_difference 1 --save "$pkg_path/tmp/diff.txt" --offset "$j" "$path_to_truth" "$sc_gmap_path")
        if (( $(echo "$res<$min_res" | bc -l) )); then
            offset=$j
            min_res=$res
        else
            break
        fi
    done
    echo 'sc_gmap RMSE =' $min_res
fi

if [ $1 = 'full' ] || [ $2 = 'gmap' ]; then
    echo "Run gmapSlam..."
    gmap_path="$pkg_path/tmp/gmap.map"
    roslaunch slam_tf gmap.launch path:=$path_to_bag out_map:=$gmap_path $params >/dev/null 2>/dev/null
    echo "gmapSlam finished."
    min_res=1000.0
    cd $pkg_path/src
    for j in `seq -5 5`; do
        res=$(./evaluate_ate.py --max_difference 1 --save "$pkg_path/tmp/diff.txt" --offset "$j" "$path_to_truth" "$gmap_path")
        if (( $(echo "$res<$min_res" | bc -l) )); then
            offset=$j
            min_res=$res
        else
            break
        fi
    done
    echo 'gmap RMSE =' $min_res
fi