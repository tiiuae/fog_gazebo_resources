#!/usr/bin/env bash

SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`

set -e

ROS2_WORKSPACE=~/tii_workspace

sitl_bin="$ROS2_WORKSPACE/src/px4_firmware/build/px4_sitl_rtps/bin/px4"
debugger="none"
program="gazebo"
model="ssrc_fog_x"
models_path="$SCRIPTPATH/../models"
world="forest.world"
worlds_path="$SCRIPTPATH/../worlds"
src_path="$ROS2_WORKSPACE/src/px4_firmware"
build_path="$ROS2_WORKSPACE/src/px4_firmware/build/px4_sitl_rtps"
verbose="--verbose"

# The rest of the arguments are files to copy into the working dir.

echo SITL ARGS

echo sitl_bin: $sitl_bin
echo debugger: $debugger
echo program: $program
echo model: $model
echo world: $world
echo src_path: $src_path
echo build_path: $build_path

rootfs="$build_path/tmp/rootfs" # this is the working directory
mkdir -p "$rootfs"

# kill process names that might stil
# be running from last time
pkill -x gazebo || true

cp "$src_path/Tools/posix_lldbinit" "$rootfs/.lldbinit"
cp "$src_path/Tools/posix.gdbinit" "$rootfs/.gdbinit"

echo "Starting gazebo"

SIM_PID=0

if [ "$program" == "gazebo" ] && [ ! -n "$no_sim" ]; then
  if [ -x "$(command -v gazebo)" ]; then
    # Get the model name
    model_name="${model}"
    # Check if a 'modelname-gen.sdf' file exist for the models using jinja and generating the SDF files
    # if [ -f "${src_path}/Tools/sitl_gazebo/models/${model}/${model}-gen.sdf" ]; then
    # 	model_name="${model}-gen"
    # fi

    # Set the plugin path so Gazebo finds our model and sim
    source "$src_path/Tools/setup_gazebo.bash" "${src_path}" "${build_path}"

    # #{ MY REMOVALS
    # if [ -z $PX4_SITL_WORLD ]; then
      #Spawn predefined world

      #if [ "$world" == "none" ]; then
      #	if [ -f ${src_path}/Tools/sitl_gazebo/worlds/${model}.world ]; then
      #		echo "empty world, default world ${model}.world for model found"
      #		world_path="${src_path}/Tools/sitl_gazebo/worlds/${model}.world"
      #	else
      #		echo "empty world, setting empty.world as default"
      #		world_path="${src_path}/Tools/sitl_gazebo/worlds/empty.world"
      #	fi
      #else
      #	#Spawn empty world if world with model name doesn't exist
      #	world_path="${src_path}/Tools/sitl_gazebo/worlds/${world}.world"
      #fi
    #else
      #if [ -f ${src_path}/Tools/sitl_gazebo/worlds/${PX4_SITL_WORLD}.world ]; then
      #	# Spawn world by name if exists in the worlds directory from environment variable
      #	world_path="${src_path}/Tools/sitl_gazebo/worlds/${PX4_SITL_WORLD}.world"
      #else
      #	# Spawn world from environment variable with absolute path
      #	world_path="$PX4_SITL_WORLD"
      #fi
    #fi
    # #}

    # #{ MY ADDITIONS
    world_path="$worlds_path/$world"
    export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${models_path}:~/git/simulation/ros_packages/mrs_gazebo_common_resources/models
    # #}

    # gzserver $verbose $world_path &
    ros2 launch gazebo_ros gzserver.launch.py verbose:="true" world:=$world_path &
    SIM_PID=$!

    # #{ MY REMOVALS

    # # Check all paths in ${GAZEBO_MODEL_PATH} for specified model
    # readarray -d : -t paths <<< ${GAZEBO_MODEL_PATH}

    # for possibleModelPath in "${paths[@]}"; do
    # 	# trim \r from path
    # 	possibleModelPath=$(echo $possibleModelPath | tr -d '\r')
    # 	if test -f "${possibleModelPath}/${model}/${model}.sdf" ; then
    # 		modelpath=$possibleModelPath
    # 		break
    # 	fi
    # done
    # #}

    # #{ MY ADDITIONS
    modelpath=$models_path
    # #}

    echo "Using: ${modelpath}/${model}/model.sdf"

    while gz model --verbose --spawn-file="${modelpath}/${model}/model.sdf" --model-name=${model} -x 1.01 -y 0.98 -z 0.83 2>&1 | grep -q "An instance of Gazebo is not running."; do
      echo "gzserver not ready yet, trying again!"
      sleep 1
    done

    if [[ -n "$HEADLESS" ]]; then
      echo "not running gazebo gui"
    else
      # gzserver needs to be running to avoid a race. Since the launch
      # is putting it into the background we need to avoid it by backing off
      sleep 3
      nice -n 20 gzclient --verbose &
      GUI_PID=$!
    fi
  else
    echo "You need to have gazebo simulator installed!"
    exit 1
  fi
fi

pushd "$rootfs" >/dev/null

# Do not exit on failure now from here on because we want the complete cleanup
set +e

if [[ ${model} == test_* ]] || [[ ${model} == *_generated ]]; then
  sitl_command="\"$sitl_bin\" $no_pxh \"$src_path\"/ROMFS/px4fmu_test -s \"${src_path}\"/posix-configs/SITL/init/test/${model} -t \"$src_path\"/test_data"
else
  sitl_command="\"$sitl_bin\" $no_pxh \"$build_path\"/etc -s etc/init.d-posix/rcS -t \"$src_path\"/test_data"
fi

echo SITL COMMAND: $sitl_command

export PX4_SIM_MODEL=${model}

eval $sitl_command

popd >/dev/null

if [ "$program" == "gazebo" ]; then
  kill -9 $SIM_PID
  if [[ ! -n "$HEADLESS" ]]; then
    kill -9 $GUI_PID
  fi
fi
