#!/usr/bin/env bash

SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`

set -e

px4_firmware_path="$1"
program="gazebo"
model="ssrc_fog_x"
models_path="$SCRIPTPATH/../share/fog_gazebo_resources/models"
world="forest.world"
worlds_path="$SCRIPTPATH/../worlds"
verbose="--verbose"

src_path="$px4_firmware_path"
sitl_bin="$px4_firmware_path/build/px4_sitl_rtps/bin/px4"
build_path="$px4_firmware_path/build/px4_sitl_rtps"

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

modelpath=$models_path

echo "Using: ${modelpath}/${model}/model.sdf"

while gz model --verbose --spawn-file="${modelpath}/${model}/model.sdf" --model-name=${model} -x 1.01 -y 0.98 -z 0.83 2>&1 | grep -q "An instance of Gazebo is not running."; do
  echo "gzserver not ready yet, trying again!"
  sleep 1
done

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
