#!/bin/bash
args="$@"
while (( "$#" )); do
  shift
done
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "Launching bitbake $args"
cd $DIR/../poky
. ./oe-init-build-env
bitbake $args  | sed -u 's@| @@'
exit 0
