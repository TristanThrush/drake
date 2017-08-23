#!/bin/bash

set -e

# If we are outside the sandbox, then change to the same relative directory as
# we would be inside the sandbox.
if ! [ -d "external/drake_visualizer" ]; then
    guess_runfiles=$(dirname "$0")/drake_visualizer.runfiles/drake
    if [ -d "$guess_runfiles/external/drake_visualizer" ]; then
        cd "$guess_runfiles"
    else
        echo "$(basename $0) error: could not find drake_visualizer" 1>&2
        exit 1
    fi
fi

export PYTHONPATH="drake/bindings/python:drake/lcmtypes:external:external/drake_visualizer/lib/python2.7/dist-packages:external/lcmtypes_bot2_core/lcmtypes:external/lcmtypes_robotlocomotion/lcmtypes${PYTHONPATH:+:$PYTHONPATH}"

exec "external/drake_visualizer/bin/drake-visualizer" "$@"
