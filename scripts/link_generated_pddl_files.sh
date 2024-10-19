#!/bin/bash

# This script is used to link the generated pddl files to the pyrobosim repository for easier access
# Author: Matthias Mayr
# Date: 2024-10-19

# Get the directory where the script is located
SCRIPT_DIR="$(dirname "$(realpath "$0")")"

# Set the target files in the user's home directory
PLANNER_DIR="$HOME/.skiros/planner"
DOMAIN="$PLANNER_DIR/domain.pddl"
PROBLEM="$PLANNER_DIR/p01.pddl"
touch "$DOMAIN"
touch "$PROBLEM"

# Set the destination directory relative to the script's location
DEST_DIR="$SCRIPT_DIR/../planning"

# Create the destination directory if it doesn't exist
mkdir -p "$DEST_DIR"

# Create symbolic links if they do not exist
if [ ! -f "$DEST_DIR/generated_domain.pddl" ]; then
    ln -s "$DOMAIN" "$DEST_DIR/generated_domain.pddl"
fi
if [ ! -f "$DEST_DIR/generated_problem.pddl" ]; then
    ln -s "$PROBLEM" "$DEST_DIR/generated_problem.pddl"
fi
echo "PDDL files have been linked from '$PLANNER_DIR' to '$DEST_DIR'"
