#!/bin/bash

# Define the source (where your configs are in the repo) 
# and the target (where libbarrett expects them)
REPO_CONFIG_DIR="/home/user/haptic-imitation/barrett_cfgs"
TARGET_DIR="$HOME/.barrett"

TARGET_DIR="$HOME/.barrett"
FLAG_FILE="$HOME/.barrett_setup_complete"

# 2. Skip if already done
if [ -f "$FLAG_FILE" ]; then
    exit 0
fi

echo "--- Barrett Configuration Setup ---"
read -p "Link individual config files to $TARGET_DIR? (y/n): " -n 1 -r
echo

# Remove existing file, folder, or symlink at the target location
if [ -e "$TARGET_DIR" ] || [ -L "$TARGET_DIR" ]; then
    echo "Removing existing $TARGET_DIR..."
    rm -rf "$TARGET_DIR"
fi

# Create the folder-level symlink
ln -s "$REPO_CONFIG_DIR" "$TARGET_DIR"

echo "Linked: $TARGET_DIR -> $REPO_CONFIG_DIR"