#!/bin/bash

TARGET_DIR="$HOME/.barrett"
FLAG_FILE="$HOME/.barrett_setup_complete"

echo "--- Barrett Configuration Unlink ---"

if [ -L "$TARGET_DIR" ]; then
    echo "Removing symlink: $TARGET_DIR"
    rm "$TARGET_DIR"
    rm -f "$FLAG_FILE"
    echo "Unlinked successfully. Your files remain safe in the repository."
else
    echo "No symlink found at $TARGET_DIR (or it is a real directory)."
fi