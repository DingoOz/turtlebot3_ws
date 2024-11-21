#!/bin/bash

# Print commands and their arguments as they are executed
set -x

# Script to fix trailing whitespace and ensure newline at end of files
echo "Starting formatting fixes..."

# Directory containing the files
DIR="src/turtlebot3_webcam/src"

# Files to fix
FILES=("$DIR/camera_publisher.cpp" "$DIR/camera_viewer.cpp")

# Create backups first
echo "Creating backups..."
for file in "${FILES[@]}"; do
    if [ -f "$file" ]; then
        cp "$file" "${file}.bak"
        echo "✓ Created backup: ${file}.bak"
    else
        echo "⚠️ Warning: $file not found"
        continue
    fi
done

# Fix the files
echo "Fixing formatting..."
for file in "${FILES[@]}"; do
    if [ -f "$file" ]; then
        # Remove trailing whitespace and ensure single newline at end
        sed -i 's/[[:space:]]*$//' "$file"
        printf '%s\n' "$(cat "$file")" > "$file"
        echo "✓ Fixed formatting in $file"
    fi
done

# Verify the fixes
echo -e "\nVerifying fixes..."
for file in "${FILES[@]}"; do
    if [ -f "$file" ]; then
        echo "Checking $file..."
        # Check for trailing whitespace
        if grep -q "[[:space:]]$" "$file"; then
            echo "⚠️ Warning: Still has trailing whitespace"
        else
            echo "✓ No trailing whitespace"
        fi
        # Check for newline at end of file
        if [ -n "$(tail -c1 "$file")" ]; then
            echo "⚠️ Warning: No newline at end of file"
        else
            echo "✓ Has newline at end of file"
        fi
    fi
done

echo -e "\nAll fixes applied. You can now:"
echo "1. Review the changes"
echo "2. Run: git add $DIR/camera_publisher.cpp $DIR/camera_viewer.cpp"
echo "3. Run: git commit -m \"fixed formatting\""

# Turn off command printing
set +x
