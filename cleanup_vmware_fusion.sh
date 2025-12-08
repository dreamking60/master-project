#!/bin/bash

# VMware Fusion Complete Uninstall Script
# Based on: https://knowledge.broadcom.com/external/article?articleNumber=307074
# This script removes all VMware Fusion files and folders

set -e  # Exit on error

echo "========================================="
echo "VMware Fusion Complete Uninstall Script"
echo "========================================="
echo ""
echo "This script will remove:"
echo "1. VMware Fusion application from /Applications"
echo "2. All VMware Fusion support files and preferences"
echo ""
read -p "Do you want to continue? (y/N): " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Cancelled."
    exit 1
fi

echo ""
echo "Starting cleanup..."
echo ""

# 1. Remove VMware Fusion application bundle
if [ -d "/Applications/VMware Fusion.app" ]; then
    echo "Removing VMware Fusion application..."
    sudo rm -rf "/Applications/VMware Fusion.app"
    echo "✓ Application removed"
else
    echo "⚠ VMware Fusion.app not found in /Applications"
fi

# 2. Remove system-level support files
echo ""
echo "Removing system-level support files..."

if [ -d "/Library/Application Support/VMware/VMware Fusion" ]; then
    echo "Removing /Library/Application Support/VMware/VMware Fusion..."
    sudo rm -rf "/Library/Application Support/VMware/VMware Fusion"
    echo "✓ Removed"
fi

# Note: Usbarb.rules - only remove if not using other VMware products
read -p "Are you using any other VMware products? (y/N): " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    if [ -f "/Library/Application Support/VMware/Usbarb.rules" ]; then
        echo "Removing Usbarb.rules..."
        sudo rm -f "/Library/Application Support/VMware/Usbarb.rules"
        echo "✓ Removed"
    fi
else
    echo "⚠ Keeping Usbarb.rules (other VMware products detected)"
fi

if [ -d "/Library/Application Support/VMware Fusion" ]; then
    echo "Removing /Library/Application Support/VMware Fusion..."
    sudo rm -rf "/Library/Application Support/VMware Fusion"
    echo "✓ Removed"
fi

if [ -d "/Library/Preferences/VMware Fusion" ]; then
    echo "Removing /Library/Preferences/VMware Fusion..."
    sudo rm -rf "/Library/Preferences/VMware Fusion"
    echo "✓ Removed"
fi

# 3. Remove user-level support files
echo ""
echo "Removing user-level support files..."

USER_HOME="$HOME"

if [ -d "$USER_HOME/Library/Application Support/VMware Fusion" ]; then
    echo "Removing ~/Library/Application Support/VMware Fusion..."
    rm -rf "$USER_HOME/Library/Application Support/VMware Fusion"
    echo "✓ Removed"
fi

if [ -d "$USER_HOME/Library/Caches/com.vmware.fusion" ]; then
    echo "Removing ~/Library/Caches/com.vmware.fusion..."
    rm -rf "$USER_HOME/Library/Caches/com.vmware.fusion"
    echo "✓ Removed"
fi

if [ -d "$USER_HOME/Library/Preferences/VMware Fusion" ]; then
    echo "Removing ~/Library/Preferences/VMware Fusion..."
    rm -rf "$USER_HOME/Library/Preferences/VMware Fusion"
    echo "✓ Removed"
fi

# Remove preference plist files
PREF_FILES=(
    "$USER_HOME/Library/Preferences/com.vmware.fusion.LSSharedFileList.plist"
    "$USER_HOME/Library/Preferences/com.vmware.fusion.LSSharedFileList.plist.lockfile"
    "$USER_HOME/Library/Preferences/com.vmware.fusion.plist"
    "$USER_HOME/Library/Preferences/com.vmware.fusion.plist.lockfile"
    "$USER_HOME/Library/Preferences/com.vmware.fusionDaemon.plist"
    "$USER_HOME/Library/Preferences/com.vmware.fusionDaemon.plist.lockfile"
    "$USER_HOME/Library/Preferences/com.vmware.fusionStartMenu.plist"
    "$USER_HOME/Library/Preferences/com.vmware.fusionStartMenu.plist.lockfile"
)

echo ""
echo "Removing preference files..."
for file in "${PREF_FILES[@]}"; do
    if [ -f "$file" ] || [ -d "$file" ]; then
        echo "Removing $(basename "$file")..."
        rm -rf "$file"
        echo "✓ Removed"
    fi
done

echo ""
echo "========================================="
echo "Cleanup completed!"
echo "========================================="
echo ""
echo "Note: Virtual machines are NOT deleted by this script."
echo "If you want to remove virtual machines, they are typically located in:"
echo "  ~/Documents/Virtual Machines/"
echo ""
echo "You can manually delete them using Finder if needed."
echo ""

