#!/bin/bash
#
# WeatherKit Build Script
# A comprehensive build, flash, and monitor tool for the WeatherKit project
#
# Usage:
#   ./build.sh [command] [options]
#
# Commands:
#   build       Build project(s)
#   flash       Flash project to device
#   monitor     Monitor serial output
#   clean       Clean build artifacts
#   fullclean   Full clean (remove build directory)
#   menuconfig  Open ESP-IDF menuconfig
#   all         Build all projects
#
# Options:
#   -p, --project    Project to build: sensor, base, or all (default: all)
#   -d, --device     Serial device (default: /dev/ttyUSB0)
#   -b, --baud       Baud rate for flashing (default: 460800)
#   -h, --help       Show this help message
#

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Default values
PROJECT="all"
DEVICE="/dev/ttyUSB0"
BAUD="460800"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Project directories
SENSOR_DIR="$SCRIPT_DIR/sensorPackage"
BASE_DIR="$SCRIPT_DIR/baseStation"

# Print colored message
print_msg() {
    local color=$1
    local msg=$2
    echo -e "${color}${msg}${NC}"
}

print_header() {
    echo ""
    print_msg "$CYAN" "╔════════════════════════════════════════════════════════════╗"
    print_msg "$CYAN" "║                    WeatherKit Build System                  ║"
    print_msg "$CYAN" "╚════════════════════════════════════════════════════════════╝"
    echo ""
}

print_section() {
    echo ""
    print_msg "$BLUE" "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    print_msg "$BLUE" "  $1"
    print_msg "$BLUE" "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
}

# Show help message
show_help() {
    print_header
    cat << EOF
Usage: ./build.sh [command] [options]

Commands:
  build         Build project(s)
  flash         Flash project to device
  monitor       Monitor serial output
  flash-monitor Flash and then monitor
  clean         Clean build artifacts
  fullclean     Full clean (remove build directory)
  menuconfig    Open ESP-IDF menuconfig
  all           Build all projects

Options:
  -p, --project <name>   Project: sensor, base, or all (default: all)
  -d, --device <path>    Serial device (default: /dev/ttyUSB0)
  -b, --baud <rate>      Baud rate for flashing (default: 460800)
  -h, --help             Show this help message

Examples:
  ./build.sh build                      # Build all projects
  ./build.sh build -p sensor            # Build sensor package only
  ./build.sh build -p base              # Build base station only
  ./build.sh flash -p sensor            # Flash sensor package
  ./build.sh flash-monitor -p base      # Flash and monitor base station
  ./build.sh monitor -p sensor -d /dev/ttyACM0  # Monitor sensor on specific port
  ./build.sh clean -p all               # Clean all projects
  ./build.sh menuconfig -p sensor       # Open menuconfig for sensor package

EOF
}

# Check if ESP-IDF is set up
check_idf() {
    if [ -z "$IDF_PATH" ]; then
        print_msg "$YELLOW" "⚠ ESP-IDF environment not detected. Attempting to source..."
        
        # Try common ESP-IDF locations
        if [ -f "$HOME/esp/v5.5.1/esp-idf/export.sh" ]; then
            source "$HOME/esp/v5.5.1/esp-idf/export.sh"
        elif [ -f "$HOME/esp/esp-idf/export.sh" ]; then
            source "$HOME/esp/esp-idf/export.sh"
        elif [ -f "/opt/esp-idf/export.sh" ]; then
            source "/opt/esp-idf/export.sh"
        elif [ -f "$HOME/.espressif/esp-idf/export.sh" ]; then
            source "$HOME/.espressif/esp-idf/export.sh"
        else
            print_msg "$RED" "✗ Could not find ESP-IDF. Please run: source \$IDF_PATH/export.sh"
            exit 1
        fi
    fi
    print_msg "$GREEN" "✓ ESP-IDF environment ready ($(idf.py --version 2>/dev/null || echo 'version unknown'))"
}

# Get project directory
get_project_dir() {
    local proj=$1
    case $proj in
        sensor|sensorPackage)
            echo "$SENSOR_DIR"
            ;;
        base|baseStation)
            echo "$BASE_DIR"
            ;;
        *)
            print_msg "$RED" "✗ Unknown project: $proj"
            print_msg "$YELLOW" "  Valid options: sensor, base"
            exit 1
            ;;
    esac
}

# Get project display name
get_project_name() {
    local proj=$1
    case $proj in
        sensor|sensorPackage)
            echo "Sensor Package"
            ;;
        base|baseStation)
            echo "Base Station"
            ;;
        *)
            echo "$proj"
            ;;
    esac
}

# Build a single project
build_project() {
    local proj=$1
    local proj_dir=$(get_project_dir "$proj")
    local proj_name=$(get_project_name "$proj")
    
    print_section "Building $proj_name"
    
    if [ ! -d "$proj_dir" ]; then
        print_msg "$RED" "✗ Project directory not found: $proj_dir"
        exit 1
    fi
    
    cd "$proj_dir"
    print_msg "$CYAN" "→ Working directory: $proj_dir"
    
    if idf.py build; then
        print_msg "$GREEN" "✓ $proj_name build successful!"
        return 0
    else
        print_msg "$RED" "✗ $proj_name build failed!"
        return 1
    fi
}

# Flash a single project
flash_project() {
    local proj=$1
    local proj_dir=$(get_project_dir "$proj")
    local proj_name=$(get_project_name "$proj")
    
    print_section "Flashing $proj_name"
    
    if [ ! -d "$proj_dir" ]; then
        print_msg "$RED" "✗ Project directory not found: $proj_dir"
        exit 1
    fi
    
    cd "$proj_dir"
    print_msg "$CYAN" "→ Device: $DEVICE"
    print_msg "$CYAN" "→ Baud rate: $BAUD"
    
    if idf.py -p "$DEVICE" -b "$BAUD" flash; then
        print_msg "$GREEN" "✓ $proj_name flashed successfully!"
        return 0
    else
        print_msg "$RED" "✗ $proj_name flash failed!"
        return 1
    fi
}

# Monitor a project
monitor_project() {
    local proj=$1
    local proj_dir=$(get_project_dir "$proj")
    local proj_name=$(get_project_name "$proj")
    
    print_section "Monitoring $proj_name"
    
    if [ ! -d "$proj_dir" ]; then
        print_msg "$RED" "✗ Project directory not found: $proj_dir"
        exit 1
    fi
    
    cd "$proj_dir"
    print_msg "$CYAN" "→ Device: $DEVICE"
    print_msg "$YELLOW" "Press Ctrl+] to exit monitor"
    echo ""
    
    idf.py -p "$DEVICE" monitor
}

# Flash and monitor a project
flash_monitor_project() {
    local proj=$1
    local proj_dir=$(get_project_dir "$proj")
    local proj_name=$(get_project_name "$proj")
    
    print_section "Flash & Monitor $proj_name"
    
    if [ ! -d "$proj_dir" ]; then
        print_msg "$RED" "✗ Project directory not found: $proj_dir"
        exit 1
    fi
    
    cd "$proj_dir"
    print_msg "$CYAN" "→ Device: $DEVICE"
    print_msg "$CYAN" "→ Baud rate: $BAUD"
    print_msg "$YELLOW" "Press Ctrl+] to exit monitor"
    echo ""
    
    idf.py -p "$DEVICE" -b "$BAUD" flash monitor
}

# Clean a project
clean_project() {
    local proj=$1
    local proj_dir=$(get_project_dir "$proj")
    local proj_name=$(get_project_name "$proj")
    
    print_section "Cleaning $proj_name"
    
    if [ ! -d "$proj_dir" ]; then
        print_msg "$YELLOW" "⚠ Project directory not found: $proj_dir"
        return 0
    fi
    
    cd "$proj_dir"
    
    if idf.py clean; then
        print_msg "$GREEN" "✓ $proj_name cleaned!"
        return 0
    else
        print_msg "$RED" "✗ $proj_name clean failed!"
        return 1
    fi
}

# Full clean a project
fullclean_project() {
    local proj=$1
    local proj_dir=$(get_project_dir "$proj")
    local proj_name=$(get_project_name "$proj")
    
    print_section "Full Clean $proj_name"
    
    if [ ! -d "$proj_dir" ]; then
        print_msg "$YELLOW" "⚠ Project directory not found: $proj_dir"
        return 0
    fi
    
    cd "$proj_dir"
    
    if idf.py fullclean; then
        print_msg "$GREEN" "✓ $proj_name fully cleaned!"
        return 0
    else
        # fullclean might fail if build dir doesn't exist, that's ok
        if [ -d "build" ]; then
            rm -rf build
            print_msg "$GREEN" "✓ $proj_name build directory removed!"
        else
            print_msg "$GREEN" "✓ $proj_name already clean!"
        fi
        return 0
    fi
}

# Open menuconfig for a project
menuconfig_project() {
    local proj=$1
    local proj_dir=$(get_project_dir "$proj")
    local proj_name=$(get_project_name "$proj")
    
    print_section "Menuconfig: $proj_name"
    
    if [ ! -d "$proj_dir" ]; then
        print_msg "$RED" "✗ Project directory not found: $proj_dir"
        exit 1
    fi
    
    cd "$proj_dir"
    idf.py menuconfig
}

# Build all projects
build_all() {
    print_section "Building All Projects"
    
    local failed=0
    
    if ! build_project "sensor"; then
        failed=1
    fi
    
    if ! build_project "base"; then
        failed=1
    fi
    
    echo ""
    if [ $failed -eq 0 ]; then
        print_msg "$GREEN" "╔════════════════════════════════════════════════════════════╗"
        print_msg "$GREEN" "║              ✓ All projects built successfully!            ║"
        print_msg "$GREEN" "╚════════════════════════════════════════════════════════════╝"
    else
        print_msg "$RED" "╔════════════════════════════════════════════════════════════╗"
        print_msg "$RED" "║              ✗ Some projects failed to build               ║"
        print_msg "$RED" "╚════════════════════════════════════════════════════════════╝"
        exit 1
    fi
}

# Clean all projects
clean_all() {
    print_section "Cleaning All Projects"
    
    clean_project "sensor"
    clean_project "base"
    
    print_msg "$GREEN" "✓ All projects cleaned!"
}

# Full clean all projects
fullclean_all() {
    print_section "Full Clean All Projects"
    
    fullclean_project "sensor"
    fullclean_project "base"
    
    print_msg "$GREEN" "✓ All projects fully cleaned!"
}

# Parse command line arguments
parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            -p|--project)
                PROJECT="$2"
                shift 2
                ;;
            -d|--device)
                DEVICE="$2"
                shift 2
                ;;
            -b|--baud)
                BAUD="$2"
                shift 2
                ;;
            -h|--help)
                show_help
                exit 0
                ;;
            build|flash|monitor|flash-monitor|clean|fullclean|menuconfig|all)
                COMMAND="$1"
                shift
                ;;
            *)
                print_msg "$RED" "Unknown option: $1"
                show_help
                exit 1
                ;;
        esac
    done
}

# Main entry point
main() {
    print_header
    
    if [ $# -eq 0 ]; then
        show_help
        exit 0
    fi
    
    parse_args "$@"
    
    # Check for ESP-IDF (except for help)
    if [ "$COMMAND" != "" ]; then
        check_idf
    fi
    
    case $COMMAND in
        build)
            if [ "$PROJECT" = "all" ]; then
                build_all
            else
                build_project "$PROJECT"
            fi
            ;;
        flash)
            if [ "$PROJECT" = "all" ]; then
                print_msg "$RED" "✗ Cannot flash 'all' - please specify a single project"
                print_msg "$YELLOW" "  Use: ./build.sh flash -p sensor"
                print_msg "$YELLOW" "   or: ./build.sh flash -p base"
                exit 1
            fi
            flash_project "$PROJECT"
            ;;
        monitor)
            if [ "$PROJECT" = "all" ]; then
                print_msg "$RED" "✗ Cannot monitor 'all' - please specify a single project"
                exit 1
            fi
            monitor_project "$PROJECT"
            ;;
        flash-monitor)
            if [ "$PROJECT" = "all" ]; then
                print_msg "$RED" "✗ Cannot flash-monitor 'all' - please specify a single project"
                exit 1
            fi
            flash_monitor_project "$PROJECT"
            ;;
        clean)
            if [ "$PROJECT" = "all" ]; then
                clean_all
            else
                clean_project "$PROJECT"
            fi
            ;;
        fullclean)
            if [ "$PROJECT" = "all" ]; then
                fullclean_all
            else
                fullclean_project "$PROJECT"
            fi
            ;;
        menuconfig)
            if [ "$PROJECT" = "all" ]; then
                print_msg "$RED" "✗ Cannot run menuconfig for 'all' - please specify a single project"
                exit 1
            fi
            menuconfig_project "$PROJECT"
            ;;
        all)
            build_all
            ;;
        *)
            print_msg "$RED" "✗ Unknown command: $COMMAND"
            show_help
            exit 1
            ;;
    esac
}

main "$@"
