#!/bin/bash

# RaspRover IDF Build Script
# This script helps build and flash the RaspRover project

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if ESP-IDF is available
check_idf() {
    if ! command -v idf.py &> /dev/null; then
        print_error "ESP-IDF not found in PATH"
        print_status "Please install ESP-IDF or source the environment:"
        print_status "source \$HOME/esp/esp-idf/export.sh"
        exit 1
    fi
    
    print_success "ESP-IDF found: $(idf.py --version)"
}

# Function to clean build
clean_build() {
    print_status "Cleaning build directory..."
    idf.py fullclean
    print_success "Build directory cleaned"
}

# Function to build project
build_project() {
    print_status "Building project..."
    idf.py build
    
    if [ $? -eq 0 ]; then
        print_success "Project built successfully"
    else
        print_error "Build failed"
        exit 1
    fi
}

# Function to flash project
flash_project() {
    print_status "Flashing project to ESP32..."
    
    # Check if port is specified
    if [ -n "$1" ]; then
        idf.py -p "$1" flash
    else
        idf.py flash
    fi
    
    if [ $? -eq 0 ]; then
        print_success "Project flashed successfully"
    else
        print_error "Flash failed"
        exit 1
    fi
}

# Function to monitor serial output
monitor_serial() {
    print_status "Starting serial monitor..."
    
    # Check if port is specified
    if [ -n "$1" ]; then
        idf.py -p "$1" monitor
    else
        idf.py monitor
    fi
}

# Function to show help
show_help() {
    echo "RaspRover IDF Build Script"
    echo ""
    echo "Usage: $0 [OPTIONS] [COMMAND]"
    echo ""
    echo "Commands:"
    echo "  build           Build the project"
    echo "  clean           Clean build directory"
    echo "  flash [PORT]    Flash project to ESP32 (optional port)"
    echo "  monitor [PORT]  Monitor serial output (optional port)"
    echo "  all [PORT]      Build, flash, and monitor (optional port)"
    echo ""
    echo "Options:"
    echo "  -h, --help      Show this help message"
    echo "  -v, --version   Show version information"
    echo ""
    echo "Examples:"
    echo "  $0 build                    # Build project"
    echo "  $0 flash /dev/ttyUSB0      # Flash to specific port"
    echo "  $0 all /dev/ttyUSB0        # Build, flash, and monitor"
    echo ""
}

# Function to show version
show_version() {
    echo "RaspRover IDF Build Script v1.0.0"
    echo "ESP-IDF Project Build Helper"
}

# Main script logic
main() {
    # Check if ESP-IDF is available
    check_idf
    
    # Parse command line arguments
    case "${1:-}" in
        "build")
            build_project
            ;;
        "clean")
            clean_build
            ;;
        "flash")
            flash_project "$2"
            ;;
        "monitor")
            monitor_serial "$2"
            ;;
        "all")
            build_project
            flash_project "$2"
            monitor_serial "$2"
            ;;
        "-h"|"--help")
            show_help
            ;;
        "-v"|"--version")
            show_version
            ;;
        "")
            print_warning "No command specified"
            show_help
            exit 1
            ;;
        *)
            print_error "Unknown command: $1"
            show_help
            exit 1
            ;;
    esac
}

# Run main function with all arguments
main "$@"
