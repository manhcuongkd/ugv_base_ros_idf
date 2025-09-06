#!/bin/bash

# RaspRover IDF Quick Setup Script
# This script sets up the entire project environment
# Supports both ESP-IDF v4.4 and v5.0+

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default ESP-IDF version (can be overridden with IDF_VERSION environment variable)
DEFAULT_IDF_VERSION="v4.4.5"
IDF_VERSION=${IDF_VERSION:-$DEFAULT_IDF_VERSION}

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

# Function to check ESP-IDF installation
check_idf_installation() {
    print_status "Checking ESP-IDF installation..."
    print_status "Target ESP-IDF version: $IDF_VERSION"
    
    if ! command -v idf.py &> /dev/null; then
        print_error "ESP-IDF not found in PATH"
        print_status "Installing ESP-IDF $IDF_VERSION..."
        
        # Check if we're on Ubuntu/Debian
        if command -v apt-get &> /dev/null; then
            install_idf_ubuntu
        else
            print_error "Please install ESP-IDF manually from:"
            print_status "https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/"
            exit 1
        fi
    else
        current_version=$(idf.py --version 2>/dev/null || echo "unknown")
        print_success "ESP-IDF found: $current_version"
        
        # Check if we need to switch versions
        if [[ "$current_version" != *"$IDF_VERSION"* ]]; then
            print_warning "Current ESP-IDF version ($current_version) doesn't match target ($IDF_VERSION)"
            print_status "Switching to ESP-IDF $IDF_VERSION..."
            install_idf_ubuntu
        fi
    fi
}

# Function to install ESP-IDF on Ubuntu/Debian
install_idf_ubuntu() {
    print_status "Installing ESP-IDF $IDF_VERSION on Ubuntu/Debian..."
    
    # Install prerequisites
    sudo apt-get update
    sudo apt-get install -y \
        git \
        wget \
        flex \
        bison \
        gperf \
        python3 \
        python3-pip \
        python3-setuptools \
        python3-serial \
        python3-click \
        python3-cryptography \
        python3-future \
        python3-pyparsing \
        python3-pyelftools \
        cmake \
        ninja-build \
        ccache \
        libffi-dev \
        libssl-dev \
        dfu-util \
        libusb-1.0-0
    
    # Determine ESP-IDF directory based on version
    if [[ "$IDF_VERSION" == "v5.0"* ]] || [[ "$IDF_VERSION" == "v5.1"* ]] || [[ "$IDF_VERSION" == "v5.2"* ]]; then
        IDF_DIR="$HOME/esp/esp-idf-v5"
    else
        IDF_DIR="$HOME/esp/esp-idf"
    fi
    
    # Clone or update ESP-IDF
    if [ ! -d "$IDF_DIR" ]; then
        mkdir -p "$HOME/esp"
        cd "$HOME/esp"
        git clone --recursive https://github.com/espressif/esp-idf.git "$(basename "$IDF_DIR")"
        cd "$IDF_DIR"
        git checkout "$IDF_VERSION"
        ./install.sh esp32
        echo "source $IDF_DIR/export.sh" >> ~/.bashrc
        cd - > /dev/null
    else
        print_status "ESP-IDF already installed, updating to $IDF_VERSION..."
        cd "$IDF_DIR"
        git fetch
        git checkout "$IDF_VERSION"
        git submodule update --init --recursive
        ./install.sh esp32
        cd - > /dev/null
    fi
    
    # Source ESP-IDF environment
    source "$IDF_DIR/export.sh"
    print_success "ESP-IDF $IDF_VERSION installed and configured"
}

# Function to run setup
run_setup() {
    print_status "Starting RaspRover IDF setup..."
    
    # Check and install ESP-IDF if needed
    check_idf_installation
    
    # Source ESP-IDF environment
    if [ -f "$HOME/esp/esp-idf/export.sh" ]; then
        source "$HOME/esp/esp-idf/export.sh"
    fi
    
    # Install dependencies
    print_status "Installing project dependencies..."
    ./install_dependencies.sh --all
    
    # Initial build test
    print_status "Testing initial build..."
    ./build.sh build
    
    print_success "Setup completed successfully!"
    print_status ""
    print_status "Next steps:"
    print_status "1. Configure your project: ./build.sh menuconfig"
    print_status "2. Build the project: ./build.sh build"
    print_status "3. Flash to ESP32: ./build.sh flash [PORT]"
    print_status "4. Monitor output: ./build.sh monitor [PORT]"
    print_status ""
    print_status "For help: ./build.sh --help"
}

# Function to show help
show_help() {
    echo "RaspRover IDF Quick Setup Script"
    echo "Supports both ESP-IDF v4.4 and v5.0+"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --help, -h         Show this help message"
    echo "  --idf-only         Install only ESP-IDF"
    echo "  --deps-only        Install only dependencies"
    echo "  --idf-version VER  Specify ESP-IDF version (e.g., v4.4.5, v5.0)"
    echo ""
    echo "Environment Variables:"
    echo "  IDF_VERSION        ESP-IDF version to install (default: v4.4.5)"
    echo ""
    echo "Examples:"
    echo "  $0                                    # Install with default v4.4.5"
    echo "  IDF_VERSION=v5.0 $0                  # Install with ESP-IDF v5.0"
    echo "  $0 --idf-version v5.1                # Install with ESP-IDF v5.1"
    echo ""
    echo "This script will:"
    echo "1. Check/install ESP-IDF if needed"
    echo "2. Install all project dependencies"
    echo "3. Test the initial build"
    echo ""
}

# Main script logic
case "${1:-}" in
    "--help"|"-h")
        show_help
        ;;
    "--idf-only")
        check_idf_installation
        ;;
    "--deps-only")
        ./install_dependencies.sh --all
        ;;
    "--idf-version")
        if [ -z "${2:-}" ]; then
            print_error "ESP-IDF version not specified"
            show_help
            exit 1
        fi
        IDF_VERSION="$2"
        check_idf_installation
        ;;
    "")
        run_setup
        ;;
    *)
        print_error "Unknown option: $1"
        show_help
        exit 1
        ;;
esac
