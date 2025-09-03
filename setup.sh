#!/bin/bash

# RaspRover IDF Quick Setup Script
# This script sets up the entire project environment

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

# Function to check ESP-IDF installation
check_idf_installation() {
    print_status "Checking ESP-IDF installation..."
    
    if ! command -v idf.py &> /dev/null; then
        print_error "ESP-IDF not found in PATH"
        print_status "Installing ESP-IDF..."
        
        # Check if we're on Ubuntu/Debian
        if command -v apt-get &> /dev/null; then
            install_idf_ubuntu
        else
            print_error "Please install ESP-IDF manually from:"
            print_status "https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/"
            exit 1
        fi
    else
        print_success "ESP-IDF found: $(idf.py --version)"
    fi
}

# Function to install ESP-IDF on Ubuntu/Debian
install_idf_ubuntu() {
    print_status "Installing ESP-IDF on Ubuntu/Debian..."
    
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
    
    # Clone ESP-IDF
    if [ ! -d "$HOME/esp/esp-idf" ]; then
        mkdir -p "$HOME/esp"
        cd "$HOME/esp"
        git clone --recursive https://github.com/espressif/esp-idf.git
        cd esp-idf
        git checkout v4.4.5  # Use stable version
        ./install.sh esp32
        echo 'source $HOME/esp/esp-idf/export.sh' >> ~/.bashrc
        source ~/.bashrc
        cd - > /dev/null
    else
        print_status "ESP-IDF already installed, updating..."
        cd "$HOME/esp/esp-idf"
        git pull
        ./install.sh esp32
        cd - > /dev/null
    fi
    
    # Source ESP-IDF environment
    source "$HOME/esp/esp-idf/export.sh"
    print_success "ESP-IDF installed and configured"
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
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --help, -h         Show this help message"
    echo "  --idf-only         Install only ESP-IDF"
    echo "  --deps-only        Install only dependencies"
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
    "")
        run_setup
        ;;
    *)
        print_error "Unknown option: $1"
        show_help
        exit 1
        ;;
esac
