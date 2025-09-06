#!/bin/bash

# RaspRover IDF Dependencies Installation Script
# This script automatically installs all required dependencies
# Supports both ESP-IDF v4.4 and v5.0+

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
        print_status "or for ESP-IDF v5: source \$HOME/esp/esp-idf-v5/export.sh"
        exit 1
    fi
    
    current_version=$(idf.py --version 2>/dev/null || echo "unknown")
    print_success "ESP-IDF found: $current_version"
    
    # Check if it's a supported version
    if [[ "$current_version" == *"v4.4"* ]] || [[ "$current_version" == *"v5.0"* ]] || [[ "$current_version" == *"v5.1"* ]] || [[ "$current_version" == *"v5.2"* ]]; then
        print_success "ESP-IDF version is supported"
    else
        print_warning "ESP-IDF version may not be fully supported. Recommended: v4.4.5 or v5.0+"
    fi
}

# Function to check if git is available
check_git() {
    if ! command -v git &> /dev/null; then
        print_error "Git not found. Please install git first."
        exit 1
    fi
    
    print_success "Git found: $(git --version)"
}

# Function to install dependencies using ESP-IDF Component Manager
install_idf_dependencies() {
    print_status "Installing dependencies using ESP-IDF Component Manager..."
    
    # Update component registry
    idf.py reconfigure
    
    # Install dependencies
    idf.py reconfigure --recursive
    
    print_success "ESP-IDF dependencies installed"
}

# Function to install custom components manually
install_custom_components() {
    print_status "Installing custom components..."
    
    local components_dir="components"
    mkdir -p "$components_dir"
    
    # Function to clone or update a repository
    install_component() {
        local name=$1
        local repo=$2
        local branch=${3:-main}
        local target_dir="$components_dir/$name"
        
        if [ -d "$target_dir" ]; then
            print_status "Updating $name..."
            cd "$target_dir"
            git fetch origin
            git checkout "$branch"
            git pull origin "$branch"
            cd - > /dev/null
        else
            print_status "Installing $name..."
            git clone -b "$branch" "$repo" "$target_dir"
        fi
    }
    
    # Install custom components
    install_component "SCServo" "https://github.com/yourusername/SCServo.git" "master"
    
    print_success "Custom components installed"
}

# Function to install system dependencies (Ubuntu/Debian)
install_system_dependencies() {
    print_status "Checking system dependencies..."
    
    # Check if we're on a Debian-based system
    if command -v apt-get &> /dev/null; then
        print_status "Installing system dependencies (Ubuntu/Debian)..."
        
        # Update package list
        sudo apt-get update
        
        # Install required packages
        sudo apt-get install -y \
            build-essential \
            cmake \
            git \
            python3 \
            python3-pip \
            python3-venv \
            libusb-1.0-0-dev \
            pkg-config \
            libssl-dev \
            libffi-dev \
            python3-dev
        
        print_success "System dependencies installed"
    else
        print_warning "Not on a Debian-based system. Please install dependencies manually."
    fi
}

# Function to install Python dependencies
install_python_dependencies() {
    print_status "Installing Python dependencies..."
    
    # Install required Python packages
    pip3 install --user \
        esptool \
        adafruit-circuitpython-busdevice \
        adafruit-circuitpython-register \
        adafruit-blinka
    
    print_success "Python dependencies installed"
}

# Function to verify installation
verify_installation() {
    print_status "Verifying installation..."
    
    # Check if components directory exists
    if [ -d "components" ]; then
        print_success "Components directory created"
    else
        print_error "Components directory not found"
        return 1
    fi
    
    # Check if idf_component.yml exists
    if [ -f "idf_component.yml" ]; then
        print_success "Component configuration file found"
    else
        print_error "Component configuration file not found"
        return 1
    fi
    
    # Try to build the project
    print_status "Testing build configuration..."
    if idf.py reconfigure > /dev/null 2>&1; then
        print_success "Build configuration verified"
    else
        print_warning "Build configuration may have issues"
    fi
    
    print_success "Installation verification completed"
}

# Function to show help
show_help() {
    echo "RaspRover IDF Dependencies Installation Script"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --all              Install all dependencies (default)"
    echo "  --idf-only         Install only ESP-IDF dependencies"
    echo "  --custom-only      Install only custom components"
    echo "  --system-only      Install only system dependencies"
    echo "  --python-only      Install only Python dependencies"
    echo "  --verify           Verify installation only"
    echo "  -h, --help         Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                 # Install all dependencies"
    echo "  $0 --idf-only      # Install only ESP-IDF dependencies"
    echo "  $0 --verify        # Verify installation"
    echo ""
}

# Main installation function
main_installation() {
    print_status "Starting RaspRover IDF dependencies installation..."
    
    # Check prerequisites
    check_idf
    check_git
    
    # Install system dependencies
    install_system_dependencies
    
    # Install Python dependencies
    install_python_dependencies
    
    # Install ESP-IDF dependencies
    install_idf_dependencies
    
    # Install custom components
    install_custom_components
    
    # Verify installation
    verify_installation
    
    print_success "All dependencies installed successfully!"
    print_status "You can now build the project with: ./build.sh build"
}

# Main script logic
case "${1:-}" in
    "--all"|"")
        main_installation
        ;;
    "--idf-only")
        check_idf
        install_idf_dependencies
        ;;
    "--custom-only")
        check_git
        install_custom_components
        ;;
    "--system-only")
        install_system_dependencies
        ;;
    "--python-only")
        install_python_dependencies
        ;;
    "--verify")
        verify_installation
        ;;
    "-h"|"--help")
        show_help
        ;;
    *)
        print_error "Unknown option: $1"
        show_help
        exit 1
        ;;
esac
