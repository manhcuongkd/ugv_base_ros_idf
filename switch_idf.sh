#!/bin/bash

# ESP-IDF Version Switcher for RaspRover IDF
# This script helps switch between ESP-IDF v4.4 and v5.0+

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

# Function to show help
show_help() {
    echo "ESP-IDF Version Switcher for RaspRover IDF"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --help, -h         Show this help message"
    echo "  --list             List available ESP-IDF versions"
    echo "  --current          Show current ESP-IDF version"
    echo "  --switch VERSION   Switch to specified ESP-IDF version"
    echo "  --v4               Switch to ESP-IDF v4.4.5"
    echo "  --v5               Switch to ESP-IDF v5.0"
    echo ""
    echo "Examples:"
    echo "  $0 --list                    # List available versions"
    echo "  $0 --current                 # Show current version"
    echo "  $0 --v4                      # Switch to ESP-IDF v4.4.5"
    echo "  $0 --v5                      # Switch to ESP-IDF v5.0"
    echo "  $0 --switch v5.1             # Switch to ESP-IDF v5.1"
    echo ""
}

# Function to list available ESP-IDF versions
list_versions() {
    print_status "Checking for available ESP-IDF installations..."
    
    if [ -d "$HOME/esp/esp-idf" ]; then
        cd "$HOME/esp/esp-idf"
        current_branch=$(git branch --show-current 2>/dev/null || echo "unknown")
        print_status "ESP-IDF v4.x: $HOME/esp/esp-idf (current: $current_branch)"
    fi
    
    if [ -d "$HOME/esp/esp-idf-v5" ]; then
        cd "$HOME/esp/esp-idf-v5"
        current_branch=$(git branch --show-current 2>/dev/null || echo "unknown")
        print_status "ESP-IDF v5.x: $HOME/esp/esp-idf-v5 (current: $current_branch)"
    fi
    
    if [ ! -d "$HOME/esp/esp-idf" ] && [ ! -d "$HOME/esp/esp-idf-v5" ]; then
        print_warning "No ESP-IDF installations found"
        print_status "Run ./setup.sh to install ESP-IDF"
    fi
}

# Function to show current ESP-IDF version
show_current() {
    if command -v idf.py &> /dev/null; then
        current_version=$(idf.py --version 2>/dev/null || echo "unknown")
        print_success "Current ESP-IDF version: $current_version"
        
        # Try to determine which installation is active
        if [[ "$PATH" == *"esp-idf-v5"* ]]; then
            print_status "Active installation: ESP-IDF v5.x"
        elif [[ "$PATH" == *"esp-idf"* ]]; then
            print_status "Active installation: ESP-IDF v4.x"
        else
            print_status "Active installation: Unknown"
        fi
    else
        print_error "ESP-IDF not found in PATH"
        print_status "Please run ./setup.sh to install ESP-IDF"
    fi
}

# Function to switch ESP-IDF version
switch_version() {
    local target_version="$1"
    
    print_status "Switching to ESP-IDF $target_version..."
    
    case "$target_version" in
        "v4"|"v4.4"|"v4.4.5")
            if [ -d "$HOME/esp/esp-idf" ]; then
                source "$HOME/esp/esp-idf/export.sh"
                print_success "Switched to ESP-IDF v4.x"
                print_status "Current version: $(idf.py --version)"
            else
                print_error "ESP-IDF v4.x not found at $HOME/esp/esp-idf"
                print_status "Run: ./setup.sh --idf-version v4.4.5"
            fi
            ;;
        "v5"|"v5.0"|"v5.1"|"v5.2")
            if [ -d "$HOME/esp/esp-idf-v5" ]; then
                source "$HOME/esp/esp-idf-v5/export.sh"
                print_success "Switched to ESP-IDF v5.x"
                print_status "Current version: $(idf.py --version)"
            else
                print_error "ESP-IDF v5.x not found at $HOME/esp/esp-idf-v5"
                print_status "Run: ./setup.sh --idf-version v5.0"
            fi
            ;;
        *)
            print_error "Unsupported ESP-IDF version: $target_version"
            print_status "Supported versions: v4.4.5, v5.0, v5.1, v5.2"
            exit 1
            ;;
    esac
}

# Main script logic
case "${1:-}" in
    "--help"|"-h")
        show_help
        ;;
    "--list")
        list_versions
        ;;
    "--current")
        show_current
        ;;
    "--switch")
        if [ -z "${2:-}" ]; then
            print_error "ESP-IDF version not specified"
            show_help
            exit 1
        fi
        switch_version "$2"
        ;;
    "--v4")
        switch_version "v4.4.5"
        ;;
    "--v5")
        switch_version "v5.0"
        ;;
    "")
        show_help
        ;;
    *)
        print_error "Unknown option: $1"
        show_help
        exit 1
        ;;
esac
