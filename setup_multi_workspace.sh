#!/bin/bash
# Multi-Workspace Setup and Management Scripts

# =============================================================================
# Script 1: setup_multi_workspaces.sh - Reorganize existing workspace into multiple workspaces
# =============================================================================
#!/bin/bash
# setup_multi_workspaces.sh

echo "Setting up multiple ROS2 workspaces in repository..."

# Check if we're in a multi-workspace repo or single workspace
if [ -d "arduino_workspace" ] && [ -d "learning_workspace" ]; then
    echo "Multi-workspace structure detected. Re-organizing packages..."
    MULTI_WORKSPACE_MODE=true
else
    # Check if we're in the right location (look for ROS2 workspace indicators)
    if [ ! -d "src" ] && [ ! -d "inactive_packages" ] && [ ! -d "build" ] && [ ! -d "install" ]; then
        echo "Error: This doesn't appear to be a ROS2 workspace directory"
        echo "Current directory contents:"
        ls -la
        echo ""
        echo "Expected to find at least one of: src/, inactive_packages/, build/, install/"
        exit 1
    fi
    MULTI_WORKSPACE_MODE=false
fi

# Create src directory if it doesn't exist (for single workspace mode)
if [ "$MULTI_WORKSPACE_MODE" = false ] && [ ! -d "src" ]; then
    echo "Creating src/ directory..."
    mkdir -p src
fi

# Create workspace directories
echo "Creating/ensuring workspace directories..."
mkdir -p arduino_workspace/src
mkdir -p differential_robot_workspace/src  
mkdir -p learning_workspace/src
mkdir -p robot_arm_workspace/src
mkdir -p current_ws/src

# Move packages to appropriate workspaces based on your current packages
echo "Organizing packages into workspaces..."

# Look for packages in all possible locations (src/, inactive_packages/, or existing workspaces)
search_locations=("src" "inactive_packages" "arduino_workspace/src" "differential_robot_workspace/src" "learning_workspace/src" "robot_arm_workspace/src" "current_ws/src" ".")

for location in "${search_locations[@]}"; do
    if [ -d "$location" ]; then
        echo "Checking $location for packages..."
        
        # Arduino-related packages
        for pkg in "arduinobot_description" "arduino_bot_description" "arduinobot_py_examples" "arduinorobot_cpp_ex"; do
            if [ -d "$location/$pkg" ] && [ "$location" != "arduino_workspace/src" ]; then
                echo "Moving $pkg from $location to arduino_workspace"
                mv "$location/$pkg" arduino_workspace/src/ 2>/dev/null || true
            fi
        done
        
        # Differential robot packages  
        for pkg in "diffrobot_description" "diffrobot_gazebo"; do
            if [ -d "$location/$pkg" ] && [ "$location" != "differential_robot_workspace/src" ]; then
                echo "Moving $pkg from $location to differential_robot_workspace"
                mv "$location/$pkg" differential_robot_workspace/src/ 2>/dev/null || true
            fi
        done
        
        # Handle diffrobot folder specially
        if [ -d "$location/diffrobot" ] && [ "$location" != "differential_robot_workspace/src" ]; then
            echo "Moving diffrobot/* from $location to differential_robot_workspace"
            mv "$location/diffrobot"/* differential_robot_workspace/src/ 2>/dev/null || true
            rmdir "$location/diffrobot" 2>/dev/null || true
        fi
        
        # Learning/tutorial packages
        for pkg in "cpp_node" "py_node" "custom_msg" "use_custom_msg"; do
            if [ -d "$location/$pkg" ] && [ "$location" != "learning_workspace/src" ]; then
                echo "Moving $pkg from $location to learning_workspace"
                mv "$location/$pkg" learning_workspace/src/ 2>/dev/null || true
            fi
        done
        
        # Robot arm packages
        for pkg in "robotArm_4DOF" "robot_arm_4dof"; do
            if [ -d "$location/$pkg" ] && [ "$location" != "robot_arm_workspace/src" ]; then
                echo "Moving $pkg from $location to robot_arm_workspace"
                mv "$location/$pkg" robot_arm_workspace/src/ 2>/dev/null || true
            fi
        done
    fi
done

# Clean up old structure
echo "Cleaning up old workspace structure..."
rm -rf src build install log inactive_packages 2>/dev/null || true

# Create .gitignore for build artifacts
echo "Creating .gitignore for build artifacts..."
cat > .gitignore << 'EOF'
# ROS2 build artifacts
*/build/
*/install/
*/log/

# IDE files
.vscode/
.idea/
*.swp
*.swo

# OS files
.DS_Store
Thumbs.db
EOF

echo "Setup complete!"
echo ""
echo "Your workspaces:"
find . -maxdepth 2 -name "src" -type d | sort

# =============================================================================
# Script 2: workspace_manager.sh - Switch between and manage workspaces
# =============================================================================
#!/bin/bash
# workspace_manager.sh

show_usage() {
    echo "ROS2 Multi-Workspace Manager"
    echo "Usage: $0 <command> [workspace_name]"
    echo ""
    echo "Commands:"
    echo "  list                    - List all workspaces"
    echo "  status                  - Show status of all workspaces"
    echo "  build <workspace>       - Build a specific workspace"
    echo "  clean <workspace>       - Clean a specific workspace"
    echo "  shell <workspace>       - Open shell in workspace with sourced environment"
    echo "  create <workspace>      - Create a new workspace"
    echo ""
    echo "Available workspaces:"
    find . -maxdepth 1 -name "*_workspace" -type d -printf "  - %f\n" | sort
    echo ""
    echo "Examples:"
    echo "  $0 build arduino_workspace"
    echo "  $0 status"
    echo "  $0 shell learning_workspace"
}

list_workspaces() {
    echo "Available workspaces:"
    for ws in *_workspace/; do
        if [ -d "$ws" ]; then
            ws_name=$(basename "$ws")
            pkg_count=$(find "$ws/src" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | wc -l)
            echo "  ✓ $ws_name ($pkg_count packages)"
            
            if [ -d "$ws/src" ]; then
                find "$ws/src" -maxdepth 1 -mindepth 1 -type d -printf "    - %f\n" | sort
            fi
            echo ""
        fi
    done
}

show_status() {
    echo "Workspace Status:"
    echo "================"
    
    for ws in *_workspace/; do
        if [ -d "$ws" ]; then
            ws_name=$(basename "$ws")
            echo ""
            echo "📁 $ws_name:"
            
            # Check if built
            if [ -d "$ws/build" ] && [ "$(ls -A $ws/build 2>/dev/null)" ]; then
                echo "  ✅ Built (has build artifacts)"
                build_count=$(find "$ws/build" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | wc -l)
                echo "     Built packages: $build_count"
            else
                echo "  ⚪ Not built"
            fi
            
            # List packages
            if [ -d "$ws/src" ]; then
                pkg_count=$(find "$ws/src" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | wc -l)
                echo "  📦 Packages: $pkg_count"
            fi
        fi
    done
}

build_workspace() {
    local workspace=$1
    
    if [ -z "$workspace" ]; then
        echo "Error: Workspace name required"
        echo "Usage: $0 build <workspace_name>"
        return 1
    fi
    
    if [ ! -d "$workspace" ]; then
        echo "Error: Workspace '$workspace' not found"
        echo "Available workspaces:"
        find . -maxdepth 1 -name "*_workspace" -type d -printf "  - %f\n"
        return 1
    fi
    
    echo "Building workspace: $workspace"
    echo "==============================="
    
    cd "$workspace"
    
    # Source ROS2 setup if available
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    elif [ -f "/opt/ros/foxy/setup.bash" ]; then
        source /opt/ros/foxy/setup.bash
    fi
    
    # Build
    colcon build
    
    echo ""
    echo "✅ Build completed for $workspace"
    echo "To use this workspace, run:"
    echo "   cd $workspace && source install/setup.bash"
    
    cd ..
}

clean_workspace() {
    local workspace=$1
    
    if [ -z "$workspace" ]; then
        echo "Error: Workspace name required"
        return 1
    fi
    
    if [ ! -d "$workspace" ]; then
        echo "Error: Workspace '$workspace' not found"
        return 1
    fi
    
    echo "Cleaning workspace: $workspace"
    cd "$workspace"
    rm -rf build/ install/ log/
    echo "✅ Cleaned $workspace"
    cd ..
}

open_workspace_shell() {
    local workspace=$1
    
    if [ -z "$workspace" ]; then
        echo "Error: Workspace name required"
        return 1
    fi
    
    if [ ! -d "$workspace" ]; then
        echo "Error: Workspace '$workspace' not found"
        return 1
    fi
    
    echo "Opening shell in $workspace..."
    echo "Environment will be sourced automatically."
    
    cd "$workspace"
    
    # Create a temporary shell script that sources the environment
    cat > .temp_shell_setup.sh << 'EOF'
#!/bin/bash
# Auto-generated shell setup

# Source ROS2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
fi

# Source workspace if built
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✅ Workspace environment sourced"
else
    echo "⚠ Workspace not built yet. Run 'colcon build' first."
fi

# Show current workspace info
echo "📁 Current workspace: $(basename $(pwd))"
if [ -d "src" ]; then
    echo "📦 Packages: $(find src -maxdepth 1 -mindepth 1 -type d | wc -l)"
fi

# Clean up this script on exit
trap 'rm -f .temp_shell_setup.sh' EXIT

# Start interactive shell
exec bash
EOF
    
    chmod +x .temp_shell_setup.sh
    ./.temp_shell_setup.sh
}

create_workspace() {
    local workspace=$1
    
    if [ -z "$workspace" ]; then
        echo "Error: Workspace name required"
        return 1
    fi
    
    if [ -d "$workspace" ]; then
        echo "Error: Workspace '$workspace' already exists"
        return 1
    fi
    
    echo "Creating new workspace: $workspace"
    mkdir -p "$workspace/src"
    
    # Create basic package template
    cat > "$workspace/README.md" << EOF
# $workspace

## Description
ROS2 workspace for $workspace

## Build
\`\`\`bash
cd $workspace
colcon build
source install/setup.bash
\`\`\`

## Packages
- Add your packages to src/
EOF
    
    echo "✅ Created workspace: $workspace"
    echo "Add packages to $workspace/src/"
}

# Main script logic
case $1 in
    "list")
        list_workspaces
        ;;
    "status")
        show_status
        ;;
    "build")
        build_workspace $2
        ;;
    "clean")
        clean_workspace $2
        ;;
    "shell")
        open_workspace_shell $2
        ;;
    "create")
        create_workspace $2
        ;;
    *)
        show_usage
        ;;
esac

# =============================================================================
# Script 3: quick_workspace.sh - Quick workspace operations
# =============================================================================
#!/bin/bash
# quick_workspace.sh

echo "ROS2 Quick Workspace Selector"
echo "============================="

# List available workspaces
workspaces=($(find . -maxdepth 1 -name "*_workspace" -type d -printf "%f\n" | sort))

if [ ${#workspaces[@]} -eq 0 ]; then
    echo "No workspaces found. Run setup_multi_workspaces.sh first."
    exit 1
fi

echo "Available workspaces:"
for i in "${!workspaces[@]}"; do
    ws=${workspaces[$i]}
    pkg_count=$(find "$ws/src" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | wc -l)
    echo "  $((i+1)). $ws ($pkg_count packages)"
done

echo ""
read -p "Select workspace (1-${#workspaces[@]}): " choice

if [[ $choice =~ ^[0-9]+$ ]] && [ $choice -ge 1 ] && [ $choice -le ${#workspaces[@]} ]; then
    selected_ws=${workspaces[$((choice-1))]}
    
    echo ""
    echo "Selected: $selected_ws"
    echo "What would you like to do?"
    echo "1. Build workspace"
    echo "2. Open shell in workspace"
    echo "3. Clean workspace"
    echo "4. Show workspace info"
    
    read -p "Choose action (1-4): " action
    
    case $action in
        1)
            echo "Building $selected_ws..."
            cd "$selected_ws"
            source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null
            colcon build
            echo "✅ Build complete!"
            ;;
        2)
            echo "Opening shell in $selected_ws..."
            cd "$selected_ws"
            exec bash -c "
                source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null
                [ -f install/setup.bash ] && source install/setup.bash
                echo '✅ Workspace environment loaded'
                echo '📁 Current workspace: $selected_ws'
                exec bash
            "
            ;;
        3)
            echo "Cleaning $selected_ws..."
            cd "$selected_ws"
            rm -rf build/ install/ log/
            echo "✅ Cleaned!"
            ;;
        4)
            echo "Workspace info for $selected_ws:"
            echo "Packages:"
            find "$selected_ws/src" -maxdepth 1 -mindepth 1 -type d -printf "  - %f\n" | sort
            ;;
    esac
else
    echo "Invalid selection"
    exit 1
fi
