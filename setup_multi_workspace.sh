#!/bin/bash
# Multi-Workspace Setup and Management Scripts for ROS2_Repo

# =============================================================================
# Script 1: setup_multi_workspaces.sh - Reorganize existing workspace into multiple workspaces
# =============================================================================
#!/bin/bash
# setup_multi_workspaces.sh

echo "Setting up multiple ROS2 workspaces in repository..."
echo "Current directory: $(pwd)"
echo "Repository structure detected:"
ls -la

# Check if we're already in a multi-workspace setup
EXISTING_WORKSPACES=($(find . -maxdepth 1 -name "*_workspace" -type d 2>/dev/null))
if [ ${#EXISTING_WORKSPACES[@]} -gt 0 ]; then
    echo "Existing workspaces detected:"
    printf "  - %s\n" "${EXISTING_WORKSPACES[@]}"
    echo "Re-organizing and ensuring all packages are properly placed..."
    MULTI_WORKSPACE_MODE=true
else
    echo "Setting up multi-workspace structure from scratch..."
    MULTI_WORKSPACE_MODE=false
fi

# Ensure all workspace directories exist
echo "Creating/ensuring workspace directories..."
mkdir -p arduino_workspace/src
mkdir -p differential_robot_workspace/src  
mkdir -p learning_workspace/src
mkdir -p robot_arm_workspace/src
mkdir -p current_ws/src

# Preserve important repository files
echo "Preserving repository files..."
PRESERVE_FILES=("README.md" "git_auto_push.sh" "setup_multi_workspace.sh" ".git" ".gitignore")

# Create a comprehensive search for packages in all possible locations
search_locations=("." "src" "inactive_packages")
for ws in "${EXISTING_WORKSPACES[@]}"; do
    if [ -d "$ws/src" ]; then
        search_locations+=("$ws/src")
    fi
done

echo "Scanning locations for packages: ${search_locations[*]}"

# Function to move package safely
move_package() {
    local pkg=$1
    local source_location=$2
    local target_workspace=$3
    
    if [ -d "$source_location/$pkg" ] && [ "$source_location" != "$target_workspace/src" ]; then
        echo "  Moving $pkg from $source_location to $target_workspace/src/"
        # Create backup if target exists
        if [ -d "$target_workspace/src/$pkg" ]; then
            echo "    Warning: $pkg already exists in $target_workspace, creating backup"
            mv "$target_workspace/src/$pkg" "$target_workspace/src/${pkg}_backup_$(date +%s)" 2>/dev/null
        fi
        mv "$source_location/$pkg" "$target_workspace/src/" 2>/dev/null
        if [ $? -eq 0 ]; then
            echo "    ✅ Successfully moved $pkg"
        else
            echo "    ❌ Failed to move $pkg"
        fi
    fi
}

# Scan and organize packages
echo "Organizing packages into appropriate workspaces..."

for location in "${search_locations[@]}"; do
    if [ -d "$location" ]; then
        echo "  Checking $location for packages..."
        
        # Arduino-related packages
        arduino_packages=("arduinobot_description" "arduino_bot_description" "arduinobot_py_examples" "arduinorobot_cpp_ex" "arduino_robot_description")
        for pkg in "${arduino_packages[@]}"; do
            move_package "$pkg" "$location" "arduino_workspace"
        done
        
        # Differential robot packages  
        diffrobot_packages=("diffrobot_description" "diffrobot_gazebo" "differential_robot_description" "diff_robot_control")
        for pkg in "${diffrobot_packages[@]}"; do
            move_package "$pkg" "$location" "differential_robot_workspace"
        done
        
        # Handle diffrobot folder specially (if it contains multiple packages)
        if [ -d "$location/diffrobot" ] && [ "$location" != "differential_robot_workspace/src" ]; then
            echo "  Processing diffrobot directory from $location"
            if [ "$(ls -A $location/diffrobot 2>/dev/null)" ]; then
                for item in "$location/diffrobot"/*; do
                    if [ -d "$item" ]; then
                        pkg_name=$(basename "$item")
                        echo "    Moving $pkg_name from diffrobot/ to differential_robot_workspace/src/"
                        move_package "$pkg_name" "$location/diffrobot" "differential_robot_workspace"
                    fi
                done
            fi
            # Remove empty diffrobot directory
            if [ -z "$(ls -A $location/diffrobot 2>/dev/null)" ]; then
                rmdir "$location/diffrobot" 2>/dev/null && echo "    Removed empty diffrobot directory"
            fi
        fi
        
        # Learning/tutorial packages
        learning_packages=("cpp_node" "py_node" "custom_msg" "use_custom_msg" "ros2_tutorials" "beginner_tutorials")
        for pkg in "${learning_packages[@]}"; do
            move_package "$pkg" "$location" "learning_workspace"
        done
        
        # Robot arm packages
        arm_packages=("robotArm_4DOF" "robot_arm_4dof" "robot_arm_description" "robot_arm_control")
        for pkg in "${arm_packages[@]}"; do
            move_package "$pkg" "$location" "robot_arm_workspace"
        done
        
        # Current workspace packages (active development)
        current_packages=("current_project" "test_package" "experimental_package")
        for pkg in "${current_packages[@]}"; do
            move_package "$pkg" "$location" "current_ws"
        done
    fi
done

# Clean up old structure (but preserve important files)
echo "Cleaning up old workspace structure..."
for item in src build install log inactive_packages; do
    if [ -d "$item" ]; then
        # Check if it's empty or only contains hidden files
        if [ -z "$(find $item -mindepth 1 -not -path '*/.*' 2>/dev/null)" ]; then
            echo "  Removing empty directory: $item"
            rm -rf "$item" 2>/dev/null
        else
            echo "  Warning: $item still contains files, skipping removal"
            echo "    Contents: $(ls -la $item 2>/dev/null | head -5)"
        fi
    fi
done

# Create comprehensive .gitignore for build artifacts
echo "Creating/updating .gitignore for build artifacts..."
cat > .gitignore << 'EOF'
# ROS2 build artifacts
*/build/
*/install/
*/log/
build/
install/
log/

# IDE files
.vscode/
.idea/
*.swp
*.swo
*~

# Python cache
__pycache__/
*.pyc
*.pyo
*.pyd
.Python
*.so

# OS files
.DS_Store
Thumbs.db
*.tmp

# Backup files
*_backup_*
*.orig
EOF

# Create README files for each workspace
echo "Creating README files for workspaces..."

# Arduino workspace README
cat > arduino_workspace/README.md << 'EOF'
# Arduino Workspace

This workspace contains Arduino-related ROS2 packages for robot control and simulation.

## Packages
- `arduinobot_description`: Robot description files (URDF, meshes)
- `arduinobot_py_examples`: Python examples for Arduino robot control
- `arduinorobot_cpp_ex`: C++ examples for Arduino robot control

## Build and Run
```bash
cd arduino_workspace
colcon build
source install/setup.bash
```

## Usage
```bash
# Launch robot description
ros2 launch arduinobot_description display.launch.py

# Run Python examples
ros2 run arduinobot_py_examples simple_publisher
```
EOF

# Differential robot workspace README
cat > differential_robot_workspace/README.md << 'EOF'
# Differential Robot Workspace

This workspace contains packages for differential drive robot simulation and control.

## Packages
- `diffrobot_description`: Robot description and URDF files
- `diffrobot_gazebo`: Gazebo simulation configurations

## Build and Run
```bash
cd differential_robot_workspace
colcon build
source install/setup.bash
```

## Usage
```bash
# Launch in Gazebo
ros2 launch diffrobot_gazebo diffrobot_world.launch.py

# Control the robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
EOF

# Learning workspace README
cat > learning_workspace/README.md << 'EOF'
# Learning Workspace

This workspace contains ROS2 learning and tutorial packages.

## Packages
- `cpp_node`: C++ node examples
- `py_node`: Python node examples  
- `custom_msg`: Custom message definitions
- `use_custom_msg`: Examples using custom messages

## Build and Run
```bash
cd learning_workspace
colcon build
source install/setup.bash
```

## Learning Path
1. Start with basic nodes (`cpp_node`, `py_node`)
2. Learn about custom messages (`custom_msg`)
3. Practice using custom messages (`use_custom_msg`)
EOF

# Robot arm workspace README
cat > robot_arm_workspace/README.md << 'EOF'
# Robot Arm Workspace

This workspace contains 4-DOF robot arm packages for simulation and control.

## Packages
- `robotArm_4DOF`: Main robot arm package
- `robot_arm_4dof`: Alternative robot arm implementation

## Build and Run
```bash
cd robot_arm_workspace
colcon build
source install/setup.bash
```

## Usage
```bash
# Launch robot arm
ros2 launch robotArm_4DOF robot_arm.launch.py

# Control joints
ros2 topic pub /joint_commands std_msgs/Float64MultiArray ...
```
EOF

# Current workspace README
cat > current_ws/README.md << 'EOF'
# Current Workspace

This workspace is for active development and testing of new packages.

## Build and Run
```bash
cd current_ws
colcon build
source install/setup.bash
```

## Usage
Add your current development packages here for quick iteration and testing.
EOF

echo ""
echo "🎉 Multi-workspace setup complete!"
echo ""
echo "📁 Your workspaces:"
for ws in *_workspace current_ws; do
    if [ -d "$ws" ]; then
        pkg_count=$(find "$ws/src" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | wc -l)
        echo "  ✓ $ws ($pkg_count packages)"
        if [ $pkg_count -gt 0 ]; then
            find "$ws/src" -maxdepth 1 -mindepth 1 -type d -printf "    - %f\n" | sort
        fi
    fi
done

echo ""
echo "📖 Next steps:"
echo "  1. Use './workspace_manager.sh status' to check workspace status"
echo "  2. Use './workspace_manager.sh build <workspace_name>' to build a workspace"
echo "  3. Use './quick_workspace.sh' for interactive workspace selection"

# =============================================================================
# Script 2: workspace_manager.sh - Enhanced workspace management
# =============================================================================

show_usage() {
    echo "🤖 ROS2 Multi-Workspace Manager"
    echo "================================"
    echo "Usage: $0 <command> [workspace_name] [options]"
    echo ""
    echo "📋 Commands:"
    echo "  list                    - List all workspaces and their packages"
    echo "  status                  - Show detailed status of all workspaces"
    echo "  build <workspace>       - Build a specific workspace"
    echo "  build-all              - Build all workspaces"
    echo "  clean <workspace>       - Clean a specific workspace"
    echo "  clean-all              - Clean all workspaces"
    echo "  shell <workspace>       - Open shell in workspace with sourced environment"
    echo "  create <workspace>      - Create a new workspace"
    echo "  packages <workspace>    - List packages in a workspace"
    echo "  deps <workspace>        - Show dependencies for workspace"
    echo ""
    echo "📁 Available workspaces:"
    find . -maxdepth 1 -name "*_workspace" -o -name "current_ws" -type d 2>/dev/null | sort | while read ws; do
        ws_name=$(basename "$ws")
        pkg_count=$(find "$ws/src" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | wc -l)
        echo "  • $ws_name ($pkg_count packages)"
    done
    echo ""
    echo "💡 Examples:"
    echo "  $0 build arduino_workspace"
    echo "  $0 status"
    echo "  $0 shell learning_workspace"
    echo "  $0 build-all"
}

list_workspaces() {
    echo "🏗️  Available ROS2 Workspaces"
    echo "============================="
    
    for ws in *_workspace current_ws; do
        if [ -d "$ws" ]; then
            ws_name=$(basename "$ws")
            pkg_count=$(find "$ws/src" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | wc -l)
            
            # Determine status
            status="📦"
            if [ -d "$ws/build" ] && [ "$(ls -A $ws/build 2>/dev/null)" ]; then
                status="✅"
            fi
            
            echo ""
            echo "$status $ws_name ($pkg_count packages)"
            
            if [ -d "$ws/src" ] && [ $pkg_count -gt 0 ]; then
                find "$ws/src" -maxdepth 1 -mindepth 1 -type d -printf "    └── %f\n" | sort
            else
                echo "    └── (no packages)"
            fi
            
            # Show README if exists
            if [ -f "$ws/README.md" ]; then
                echo "    📝 README available"
            fi
        fi
    done
}

show_status() {
    echo "🔍 Workspace Status Report"
    echo "=========================="
    
    total_workspaces=0
    built_workspaces=0
    total_packages=0
    
    for ws in *_workspace current_ws; do
        if [ -d "$ws" ]; then
            total_workspaces=$((total_workspaces + 1))
            ws_name=$(basename "$ws")
            echo ""
            echo "📁 $ws_name"
            echo "   ├── Location: $(pwd)/$ws"
            
            # Check build status
            if [ -d "$ws/build" ] && [ "$(ls -A $ws/build 2>/dev/null)" ]; then
                echo "   ├── Status: ✅ Built"
                built_workspaces=$((built_workspaces + 1))
                build_count=$(find "$ws/build" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | wc -l)
                echo "   ├── Built packages: $build_count"
                
                # Check if install directory exists
                if [ -d "$ws/install" ]; then
                    echo "   ├── Install: ✅ Available"
                else
                    echo "   ├── Install: ❌ Missing"
                fi
            else
                echo "   ├── Status: ⚪ Not built"
            fi
            
            # Count packages
            if [ -d "$ws/src" ]; then
                pkg_count=$(find "$ws/src" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | wc -l)
                total_packages=$((total_packages + pkg_count))
                echo "   ├── Source packages: $pkg_count"
                
                if [ $pkg_count -gt 0 ]; then
                    echo "   └── Packages:"
                    find "$ws/src" -maxdepth 1 -mindepth 1 -type d -printf "       • %f\n" | sort
                fi
            fi
        fi
    done
    
    echo ""
    echo "📊 Summary"
    echo "=========="
    echo "Total workspaces: $total_workspaces"
    echo "Built workspaces: $built_workspaces"
    echo "Total packages: $total_packages"
}

build_workspace() {
    local workspace=$1
    
    if [ -z "$workspace" ]; then
        echo "❌ Error: Workspace name required"
        echo "Usage: $0 build <workspace_name>"
        return 1
    fi
    
    if [ ! -d "$workspace" ]; then
        echo "❌ Error: Workspace '$workspace' not found"
        echo "Available workspaces:"
        find . -maxdepth 1 -name "*_workspace" -o -name "current_ws" -type d -printf "  • %f\n"
        return 1
    fi
    
    echo "🔨 Building workspace: $workspace"
    echo "================================="
    
    cd "$workspace"
    
    # Check for packages
    pkg_count=$(find "src" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | wc -l)
    if [ $pkg_count -eq 0 ]; then
        echo "⚠️  Warning: No packages found in src/ directory"
        cd ..
        return 1
    fi
    
    echo "📦 Found $pkg_count package(s) to build"
    
    # Source ROS2 setup
    ROS_SOURCED=false
    for ros_setup in "/opt/ros/humble/setup.bash" "/opt/ros/foxy/setup.bash" "/opt/ros/galactic/setup.bash"; do
        if [ -f "$ros_setup" ]; then
            echo "🔗 Sourcing ROS2: $ros_setup"
            source "$ros_setup"
            ROS_SOURCED=true
            break
        fi
    done
    
    if [ "$ROS_SOURCED" = false ]; then
        echo "⚠️  Warning: No ROS2 installation found"
    fi
    
    # Build with colcon
    echo "🏗️  Starting build process..."
    if colcon build --symlink-install; then
        echo ""
        echo "✅ Build completed successfully for $workspace"
        echo ""
        echo "🚀 To use this workspace:"
        echo "   cd $workspace"
        echo "   source install/setup.bash"
        echo ""
        echo "📋 Available packages:"
        find src -maxdepth 1 -mindepth 1 -type d -printf "   • %f\n" | sort
    else
        echo ""
        echo "❌ Build failed for $workspace"
        echo "Check the error messages above for details"
        cd ..
        return 1
    fi
    
    cd ..
}

build_all_workspaces() {
    echo "🔨 Building all workspaces"
    echo "========================="
    
    success_count=0
    failure_count=0
    
    for ws in *_workspace current_ws; do
        if [ -d "$ws" ]; then
            echo ""
            echo "Building $ws..."
            if build_workspace "$ws"; then
                success_count=$((success_count + 1))
            else
                failure_count=$((failure_count + 1))
            fi
        fi
    done
    
    echo ""
    echo "📊 Build Summary"
    echo "==============="
    echo "✅ Successful: $success_count"
    echo "❌ Failed: $failure_count"
}

clean_workspace() {
    local workspace=$1
    
    if [ -z "$workspace" ]; then
        echo "❌ Error: Workspace name required"
        return 1
    fi
    
    if [ ! -d "$workspace" ]; then
        echo "❌ Error: Workspace '$workspace' not found"
        return 1
    fi
    
    echo "🧹 Cleaning workspace: $workspace"
    cd "$workspace"
    
    # Show what will be removed
    dirs_to_remove=()
    for dir in build install log; do
        if [ -d "$dir" ]; then
            dirs_to_remove+=("$dir")
        fi
    done
    
    if [ ${#dirs_to_remove[@]} -gt 0 ]; then
        echo "Will remove: ${dirs_to_remove[*]}"
        rm -rf "${dirs_to_remove[@]}"
        echo "✅ Cleaned $workspace"
    else
        echo "✨ $workspace is already clean"
    fi
    
    cd ..
}

clean_all_workspaces() {
    echo "🧹 Cleaning all workspaces"
    echo "========================="
    
    for ws in *_workspace current_ws; do
        if [ -d "$ws" ]; then
            clean_workspace "$ws"
        fi
    done
    
    echo "✅ All workspaces cleaned"
}

open_workspace_shell() {
    local workspace=$1
    
    if [ -z "$workspace" ]; then
        echo "❌ Error: Workspace name required"
        return 1
    fi
    
    if [ ! -d "$workspace" ]; then
        echo "❌ Error: Workspace '$workspace' not found"
        return 1
    fi
    
    echo "🐚 Opening interactive shell in $workspace..."
    echo "Environment will be sourced automatically."
    
    cd "$workspace"
    
    # Create enhanced shell setup script
    cat > .temp_shell_setup.sh << 'SHELL_EOF'
#!/bin/bash
# Enhanced shell setup for ROS2 workspace

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🤖 ROS2 Workspace Shell${NC}"
echo -e "${BLUE}======================${NC}"

# Source ROS2
ROS_SOURCED=false
for ros_setup in "/opt/ros/humble/setup.bash" "/opt/ros/foxy/setup.bash" "/opt/ros/galactic/setup.bash"; do
    if [ -f "$ros_setup" ]; then
        source "$ros_setup"
        echo -e "${GREEN}✅ ROS2 sourced: $ros_setup${NC}"
        ROS_SOURCED=true
        break
    fi
done

if [ "$ROS_SOURCED" = false ]; then
    echo -e "${YELLOW}⚠️  Warning: No ROS2 installation found${NC}"
fi

# Source workspace if built
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo -e "${GREEN}✅ Workspace environment sourced${NC}"
else
    echo -e "${YELLOW}⚠️  Workspace not built yet. Run 'colcon build' first.${NC}"
fi

# Show workspace info
echo ""
echo -e "${BLUE}📁 Current workspace: $(basename $(pwd))${NC}"
if [ -d "src" ]; then
    pkg_count=$(find src -maxdepth 1 -mindepth 1 -type d | wc -l)
    echo -e "${BLUE}📦 Packages: $pkg_count${NC}"
    if [ $pkg_count -gt 0 ]; then
        echo -e "${BLUE}   Packages:${NC}"
        find src -maxdepth 1 -mindepth 1 -type d -printf "   • %f\n" | sort
    fi
fi

echo ""
echo -e "${BLUE}💡 Quick commands:${NC}"
echo "   colcon build              - Build all packages"
echo "   colcon build --packages-select <pkg> - Build specific package"
echo "   ros2 pkg list             - List available packages"
echo "   ros2 launch <pkg> <launch> - Launch a package"
echo "   exit                      - Exit workspace shell"
echo ""

# Set a custom PS1 to show we're in a workspace
export PS1="($(basename $(pwd))) \[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ "

# Clean up this script on exit
trap 'rm -f .temp_shell_setup.sh' EXIT

# Start interactive shell
exec bash
SHELL_EOF
    
    chmod +x .temp_shell_setup.sh
    ./.temp_shell_setup.sh
}

create_workspace() {
    local workspace=$1
    
    if [ -z "$workspace" ]; then
        echo "❌ Error: Workspace name required"
        return 1
    fi
    
    # Ensure workspace name ends with _workspace or is current_ws
    if [[ ! "$workspace" =~ _workspace$ ]] && [ "$workspace" != "current_ws" ]; then
        workspace="${workspace}_workspace"
    fi
    
    if [ -d "$workspace" ]; then
        echo "❌ Error: Workspace '$workspace' already exists"
        return 1
    fi
    
    echo "🆕 Creating new workspace: $workspace"
    mkdir -p "$workspace/src"
    
    # Create comprehensive README
    cat > "$workspace/README.md" << EOF
# $workspace

## Description
ROS2 workspace for $workspace

Created on: $(date)

## Structure
\`\`\`
$workspace/
├── src/          # Source packages go here
├── build/        # Build artifacts (auto-generated)
├── install/      # Install space (auto-generated)
├── log/          # Build logs (auto-generated)
└── README.md     # This file
\`\`\`

## Quick Start

### Build the workspace
\`\`\`bash
cd $workspace
colcon build
\`\`\`

### Source the workspace
\`\`\`bash
source install/setup.bash
\`\`\`

### Add a new package
\`\`\`bash
cd src
ros2 pkg create <package_name> --build-type ament_cmake  # For C++
ros2 pkg create <package_name> --build-type ament_python # For Python
\`\`\`

## Usage Examples
\`\`\`bash
# Build specific packages
colcon build --packages-select <package_name>

# Build with verbose output
colcon build --event-handlers console_direct+

# Clean build
rm -rf build install log
colcon build
\`\`\`

## Packages
Add your package documentation here as you create them.

EOF
    
    echo "✅ Created workspace: $workspace"
    echo "📝 README created with usage instructions"
    echo "📁 Add your packages to $workspace/src/"
}

show_packages() {
    local workspace=$1
    
    if [ -z "$workspace" ]; then
        echo "❌ Error: Workspace name required"
        return 1
    fi
    
    if [ ! -d "$workspace" ]; then
        echo "❌ Error: Workspace '$workspace' not found"
        return 1
    fi
    
    echo "📦 Packages in $workspace"
    echo "========================="
    
    if [ -d "$workspace/src" ]; then
        pkg_count=$(find "$workspace/src" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | wc -l)
        
        if [ $pkg_count -eq 0 ]; then
            echo "No packages found in $workspace/src"
        else
            echo "Found $pkg_count package(s):"
            echo ""
            
            find "$workspace/src" -maxdepth 1 -mindepth 1 -type d | sort | while read pkg_dir; do
                pkg_name=$(basename "$pkg_dir")
                echo "📁 $pkg_name"
                
                # Check for package.xml
                if [ -f "$pkg_dir/package.xml" ]; then
                    # Try to extract description from package.xml
                    desc=$(grep -o '<description>[^<]*</description>' "$pkg_dir/package.xml" 2>/dev/null | sed 's/<[^>]*>//g')
                    if [ -n "$desc" ]; then
                        echo "   📝 $desc"
                    fi
                    
                    # Check build type
                    if [ -f "$pkg_dir/CMakeLists.txt" ]; then
                        echo "   🔧 Build: ament_cmake"
                    elif [ -f "$pkg_dir/setup.py" ]; then
                        echo "   🐍 Build: ament_python"
                    fi
                else
                    echo "   ⚠️  No package.xml found"
                fi
                echo ""
            done
        fi
    else
        echo "❌ No src directory found in $workspace"
    fi
}

show_dependencies() {
    local workspace=$1
    
    if [ -z "$workspace" ]; then
        echo "❌ Error: Workspace name required"
        return 1
    fi
    
    if [ ! -d "$workspace" ]; then
        echo "❌ Error: Workspace '$workspace' not found"
        return 1
    fi
    
    echo "🔗 Dependencies for $workspace"
    echo "=============================="
    
    cd "$workspace"
    
    # Check if rosdep is available
    if command -v rosdep >/dev/null 2>&1; then
        echo "Running rosdep check..."
        rosdep check --from-paths src --ignore-src -r || true
    else
        echo "⚠️  rosdep not available, checking package.xml files manually..."
        
        find src -name "package.xml" | while read pkg_xml; do
            pkg_name=$(basename $(dirname "$pkg_xml"))
            echo ""
            echo "📁 $pkg_name dependencies:"
            
            # Extract dependencies from package.xml
            for dep_type in "build_depend" "exec_depend" "depend"; do
                deps=$(grep -o "<$dep_type>[^<]*</$dep_type>" "$pkg_xml" 2>/dev/null | sed 's/<[^>]*>//g')
                if [ -n "$deps" ]; then
                    echo "  $dep_type:"
                    echo "$deps" | while read dep; do
                        echo "    - $dep"
                    done
                fi
            done
        done
    fi
    
    cd ..
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
    "build-all")
        build_all_workspaces
        ;;
    "clean")
        clean_workspace $2
        ;;
    "clean-all")
        clean_all_workspaces
        ;;
    "shell")
        open_workspace_shell $2
        ;;
    "create")
        create_workspace $2
        ;;
    "packages")
        show_packages $2
        ;;
    "deps")
        show_dependencies $2
        ;;
    *)
        show_usage
        ;;
esac

# =============================================================================
# Script 3: quick_workspace.sh - Enhanced interactive workspace selector
# =============================================================================
#!/bin/bash
# quick_workspace.sh

echo "🚀 ROS2 Quick Workspace Selector"
echo "================================="

# Colors for better UX
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Find all workspaces
workspaces=($(find . -maxdepth 1 \( -name "*_workspace" -o -name "current_ws" \) -type d -printf "%f\n" | sort))

if [ ${#workspaces[@]} -eq 0 ]; then
    echo -e "${RED}❌ No workspaces found.${NC}"
    echo -e "${YELLOW}💡 Run setup_multi_workspaces.sh first to create workspaces.${NC}"
    exit 1
fi

echo -e "${CYAN}📁 Available workspaces:${NC}"
echo ""

# Display workspaces with detailed info
for i in "${!workspaces[@]}"; do
    ws=${workspaces[$i]}
    pkg_count=$(find "$ws/src" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | wc -l)
    
    # Check build status
    build_status="⚪"
    if [ -d "$ws/build" ] && [ "$(ls -A $ws/build 2>/dev/null)" ]; then
        build_status="✅"
    fi
    
    # Get workspace description from README if available
    description=""
    if [ -f "$ws/README.md" ]; then
        description=$(grep -m 1 "^This workspace" "$ws/README.md" 2>/dev/null | head -c 50)
        if [ -n "$description" ]; then
            description="- ${description}..."
        fi
    fi
    
    echo -e "${BLUE}$((i+1)).${NC} ${GREEN}$ws${NC} $build_status"
    echo -e "    📦 $pkg_count packages $description"
    
    # Show top 3 packages if any exist
    if [ $pkg_count -gt 0 ]; then
        echo -e "${PURPLE}    Packages:${NC}"
        find "$ws/src" -maxdepth 1 -mindepth 1 -type d -printf "      • %f\n" | sort | head -3
        if [ $pkg_count -gt 3 ]; then
            echo -e "      • ... and $((pkg_count-3)) more"
        fi
    fi
    echo ""
done

echo -e "${YELLOW}💡 Legend: ✅ = Built, ⚪ = Not built${NC}"
echo ""

# Get user selection
while true; do
    echo -e "${CYAN}Select workspace (1-${#workspaces[@]}) or 'q' to quit:${NC}"
    read -p "👉 " choice
    
    if [ "$choice" = "q" ] || [ "$choice" = "Q" ]; then
        echo "Goodbye! 👋"
        exit 0
    fi
    
    if [[ $choice =~ ^[0-9]+$ ]] && [ $choice -ge 1 ] && [ $choice -le ${#workspaces[@]} ]; then
        break
    else
        echo -e "${RED}❌ Invalid selection. Please choose 1-${#workspaces[@]} or 'q' to quit.${NC}"
    fi
done

selected_ws=${workspaces[$((choice-1))]}

echo ""
echo -e "${GREEN}✅ Selected: $selected_ws${NC}"

# Show current workspace info
pkg_count=$(find "$selected_ws/src" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | wc -l)
build_status="Not built"
if [ -d "$selected_ws/build" ] && [ "$(ls -A $selected_ws/build 2>/dev/null)" ]; then
    build_status="Built"
fi

echo -e "${BLUE}📊 Workspace Info:${NC}"
echo "   📁 Path: $(pwd)/$selected_ws"
echo "   📦 Packages: $pkg_count"
echo "   🔧 Status: $build_status"
echo ""

# Action menu
echo -e "${CYAN}🛠️  What would you like to do?${NC}"
echo ""
echo "1. 🔨 Build workspace"
echo "2. 🐚 Open interactive shell"
echo "3. 🧹 Clean workspace"
echo "4. 📋 Show detailed workspace info"
echo "5. 📦 List all packages"
echo "6. 🔍 Show dependencies"
echo "7. 📖 View README"
echo "8. 🚀 Quick launch (build + shell)"
echo "9. ❌ Cancel"
echo ""

while true; do
    read -p "👉 Choose action (1-9): " action
    
    case $action in
        1)
            echo -e "${YELLOW}🔨 Building $selected_ws...${NC}"
            cd "$selected_ws"
            
            # Check for packages first
            if [ $pkg_count -eq 0 ]; then
                echo -e "${RED}❌ No packages found in src/ directory${NC}"
                echo -e "${YELLOW}💡 Add packages to src/ first${NC}"
                cd ..
                break
            fi
            
            # Source ROS2
            ROS_SOURCED=false
            for ros_setup in "/opt/ros/humble/setup.bash" "/opt/ros/foxy/setup.bash" "/opt/ros/galactic/setup.bash"; do
                if [ -f "$ros_setup" ]; then
                    source "$ros_setup"
                    echo -e "${GREEN}🔗 Sourced: $ros_setup${NC}"
                    ROS_SOURCED=true
                    break
                fi
            done
            
            if [ "$ROS_SOURCED" = false ]; then
                echo -e "${YELLOW}⚠️  Warning: No ROS2 installation found${NC}"
            fi
            
            # Build with progress indication
            echo -e "${BLUE}🏗️  Building packages...${NC}"
            if colcon build --symlink-install; then
                echo -e "${GREEN}✅ Build completed successfully!${NC}"
                echo -e "${BLUE}📋 Built packages:${NC}"
                find src -maxdepth 1 -mindepth 1 -type d -printf "   ✅ %f\n" | sort
            else
                echo -e "${RED}❌ Build failed. Check the error messages above.${NC}"
            fi
            cd ..
            break
            ;;
        2)
            echo -e "${BLUE}🐚 Opening interactive shell in $selected_ws...${NC}"
            ./workspace_manager.sh shell "$selected_ws"
            break
            ;;
        3)
            echo -e "${YELLOW}🧹 Cleaning $selected_ws...${NC}"
            cd "$selected_ws"
            dirs_to_remove=()
            for dir in build install log; do
                if [ -d "$dir" ]; then
                    dirs_to_remove+=("$dir")
                fi
            done
            
            if [ ${#dirs_to_remove[@]} -gt 0 ]; then
                echo -e "${YELLOW}Will remove: ${dirs_to_remove[*]}${NC}"
                rm -rf "${dirs_to_remove[@]}"
                echo -e "${GREEN}✅ Cleaned successfully!${NC}"
            else
                echo -e "${GREEN}✨ Already clean!${NC}"
            fi
            cd ..
            break
            ;;
        4)
            echo -e "${BLUE}📋 Detailed info for $selected_ws:${NC}"
            echo ""
            ./workspace_manager.sh status | grep -A 20 "📁 $selected_ws"
            break
            ;;
        5)
            echo -e "${BLUE}📦 Packages in $selected_ws:${NC}"
            ./workspace_manager.sh packages "$selected_ws"
            break
            ;;
        6)
            echo -e "${BLUE}🔍 Dependencies for $selected_ws:${NC}"
            ./workspace_manager.sh deps "$selected_ws"
            break
            ;;
        7)
            if [ -f "$selected_ws/README.md" ]; then
                echo -e "${BLUE}📖 README for $selected_ws:${NC}"
                echo ""
                cat "$selected_ws/README.md"
            else
                echo -e "${YELLOW}⚠️  No README.md found in $selected_ws${NC}"
            fi
            break
            ;;
        8)
            echo -e "${YELLOW}🚀 Quick launch: Building and opening shell for $selected_ws...${NC}"
            
            # First build
            cd "$selected_ws"
            if [ $pkg_count -eq 0 ]; then
                echo -e "${RED}❌ No packages found. Add packages to src/ first.${NC}"
                cd ..
                break
            fi
            
            # Source ROS2
            for ros_setup in "/opt/ros/humble/setup.bash" "/opt/ros/foxy/setup.bash" "/opt/ros/galactic/setup.bash"; do
                if [ -f "$ros_setup" ]; then
                    source "$ros_setup"
                    break
                fi
            done
            
            echo -e "${BLUE}🔨 Building...${NC}"
            if colcon build --symlink-install; then
                echo -e "${GREEN}✅ Build successful! Opening shell...${NC}"
                cd ..
                ./workspace_manager.sh shell "$selected_ws"
            else
                echo -e "${RED}❌ Build failed.${NC}"
                cd ..
            fi
            break
            ;;
        9)
            echo -e "${YELLOW}Cancelled. 👋${NC}"
            break
            ;;
        *)
            echo -e "${RED}❌ Invalid selection. Please choose 1-9.${NC}"
            ;;
    esac
done

# =============================================================================
# Script 4: workspace_tools.sh - Additional utility functions
# =============================================================================
#!/bin/bash
# workspace_tools.sh

show_tools_usage() {
    echo "🔧 ROS2 Workspace Tools"
    echo "======================"
    echo "Additional utilities for workspace management"
    echo ""
    echo "Usage: $0 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  backup <workspace>      - Create backup of workspace"
    echo "  restore <backup_file>   - Restore workspace from backup"
    echo "  migrate <old> <new>     - Migrate packages from old to new workspace"
    echo "  sync                    - Synchronize all workspace dependencies"
    echo "  doctor                  - Check workspace health"
    echo "  template <name>         - Create workspace from template"
    echo "  export <workspace>      - Export workspace configuration"
    echo "  import <config_file>    - Import workspace configuration"
    echo ""
    echo "Examples:"
    echo "  $0 backup arduino_workspace"
    echo "  $0 doctor"
    echo "  $0 migrate learning_workspace experimental_workspace"
}

backup_workspace() {
    local workspace=$1
    
    if [ -z "$workspace" ]; then
        echo "❌ Error: Workspace name required"
        return 1
    fi
    
    if [ ! -d "$workspace" ]; then
        echo "❌ Error: Workspace '$workspace' not found"
        return 1
    fi
    
    local backup_name="${workspace}_backup_$(date +%Y%m%d_%H%M%S).tar.gz"
    
    echo "💾 Creating backup of $workspace..."
    echo "Backup file: $backup_name"
    
    # Create backup excluding build artifacts
    tar -czf "$backup_name" \
        --exclude="$workspace/build" \
        --exclude="$workspace/install" \
        --exclude="$workspace/log" \
        "$workspace"
    
    if [ $? -eq 0 ]; then
        echo "✅ Backup created successfully: $backup_name"
        echo "📊 Backup size: $(du -h $backup_name | cut -f1)"
    else
        echo "❌ Backup failed"
        return 1
    fi
}

doctor_check() {
    echo "🏥 ROS2 Workspace Health Check"
    echo "=============================="
    
    local issues_found=0
    
    # Check ROS2 installation
    echo "🔍 Checking ROS2 installation..."
    ros2_found=false
    for ros_setup in "/opt/ros/humble/setup.bash" "/opt/ros/foxy/setup.bash" "/opt/ros/galactic/setup.bash"; do
        if [ -f "$ros_setup" ]; then
            echo "  ✅ Found ROS2: $ros_setup"
            ros2_found=true
            break
        fi
    done
    
    if [ "$ros2_found" = false ]; then
        echo "  ❌ No ROS2 installation found"
        issues_found=$((issues_found + 1))
    fi
    
    # Check colcon
    echo "🔍 Checking build tools..."
    if command -v colcon >/dev/null 2>&1; then
        echo "  ✅ colcon available"
    else
        echo "  ❌ colcon not found"
        echo "     Install with: pip install colcon-common-extensions"
        issues_found=$((issues_found + 1))
    fi
    
    # Check workspaces
    echo "🔍 Checking workspaces..."
    workspace_count=0
    for ws in *_workspace current_ws; do
        if [ -d "$ws" ]; then
            workspace_count=$((workspace_count + 1))
            echo "  📁 Checking $ws..."
            
            # Check src directory
            if [ ! -d "$ws/src" ]; then
                echo "    ❌ Missing src/ directory"
                issues_found=$((issues_found + 1))
            else
                pkg_count=$(find "$ws/src" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | wc -l)
                if [ $pkg_count -eq 0 ]; then
                    echo "    ⚠️  No packages in src/"
                else
                    echo "    ✅ $pkg_count packages found"
                fi
            fi
            
            # Check for broken symlinks in install
            if [ -d "$ws/install" ]; then
                broken_links=$(find "$ws/install" -type l -! -exec test -e {} \; -print 2>/dev/null | wc -l)
                if [ $broken_links -gt 0 ]; then
                    echo "    ⚠️  $broken_links broken symlinks in install/"
                fi
            fi
            
            # Check package.xml files
            find "$ws/src" -name "package.xml" | while read pkg_xml; do
                if ! xmllint --noout "$pkg_xml" 2>/dev/null; then
                    echo "    ❌ Invalid XML in $(dirname $pkg_xml)"
                    issues_found=$((issues_found + 1))
                fi
            done
        fi
    done
    
    echo "📊 Summary:"
    echo "  Workspaces found: $workspace_count"
    echo "  Issues found: $issues_found"
    
    if [ $issues_found -eq 0 ]; then
        echo "🎉 All checks passed! Your workspace setup looks healthy."
    else
        echo "⚠️  Found $issues_found issue(s) that may need attention."
    fi
}

sync_dependencies() {
    echo "🔄 Synchronizing workspace dependencies..."
    
    if ! command -v rosdep >/dev/null 2>&1; then
        echo "❌ rosdep not found. Install with:"
        echo "   sudo apt install python3-rosdep"
        return 1
    fi
    
    # Initialize rosdep if needed
    if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
        echo "🔧 Initializing rosdep..."
        sudo rosdep init
    fi
    
    echo "🔄 Updating rosdep database..."
    rosdep update
    
    # Check each workspace
    for ws in *_workspace current_ws; do
        if [ -d "$ws" ] && [ -d "$ws/src" ]; then
            pkg_count=$(find "$ws/src" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | wc -l)
            if [ $pkg_count -gt 0 ]; then
                echo "🔍 Checking dependencies for $ws..."
                cd "$ws"
                rosdep install --from-paths src --ignore-src -r -y
                cd ..
            fi
        fi
    done
    
    echo "✅ Dependency synchronization complete!"
}

create_template_workspace() {
    local template_name=$1
    local workspace_name="${template_name}_workspace"
    
    if [ -z "$template_name" ]; then
        echo "❌ Error: Template name required"
        return 1
    fi
    
    if [ -d "$workspace_name" ]; then
        echo "❌ Error: Workspace '$workspace_name' already exists"
        return 1
    fi
    
    echo "🏗️  Creating template workspace: $workspace_name"
    mkdir -p "$workspace_name/src"
    
    # Create example package structure
    pkg_name="${template_name}_example"
    pkg_dir="$workspace_name/src/$pkg_name"
    mkdir -p "$pkg_dir"
    
    # Create package.xml
    cat > "$pkg_dir/package.xml" << EOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypeid="package_format3"?>
<package format="3">
  <name>$pkg_name</name>
  <version>1.0.0</version>
  <description>Example package for $template_name workspace</description>
  
  <maintainer email="user@example.com">Your Name</maintainer>
  <license>MIT</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF
    
    # Create CMakeLists.txt
    cat > "$pkg_dir/CMakeLists.txt" << EOF
cmake_minimum_required(VERSION 3.8)
project($pkg_name)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable
add_executable(example_node src/example_node.cpp)
ament_target_dependencies(example_node rclcpp std_msgs)

# Install targets
install(TARGETS
  example_node
  DESTINATION lib/\${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
EOF
    
    # Create source directory and example node
    mkdir -p "$pkg_dir/src"
    cat > "$pkg_dir/src/example_node.cpp" << EOF
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class ExampleNode : public rclcpp::Node
{
public:
    ExampleNode() : Node("example_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("example_topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&ExampleNode::publish_message, this));
        
        RCLCPP_INFO(this->get_logger(), "Example node started");
    }

private:
    void publish_message()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello from $template_name workspace!";
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published: '%s'", message.data.c_str());
    }
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExampleNode>());
    rclcpp::shutdown();
    return 0;
}
EOF
    
    # Create workspace README
    cat > "$workspace_name/README.md" << EOF
# $workspace_name

Template workspace created on $(date)

## Quick Start

1. Build the workspace:
\`\`\`bash
cd $workspace_name
colcon build
source install/setup.bash
\`\`\`

2. Run the example:
\`\`\`bash
ros2 run $pkg_name example_node
\`\`\`

3. In another terminal, listen to the topic:
\`\`\`bash
source $workspace_name/install/setup.bash
ros2 topic echo /example_topic
\`\`\`

## Development

Add your packages to the \`src/\` directory and modify the example package as needed.

## Template Structure
- \`$pkg_name/\`: Example C++ package with publisher node
- Configured with proper dependencies and build files
- Ready to extend with your own functionality
EOF
    
    echo "✅ Template workspace created: $workspace_name"
    echo "📁 Example package: $pkg_name"
    echo "🚀 Ready to build and run!"
}

# Main tools script logic
case $1 in
    "backup")
        backup_workspace $2
        ;;
    "doctor")
        doctor_check
        ;;
    "sync")
        sync_dependencies
        ;;
    "template")
        create_template_workspace $2
        ;;
    *)
        show_tools_usage
        ;;
esac

echo ""
echo "🎯 Multi-workspace setup complete!"
echo "=================================="
echo ""
echo "📝 Created files:"
echo "  • setup_multi_workspaces.sh    - Main setup script"
echo "  • workspace_manager.sh          - Workspace management"
echo "  • quick_workspace.sh            - Interactive selector"
echo "  • workspace_tools.sh            - Additional utilities"
echo ""
echo "🚀 Quick start:"
echo "  1. chmod +x *.sh"
echo "  2. ./setup_multi_workspaces.sh"
echo "  3. ./quick_workspace.sh"
echo ""
echo "💡 Pro tips:"
echo "  • Use './workspace_manager.sh status' for overview"
echo "  • Use './workspace_manager.sh build-all' to build everything"
echo "  • Use './workspace_tools.sh doctor' to check health"