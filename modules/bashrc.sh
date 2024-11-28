#!/bin/bash

# Check if "## Macro" is present in ~/.bashrc
if ! grep -Fxq "## Macro" ~/.bashrc
then
    # Append the entire block of macros and aliases to ~/.bashrc
    cat << 'EOF' >> ~/.bashrc

## Macro
export ROSCONSOLE_FORMAT='[${severity}] [${node}]: ${message}'
alias killr='ps aux | grep ros | grep -v grep | awk '\''{ print "kill -9", $2 }'\'' | sh && killall -9 roscore && killall -9 rosmaster && killall -9 rosout && killall -9 rviz'
alias killg="killall gzclient; killall gzserver; killall rosmaster; ps aux | grep ros | grep -v grep | awk '{ print \"kill -9\", \$2 }' | sh;ps aux | grep gazebo | grep -v grep | awk '{ print \"kill -9\", \$2 }' | sh"
alias killb="ps aux | grep blender | grep -v grep | awk '{ print \"kill -9\", \$2 }' | sh"
alias yamlfix="yamlfixer --recurse -1 ."
alias xmlfix="find . -maxdepth 3 -type f -name \"*.xml\" -o -name \"*.launch\" | xargs -I '{}' xmllint --format '{}' -output '{}'"
alias hooks_disable="git config --global core.hooksPath no-hooks"
alias hooks_enable="git config --global --unset core.hooksPath"
alias update_rosdep="catkin source; roscd; cd ..; rosdep install --from-paths src --ignore-src -r -y"

function update_rosinstall() {
    source $(catkin locate --shell-verbs)
    catkin source
    roscd
    cd ../src

    # Files to ignore, including robotis pattern
    ignore_pattern="(\./eband_local_planner/.*\.rosinstall|\./moveit/.*\.rosinstall|\./robotis/.*\.rosinstall)"
    
    # Get all .rosinstall files
    files=$(find . -type f -regextype posix-egrep -regex "\./.+\.rosinstall" | sort)
    pre_n=0
    n=$(echo "${files}" | wc -w)

    while [ ${n} -ne ${pre_n} ]; do
        # Filter out the ignored files
        filtered_files=$(echo "${files}" | grep -Ev "${ignore_pattern}")
        
        # If no files are left after filtering, break the loop
        if [ -z "$filtered_files" ]; then
            break
        fi

        for f in ${filtered_files}; do
            echo "Processing ${f}"
            vcs import --skip-existing --recursive --debug < "${f}"
        done
        
        # Update the file list and counts for the next iteration
        files=$(find . -type f -regextype posix-egrep -regex "\./.+\.rosinstall" | sort)
        pre_n=${n}
        n=$(echo "${files}" | wc -w)
    done
}
EOF

    # Reload ~/.bashrc to apply the changes immediately
    source ~/.bashrc
fi

