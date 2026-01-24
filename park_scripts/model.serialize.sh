#!/bin/bash

xml_file="/apollo/modules/perception/launch/perception_serialize.launch"

# Check if the file exists
if [ ! -f "$xml_file" ]; then
    echo "File $xml_file does not exist."
    exit 1
fi

declare -A process_name_map
process_name_map=(
    ["semantics"]=2
    ["motion"]=2
    ["cpdet"]=2
    ["traffic_detection"]=1
    ["traffic_recognition"]=1
    ["centerpoint"]=4
)

has_paddle="false"

# Retrieve the<process_name>defined in the file
process_names=($(grep -oP '(?<=<process_name>).*?(?=</process_name>)' "$xml_file"))

# Initialize the expected total number of files
expected_total_count=0
for process_name in "${process_names[@]}"; do
    if [ "$process_name" == "centerpoint" ]; then
        has_paddle="true"
    fi

    if [[ -n "${process_name_map[$process_name]}" ]]; then
        expected_total_count=$((expected_total_count + process_name_map[$process_name]))
    fi
done
echo "The model needs to generate a total number of engines: $expected_total_count"

# Check function
function check_files() {
    # Get the actual total number of. rt.Engine files
    actual_total_count=$(find /apollo/modules/perception/data/models -type f -name "*.trt.engine" | wc -l)
    if [ "$has_paddle" == "true" ]; then
        paddle_trt_count=$(find /apollo/modules/perception/data/models/center_point_paddle -type f -name "trt_serialized*" | wc -l) 
        actual_total_count=$((actual_total_count + paddle_trt_count))
    fi    

    if [[ "$actual_total_count" -eq "$expected_total_count" ]]; then
        return 0
    else
        echo "Current number of engines: $actual_total_count"
        return 1
    fi
}

# Start the cyber_1aunch process
cyber_launch start $xml_file > /apollo_workspace/data/perception_model.log 2>&1 &
CYBER_PID=$!

# Loop check until the quantity matches
while true; do
    if ! ps -p "$CYBER_PID" > /dev/null; then
        echo "Error: cyber_launch process has stopped unexpectedly."
        exit 1
    fi
    
    if check_files; then
        echo "All model engines have been generated"
        break
    fi
    
    sleep 120
done

# kill cyber_launch
kill "$CYBER_PID"
echo "cyber_launch process terminated successfully."
