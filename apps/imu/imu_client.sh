#!/bin/bash

# Default connection parameters
HOST="10.42.0.199"
PORT=5000

# Check if custom host/port provided
if [ $# -eq 2 ]; then
    HOST=$1
    PORT=$2
fi

echo "Connecting to IMU server at $HOST:$PORT..."

# Connect to server and process data
nc $HOST $PORT | while IFS="," read -r x y z; do
    printf "X: %6s  Y: %6s  Z: %6s\r" "$x" "$y" "$z"
done

echo -e "\nConnection closed." 