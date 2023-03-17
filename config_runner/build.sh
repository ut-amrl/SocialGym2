#!/bin/bash

if command -v podman &> /dev/null
then
    # Podman is installed
    docker build --format docker -t social_gym_config_runner:1.0 -f ./config_runner/Dockerfile .
else
    if command -v docker &> /dev/null
    then
        # Docker is installed
        docker build -t social_gym_config_runner:1.0 -f ./config_runner/Dockerfile .
    else
        # Neither Podman nor Docker is installed
        echo "Neither Podman nor Docker is installed."
        exit 1
    fi
fi
