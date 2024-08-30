#!/bin/bash

docker buildx build \
    -t laarma:latest \
    -f Dockerfile .
