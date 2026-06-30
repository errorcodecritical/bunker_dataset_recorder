#!/usr/bin/env bash
set -e

cleanup() {
    local exit_code=$?
    docker compose down
    return $exit_code
}
# Trap EXIT signal to ensure cleanup runs
trap cleanup EXIT
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
cd $SCRIPT_DIR/docker
docker compose up -d
sleep 1
docker compose run --rm -i recorder
