#!/usr/bin/env bash

set -e

source ~/export-esp.sh >/dev/null 2>&1

BUILD_MODE=""
case "$1" in
"" | "release")
    cargo run --release
    ;;
"debug")
     cargo run 
    ;;
*)
    echo "Wrong argument. Only \"debug\"/\"release\" arguments are supported"
    exit 1
    ;;
esac