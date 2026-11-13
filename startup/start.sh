#!/usr/bin/env bash

cd "$(dirname "$(readlink -f "$0")")" && tmuxinator start -p session.yml "$@"