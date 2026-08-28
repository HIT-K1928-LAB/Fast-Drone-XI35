#!/usr/bin/env bash

set -eu

BRINGUP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROFILES_ROOT="$BRINGUP_ROOT/profiles"

list_profiles() {
    local profile_env

    for profile_env in "$PROFILES_ROOT"/*/profile.env; do
        if [ -f "$profile_env" ]; then
            basename "$(dirname "$profile_env")"
        fi
    done | sort
}

show_help() {
    cat <<EOF
Usage:
  $(basename "$0") <profile>
  $(basename "$0") --list-profiles
  $(basename "$0") completion

The profile is the only public flight argument. Change localization and other
vehicle modes in bringup/profiles/<profile>/profile.env.

Available profiles:
$(list_profiles | sed 's/^/  /')
EOF
}

print_completion() {
    cat <<'EOF'
_fast_drone_flight_completion() {
    local cur profiles
    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"
    if [ "$COMP_CWORD" -eq 1 ]; then
        profiles="$("${COMP_WORDS[0]}" --list-profiles 2>/dev/null)"
        COMPREPLY=( $(compgen -W "$profiles --help --list-profiles completion" -- "$cur") )
    fi
}
EOF
    printf 'complete -F _fast_drone_flight_completion %q\n' "$BRINGUP_ROOT/flight.sh"
}

case "${1:-}" in
    -h|--help)
        if [ "$#" -ne 1 ]; then
            show_help >&2
            exit 64
        fi
        show_help
        exit 0
        ;;
    --list-profiles)
        if [ "$#" -ne 1 ]; then
            show_help >&2
            exit 64
        fi
        list_profiles
        exit 0
        ;;
    completion)
        if [ "$#" -ne 1 ]; then
            show_help >&2
            exit 64
        fi
        print_completion
        exit 0
        ;;
esac

if [ "$#" -ne 1 ]; then
    show_help >&2
    exit 64
fi

PROFILE="$1"
PROFILE_DIR="$PROFILES_ROOT/$PROFILE"
PROFILE_ENV="$PROFILE_DIR/profile.env"
PROFILE_TMUX="$PROFILE_DIR/tmux.sh"

if [ ! -f "$PROFILE_ENV" ]; then
    echo "Unknown profile: $PROFILE" >&2
    echo "Available profiles:" >&2
    list_profiles | sed 's/^/  /' >&2
    exit 66
fi

if [ "${FASTDRONE_DISPATCH_DRY_RUN:-0}" = "1" ]; then
    printf '%s\n' "$PROFILE_TMUX"
    exit 0
fi

if [ ! -f "$PROFILE_TMUX" ]; then
    echo "Profile tmux script not found: $PROFILE_TMUX" >&2
    exit 66
fi

exec bash "$PROFILE_TMUX"
