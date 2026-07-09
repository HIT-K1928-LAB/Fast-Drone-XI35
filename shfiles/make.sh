#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SRC_DIR="${WORKSPACE_ROOT}/src"

DEFAULT_BUILD_TYPE="Release"

usage() {
  cat <<'EOF'
Usage:
  shfiles/make.sh all [options]              Build all catkin packages
  shfiles/make.sh pkg <package> [options]    Build one catkin package
  shfiles/make.sh list [--paths]             List packages in this workspace
  shfiles/make.sh completion                 Print bash completion script

Options:
  -t, --type <Release|Debug|RelWithDebInfo|MinSizeRel>
                                      CMake build type. Default: Release
  -c, --compile-commands             Generate compile_commands.json
  -j, --jobs <N>                     Pass -jN to make
  -n, --dry-run                      Print the command without running it
  -h, --help                         Show help
  -- <extra cmake args...>           Forward extra args to catkin_make

Examples:
  shfiles/make.sh all
  shfiles/make.sh all -c
  shfiles/make.sh pkg yopo_planner
  shfiles/make.sh pkg quadrotor_msgs -c -j8
  shfiles/make.sh list --paths

Bash completion:
  source <(shfiles/make.sh completion)

  To enable it automatically, add this line to ~/.bashrc:
  source <(/root/Fast-Drone-XI35/shfiles/make.sh completion)
EOF
}

package_entries() {
  find "${SRC_DIR}" -name package.xml \
    ! -path "*/catkin_simple/test/*" \
    -print | sort | while IFS= read -r package_xml; do
      local name rel_path
      name="$(sed -n 's/.*<name>[[:space:]]*\([^<[:space:]][^<]*[^<[:space:]]\)[[:space:]]*<\/name>.*/\1/p' "${package_xml}" | head -n 1)"
      if [[ -z "${name}" ]]; then
        continue
      fi
      rel_path="${package_xml#${WORKSPACE_ROOT}/}"
      rel_path="${rel_path%/package.xml}"
      printf '%s\t%s\n' "${name}" "${rel_path}"
    done
}

package_names() {
  package_entries | cut -f1
}

package_exists() {
  local package="$1"
  package_names | grep -Fxq "${package}"
}

list_packages() {
  local with_paths=false
  local names_only=false

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --paths|-p)
        with_paths=true
        ;;
      --names-only)
        names_only=true
        ;;
      -h|--help)
        cat <<'EOF'
Usage:
  shfiles/make.sh list [--paths] [--names-only]
EOF
        return 0
        ;;
      *)
        echo "Unknown list option: $1" >&2
        return 2
        ;;
    esac
    shift
  done

  if [[ "${names_only}" == true ]]; then
    package_names
  elif [[ "${with_paths}" == true ]]; then
    package_entries | awk -F '\t' '{ printf "%-28s %s\n", $1, $2 }'
  else
    package_names | nl -w2 -s'. '
  fi
}

parse_build_options() {
  BUILD_TYPE="${DEFAULT_BUILD_TYPE}"
  EXPORT_COMPILE_COMMANDS=false
  DRY_RUN=false
  CATKIN_ARGS=()
  EXTRA_CMAKE_ARGS=()

  while [[ $# -gt 0 ]]; do
    case "$1" in
      -t|--type|--build-type)
        if [[ $# -lt 2 ]]; then
          echo "$1 requires a value" >&2
          return 2
        fi
        BUILD_TYPE="$2"
        shift 2
        ;;
      -c|--compile-commands)
        EXPORT_COMPILE_COMMANDS=true
        shift
        ;;
      -j|--jobs)
        if [[ $# -lt 2 ]]; then
          echo "$1 requires a value" >&2
          return 2
        fi
        CATKIN_ARGS+=("-j$2")
        shift 2
        ;;
      -n|--dry-run)
        DRY_RUN=true
        shift
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      --)
        shift
        EXTRA_CMAKE_ARGS+=("$@")
        break
        ;;
      *)
        EXTRA_CMAKE_ARGS+=("$1")
        shift
        ;;
    esac
  done
}

run_catkin_make() {
  local cmd=(catkin_make)

  if [[ $# -gt 0 ]]; then
    cmd+=("$@")
  fi

  cmd+=("-DCMAKE_BUILD_TYPE=${BUILD_TYPE}")

  if [[ "${EXPORT_COMPILE_COMMANDS}" == true ]]; then
    cmd+=("-DCMAKE_EXPORT_COMPILE_COMMANDS=ON")
  fi

  if [[ ${#EXTRA_CMAKE_ARGS[@]} -gt 0 ]]; then
    cmd+=("${EXTRA_CMAKE_ARGS[@]}")
  fi

  if [[ ${#CATKIN_ARGS[@]} -gt 0 ]]; then
    cmd+=("${CATKIN_ARGS[@]}")
  fi

  echo "Workspace: ${WORKSPACE_ROOT}"
  printf 'Command:'
  printf ' %q' "${cmd[@]}"
  printf '\n'

  if [[ "${DRY_RUN}" == true ]]; then
    return 0
  fi

  cd "${WORKSPACE_ROOT}"
  "${cmd[@]}"
}

build_all() {
  parse_build_options "$@"
  run_catkin_make
}

build_package() {
  if [[ $# -lt 1 || "$1" == "-h" || "$1" == "--help" ]]; then
    cat <<'EOF'
Usage:
  shfiles/make.sh pkg <package> [options]
EOF
    return 0
  fi

  local package="$1"
  shift

  if ! package_exists "${package}"; then
    echo "Unknown package: ${package}" >&2
    echo "Run 'shfiles/make.sh list' to see available packages." >&2
    return 2
  fi

  parse_build_options "$@"
  run_catkin_make --pkg "${package}"
}

print_completion() {
  cat <<'EOF'
_fast_drone_make_completion() {
  local cur prev words cword script commands packages

  COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"
  prev="${COMP_WORDS[COMP_CWORD-1]}"
  script="${COMP_WORDS[0]}"
  commands="all pkg package list ls completion help"

  case "${COMP_CWORD}" in
    1)
      COMPREPLY=( $(compgen -W "${commands}" -- "${cur}") )
      return 0
      ;;
  esac

  case "${COMP_WORDS[1]}" in
    pkg|package)
      if [[ "${COMP_CWORD}" -eq 2 ]]; then
        packages="$("${script}" list --names-only 2>/dev/null)"
        COMPREPLY=( $(compgen -W "${packages}" -- "${cur}") )
        return 0
      fi
      ;;
    list|ls)
      COMPREPLY=( $(compgen -W "--paths -p --names-only --help -h" -- "${cur}") )
      return 0
      ;;
  esac

  case "${prev}" in
    -t|--type|--build-type)
      COMPREPLY=( $(compgen -W "Release Debug RelWithDebInfo MinSizeRel" -- "${cur}") )
      return 0
      ;;
    -j|--jobs)
      COMPREPLY=( $(compgen -W "2 4 6 8 12 16" -- "${cur}") )
      return 0
      ;;
  esac

  COMPREPLY=( $(compgen -W "-t --type --build-type -c --compile-commands -j --jobs -n --dry-run -h --help --" -- "${cur}") )
}

complete -F _fast_drone_make_completion make.sh
complete -F _fast_drone_make_completion ./shfiles/make.sh
complete -F _fast_drone_make_completion shfiles/make.sh
EOF
  printf 'complete -F _fast_drone_make_completion %q\n' "${SCRIPT_DIR}/make.sh"
}

main() {
  if [[ $# -eq 0 ]]; then
    usage
    exit 0
  fi

  local command="$1"
  shift

  case "${command}" in
    all)
      build_all "$@"
      ;;
    pkg|package)
      build_package "$@"
      ;;
    list|ls)
      list_packages "$@"
      ;;
    completion)
      print_completion
      ;;
    help|-h|--help)
      usage
      ;;
    *)
      echo "Unknown command: ${command}" >&2
      echo >&2
      usage >&2
      exit 2
      ;;
  esac
}

main "$@"
