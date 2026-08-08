#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SRC_DIR="${WORKSPACE_ROOT}/src"

DEFAULT_BUILD_TYPE="Release"
CATKIN_LOG_SPACE="build/logs"

# RK3588 Fast-LIVO build set. Add or remove ROS package names here as needed.
# catkin_tools will also build any required package dependencies automatically.
RK3588_FASTLIVO_PACKAGES=(
  vikit_common
  vikit_ros
  fast_livo
)

usage() {
  cat <<'EOF'
Usage:
  tools/make.sh all [options]              Build all catkin packages with catkin build
  tools/make.sh pkg <package> [options]    Build one catkin package with catkin build
  tools/make.sh fastlivo [options]         Build the RK3588 Fast-LIVO package set
  tools/make.sh clean [options]            Clean catkin build products
  tools/make.sh list [--paths]             List packages in this workspace
  tools/make.sh completion                 Print bash completion script

Options:
  -t, --type <Release|Debug|RelWithDebInfo|MinSizeRel>
                                      CMake build type. Default: Release
  -c, --compile-commands             Generate compile_commands.json
  -j, --jobs <N>                     Pass -j N to catkin build
  -n, --dry-run                      Print the command without running it
  -h, --help                         Show help
  -- <extra cmake args...>           Forward extra args to catkin build --cmake-args

Build logs:
  Catkin build logs are stored in build/logs.

Clean options:
  tools/make.sh clean              Clean build/devel/install/log products
  tools/make.sh clean --logs       Clean only logs
  tools/make.sh clean --build      Clean only build space
  tools/make.sh clean --devel      Clean only devel space
  tools/make.sh clean --install    Clean only install space
  tools/make.sh clean -n           Show what would be cleaned

Examples:
  tools/make.sh all
  tools/make.sh all -c
  tools/make.sh fastlivo -c
  tools/make.sh pkg yopo_planner
  tools/make.sh pkg quadrotor_msgs -c -j8
  tools/make.sh clean
  tools/make.sh list --paths

Bash completion:
  source <(tools/make.sh completion)

  To enable it automatically, add this line to ~/.bashrc:
  source <(/root/Fast-Drone-XI35/tools/make.sh completion)
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
  tools/make.sh list [--paths] [--names-only]
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

merge_compile_commands() {
  python3 - "${WORKSPACE_ROOT}" <<'PY'
import glob
import json
import os
import sys

workspace = sys.argv[1]
databases = sorted(glob.glob(os.path.join(workspace, "build", "*", "compile_commands.json")))
commands_by_file = {}

for database in databases:
    with open(database, encoding="utf-8") as stream:
        for command in json.load(stream):
            source = command.get("file")
            if not source:
                continue
            if not os.path.isabs(source):
                source = os.path.normpath(os.path.join(command.get("directory", ""), source))
            commands_by_file[source] = command

output = os.path.join(workspace, "compile_commands.json")
temporary = output + ".tmp"
with open(temporary, "w", encoding="utf-8") as stream:
    json.dump(list(commands_by_file.values()), stream, indent=2)
    stream.write("\n")
os.replace(temporary, output)
print(f"Merged {len(commands_by_file)} entries from {len(databases)} databases into {output}")
PY
}

run_catkin_build() {
  local cmd=(catkin build)
  local config_cmd=(catkin config --workspace "${WORKSPACE_ROOT}" --log-space "${CATKIN_LOG_SPACE}")
  local cmake_args=()

  if [[ $# -gt 0 ]]; then
    cmd+=("$@")
  fi

  cmake_args+=("-DCMAKE_BUILD_TYPE=${BUILD_TYPE}")

  if [[ "${EXPORT_COMPILE_COMMANDS}" == true ]]; then
    cmake_args+=("-DCMAKE_EXPORT_COMPILE_COMMANDS=ON")
  fi

  if [[ ${#EXTRA_CMAKE_ARGS[@]} -gt 0 ]]; then
    cmake_args+=("${EXTRA_CMAKE_ARGS[@]}")
  fi

  if [[ ${#CATKIN_ARGS[@]} -gt 0 ]]; then
    cmd+=("${CATKIN_ARGS[@]}")
  fi

  if [[ ${#cmake_args[@]} -gt 0 ]]; then
    cmd+=("--cmake-args" "${cmake_args[@]}")
  fi

  echo "Workspace: ${WORKSPACE_ROOT}"
  echo "Build logs: ${WORKSPACE_ROOT}/${CATKIN_LOG_SPACE}"
  printf 'Config command:'
  printf ' %q' "${config_cmd[@]}"
  printf '\n'
  printf 'Command:'
  printf ' %q' "${cmd[@]}"
  printf '\n'

  if [[ "${DRY_RUN}" == true ]]; then
    return 0
  fi

  cd "${WORKSPACE_ROOT}"
  "${config_cmd[@]}"
  "${cmd[@]}"

  if [[ "${EXPORT_COMPILE_COMMANDS}" == true ]]; then
    merge_compile_commands
  fi
}

build_all() {
  parse_build_options "$@"
  run_catkin_build
}

build_package() {
  if [[ $# -lt 1 || "$1" == "-h" || "$1" == "--help" ]]; then
    cat <<'EOF'
Usage:
  tools/make.sh pkg <package> [options]
EOF
    return 0
  fi

  local package="$1"
  shift

  if ! package_exists "${package}"; then
    echo "Unknown package: ${package}" >&2
    echo "Run 'tools/make.sh list' to see available packages." >&2
    return 2
  fi

  parse_build_options "$@"
  run_catkin_build "${package}"
}

build_fastlivo() {
  local package

  for package in "${RK3588_FASTLIVO_PACKAGES[@]}"; do
    if ! package_exists "${package}"; then
      echo "Unknown Fast-LIVO package in RK3588_FASTLIVO_PACKAGES: ${package}" >&2
      return 2
    fi
  done

  parse_build_options "$@"
  run_catkin_build "${RK3588_FASTLIVO_PACKAGES[@]}"
}

clean_products() {
  local cmd=(catkin clean --workspace "${WORKSPACE_ROOT}" -y)
  local clean_build=false
  local clean_devel=false
  local clean_install=false
  local clean_logs=false
  local clean_space_selected=false
  local dry_run=false

  while [[ $# -gt 0 ]]; do
    case "$1" in
      -n|--dry-run)
        dry_run=true
        ;;
      -b|--build|--build-space)
        clean_build=true
        clean_space_selected=true
        ;;
      -d|--devel|--devel-space)
        clean_devel=true
        clean_space_selected=true
        ;;
      -i|--install|--install-space)
        clean_install=true
        clean_space_selected=true
        ;;
      -L|--logs|--log-space)
        clean_logs=true
        clean_space_selected=true
        ;;
      -h|--help)
        cat <<'EOF'
Usage:
  tools/make.sh clean [options]

Options:
  -n, --dry-run      Show what would be cleaned
  -b, --build        Clean only build space
  -d, --devel        Clean only devel space
  -i, --install      Clean only install space
  -L, --logs         Clean only log space
  -h, --help         Show help
EOF
        return 0
        ;;
      -*)
        echo "Unknown clean option: $1" >&2
        return 2
        ;;
      *)
        echo "Unknown clean argument: $1" >&2
        return 2
        ;;
    esac
    shift
  done

  if [[ "${clean_space_selected}" == false ]]; then
    clean_build=true
    clean_devel=true
    clean_install=true
  fi

  [[ "${dry_run}" == true ]] && cmd+=("--dry-run")
  [[ "${clean_build}" == true ]] && cmd+=("--build")
  [[ "${clean_devel}" == true ]] && cmd+=("--devel")
  [[ "${clean_install}" == true ]] && cmd+=("--install")

  # The log space is nested under the build space (build/logs). Cleaning both
  # as independent catkin spaces makes catkin_tools remove build first and then
  # fail when build/logs no longer exists.
  if [[ "${clean_logs}" == true && "${clean_build}" == false ]]; then
    cmd+=("--logs")
  fi

  echo "Workspace: ${WORKSPACE_ROOT}"
  printf 'Command:'
  printf ' %q' "${cmd[@]}"
  printf '\n'

  cd "${WORKSPACE_ROOT}"
  "${cmd[@]}"
  if [[ "${dry_run}" == false && "${clean_build}" == true ]]; then
    rm -f "${WORKSPACE_ROOT}/compile_commands.json"
  fi
}

print_completion() {
  cat <<'EOF'
_fast_drone_make_completion() {
  local cur prev words cword script commands packages

  COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"
  prev="${COMP_WORDS[COMP_CWORD-1]}"
  script="${COMP_WORDS[0]}"
  commands="all pkg package fastlivo clean list ls completion help"

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
    fastlivo)
      ;;
    list|ls)
      COMPREPLY=( $(compgen -W "--paths -p --names-only --help -h" -- "${cur}") )
      return 0
      ;;
    clean)
      COMPREPLY=( $(compgen -W "-n --dry-run -b --build --build-space -d --devel --devel-space -i --install --install-space -L --logs --log-space -h --help" -- "${cur}") )
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
complete -F _fast_drone_make_completion ./tools/make.sh
complete -F _fast_drone_make_completion tools/make.sh
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
    fastlivo)
      build_fastlivo "$@"
      ;;
    clean)
      clean_products "$@"
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
