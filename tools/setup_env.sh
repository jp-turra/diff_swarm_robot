script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
root_dir="$( cd "$( dirname "${script_dir}" )" &> /dev/null && pwd )"

export COLCON_DEFAULTS_FILE="${root_dir}/defaults.yaml"