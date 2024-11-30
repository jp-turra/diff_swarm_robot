script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
root_dir="$( cd "$( dirname "${script_dir}" )" &> /dev/null && pwd )"

extra_str=""
setup_colcon_defaults() {
    if [[ -z "$COLCON_DEFAULTS_FILE" ]]; then
        if [["$(cat ~/.bashrc | grep COLCON_DEFAULTS_FILE)" != "*COLCON_DEFAULTS_FILE*"  ]]; then
            echo "Setting COLCON_DEFAULTS_FILE"
            echo "export COLCON_DEFAULTS_FILE=${root_dir}/defaults.yaml" >> ~/.bashrc
            export COLCON_DEFAULTS_FILE="${root_dir}/defaults.yaml"

            extra_str=" Please source ~/.bashrc or restart your shell to apply the changes."
        fi
    fi
}

source_project() {
    if [[ -f "$root_dir/colcon/install/setup.bash" ]]; then
        echo "Sourcing the project"
        source "$root_dir/colcon/install/setup.bash"
    fi
}

setup_colcon_defaults
source_project

echo "Environment setup done.${extra_str}"
