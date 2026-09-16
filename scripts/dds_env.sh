# Shared Bash/Zsh fragment; caller sets fastlio_workspace_dir.
case "${FASTLIO_DDS:-${RMW_IMPLEMENTATION:-fastdds}}" in
  fastdds|rmw_fastrtps_cpp)
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    unset CYCLONEDDS_URI
    ;;
  cyclone|rmw_cyclonedds_cpp)
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$fastlio_workspace_dir/dds_config/cyclonedds.xml}"
    ;;
  *) echo "FASTLIO_DDS must be fastdds or cyclone (only these RMWs are supported by the helper)." >&2; return 2 ;;
esac
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-18}"
# Do not force localhost-only: PC/Jetson discovery is normally required.
