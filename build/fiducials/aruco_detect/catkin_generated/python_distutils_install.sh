#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/my_ros_data/pa_ws/src/fiducials/aruco_detect"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/my_ros_data/pa_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/my_ros_data/pa_ws/install/lib/python3/dist-packages:/my_ros_data/pa_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/my_ros_data/pa_ws/build" \
    "/usr/bin/python3" \
    "/my_ros_data/pa_ws/src/fiducials/aruco_detect/setup.py" \
     \
    build --build-base "/my_ros_data/pa_ws/build/fiducials/aruco_detect" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/my_ros_data/pa_ws/install" --install-scripts="/my_ros_data/pa_ws/install/bin"
