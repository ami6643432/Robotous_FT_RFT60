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

echo_and_run cd "/home/mick/Robotous_FT_RFT60/catkin_ft/src/DynamixelSDK/ros/dynamixel_sdk"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/mick/Robotous_FT_RFT60/catkin_ft/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/mick/Robotous_FT_RFT60/catkin_ft/install/lib/python3/dist-packages:/home/mick/Robotous_FT_RFT60/catkin_ft/build/dynamixel_sdk/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/mick/Robotous_FT_RFT60/catkin_ft/build/dynamixel_sdk" \
    "/usr/bin/python3" \
    "/home/mick/Robotous_FT_RFT60/catkin_ft/src/DynamixelSDK/ros/dynamixel_sdk/setup.py" \
     \
    build --build-base "/home/mick/Robotous_FT_RFT60/catkin_ft/build/dynamixel_sdk" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/mick/Robotous_FT_RFT60/catkin_ft/install" --install-scripts="/home/mick/Robotous_FT_RFT60/catkin_ft/install/bin"
