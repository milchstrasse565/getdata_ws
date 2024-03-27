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

echo_and_run cd "/home/lmk/getdata_ws/src/vision_opencv-noetic/image_geometry"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/lmk/getdata_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/lmk/getdata_ws/install/lib/python3/dist-packages:/home/lmk/getdata_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/lmk/getdata_ws/build" \
    "/usr/bin/python3" \
    "/home/lmk/getdata_ws/src/vision_opencv-noetic/image_geometry/setup.py" \
    egg_info --egg-base /home/lmk/getdata_ws/build/vision_opencv-noetic/image_geometry \
    build --build-base "/home/lmk/getdata_ws/build/vision_opencv-noetic/image_geometry" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/lmk/getdata_ws/install" --install-scripts="/home/lmk/getdata_ws/install/bin"
