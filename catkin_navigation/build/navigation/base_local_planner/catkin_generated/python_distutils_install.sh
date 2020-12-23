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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/pengyang/catkin_navigation/src/navigation/base_local_planner"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/pengyang/catkin_navigation/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/pengyang/catkin_navigation/install/lib/python2.7/dist-packages:/home/pengyang/catkin_navigation/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/pengyang/catkin_navigation/build" \
    "/usr/bin/python2" \
    "/home/pengyang/catkin_navigation/src/navigation/base_local_planner/setup.py" \
    build --build-base "/home/pengyang/catkin_navigation/build/navigation/base_local_planner" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/pengyang/catkin_navigation/install" --install-scripts="/home/pengyang/catkin_navigation/install/bin"
