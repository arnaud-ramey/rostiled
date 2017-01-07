rostiled
=======

[![Build Status](https://travis-ci.org/arnaud-ramey/rostiled.svg)](https://travis-ci.org/arnaud-ramey/rostiled)

An integration of tiled within ROS.

Licence
=======

BSD


Authors
=======

  - Package maintainer: Arnaud Ramey (arnaud.a.ramey@gmail.com)

Compile and install
===================

ROS Kinetic + catkin
-------------------

Compile with [catkin_make](http://wiki.ros.org/catkin/commands/catkin_make):

```bash
$ rosdep install rostiled --ignore-src
$ catkin_make --only-pkg-with-deps rostiled
```

Run
===

```bash
$ rosrun rostiled rostiled.exe
```

Parameters
==========

 * ```~ivona_credentials``` [std_msgs/String]
  a text file containing two lines,
  the first being the access key, the second the secret key.

Subscriptions
=============

 * ```/tts``` [std_msgs/String]
 Sentences to be said.

Publications
============

None.
