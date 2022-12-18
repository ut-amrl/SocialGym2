# Full Install

Welcome to the Full Install doc!  Here, we will be setting up Social Gym so that each submodule runs on it's
own, allowing you to set up debuggers and step through each C submodule and the python code!

### Requirements

We have only tested this full process on Pycharm and CLion Professional IDEs.  However, what's really needed
is the ability to have a remote source or remote server that you can plug your IDE into.  Specifically, 
we are going to plug our IDE (Pycharm/CLion) into a docker container via SSH.  Our IDE's allow us to do this 
somewhat trivially with run/debug configurations pretty much already set up.  If you do not have this, you may
need to look more stuff up on how to get these to work :/ 

### The Full Install (buckle up!)

#### 1.) From the basic installation guide follow steps 1-5

They are not repeated here in case they change at some time

#### 2.) Build and "up" the docker containers

From the root directory of the project

```shell
docker-compose build && docker-compose up
```

Make sure you see an RVIS window after the `docker-up` command finishes.  If you don't, check your logs for build 
errors.

#### 3.) SSH into the ROS submodules container

```shell
ssh rosdev@localhost -p 2222
```

This is the container where all the C projects will be built and ran.  We are diving into their container now
so we can set some stuff up manually before we plug our IDE into it.

#### 4.) 