# Advanced Usage

---

There's a FAQ at the bottom (maybe worth reading first).  Expect this process to take about 3-6 hours :/ 

**WARNING**: The authors of this project have had numerous issues with Docker and Docker-Compose when installed via snap.  We
recommend uninstalling docker and docker-compose if you have GPU-related issues with the docker images and installing them via
`apt`.

---

Welcome to the Advanced Usage doc!  Here, we will be setting up Social Gym so that each submodule runs on it's
own, allowing you to set up debuggers and step through each C submodule and the python code!

## Requirements

We have only tested this full process on Pycharm and CLion Professional IDEs.  However, what's really needed
is the ability to have a remote source or remote server that you can plug your IDE into.  Specifically, 
we are going to plug our IDE (Pycharm/CLion) into a docker container via SSH.  Our IDE's allow us to do this 
somewhat trivially with run/debug configurations pretty much already set up.  If you do not have this, you may
need to look more stuff up on how to get these to work :/ 

## Advanced Usage setup (buckle up!)

We've broken this tutorial into 3 parts, the first is the basic setup, the second will get the submodules
running, and the final part will focus on the python code and running the actual simulator end to end.

### Part 1: Setting up the basic Social Gym dependencies and containers

#### 1.) From the basic installation guide follow steps 1-5

They are not repeated here in case they change at some time

#### 2.) Build and "up" the docker containers

From the root directory of the project

```shell
docker-compose build && docker-compose up
```

Make sure you see an RVIS window after the `docker-up` command finishes.  If you don't, check your logs for build 
errors.

#### 3.) Open the submodule projects `ut_multirobot_sim` and `graph_navigation` in a separate IDE window (I use CLion for these)

Make sure you are opening them from the directory `{PROJECT_ROOT}/submodules/ut_multirobot_sim` and 
`{PROJECT_ROOT}/submodules/graph_navigation` -- we will be running the submodules from these windows (you could also
run them from the ssh terminal in the subsequent steps)

### Part 2: Setting up the submodules

This part of the setup was heavily inspired by https://github.com/nocoinman/ros-docker-clion -- big shoutout to this
project :)

#### 1.) SSH into the ROS submodules container

```shell
ssh rosdev@localhost -p 2222
yes
12345
```

The password for the container is `12345` for all containers - it can be changed in the dockerfiles found in 
`{PROJECT_ROOT}/docker`

This is the container where all the C projects will be built and ran.  We are diving into their container now
so we can set some stuff up manually before we plug our IDE into it.

#### 2.) Build static dependencies.

There are a few submodules that really just contain libraries or code that does not need to be ran.  We still
need to compile them so that we can reference them in the other sobumodules.

```shell
source ~/set_paths.sh
rosdep update
cd social_gym/submodules/amrl_msgs
make
cd ../pedsim_ros
catkin_make
cd ../
```

Make sure `rosdep udpate` succeeds! If it fails, you won't be able to build the submodules.

#### 3.) Build the Graph Navigation submodule

This submodule handles low level navigation for the agents.

Assuming you are starting from the `{PROJECT_ROOT}/submodules` directory
```shell
cd graph_navigation
make
```

Expect a small delay for this make, maybe 5-10m depending on your machine.

---

**NOTE**: If at any part you get an error, especially with a submodule, it's usually a good idea to run
`make clean` before re-running `make`.

---

#### 4.) Setup Graph Navigation with your IDE

If you have a different IDE other than CLION Commercial or Professional find out how to configure a remote host via
ssh.  Some of this will be relevant (like setting global env variables) but most of it may be CLION specific.

Before following the steps below, run 

```shell
~/get_vars.sh
```

In the SSH terminal and copy the output. It should look similar to 
`ROS_ROOT=/opt/ros/noetic/share/ros;ROS_MASTER_URI=...` and be very long.  This is important to have ready and we will
use it to set environment variables for the IDE.

1.) In CLION go to `Settings`

2.) Select `Build,Execution,Deployment < Toolchains`

3.) Add a new Toolchain via the + symbol and make it a `remote host`

4.) Select the cog icon next to the `Credentials` drop down.

5.) SSH Configurations window should have opened and you can fill out the form, specifically set the Host to `localhost`
the username to `rosdev`, the port to `2222`, and the password to `12345`.  Test your connection to make suer you filled
it out correctly then hit Apply and OK.

6 \[Optional\].) Name the remote host something memorable 

7.) Now navigate the settings window to `Build,Execution,Deployment < CMake`

8.) Here, copy the `Debug` profile and name it something like `Remote Debugger`

9.) In the `Toolchain` dropdown, select your new remote host you just created. (This is why a unique name is useful)

10.) Towards the bottom of the profile configuration for the new remote debugger, paste the environment variables we 
saved in our clipboard into the `Environment` text box. 

11.) Click apply and OK 

12.) Now in the settings window navigate to `Build,Execution,Deployment < Deployment`

13.) You should see your new remote host in the `remote development` panel, select it and move to the `mappings` tab in
the right most panel.

14.) In the `Local path:` box place the full path to your graph navigation submodule i.e. 
`{YOUR PATH}/social_gym/submodules/graph_navigation`

15.) In the `Deployment Path:` fill in `/home/rosdev/social_gym/submodules/graph_navigation` (you might have tmp 
something in there currently, make sure to delete that.)

16.) Click OK in the settings window and go back to your terminal with the SSH connection.

17.) Run 

```shell
sudo rm -r cmake-build-debug/
sudo rm -r cmake-build-remotedebugger/
```

remember the password is `12345` if you have not changed it.

18.) Move to the CMake tab in the CLION idea (usually by the terminal tab on the bottom of your IDE) and click the 
refresh button to rebuild your cmake project.  (It should produce two tabs, `Debug` and `RemoteDebugger`, `Debug` will
FAIL but the `RemoteDebugger` tab should SUCCEED).

19.) You should now have a list of run configurations found near the play button on the top of your IDE.  Click the drop
down and select `social_nav`.

20.) Now click the dropdown again and edit the currently selected config.  In the `Environment Varaibles` text box paste
the environment variables we copied before starting this process.

Set the `working directory` to `/home/rosdev/social_gym/submodules/graph_navigation`

Set the `Program arguments` to `-service_mode=true -map=exp2/train/easy` 

---

*NOTE*: If you are going to be using custom maps, this is one of the places you'll have to update the name (`-map`)

21.) Hit Apply and OK then hit Run.  You should see the IDE build the project and eventually produce 

```shell
/home/rosdev/social_gym/submodules/graph_navigation/cmake-build-remotedebugger/../bin/social_nav -service_mode=true -map=exp2/train/easy
Loading /home/rosdev/social_gym/submodules/amrl_maps/exp2/train/easy/exp2/train/easy.navigation.json...
Loaded /home/rosdev/social_gym/submodules/amrl_maps/exp2/train/easy/exp2/train/easy.navigation.json with 18 states, 17 edges
```

If you see anything other than what's shown here (except for maybe vertices counts) double check your configuration and 
make sure you have the right variables set!

22.) Congratulations you now have a running Graph Navigation project that you can set breakpoints on using CLION :)


#### 5.) Setup UT Multirobot Sim with your IDE

For this next step I like creating a new terminal in a new IDE window, but that's entirely up to you.

Keep those environment parameters we copied from the Graph Navigation setup for this step as well!

I am going to go a bit faster here since almost all the steps are repeating what we've already done for Graph Nav.

1.) Navigate to your UT_Multirobot_sim IDE window and open a new terminal with a new ssh connection into the same 
container `ssh rosdev@localhost -p 2222` (password is `12345` unless you changed it)

2.) Set up the shell
```shell
cd social_gym/submodules/ut_multirobot_sim/
```

3.) Edit the source file to include a new submodule
```shell
vim ~/set_paths.sh
```
Uncomment the second line
write and quit

4.) Source variables.
```shell 
source ./set_paths.sh
```

5.) Update rosdep
```shell
rosdep update
```

6.) Make the simulator
```shell
make
```

---

**NOTE**: The next steps assume you've followed the Graph Navigation setup meaning that your remote host toolchain is
already created

---

7.) Open the settings window `file < settings`

8.) Navigate to `Build,Execution,Deploy < CMake` and copy your `Debug` profile and then rename the copied debug profile
to something memorable (I use `RemoteDebugger`)

9.) In the `Environment` text' box paste your environment variables (remember you can get them by running `~/get_vars.sh`)

10.) Change the toolchain dropdown to the remote host toolchain you created in the Graph Navigation setup.

11.) Click apply then navigate to `Build,Execution,Deployment < Deployment` and navigate to the `Mappings` tab for your 
remote host toolchain. Update the `Deployment Path` textbox to the value 
`/home/rosdev/social_gym/submodules/ut_multirobot_sim`

12.) Click Apply and OK to close the settings window.

13.) Clean the projects CMAKE configs

```shell
sudo rm -r cmake-build-debug/
sudo rm -r cmake-build-remotedebugger/
```

14.) Go to the CMake tab on your IDE (by the terminal window tab)

15.) Click the refresh button 

16.) You should now have Run Configurations you can select in the dropdown by the run button in your IDE.  Go to it and
find the run config `simulator` (it may be defaulted to that value)

17.) Click the dropdown again and edit your configuration

18.) Update the configuration values

`Program arguments:` should have `-sim_config /home/rosdev/social_gym/config/gym_gen/sim_config.lua -scene_config /home/rosdev/social_gym/config/gym_gen/scene.xml -speedup_factor 1.0 --localize --use_pedsim`

`Working Directory` should have `/home/rosdev/social_gym/submodules/ut_multirobot_sim`

`Environment Variables` should have the pasted environment variables from `~/get_vars.sh`

19.) Click the Run button

20.) You should see the project rebuild and then an error message

```shell
/home/rosdev/social_gym/submodules/ut_multirobot_sim/cmake-build-debug-docremotetest/../bin/simulator -sim_config /home/rosdev/social_gym/config/gym_gen/sim_config.lua -scene_config /home/rosdev/social_gym/config/gym_gen/scene.xml -speedup_factor 1.0 --localize --use_pedsim

UT Multi-Robot Simulator

Publish GoAlone message
Publish GoAlone message
ERROR: Unable to load map maps/closed/door/t1/closed/door/t1.vectormap.txt
Run-time stats for Pass : mean run time = -nan ms, invocations = 0
Run-time stats for Follow : mean run time = -nan ms, invocations = 0
Run-time stats for GoAlone : mean run time = 0.017613 ms, invocations = 2
Run-time stats for Halt : mean run time = -nan ms, invocations = 0
Run-time stats for Publishing Halt : mean run time = -nan ms, invocations = 0
Run-time stats for Step : mean run time = -nan ms, invocations = 0
Run-time stats for StepUTMRS : mean run time = -nan ms, invocations = 0
Run-time stats for StepPedsim : mean run time = -nan ms, invocations = 0

Process finished with exit code 1
```

This is okay! This means the project was built correctly but the current configuration in the python project
is incorrect, which is fine since we haven't touched it yet.  We are close to the end now :)

21.) Congratulations, you now have a UT Multirobot Sim project you can place breakpoints on and debug with.  Of course
to actually use it, we have to update the python project a little bit and run it -- then the ut multirobot sim will 
work correctly.


### Part 3: Setting up Python and Running everything!

#### 1.) Open up Pycharm to the root folder of the project

#### 2.) Set up your IDE with the container via SSH (Pycharm Specific)

Similar to the UT Multirobot Sim and Graph Navigation guides, we assume your IDE has remote host available
and if not, there's some useful stuff here but you may have to translate the steps for your ide.

1.) Open the settings window `file < settings`

2.) Navigate to `tools < ssh configurations`

3.) Create a new ssh configuration exactly the same way we did for Graph Navigation EXCEPT make the port
`2223` (this is important!)

4.) Click Apply and navigate the settings window to `Project < interpreter` create a new interpreter by clicking the cog
icon and hitting `add new` then in the types of interpreters you can select on the left most panel select the 
`SSH Terminal` option

5.) Select existing ssh terminal and choose the configuration you just made in the previous steps

6.) Finish the guide for setting up the interpreter and then double check that you see python packages installed in 
the interpreter window.

7.) In the interpreter window select the `path mappings` text box and edit it.  Make sure to set the local path to your
project root and the mapped path to the location `/home/rosdev/social_gym` (you can find this through the folder 
explore to ensure there are no typos)

8.) Open the file `config_run.py` and try to run it (so that it will create the run config, it should error for now)

9.) Edit the run config that was just made

`Parameters` should have `-c ./tmp_config.json`

`Environment variables` should have the pasted env vars we keep getting in the previous steps.

10.) Open a new ssh terminal 

```shell
ssh rosdev@localhost -p 2223
cd ./social_gym/submodules
vim ~/set_paths.sh
# uncomment the second line then save the file
source ~/set_paths.sh
rosdep update
pip install sb3_contrib
pip install supersuit==3.5.0
pip install gym==0.26.2
```

11.) Click run in your pycharm editor. It should hang around the input

```shell
...
Waiting on UTMRS
[WARN] [1671386774.168961]: wait_for_service(/utmrsStepper): failed to contact, will keep trying
QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-rosdev'
[ INFO] [1671386774.188148416]: Updated sim with live config: Rate=40 incoming rate=40
[ INFO] [1671386774.192522299]: Using default queue size of 10 for publisher queues... 
[ INFO] [1671386774.199740381]: Loading scene [/home/rosdev/social_gym/config/gym_gen/scene.xml] for simulation
[ INFO] [1671386774.204534827]: node initialized, now running 
Service Mode
```

This means we can fire up the UT Multirobot simulator now

12.) Run your ut multirobot sim simulator run config.

13.) If the python program crashed, rerun it.

14.) You should now see cubes loaded into the RVIS window after awhile, and eventually they'll start
moving around!

## FAQ

---

**All my files are missing!**: If at any point your files are missing or empty check out your path mappings for your 
project and make sure they are valid.  Then, do `git stash` for every project that has missing or empty files

This includes SUB folders as well! i.e. `social_gym/ut_multirobot_sim/submodules/config_reader`

You will get tons of compile errors when you make a submodule saying "geometry" is undefined etc.  This means you have
to do `git stash` anywhere a thirdparty source is being used.  

**ROSDEP errors**: If you see anything during the install about ROSDEP not working correctly, make sure you are doing 

```shell
source ~/set_paths.sh
rosedep update
```

often -- once you have built pedsim via `catkin_make`, you should remove the comment on the 2nd line of `~/set_paths.sh`
then rerun the commands above. This is also pretty important for when you run `get_vars.sh`, since that commented out
line has env variables you want your IDE to know.

**CMAKE errors**: CMake is not fun.  If you see weird issues with cmake not building or not finding stuff make sure you
are removing the cmake folders then rebuilding.

**This is hard**: Yep... it's an art, and you should expect this to take at least 3-6hours if it's your first time :( 
I would like to make this process easier, but it's difficult to automate... maybe a video at some point?  Open to
suggestions!! I promise it wasn't made difficult intentionally lol :)



