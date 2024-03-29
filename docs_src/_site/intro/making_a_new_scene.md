# How to make a new Scene

Scene (or environments) are what the agents are trained on.  They consist of two important objects, the first are walls
(denoted by blue lines).  The next are navigation paths (denoted by pink/purple lines).  These are explained a bit 
below.

---

Walls are pretty straight forward, an agent cannot move through them and they are static.

Navigation graphs act as a local plan for agents to follow.  This is why we are able to reduce our actions to just 0 and
1 (0 for stop and 1 for go), where go implies "follow the navigation graph". A navigation graph is a collection of nodes
connected by straight line edges.  An agent is then given a set of nodes to reach, where the first in the list is the
starting position and each node after should lead the agent to the last defining a trajectory. 

So, for example, an agent may have the set of nodes \[0, 1, 2, 3\] which would mean the agent will initialize at the 
node 0, then traverse the edge connecting nodes 0 and 1, then 1 and 2, then 2 and 3, once the agent has reached node 3
a success observation will be triggered from the simulation.  

You can manually set paths for agents using the ManualScenario class or you can allow the GraphNav ROS Submodule to 
randomly assign valid trajectories to each agent.

When creating a scene, it's important to keep the navigation graph in mind so you can create complex environments 
effectively!

---

## Creating a new environment

We have an easy script for creating brand new scenes!  It also allows editing scenes, or using an existing scene as a
template for a new one.  We'll give a brief intro into how to make a new scene below.

#### 1.) Using your local python interpreter of choice, run
```shell
python scripts/create_env_template --name {NAME YOUR NEW ENV HERE}
```

#### 2.) If you have followed the Installation guide you should see a window pop up (if not double check that Social Gym was installed correctly).

This window is the Walls editor!  It's most likely blank on your screen, but you can add new walls by using 
`[Shift] + [Click] + Mouse Drag`

Remove walls by `[Control] + [Click]`

When you are satisfied with your walls, hit `[Escape]`

#### 3.) A new window should have popped up that looks exactly like the last, but now we are in the Graph Navigation editor!

The controls are the same except for one crucial difference.  In the Graph Navigation editor, you create "Nodes" that 
can connect to each other through "Edges".  You must place "Nodes" first before you can draw an "Edge".

Create a node by hitting `[Shift] + [Click]`, Be advised - the nodes are literally the size of a pixel so they may be hard
to see.

Once two nodes are placed, draw a line between them via `[Shift] + [Click] + Mouse Drag`.  If you did it right, a 
dialogue box will open.  However, it does require that the starting point of the edge and the ending point of the edge
are sufficiently close to the nodes you want to connect for the dialogue box to show.  So if you don't see one, try to 
draw the edge closer to the nodes.

Once the dialogue box has opened, you can simply accept all the default values (Social Gym makes no distinctions on
these values).

To remove nodes or edges use `[Control] + [Click]`

Hit `[Escape]` when done.

#### 4.) Your Scene has been created and the files are in the right spot, but now you have to update the Social Gym code to
reference your new scene.  The easiest way to do this is to define a new `GraphNavScenario` scenario like so

```python
   #...
   scenario = GraphNavScenario(
       "NAME OF YOUR ENVIRONMENT HERE, same as the --name param you passed in before",
   )
  #...
  env = ENV_CLASS(observer=observer, rewarder=rewarder, scenario=scenario, num_humans=0, num_agents=num_agents, debug=debug)
```

In the `config_run.py` file.  A better way of choosing scenes is in progress, but for now you can specify your scenes 
in the config_run file directly.


That's it!

---

NOTE:  If you use the `ManualScenario` class or the `ManualZoneEnv` you will have to distinguish your nodes.  Meaning 
that you need to know which node is which (what is node 0, 1, 2, etc.)

This is because, `ManualScenario` requires that you give it valid paths for your agents to traverse in the navigation
graph.  And the `ManualZoneEnv` requires you to pick two points that define a conflict zone (like two opposite points
on either side of a hallway).  In order to know these, you need to know the values of the nodes on the graph and it's
not guaranteed to be the order that you placed them.

We have a way to tell which node is which via the `CycleScenario` class.  This will effectively place an agent at each
node starting from node 0 to N and it will print which node it is currently on.  We recommend using this scenario to 
build out a map and label your navigation graph (though a better way is being developed).  In order to get `CycleScenario`
to work properly you should use a timeout wrapper and avoid the entropyender wrapper.

---