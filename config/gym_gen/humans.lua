function Vector2(x, y)
    return {x = x, y = y}
end

-- Human shape information
hu_radius = 0.2
hu_num_segments = 20

-- Human speed information
hu_max_speed = 0.8 --
hu_avg_speed = 0.8 --
hu_max_omega = 0.2
hu_avg_omega = 0.
hu_reach_goal_threshold = 0.1

-- Human walking mode
local HumanMode = {
    Singleshot=0,
    Repeat=1,
    Controlled=2,
    Cycle=3,
}

hu_mode = HumanMode.Controlled
hu_control_topic = "/command"


    hu0_waypoints = {
        { -20.78, 10.35, 4 }
    }

    hu1_waypoints = {
        { -19, 6.36, 11 }
    }

    hu2_waypoints = {
        { -6.38, 6.38, 14 }
    }

    hu3_waypoints = {
        { -10.06, 6.36, 22 }
    }

    hu4_waypoints = {
        { 0.74, 10.35, 23 }
    }

    hu5_waypoints = {
        { 15.09, 10.35, 19 }
    }

    hu6_waypoints = {
        { -24.39, 16.1, 13 }
    }

    hu7_waypoints = {
        { 36.124, 15.972, 24 }
    }

    hu8_waypoints = {
        { -1.45, 10.35, 19 }
    }

    hu9_waypoints = {
        { -20.78, 10.35, 10 }
    }

    hu10_waypoints = {
        { -13.71, 16.24, 24 }
    }

    hu11_waypoints = {
        { -10.06, 6.36, 11 }
    }

    hu12_waypoints = {
        { 15.09, 10.35, 6 }
    }
