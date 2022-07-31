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
        { -1.6, 16.24, 2 }
    }

    hu1_waypoints = {
        { -13.68, 6.36, 3 }
    }

    hu2_waypoints = {
        { -24.39, 16.1, 24 }
    }

    hu3_waypoints = {
        { -36.1, 20.65, 15 }
    }

    hu4_waypoints = {
        { 9.9, 10.35, 12 }
    }

    hu5_waypoints = {
        { -13.71, 16.24, 6 }
    }

    hu6_waypoints = {
        { 29.15, 15.69, 15 }
    }

    hu7_waypoints = {
        { 34.3, 7.04, 12 }
    }

    hu8_waypoints = {
        { -36.1, 18.18, 19 }
    }
