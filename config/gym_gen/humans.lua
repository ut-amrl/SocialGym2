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
        { -0.27310073375701904, 0.646902322769165, 8 }
    }
