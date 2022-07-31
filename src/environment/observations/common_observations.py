from src.environment.observations import AgentsPose, AgentsVelocity, AgentsHeadingDirection, AgentsOthersDistance,\
    OthersHeadingDirection, OthersVelocities, OthersPoses, AgentsPreferredVelocity, AgentsGoalDistance


def dsacadrl():
    """
    Helper for building the observations needed for the D-Sacadrl Policy.
    """

    return [
        AgentsGoalDistance(),
        AgentsPose(),
        AgentsVelocity(),
        AgentsHeadingDirection(),
        AgentsOthersDistance(),
        AgentsPreferredVelocity(preferred_velocity=1.0),
        OthersPoses(),
        OthersVelocities(),
        OthersHeadingDirection()
    ]
