from src.environment.rewards import GoalDistance, Success, SocialNormPass, SocialNormOvertake, SocialNormCross


def dsacadrl_rewards():
    return [GoalDistance(), Success(), SocialNormPass(), SocialNormOvertake(), SocialNormCross()]
