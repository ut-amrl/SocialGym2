FROM social_gym_social_gym_image

WORKDIR ${ROS_WS}/submodules/ut_multirobot_sim

RUN chmod -R 777 ./

ENTRYPOINT ["/bin/bash", "/home/rosdev/entrypoint.sh"]
CMD ["/usr/sbin/sshd", "-D", "-e", "-f", "/etc/ssh/sshd_config_test_clion"]