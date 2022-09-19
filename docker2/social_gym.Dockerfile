FROM social_gym_social_gym_image

ENV DISPLAY=:0

#ENTRYPOINT ["/bin/bash", "-c", "cd /home/rosdev/social_gym && source /home/rosdev/set_paths.sh && \"$@\"", "-s"]
ENTRYPOINT ["/bin/bash", "/home/rosdev/entrypoint.sh"]
CMD ["/usr/sbin/sshd", "-D", "-e", "-f", "/etc/ssh/sshd_config_test_clion"]