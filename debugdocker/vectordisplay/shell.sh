#!/bin/bash
sudo docker rm -f VECTORDISPLAY_shell

sudo docker run -d --name VECTORDISPLAY_shell -w /root/vector_display vector_display:custom bash -c 'python -c "while True: print(1)"'
docker exec -it VECTORDISPLAY_shell /bin/bash
sudo docker rm -f VECTORDISPLAY_shell
