FROM gitlab.laas.fr:4567/humanoid-path-planner/hpp-doc/isae-2020
RUN apt-get update && apt-get install -y firefox vim
RUN echo 'source /home/langlade_louahadj/config.sh' >> /root/.bashrc
WORKDIR /home
