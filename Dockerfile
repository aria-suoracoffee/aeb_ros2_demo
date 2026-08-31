# Reproducible ROS 2 Humble environment for the AEB demo.
#   docker build -t aeb_demo .
#   docker run -it --rm --shm-size=1g -v "$PWD":/ws -w /ws aeb_demo bash
FROM ros:humble

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-pytest \
    && rm -rf /var/lib/apt/lists/*

# Source ROS (and the workspace overlay, once built) in every shell.
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
 && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc

WORKDIR /ws
SHELL ["/bin/bash", "-c"]
CMD ["bash"]
