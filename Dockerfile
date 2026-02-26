FROM osrf/ros:humble-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

COPY packages.txt packages.txt
COPY ros_packages.txt ros_packages.txt
COPY requirements.txt requirements.txt

RUN apt-get update && apt-get install -y --no-install-recommends \
    $(cat packages.txt) \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    $(cat ros_packages.txt) \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# libconfig
RUN wget http://web.barrett.com/support/WAM_Installer/libconfig-1.4.5-PATCHED-NOTHROW.tar.gz && \
    tar -xf libconfig-1.4.5-PATCHED-NOTHROW.tar.gz && \
    cd libconfig-1.4.5 && ./configure && make -j$(nproc) && \
    sudo make install && sudo ldconfig

# libbarrett
RUN git clone https://git.barrett.com/software/libbarrett && \
    cd libbarrett && \
    git checkout feature/clearcan-and-nothrow && \
    cmake . && make -j$(nproc) && sudo make install && sudo ldconfig

# realsense
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | \
    tee /etc/apt/keyrings/librealsense.pgp > /dev/null && \
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/librealsense.list && \
    sudo apt-get update && \
    sudo apt-get install -y \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg && \
    sudo apt-get clean && rm -rf /var/lib/apt/lists/*

# python
RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install -r requirements.txt

# sergey nvim stuff :)
RUN git clone https://github.com/sergey-khl/bear_ide.git \
    && cd bear_ide \
    && ./install.sh

RUN echo "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "alias die='tmux kill-session'" >> ~/.bashrc
RUN echo "source /home/user/haptic-imitation/wam_ws/install/setup.bash" >> ~/.bashrc


# formats terminal to look normal, but with a red (fr3-docker) prefix to indicate docker environment
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' ~/.bashrc
RUN echo "export PS1='\[\e[1;31m\](wam-docker) \e[1;32m\]\u@\h\[\e[0m\]:\[\e[1;34m\]\w\[\e[0m\]\$ '" >> /root/.bashrc
RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc
RUN echo "export TERM=xterm-256color" >> ~/.bashrc

# Build alis
RUN echo "alias build_ros='colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'" >> /root/.bashrc
