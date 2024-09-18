FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    git \
    make \
    wget \
    sudo \
    python3 \
    g++ \
    unzip \
    curl

ENV USERNAME developer
# Replace 1000 with your user/group id
ARG UID=1000
RUN useradd -m $USERNAME && \
    echo "$USERNAME:$USERNAME" | chpasswd && \
    usermod --shell /bin/bash $USERNAME && \
    usermod -aG sudo $USERNAME && \
    mkdir -p /etc/sudoers.d/ && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    usermod --uid $UID $USERNAME && \
    groupmod --gid $UID $USERNAME

ENV HOME /home/$USERNAME

USER root

RUN wget https://go.dev/dl/go1.21.0.linux-amd64.tar.gz && \
    mkdir -p /usr/local/go && \
    tar -C /usr/local -xzf go1.21.0.linux-amd64.tar.gz && \
    rm go1.21.0.linux-amd64.tar.gz

ENV GOROOT=/usr/local/go
ENV GOPATH=/home/$USERNAME/go
ENV PATH=$GOROOT/bin:$GOPATH/bin:$PATH

RUN echo "export GOROOT=$GOROOT" >> /home/$USERNAME/.bashrc && \
    echo "export GOPATH=$GOPATH" >> /home/$USERNAME/.bashrc && \
    echo "export PATH=$PATH" >> /home/$USERNAME/.bashrc

RUN mkdir -p $GOPATH && chown -R $USERNAME:$USERNAME $GOPATH

RUN go version

USER $USERNAME

# Install nvm
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | bash

# Install Node.js using nvm
RUN bash -c "source $HOME/.nvm/nvm.sh && nvm install 20"

# Set NODE_OPTIONS for memory limit
ENV NODE_OPTIONS="--max-old-space-size=4096"

# Download and set up Grafana
RUN wget -O /tmp/grafana.zip https://github.com/CMU-cabot/grafana/archive/refs/heads/main.zip && \
    unzip /tmp/grafana.zip -d /tmp && \
    mkdir -p $HOME/src && \
    mv /tmp/grafana-main $HOME/src/grafana && \
    rm /tmp/grafana.zip
RUN sudo chown -R $USERNAME:$USERNAME $HOME/src/grafana

WORKDIR $HOME/src/grafana

# Install Yarn and build the project
RUN bash -c "source $HOME/.nvm/nvm.sh && npm install --global yarn && yarn install --immutable"

# Ensure Go is available in the environment and build the project
RUN bash -c "source $HOME/.bashrc && source $HOME/.nvm/nvm.sh && echo 'Building the project...' && make build -j1 VERBOSE=1 && echo 'grafana cli install plugin ...' && ./bin/linux-amd64/grafana cli plugins install volkovlabs-image-panel"

EXPOSE 3000

CMD ["/home/developer/src/grafana/bin/linux-amd64/grafana", "server"]
