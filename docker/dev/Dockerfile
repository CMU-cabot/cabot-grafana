FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
	git \
	make \
	wget \
	sudo \
	python3 \
	g++

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
        usermod  --uid $UID $USERNAME && \
        groupmod --gid $UID $USERNAME

USER $USERNAME

ENV HOME /home/$USERNAME
WORKDIR $HOME

## install go
RUN wget https://go.dev/dl/go1.21.0.linux-amd64.tar.gz && \
	mkdir -p $HOME/go && \
	tar -C $HOME/go -xzf go1.21.0.linux-amd64.tar.gz
ENV GOROOT $HOME/go/go
ENV GOPATH $HOME/go_projects
ENV PATH $GOROOT/bin:$GOPATH/bin:$PATH
RUN go install github.com/evilmartians/lefthook@latest

## install nvm
RUN wget -qO- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | sh
RUN . $HOME/.profile && nvm install 20

RUN git clone https://github.com/grafana/grafana.git

WORKDIR $HOME/grafana

RUN make lefthook-install
RUN . $HOME/.profile && \
	npm install --global yarn && \
	yarn install --immutable

COPY launch.sh $HOME/grafana

