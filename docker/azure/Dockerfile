# Build stage
FROM ubuntu:22.04 AS builder

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    git \
    make \
    wget \
    sudo \
    python3 \
    g++ \
    unzip \
    curl \
    && rm -rf /var/lib/apt/lists/*

ENV USERNAME=developer
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

USER $USERNAME
ENV HOME=/home/$USERNAME

RUN wget https://go.dev/dl/go1.21.8.linux-amd64.tar.gz -O /tmp/go1.21.8.linux-amd64.tar.gz && \
    mkdir -p $HOME/go && \
    tar -C $HOME/go -xzf /tmp/go1.21.8.linux-amd64.tar.gz && \
    rm /tmp/go1.21.8.linux-amd64.tar.gz

ENV GOROOT=$HOME/go/go
ENV GOPATH=$HOME/go_projects
ENV PATH=$GOROOT/bin:$GOPATH/bin:$PATH

RUN mkdir -p $GOPATH && chown -R $USERNAME:$USERNAME $GOPATH

RUN go install github.com/evilmartians/lefthook@latest

ENV NODE_VERSION 20.17.0
ENV NVM_DIR /home/developer/.nvm

RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | bash && \
    . $NVM_DIR/nvm.sh && \
    nvm install $NODE_VERSION && \
    nvm alias default $NODE_VERSION && \
    nvm use default

ENV PATH $NVM_DIR/versions/node/v$NODE_VERSION/bin:$PATH

RUN wget -O /tmp/grafana.zip https://github.com/CMU-cabot/grafana/archive/refs/heads/main.zip && \
    unzip /tmp/grafana.zip -d /tmp && \
    mkdir -p $HOME/src && \
    mv /tmp/grafana-main $HOME/src/grafana && \
    rm /tmp/grafana.zip

WORKDIR $HOME/src/grafana

ENV NODE_OPTIONS="--max-old-space-size=8192"

RUN npm install --global yarn && \
    yarn install --immutable

RUN make build
RUN ./bin/linux-amd64/grafana cli plugins install volkovlabs-image-panel

# Runtime stage
FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    ca-certificates \
    curl \
    mysql-client \
    && rm -rf /var/lib/apt/lists/*

RUN curl -o /etc/ssl/certs/DigiCertGlobalRootCA.crt.pem https://dl.cacerts.digicert.com/DigiCertGlobalRootCA.crt.pem

# Copy Grafana files
COPY --from=builder /home/developer/src/grafana/bin/linux-amd64/grafana /usr/local/bin/grafana
COPY --from=builder /home/developer/src/grafana/conf /usr/share/grafana/conf

# Create necessary directories and copy conf
RUN mkdir -p /etc/grafana /var/lib/grafana/data /var/log/grafana && \
    cp -r /usr/share/grafana/conf/* /etc/grafana/

COPY --from=builder /home/developer/src/grafana/public /usr/share/grafana/public
COPY --from=builder /home/developer/src/grafana/data/plugins /var/lib/grafana/plugins

# Set permissions
RUN chown -R nobody:nogroup /etc/grafana /var/lib/grafana /var/log/grafana /usr/share/grafana

# Set up entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Verify final state
RUN echo "Final state:" && \
    ls -la /usr/share/grafana && \
    ls -la /usr/local/bin/grafana && \
    ls -la /var/lib/grafana/plugins && \
    ls -la /etc/grafana

# Copy default configuration
RUN cp /etc/grafana/defaults.ini /etc/grafana/grafana.ini

# Verify the configuration file
RUN ls -la /etc/grafana && head -n 20 /etc/grafana/grafana.ini

USER nobody
WORKDIR /usr/share/grafana

ENV PATH="/usr/local/bin:${PATH}" \
    GF_PATHS_CONFIG="/etc/grafana/grafana.ini" \
    GF_PATHS_DATA="/var/lib/grafana/data" \
    GF_PATHS_HOME="/usr/share/grafana" \
    GF_PATHS_LOGS="/var/log/grafana" \
    GF_PATHS_PLUGINS="/var/lib/grafana/plugins" \
    GF_PATHS_PROVISIONING="/etc/grafana/provisioning"

EXPOSE 3000

ENTRYPOINT ["/entrypoint.sh"]
CMD ["grafana", "server", "--config=/etc/grafana/grafana.ini"]
