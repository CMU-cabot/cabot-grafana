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

ENV HOME=/home/$USERNAME

# Install Go
RUN wget https://go.dev/dl/go1.21.0.linux-amd64.tar.gz && \
    tar -C /usr/local -xzf go1.21.0.linux-amd64.tar.gz && \
    rm go1.21.0.linux-amd64.tar.gz

ENV GOROOT=/usr/local/go
ENV GOPATH=/home/$USERNAME/go
ENV PATH=$GOROOT/bin:$GOPATH/bin:$PATH

RUN mkdir -p $GOPATH && chown -R $USERNAME:$USERNAME $GOPATH

# Install Node.js and corepack
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - && \
    apt-get install -y nodejs && \
    npm install -g corepack && \
    corepack enable

USER $USERNAME

ENV NODE_OPTIONS="--max-old-space-size=8192"

# Download and set up Grafana
RUN wget -O /tmp/grafana.zip https://github.com/CMU-cabot/grafana/archive/refs/heads/main.zip && \
    unzip /tmp/grafana.zip -d /tmp && \
    mkdir -p $HOME/src && \
    mv /tmp/grafana-main $HOME/src/grafana && \
    rm /tmp/grafana.zip

WORKDIR $HOME/src/grafana

# Build Grafana
RUN yarn install --immutable && \
    make build -j1 VERBOSE=1 && \
    ./bin/linux-amd64/grafana cli plugins install volkovlabs-image-panel

# Check the contents of the build stage
RUN echo "Build stage: Content of $HOME/src/grafana:" && \
    ls -la $HOME/src/grafana && \
    echo "Build stage: Content of $HOME/src/grafana/conf:" && \
    ls -la $HOME/src/grafana/conf && \
    echo "Build stage: Tree structure of $HOME/src/grafana:" && \
    find $HOME/src/grafana -type d


# Runtime stage
FROM ubuntu:22.04

# Copy Grafana files
COPY --from=builder /home/developer/src/grafana/bin/linux-amd64/grafana /usr/local/bin/grafana
COPY --from=builder /home/developer/src/grafana/conf /usr/share/grafana/conf
RUN echo "After copying conf:" && \
    ls -la /usr/share/grafana && \
    ls -laR /usr/share/grafana/conf

# Create necessary directories and copy conf
RUN mkdir -p /etc/grafana /var/lib/grafana /var/log/grafana && \
    cp -r /usr/share/grafana/conf/* /etc/grafana/ && \
    echo "After creating directories and copying conf:" && \
    ls -la /etc/grafana && \
    ls -la /var/lib/grafana && \
    ls -la /var/log/grafana

COPY --from=builder /home/developer/src/grafana/public /usr/share/grafana/public
COPY --from=builder /home/developer/src/grafana/plugins-bundled /var/lib/grafana/plugins

# Set permissions
RUN chown -R nobody:nogroup /etc/grafana /var/lib/grafana /var/log/grafana /usr/share/grafana && \
    echo "After setting permissions:" && \
    ls -la /etc/grafana && \
    ls -la /var/lib/grafana && \
    ls -la /var/log/grafana && \
    ls -la /usr/share/grafana

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
    GF_PATHS_DATA="/var/lib/grafana" \
    GF_PATHS_HOME="/usr/share/grafana" \
    GF_PATHS_LOGS="/var/log/grafana" \
    GF_PATHS_PLUGINS="/var/lib/grafana/plugins" \
    GF_PATHS_PROVISIONING="/etc/grafana/provisioning"

EXPOSE 3000

ENTRYPOINT ["/entrypoint.sh"]
CMD ["grafana", "server", "--config=/etc/grafana/grafana.ini"]
