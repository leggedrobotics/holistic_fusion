#!/bin/bash

apt update && apt install -y \
  sudo \
  tzdata \
  lsb-release \
  ca-certificates \
  apt-utils \
  gnupg2 \
  locate \
  curl \
  wget \
  git \
  vim \
  gedit \
  tmux \
  unzip \
  iputils-ping \
  net-tools \
  htop \
  iotop \
  iftop \
  nmap \
  software-properties-common \
  build-essential \
  gdb \
  pkg-config \
  cmake \
  zsh \
  clang-format \
  clang-tidy \
  xterm \
  gnome-terminal \
  dialog \
  tasksel \
  meld \
  figlet \
  && \
  rm -rf /var/lib/apt/lists/*