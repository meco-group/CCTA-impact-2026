# CCTA Impact Workshop 2026
Numerical Optimization and Model Predictive Control: A Hands On Journey with CasADi and Impact.

## Introduction

This repository contains source of training material for the `Impact Workshop 2026`.

Program to be announced.

## Software installation

The easiest way to follow and complete all assignments in this hands-on workshop is to bring a machine with linux or WSL, along with Docker.

### Furuta Pendulum Assignment
This repo contains a `Dockerfile` and a `docker-compose.yml`. To pull the required image, navigate to `furuta_pendulum` and run

```
docker compose up -d
```

This can take a few minutes. Within the container, navigate to /home/furuta-stepper-cpp/.

If you prefer not to use docker and do the installation yourself, you can find instructions in `furuta_pendulum/README.md`
