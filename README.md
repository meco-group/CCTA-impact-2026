# CCTA Impact Workshop 2026
Numerical Optimization and Model Predictive Control: A Hands On Journey with CasADi and Impact.

## Introduction

This repository contains source of training material for the `Impact Workshop 2026`.

Program to be announced.

## Program
| Session | Content |
|---|---|
| 8:30am - 10:00am | Introduction to workshop <br>CasADi part 1 |
| 10:30am - 12:00am | CasADi part 2 <br>static optimization on Arduino Alvik |
| 1:30am - 3:00pm | Introduction on MPC <br>assignment 1: dynamic optimization on Arduino Alvik |
| 3:30pm - 5:00pm | assignment 2: swingup control and stabilization of a furuta pendulum |

## Software installation

The easiest way to follow and complete all assignments in this hands-on workshop is to bring a machine with linux or WSL, along with Docker.
For vscode users, a dev container can be used to access all the necessary code. After cloning this repository, opening vscode in this repository.
Then, open the command palette (Ctrl + Shift + P) and execute `Dev Containers: Rebuild and Reopen in Container`. This will open a new window with all required software and files. Note that downloading the docker image can take a few minutes.

Instead of using a dev container, you can build your own docker container by navigating to `/software_installation` and running 
```
docker compose up -d
```

This can take a few minutes.
If you prefer not to use docker and do the installation yourself, you can find instructions in `software_installation/README.md`

### Arduino Alvik Assignment (morning session)

Follow the Setup instructions at [docs.arduino.cc](https://docs.arduino.cc/tutorials/alvik/setting-alvik-arduino-ide).

- Install Arduino IDE
- Install Alvik library (From the IDE vertical toolbar: "Library manager" -> search for Alvik)
- Install Arduino Nano ESP32 board (From the IDE vertical toolbar: "Board manager" -> search for esp32)

![me](furuta_pendulum/furuta-ccta-gif.gif)


