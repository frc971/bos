# BOS README
***
## What is Docker?
###  What does it do? 
Basically, Docker lets you package an application <br>
into a container that can be distributed as images to <br>
run on other platforms. For example, let's say you have <br>
an application that works on your computer, but not on <br>
the server you want to eventually run it on. Docker <br>
solves that. It lets you package an application and <br>
the environment it needs to run on into a bundle that <br>
can then be run anywhere.

## Docker commands you need to know
### How to pull images using docker pull
On an ORINS' terminal, while it has power and ethernet,<br> 
run `docker pull ghcr.io/frc971/bos/orin:latest`.

This will pull a docker image from the bos repository <br>
to make sure the code on the ORIN is up to date.

### How to run the container on the ORIN
To run the docker container that is on the ORIN, <br>
run the following commands in the terminal:
```bash
    docker run --priviliged --network host --name orin -it ghcr.io/frc971/bos/orin:latest
    docker exec -it orin /bin/bash
```
This will run the docker container on the ORIN.
