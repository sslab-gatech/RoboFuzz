1. Install docker-ce

Skip if you already have docker-ce installed on your system.
The following is copied from https://docs.docker.com/engine/install/ubuntu/

```sh
$ sudo apt-get remove docker docker-engine docker.io containerd runc

$ sudo apt-get update
$ sudo apt-get install \
    ca-certificates \
    curl \
    gnupg \
    lsb-release

$ sudo mkdir -p /etc/apt/keyrings
$ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
$ echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

$ sudo apt-get update
$ sudo apt-get install docker-ce docker-ce-cli containerd.io docker-compose-plugin
```

2. Pull our docker image and tag it
```
$ docker pull ghcr.io/sslab-gatech/robofuzz:latest
$ docker tag ghcr.io/sslab-gatech/robofuzz:latest robofuzz
```
(Due to the pre-compiled target robotic systems, the size of the image is
 approximately 20 GB, so docker pull might take a while.)

3. Run docker
```
$ xhost +
$ docker run --rm -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --name robofuzz robofuzz
```

4. Check installation

```sh
root@container_id:/robofuzz/src# ./fuzzer.py --help
```
