# Faster localization with more assumptions

Remember to: `chmod +x ./packages/[package]/src/[node script]`


How to build:
```
dts devel build -f --arch amd64
```

How to run:
```
docker run --name localization --network=host -it --rm -e ROS_MASTER_IP=http://<LAB_SERVER_IP>:11311 duckietown/simple-localization:v1-amd64
```

For example:
```
docker run --name localization --network=host -it --rm duckietown/simple-localization:v1-amd64
```