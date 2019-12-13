# Faster localization but with more assumptions

Remember to: `chmod +x ./packages/[package]/src/[node script]`

### How to build:
```
dts devel build -f --arch amd64
```

### How to run:
Make sure online localization is already running properly.
* Start the container
```
docker run --name localization --network=host -it --rm duckietown/simple-localization:v1-amd64
```
* [Optional but recommended] Offset measurement for watchtowers.
    1. Enter a dt-ros-commons container: `docker run -it --rm --net host duckietown/dt-ros-commons:daffy-amd64 /bin/bash` 
    2. Place autobot26 near the watchtower that you want to do the measurement for.
    3. Trigger the measurement: `rosparam set /simple_localization/loc_node/measuring_offset true`
    4. Move the bot a bit so the watchtower sees it, and then leave the bot still.
    5. When the diff values printed from the program container are stable, turn off measurement: `rosparam set /simple_localization/loc_node/measuring_offset false`
    6. Verify by moving the bot a bit, check that the diff values are much closer to 0.
    7. Move the bot to other watchtower(s) and repeat steps 2-4. Multiple watchtowers can be corrected at one time, but use visualization to verify.
 

### Node I/O:
![alt text](https://raw.githubusercontent.com/jasonhu5/simple-localization/v1/figures/rqt_graph.png?token=ACTBVS4TTLVHI5VIIXCCPHS57KUPQ)
* `/simple_loc` topic outputs `visualization_msgs.Marker` messages
    * The markers are used for result verification
* `/simple_loc_bots` topic outputs `std_msgs.Float64MultiArray` messages
    * The output is like following format
    ```
    ---
    layout:
    dim: []
    data_offset: 0
    data: [426.0, 2.930640010309501, 3.183142511143741, 24.491974855898427]
    ---
    ```
    * data[0]: bot_tag_number
    * data[1]: x
    * data[2]: y
    * data[3]: rotation angle

### How to visualize results:

On server desktop, open terminal, type `rviz`. In rviz window, add by topic, the `/simple_loc` Markers.
