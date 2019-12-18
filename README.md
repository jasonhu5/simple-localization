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

___NOTICE___: Only offset measured watchtowers contribute to publishing fast localization results.

* [Required] Offset measurement for watchtowers.
    1. Enter a dt-ros-commons container: `docker run -it --rm --net host duckietown/dt-ros-commons:daffy-amd64 /bin/bash` 
    * (Optional step) Tell the program which autobot is used for measurements by: `rosparam set /simple_localization/loc_node/offset_bot_id [AUTOBOT_ID]`. Otherwise, it's autobot26 by default.
    2. Enter measurement mode with: `rosparam set /simple_localization/loc_node/measuring_offset true`
    3. Choose the watchtower to do measurement for: `rosparam set /simple_localization/loc_node/offset_tower_id [WATCHTOWER_ID]`
        * After above step, the watchtower in RVIZ appears as: ![alt text](https://raw.githubusercontent.com/jasonhu5/simple-localization/v1/figures/watchtower_offset_being_measured.png?token=ACTBVS4USGSZCMTHMQYNIKK572TRA)
    4. Place autobot near the watchtower that you want to do the measurement for, and move the bot a bit so the watchtower sees it. And then leave the bot still.
    5. Check the diff values printed from the program container are nearly zero, commit the measurement by either: 
        * Exiting measurement mode: `rosparam set /simple_localization/loc_node/measuring_offset false`, or
        * switch to another watchtower for measurement: `rosparam set /simple_localization/loc_node/offset_tower_id [ANOTHER_WATCHTOWER_ID]`
        * After commiting the measurement, the watchtower(s) that have been measured will appear like: ![alt text](https://raw.githubusercontent.com/jasonhu5/simple-localization/v1/figures/watchtower_offset_compensated.png?token=ACTBVSYJWYB257OANQNIOXK572TRK)
    6. Choose other watchtower(s) and repeat steps 3-5.
    * Watchtower numbers in AMoD lab can be referred to here: ![alt text](https://raw.githubusercontent.com/jasonhu5/simple-localization/v1/figures/watchtowers_map.png?token=ACTBVS3JOZPTQPG5A6K2ZK2572TRW)
 

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
    data: [26.0, 2.8390284122793625, 3.050242414056867, -50.08818199367434, 1576363992.0, 919081926.0]
    ---
    ```
    * data[0]: bot_tag_number
    * data[1]: x
    * data[2]: y
    * data[3]: rotation angle
    * data[4]: timestamp seconds
    * data[5]: timestamp nanoseconds

### How to visualize results:

In online localization rviz window, add by topic, the `/simple_loc` Markers.

