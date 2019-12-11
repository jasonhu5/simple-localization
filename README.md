# Faster localization but with more assumptions

Remember to: `chmod +x ./packages/[package]/src/[node script]`

### How to build:
```
dts devel build -f --arch amd64
```

### How to run:
Make sure online localization is already running properly.
```
docker run --name localization --network=host -it --rm duckietown/simple-localization:v1-amd64
```

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
