## Usage

```shell
docker run \
    --privileged \
    --device=/dev/ttyACM0 \
    --volume=/dev/ttyACM0:/dev/ttyACM0 \
    --name="ros2_serial" \
    --rm \
    yxsakana/ros2_serial:latest \
    ros2 launch controller_io controller_io.launch.py
```

