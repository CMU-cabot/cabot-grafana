# cabot-grafana



## build

```
./setup-dependency.sh
./build-docker.sh -i -w
```

## launch cabot grafana clients

```
# modify .env file
./launch.sh
```

### Environment variables (.env)

```
CABOT_ANCHOR_FILE                  # cabot_site anchor yaml file for converting ROS coordinate to world coordinate
CABOT_INFLUXDB_ROBOT_NAME          # robot name like cabot1
CABOT_INFLUXDB_IMAGE_LEFT_TOPIC    # topic for left camera
CABOT_INFLUXDB_IMAGE_CENTER_TOPIC  # topic for center camera
CABOT_INFLUXDB_IMAGE_RIGHT_TOPIC   # topic for right camera
CABOT_INFLUXDB_POSE_INTERVAL       # <float> interval for pose
CABOT_INFLUXDB_CMD_VEL_INTERVAL    # <float> interval for cmd vel
CABOT_INFLUXDB_ODOM_INTERVAL       # <float> interval for odom
CABOT_INFLUXDB_DIAG_AGG_INTERVAL   # <float> interval for diagnostics agg
CABOT_INFLUXDB_IMAGE_INTERVAL      # <float> interval for image
CABOT_INFLUXDB_HOST                # host like http://localhost:8086
CABOT_INFLUXDB_TOKEN               # token
CABOT_INFLUXDB_ORG                 # org
CABOT_INFLUXDB_BUCKET              # bucket
```

## debug multiple robot grafana visualization with a single client
- set `CABOT_INFLUXDB_ROBOT_NAME` to comma separated names such as `cabot1,cabot2,cabot3`
  - The client will generate fake data for each robot name
  - Robot location will be shifted towards North East direction

## launch local grafana(modified)/influxDB for development
- There is [a fork of grafana in CMU-cabot](https://github.com/CMU-cabot/grafana), which is modified for visualizing multiple path in a Geomap layer
- Install [volkovlabs-image-panel](https://github.com/VolkovLabs/volkovlabs-image-panel) to show BASE64 encoded images in the panel

```
# first terminal (launch grafana and influxdb)
sudo chown -R 472:472 grafanadb
./launch.sh -s -d -i # first time only
./launch.sh -s -d
# second terminal (launch cabot_influxdb_client)
./launch.sh
```


## launch local grafana/influxDB for development
- cannot show multiple robot pathes
- cannot show camera images

```
# first terminal (launch grafana and influxdb)
sudo chown -R 472:472 grafanadb
./launch.sh -s -i # first time only
./launch.sh -s
# second terminal (launch cabot_influxdb_client)
./launch.sh
```

## Browse InfluxDB 

open [http://localhost:8086](http://localhost:8086)

## Browse Grafana

open [http://localhost:3000](http://localhost:3000)


## Trouble shooting

- If you change grafana server (origin / forked), you may need to delete `grafanadb` content and `grafanaconfig/api-key.txt` and start with `-i` option again