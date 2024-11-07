# cabot-grafana

## prerequisite

- jq
  - `sudo apt-get install jq`

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
CABOT_INFLUXDB_ROBOT_NAME          # robot name like cabot1               (default cabot)
CABOT_INFLUXDB_BATTERY_TOPIC       # battery topic name                   (default /cabot/battery)
CABOT_INFLUXDB_IMAGE_LEFT_TOPIC    # topic for left camera                (default "")
CABOT_INFLUXDB_IMAGE_CENTER_TOPIC  # topic for center camera              (default /camera/color/image_raw)
CABOT_INFLUXDB_IMAGE_RIGHT_TOPIC   # topic for right camera               (default "")
CABOT_INFLUXDB_IMAGE_ROTATE        # select image rotate from [left,...]  (default "")
CABOT_INFLUXDB_POSE_INTERVAL       # <float> interval for pose            (default 1.0)
CABOT_INFLUXDB_CMD_VEL_INTERVAL    # <float> interval for cmd vel         (default 0.2)
CABOT_INFLUXDB_ODOM_INTERVAL       # <float> interval for odom            (default 0.2)
CABOT_INFLUXDB_DIAG_AGG_INTERVAL   # <float> interval for diagnostics agg (default 1.0)
CABOT_INFLUXDB_BATTERY_INTERVAL    # <float> interval for battery         (default 1.0)
CABOT_INFLUXDB_IMAGE_INTERVAL      # <float> interval for image           (default 5.0)
CABOT_INFLUXDB_HOST                # host including http/https and port   (default http://localhost:8086)
CABOT_INFLUXDB_TOKEN               # token                                (default a54a87f7-73a0-4534-9741-ad7ff4e7d111  - development default)
CABOT_INFLUXDB_ORG                 # org                                  (default cabot)
CABOT_INFLUXDB_BUCKET              # bucket                               (default cabot)
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

open [http://localhost:8086](http://localhost:8086) - cabot/cabot-influxdb

## Browse Grafana

open [http://localhost:3000](http://localhost:3000) - admin/admin


## Trouble shooting

- If you change grafana server (origin / forked), you may need to delete `grafanadb` content and `grafanaconfig/api-key.txt` and start with `-i` option again

- `./clean.sh` to clean all the servers and data and launch from initialization
