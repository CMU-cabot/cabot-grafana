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
CABOT_ANCHOR_FILE           # cabot_site anchor yaml file for converting ROS coordinate to world coordinate
CABOT_INFLUXDB_HOST         # host like http://localhost:8086
CABOT_INFLUXDB_TOKEN        # token
CABOT_INFLUXDB_ORG          # org
CABOT_INFLUXDB_BUCKET       # bucket
```

## launch local grafana/influxDB for development

```
sudo chown -R 472:472 grafanadb
./launch.sh -s -i # first time only
./launch.sh -s
# second terminal
./launch.sh
```

## Browse InfluxDB 

open [http://localhost:8086](http://localhost:8086)

## Browse Grafana

open [http://localhost:3000](http://localhost:3000)
