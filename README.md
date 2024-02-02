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
CABOT_INFLUXDB_HOST
CABOT_INFLUXDB_TOKEN
CABOT_INFLUXDB_ORG
CABOT_INFLUXDB_BUCKET
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
