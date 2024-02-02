# grafana-test



## build

```
sudo chown -R 472:472 grafanadb
./setup-dependency.sh
./build-docker.sh -i -w
```

## launch

```
./launch.sh -i # first time only
./launch.sh
```

## Browse InfluxDB 

open [http://localhost:8086](http://localhost:8086)

## Browse Grafana

open [http://localhost:3000](http://localhost:3000)
