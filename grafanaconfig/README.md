## setup .env

- edit `.env` file

```
GRAFANA_HOST=localhost:3000
```

## make API_KEY

```
./make-key.sh -p <admin pass>
```

copy generated `API_KEY` to `.env` file


## import/export datasource dashboard

see help

```
./import.sh -h
./export.sh -h
```
