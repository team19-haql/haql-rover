# Configuration


<iframe width="560" height="315" src="https://www.youtube.com/embed/WC9gRCU1fb0?si=OQ9zCWA2G-UpCUi-" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Install QGIS

To easily configure  a GPS route, install a gis software, we use [QGIS](https://www.qgis.org/en/site/). 

[This](https://hatarilabs.com/ih-en/how-to-add-a-google-map-in-qgis-3-tutorial) is a guide to adding a map layer to gqis.

## Create a shape

Create a new shapefile layer. The layer should be a linestring type. 

## Convert into a configuration file

Use the following script to convert the shape file into a configuration file

```python
import yaml
import shapefile
import io

# open shapefile
sf = shapefile.Reader("path.shp")
shapes = sf.shapes()
print(sf)

# load the path into a file
paths = []
for shape in shapes:
    paths.append(shape.points)
print(f'found #{len(paths)} paths')

# generate configuration file
routes = []
record = {'routes': routes}
for i, path in enumerate(paths):
    name = f'path_{i}'
    routes.append(name)
    record[name] = []
    for lon, lat in path:
        record[name].append({
            'lat': lat,
            'lon': lon,
        })

# Save waypoint file
with io.open('waypoints.yml', 'w') as outfile:
    print(record)
    yaml.dump(record, outfile, default_flow_style=False)
```

The configuration file should then be copied into a file `/home/deweykai/Documents/startup/runtime_config/waypoints.yml`