# Traverse Layer

[source](https://github.com/team19-haql/haql-rover/tree/main/traverse_layer)

This package provides a system that reads pointcloud data and generates a navigation map. 
## Pointcloud to gridmap

The pointcloud to gridmap node does some basic processing to convert a 3d pointcloud into a 2d map. The internal representation of a map uses the [grid maps](https://github.com/ANYbotics/grid_map) package. This processing is in the pointcloud to gridmap component. 

## Traverse Map

Traverse layer does the processing to calculate traversability. The algorithm uses a 15 stage filter described below. This logic is in the traverse layer component. 

This component also handles map storage, instead of throwing away map data after we leave its range, we store it in a global map. We store a high resolution global map which currently doesn't do anything, and a low resolution global map which is used for long distance path planning. 

### Filter

1. Buffer normalizer (fixes indexing)
2. smoothed_elevation = box blur with radius of elevation
3. calculate surface normals from smoothed elevation
4. add color map to surface normals
5. calculate slope using `arccos` of normal z component
6. find minimum slope values within a given radius
7. `slope traversability = min_slope / critical angle`
8. find the lowest value within a given radius on the smoothed elevation layer
9. calculate stepsize as the difference between the lowest value from `8` and the actual smoothed elevation value
10. `step traversability = step size / critical step size`
11. `traversability = max(slope traversability, step traversability)`
12. set lower limit of traversability to 0
13. set upper limit of traversablity to 1
14. remove cells that don't have actual measurements
15. set traversability to be the max traversability within a radius. 

### Zigmaps

Due to the complexity of the filter, calculating the traversability purely in grid maps was causing performance issues. To combat this issue, I wrote the [zigmaps](https://github.com/deweykai/zigmaps) package. Inside the traverse layer component, we copy the data coming from the pointcloud to grid map component into a zigmaps layer. Zigmaps calculates the traversability. Finally, the the results are copied back into grid maps for further processing. 