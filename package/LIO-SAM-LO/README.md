Package based on LIO-SAM that allow to localize only using a LIO-SAM generated map. Since the package performs localization only I named the package LIO-SAM-LO (**L**ocalize **O**nly).

## How to use
1. Run the offical LIO-SAM package and save the map in `/workspace/map/` folder
2. Run the LIO-SAM-LO launch file ``:
```
roslaunch lio_sam_lo run_localize.launch
```
3. Play your bag files:
```
rosbag play your-bag.bag
```
