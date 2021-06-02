map_path = '~/catkin_ws/src/epic_drive_functions/maps/maze.png'
resolution = 0.05

map_matrix = imread(map_path)
map_matrix = ~(map_matrix)
map = binaryOccupancyMap(map_matrix, resolution)

test = occupancyMatrix(map)