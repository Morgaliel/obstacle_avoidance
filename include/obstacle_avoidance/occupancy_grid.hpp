#ifndef SRC_OCCUPANCY_GRID_HPP
#define SRC_OCCUPANCY_GRID_HPP


#include "nav_msgs/msg/occupancy_grid.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <math.h>

#define THRESHOLD 50

using namespace std;

namespace occupancy_grid{
    int xy_ind2ind(const nav_msgs::msg::OccupancyGrid& grid, int x_ind, int y_ind){
        return min(y_ind * int(grid.info.width) + x_ind, int(grid.data.size())-1);
    }

    int xy2ind(const nav_msgs::msg::OccupancyGrid& grid, float x, float y){
        int x_ind = static_cast<int>(ceil((x-grid.info.origin.position.x)/grid.info.resolution))-1;
        int y_ind = static_cast<int>(ceil((y-grid.info.origin.position.y)/grid.info.resolution))-1;

        if (x_ind <= 0 || x_ind >= grid.info.width || y_ind <= 0 || y_ind >= grid.info.height) 
        {
            return 100;  // Return an invalid index if out of range
        }

        return xy_ind2ind(grid, x_ind, y_ind);
    }

    struct Pair{
        int x_ind;
        int y_ind;
    };

    Pair ind2xy_ind(const nav_msgs::msg::OccupancyGrid& grid, int ind){
        int y_ind = ind/grid.info.width;
        int x_ind = ind - y_ind*grid.info.width;
        Pair res;
        res.x_ind = x_ind;
        res.y_ind = y_ind;
        return res;
    }

    float ind2x(const nav_msgs::msg::OccupancyGrid& grid, int ind){
        return grid.info.origin.position.x + ind2xy_ind(grid, ind).x_ind * grid.info.resolution;
    }

    float ind2y(const nav_msgs::msg::OccupancyGrid& grid, int ind){
        return grid.info.origin.position.y + ind2xy_ind(grid, ind).y_ind * grid.info.resolution;
    }

    bool is_xy_occupied(nav_msgs::msg::OccupancyGrid& grid, float x, float y){
        int temp = int(grid.data.at(xy2ind(grid, x, y)));
        if(temp < 0 || temp > THRESHOLD){
            return true;
        }
        else
        {
            return false;
        }
        // return int(grid.data.at(xy2ind(grid, x, y))) > THRESHOLD;
    }

    void set_xy_occupied(nav_msgs::msg::OccupancyGrid& grid, float x, float y){
        grid.data.at(xy2ind(grid, x, y)) = 100;
    }

    void inflate_cell(nav_msgs::msg::OccupancyGrid &grid, int i, float margin, int val) {
        int margin_cells = static_cast<int>(ceil(margin/grid.info.resolution));
        Pair res = ind2xy_ind(grid, i);
        for (int x = max(0, res.x_ind-margin_cells); x<min(int(grid.info.width-1), res.x_ind+margin_cells); x++){
            for (int y = max(0, res.y_ind-margin_cells); y<min(int(grid.info.height-1), res.y_ind+margin_cells); y++){
                if (grid.data.at(xy_ind2ind(grid,x,y))>THRESHOLD) continue;
                grid.data.at(xy_ind2ind(grid,x,y)) = val;
            }
        }
    }

    void inflate_map(nav_msgs::msg::OccupancyGrid& grid, float margin){
        vector<int> occupied_ind;
        occupied_ind.clear();
        for (int i=0; i<grid.data.size(); i++){
            if (grid.data.at(i)>THRESHOLD){
                occupied_ind.push_back(i);
            }
        }
        for (int i=0; i<occupied_ind.size(); i++){
            inflate_cell(grid, occupied_ind[i], margin, 100);
        }
    }


}

#endif //SRC_OCCUPANCY_GRID_HPP