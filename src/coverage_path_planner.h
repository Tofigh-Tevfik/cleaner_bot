#ifndef COVERAGE_PATH_PLANNER
#define COVERAGE_PATH_PLANNER

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <deque>
#include <cstdlib>
#include "linked_list.h"
#include <opencv2/core/core.hpp>
#include <cmath>


void map_callback(const nav_msgs::OccupancyGrid::ConstPtr&);
void plot_org_map(int **, unsigned int, unsigned int, int, int);
int** reshape(int8_t *, unsigned int, unsigned int);
int** reduce_resolution(int **, float, float, unsigned int, unsigned int);
bool is_occupied(int **, int, int []);
float calculateDistance(float, float, float, float);

class CoveragePath {
    public:
        // constructor for the class
        // creates a vector of the unoccupied grids
        CoveragePath(int **map_values, int height, int width) {
            height_ = height; width_ = width;
            for (int i = 0; i < height; i++) {
                for (int j = 0; j < width; j++) {
                    if (map_values[i][j] == 0) {
                        map_grids.append(i, j);
                    }
                }
            }
            map_grids.findAdjGrids();
        };
        // destructor deletes the map_values_ dynamic array
        ~CoveragePath() {};

        bool createWeights(int x, int y) {
            // if the start grid (x, y) does not exist
            if (!map_grids.has(x, y)) {
                std::cerr << "Grid does not exist. Cannot create weights" << std::endl;
                return false;
            }
            // create a weight vector to store weights for each grid
            std::deque<int> weight_vector;
            // weight the start grid as 1
            weight_vector.push_back(1);
            std::vector<grid*> adjGrids;
            grid * currGrid = map_grids.findGrid(x, y);
            startGrid = currGrid;
            currGrid->weight = weight_vector[0];
            // adjacent grids of the current grid
            std::vector<grid*> currGridAdjs = map_grids.returnAdjGrids(currGrid->index);
            for(int i = 0; i < currGridAdjs.size(); i++) {
                weight_vector.push_back(weight_vector[0]+1);
            }
            // remove the first weight
            weight_vector.erase(weight_vector.begin());
            // concatenate the adjacent grids with the adjacent grids of the current grid
            adjGrids.insert(adjGrids.end(), currGridAdjs.begin(), currGridAdjs.end());

            while (!adjGrids.empty()) {
                currGrid = adjGrids[0];
                if (currGrid->weight == 0) {
                    currGrid->weight = weight_vector[0];
                    currGridAdjs = map_grids.returnAdjGrids(currGrid->index);
                    adjGrids.insert(adjGrids.end(), currGridAdjs.begin(), currGridAdjs.end());
                    for(int i = 0; i < currGridAdjs.size(); i++) {
                        weight_vector.push_back(weight_vector[0]+1);
                    }
                }
                weight_vector.pop_front();
                adjGrids.erase(adjGrids.begin());
            }
            // map_grids.printWeights();
            return true;
        };
        
        bool createCoveragePath() {
            coveragePath.append(startGrid->x, startGrid->y);
            grid * currentGrid = startGrid;
            std::vector<grid*> currentGridAdj = map_grids.returnAdjGrids(currentGrid->index);
            grid * nextGrid;
            while (coveragePath.length() < map_grids.length()) {
                if (!currentGridAdj.empty()) {
                    nextGrid = findMaxWeightGrid(currentGridAdj);
                }

                if (!coveragePath.has(nextGrid->x, nextGrid->y)) {
                    coveragePath.append(nextGrid->x, nextGrid->y);
                    currentGrid = nextGrid;
                    currentGridAdj = map_grids.returnAdjGrids(currentGrid->index);
                }
                else if (!currentGridAdj.empty()) {
                    // std::cout << "grid has already been visited" << std::endl;
                    for (int i = 0; i < currentGridAdj.size(); i++) {
                        if (nextGrid == currentGridAdj[i]) {
                            currentGridAdj.erase(currentGridAdj.begin() + i);
                            break;
                        }
                    }
                }
                else {
                    // std::cout << "adjacent grid vector is empty" << std::endl;
                    currentGrid = this->findClosestUnvistedGrid(currentGrid);
                    coveragePath.append(currentGrid->x, currentGrid->y);
                    currentGridAdj = map_grids.returnAdjGrids(currentGrid->index);
                }
            }
            return true;
        };

        // method to find the grid with max weight inside a adjacent grid vector
        grid * findMaxWeightGrid(std::vector<grid*> adjGrids) {
            grid * maxWeightGrid = adjGrids[0];
            for (int i = 1; i < adjGrids.size(); i++) {
                if (adjGrids[i]->weight > maxWeightGrid->weight) {
                    maxWeightGrid = adjGrids[i];
                }
            }
            return maxWeightGrid;
        };

        grid * findClosestUnvistedGrid(grid * lockedGrid) {
            grid * currGrid = map_grids.getHead();
            grid * closestGrid;
            float distance = INFINITY;
            while (currGrid != nullptr) {
                float distance_toCurrGrid = calculateDistance(float (currGrid->x), float (currGrid->y), float (lockedGrid->x), float (lockedGrid->y));
                if (distance > distance_toCurrGrid && !coveragePath.has(currGrid->x, currGrid->y)) {
                    distance = distance_toCurrGrid;
                    closestGrid = currGrid;
                }
                currGrid = currGrid->next;
            }
            return closestGrid;
        }
        LinkedList coveragePath;
    private:
        grid * startGrid;
        LinkedList map_grids;
        int height_; int width_;
};

// plotting the map using opencv
void plot_org_map(int **map_values, unsigned int height, unsigned int width, int goal_x = -1, int goal_y = -1)
{
    cv::Mat image(height, width, CV_8UC3);
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            int map_value = map_values[i][j];
            cv::Vec3b color;

            if (map_value == 0) color = cv::Vec3b(255, 255, 255); // white
            else if (map_value == 100) color = cv::Vec3b(0, 0, 0); // black
            else color = cv::Vec3b(127, 127, 127); // gray
            image.at<cv::Vec3b>(cv::Point(j, i)) = color;
        }
    }
    if (goal_x != -1 && goal_y != -1) image.at<cv::Vec3b>(cv::Point(goal_y, goal_x)) = cv::Vec3b(0, 0, 255);
    cv::namedWindow("OccupancyGrid Map", cv::WINDOW_NORMAL);
    cv::resizeWindow("OccupancyGrid Map", 800, 600); //
    cv::imshow("OccupancyGrid Map", image);
    cv::waitKey(0); // Wait for the user to close the window
}

// function for reshaping the map into a 2 dimensional grid
int** reshape(int8_t* map_values, unsigned int height, unsigned int width)
{
    int** reshaped_map = new int*[height];
    for (unsigned int i = 0; i < height; i++) {
        reshaped_map[i] = new int[width];
        for (unsigned int j = 0; j < width; j++) {
            reshaped_map[i][j] = map_values[i * width + j];
        }
    }
    return reshaped_map;
}

// function for reducing the resolution of the map
// this function reduces each 8x8 original grids into 1x1 grids.
int** reduce_resolution(int **map_values, float current_resolution, float desired_resolution, unsigned int height, unsigned int width)
{
    int grid_ratio = int (desired_resolution / current_resolution);
    int search_radius = grid_ratio / 2;
    int** new_grid_map = new int*[height/grid_ratio];
    for (int i = 0; i < height / grid_ratio; i++) {
        new_grid_map[i] = new int[width / grid_ratio];
        for (int j = 0; j < width / grid_ratio; j++) {
            int center[2] = {i * grid_ratio + search_radius, j * grid_ratio + search_radius};
            new_grid_map[i][j] = is_occupied(map_values, search_radius, center);
        }
    }
    return new_grid_map;
}

// this function check a grids radius and see if the radius is occupied.
bool is_occupied(int **map_values, int search_radius, int center[])
{
    for (int i = -search_radius; i < search_radius; i++) {
        for (int j = -search_radius; j < search_radius; j++) {
            if (map_values[center[0] + i][center[1] + j]) {
                return true;
            }
        }
    }
    return false;
}

float calculateDistance(float x1, float y1, float x2, float y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

#endif