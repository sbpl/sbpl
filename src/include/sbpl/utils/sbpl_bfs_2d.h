/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Mellon University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SBPL_BFS_2D
#define SBPL_BFS_2D

#include <vector>
#include <sbpl/utils/sbpl_fifo.h>

class sbpl_bfs_2d
{
private:
    class bfs_cell_2d
    {
    public:
        bfs_cell_2d()
        {
            x = 0;
            y = 0;
        }

        bfs_cell_2d(int tempX, int tempY)
        {
            x = tempX;
            y = tempY;
        }

        int x;
        int y;
    };

public:
    sbpl_bfs_2d(unsigned int sx, unsigned int sy, int obs_thresh, int fifo_size = -1) :
        NUM_ACTIONS(8)
    {
        size_x_ = sx;
        size_y_ = sy;
        thresh_ = obs_thresh;

        //initialize the fifo
        if (fifo_size < 0) fifo_size = 2 * sx + 2 * sy;
        q_ = new sbpl_fifo<bfs_cell_2d> (fifo_size);

        //initialize the distance grid
        dist_ = new int*[size_x_];
        for (int x = 0; x < size_x_; x++)
            dist_[x] = new int[size_y_];

        //initialize the actions
        dx = new int[NUM_ACTIONS];
        dy = new int[NUM_ACTIONS];
        int idx = 0;
        for (int x = -1; x <= 1; x++) {
            for (int y = -1; y <= 1; y++) {
                if (!x && !y) continue;
                dx[idx] = x;
                dy[idx] = y;
                idx++;
            }
        }

    }

    ~sbpl_bfs_2d()
    {
        //delete the fifo
        delete q_;

        //delete the distance grid
        for (int x = 0; x < size_x_; x++)
            delete[] dist_[x];
        delete[] dist_;

        //delete the actions
        delete[] dx;
        delete[] dy;
    }

    bool compute_distance_from_point(int** grid, int x, int y)
    {
        if (x < 0 || x >= size_x_ || y < 0 || y >= size_y_) {
            printf("ERROR[compute_distance_from_point]: point is out of bounds!\n");
            return false;
        }
        q_->clear();
        clear_distances();

        bfs_cell_2d c(x, y);
        q_->insert(c);
        dist_[x][y] = 0;

        compute_distances(grid);
        return true;
    }

    bool compute_distance_from_points(int** grid, std::vector<int> x, std::vector<int> y)
    {
        if (x.size() != y.size()) {
            printf("ERROR[compute_distance_from_points]: size of x and y coordinates must agree!\n");
            return false;
        }
        q_->clear();
        clear_distances();

        for (unsigned int i = 0; i < x.size(); i++) {
            if (x[i] < 0 || x[i] >= size_x_ || y[i] < 0 || y[i] >= size_y_) {
                printf("ERROR[compute_distance_from_points]: point is out of bounds!\n");
                return false;
            }
            bfs_cell_2d c(x[i], y[i]);
            q_->insert(c);
            dist_[x[i]][y[i]] = 0;
        }

        compute_distances(grid);
        return true;
    }

    void compute_distance_from_obs(int** grid)
    {
        q_->clear();
        clear_distances();

        for (int x = 0; x < size_x_; x++) {
            for (int y = 0; y < size_y_; y++) {
                if (grid[x][y] >= thresh_) {
                    bfs_cell_2d c(x, y);
                    q_->insert(c);
                    dist_[x][y] = 0;
                }
            }
        }

        compute_distances(grid);
    }

    void clear_distances()
    {
        for (int x = 0; x < size_x_; x++)
            for (int y = 0; y < size_y_; y++)
                dist_[x][y] = -1;
    }

    void compute_distances(int** grid)
    {
        bfs_cell_2d c;
        while (!q_->empty()) {
            q_->remove(&c);
            int cost = dist_[c.x][c.y] + 1;
            if (c.x == 0 || c.x == size_x_ - 1 || c.y == 0 || c.y == size_y_ - 1) {
                //we are on a boundary so we have to bounds check each successor
                for (int i = 0; i < NUM_ACTIONS; i++) {
                    int x = c.x + dx[i];
                    int y = c.y + dy[i];
                    if (x < 0 || x >= size_x_ || y < 0 || y >= size_y_) continue;
                    if (dist_[x][y] < 0 && grid[x][y] < thresh_) {
                        dist_[x][y] = cost;
                        bfs_cell_2d temp(x, y);
                        q_->insert(temp);
                    }
                }
            }
            else {
                //we are not near a boundary so no bounds check is required
                for (int i = 0; i < NUM_ACTIONS; i++) {
                    int x = c.x + dx[i];
                    int y = c.y + dy[i];
                    if (dist_[x][y] < 0 && grid[x][y] < thresh_) {
                        dist_[x][y] = cost;
                        bfs_cell_2d temp(x, y);
                        q_->insert(temp);
                    }
                }
            }
        }
    }

    int get_distance(unsigned int x, unsigned int y)
    {
        return dist_[x][y];
    }

private:
    int** dist_;
    sbpl_fifo<bfs_cell_2d>* q_;
    int size_x_;
    int size_y_;
    int thresh_;

    const int NUM_ACTIONS;
    int* dx;
    int* dy;
};

#endif
