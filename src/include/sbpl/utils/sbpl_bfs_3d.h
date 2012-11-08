#ifndef SBPL_BFS_3D
#define SBPL_BFS_3D

#include <vector>
#include <sbpl/utils/sbpl_fifo.h>

class sbpl_bfs_3d
{
private:
    class bfs_cell_3d
    {
    public:
        bfs_cell_3d()
        {
            x = 0;
            y = 0;
            z = 0;
        }
        bfs_cell_3d(int tempX, int tempY, int tempZ)
        {
            x = tempX;
            y = tempY;
            z = tempZ;
        }
        ;
        int x;
        int y;
        int z;
    };

public:
    sbpl_bfs_3d(unsigned int sx, unsigned int sy, unsigned int sz, int obs_thresh, int fifo_size = -1) :
        NUM_ACTIONS(26)
    {
        size_x_ = sx;
        size_y_ = sy;
        size_z_ = sz;
        thresh_ = obs_thresh;

        //initialize the fifo
        if (fifo_size < 0) fifo_size = 2 * sx * sy + 2 * sx * sz + 2 * sy * sz;
        q_ = new sbpl_fifo<bfs_cell_3d> (fifo_size);

        //initialize the distance grid
        dist_ = new int**[size_x_];
        for (int x = 0; x < size_x_; x++) {
            dist_[x] = new int*[size_y_];
            for (int y = 0; y < size_y_; y++)
                dist_[x][y] = new int[size_z_];
        }

        //initialize the actions
        dx = new int[NUM_ACTIONS];
        dy = new int[NUM_ACTIONS];
        dz = new int[NUM_ACTIONS];
        int idx = 0;
        for (int x = -1; x <= 1; x++) {
            for (int y = -1; y <= 1; y++) {
                for (int z = -1; z <= 1; z++) {
                    if (!x && !y && !z) continue;
                    dx[idx] = x;
                    dy[idx] = y;
                    dz[idx] = z;
                    idx++;
                }
            }
        }

    }

    ~sbpl_bfs_3d()
    {
        //delete the fifo
        delete q_;

        //delete the distance grid
        for (int x = 0; x < size_x_; x++) {
            for (int y = 0; y < size_y_; y++)
                delete[] dist_[x][y];
            delete[] dist_[x];
        }
        delete[] dist_;

        //delete the actions
        delete[] dx;
        delete[] dy;
        delete[] dz;
    }

    bool compute_distance_from_point(int*** grid, int x, int y, int z)
    {
        if (x < 0 || x >= size_x_ || y < 0 || y >= size_y_ || z < 0 || z >= size_z_) {
            printf("ERROR[compute_distance_from_point]: point is out of bounds!\n");
            return false;
        }
        q_->clear();
        clear_distances();

        bfs_cell_3d c(x, y, z);
        q_->insert(c);
        dist_[x][y][z] = 0;

        compute_distances(grid);
        return true;
    }

    bool compute_distance_from_points(int*** grid, std::vector<int> x, std::vector<int> y, std::vector<int> z)
    {
        if (x.size() != y.size() || x.size() != z.size()) {
            printf("ERROR[compute_distance_from_points]: size of x, y, and z coordinates must agree!\n");
            return false;
        }
        q_->clear();
        clear_distances();

        for (unsigned int i = 0; i < x.size(); i++) {
            if (x[i] < 0 || x[i] >= size_x_ || y[i] < 0 || y[i] >= size_y_ || z[i] < 0 || z[i] >= size_z_) {
                printf("ERROR[compute_distance_from_points]: point is out of bounds!\n");
                return false;
            }
            bfs_cell_3d c(x[i], y[i], z[i]);
            q_->insert(c);
            dist_[x[i]][y[i]][z[i]] = 0;
        }

        compute_distances(grid);
        return true;
    }

    void compute_distance_from_obs(int*** grid)
    {
        q_->clear();
        clear_distances();

        for (int x = 0; x < size_x_; x++) {
            for (int y = 0; y < size_y_; y++) {
                for (int z = 0; z < size_z_; z++) {
                    if (grid[x][y][z] >= thresh_) {
                        bfs_cell_3d c(x, y, z);
                        q_->insert(c);
                        dist_[x][y][z] = 0;
                    }
                }
            }
        }

        compute_distances(grid);
    }

    void clear_distances()
    {
        for (int x = 0; x < size_x_; x++)
            for (int y = 0; y < size_y_; y++)
                for (int z = 0; z < size_z_; z++)
                    dist_[x][y][z] = -1;
    }

    void compute_distances(int*** grid)
    {
        bfs_cell_3d c;
        while (!q_->empty()) {
            q_->remove(&c);
            int cost = dist_[c.x][c.y][c.z] + 1;
            if (c.x == 0 || c.x == size_x_ - 1 || c.y == 0 || c.y == size_y_ - 1 || c.z == 0 || c.z == size_z_ - 1) {
                //we are on a boundary so we have to bounds check each successor
                for (int i = 0; i < NUM_ACTIONS; i++) {
                    int x = c.x + dx[i];
                    int y = c.y + dy[i];
                    int z = c.z + dz[i];
                    if (x < 0 || x >= size_x_ || y < 0 || y >= size_y_ || z < 0 || z >= size_z_) continue;
                    if (dist_[x][y][z] < 0 && grid[x][y][z] < thresh_) {
                        dist_[x][y][z] = cost;
                        bfs_cell_3d temp(x, y, z);
                        q_->insert(temp);
                    }
                }
            }
            else {
                //we are not near a boundary so no bounds check is required
                for (int i = 0; i < NUM_ACTIONS; i++) {
                    int x = c.x + dx[i];
                    int y = c.y + dy[i];
                    int z = c.z + dz[i];
                    if (dist_[x][y][z] < 0 && grid[x][y][z] < thresh_) {
                        dist_[x][y][z] = cost;
                        bfs_cell_3d temp(x, y, z);
                        q_->insert(temp);
                    }
                }
            }
        }
    }

    int get_distance(unsigned int x, unsigned int y, unsigned int z)
    {
        return dist_[x][y][z];
    }

private:
    int*** dist_;
    sbpl_fifo<bfs_cell_3d>* q_;
    int size_x_;
    int size_y_;
    int size_z_;
    int thresh_;

    const int NUM_ACTIONS;
    int* dx;
    int* dy;
    int* dz;
};

#endif
