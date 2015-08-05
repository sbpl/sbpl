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

#include <cmath>
#include <iostream>
using namespace std;

#include <sbpl/discrete_space_information/environment_precomputed_adjacency_list.h>

// 2d Points
struct Point2D
{
    Point2D(int newX, int newY) :
        x(newX), y(newY)
    {
    }

    int heuristicDistanceTo(const Point2D& p)
    {
        int dx = p.x - x;
        int dy = p.y - y;
        int dist = ((int)sqrt(dx * dx + dy * dy));
        return dist;
    }

    int x;
    int y;
};

ostream& operator<<(ostream& stream, Point2D p)
{
    stream << "(" << p.x << ", " << p.y << ")";
    return stream;
}

int operator<(const Point2D& p1, const Point2D& p2)
{
    return (p1.x < p2.x) || ((p1.x == p2.x) && (p1.y < p2.y));
}

void testPlanner(AdjacencyListSBPLEnv<Point2D>& e)
{
    int sol_cost;
    e.writeToStream();
    vector<Point2D> solution = e.findOptimalPath(&sol_cost);
    cout << "Returned plan is ";
    for (unsigned int i = 0; i < solution.size(); i++) {
        cout << solution[i] << " ";
    }
    cout << endl;
}

int main(int, char**)
{
    AdjacencyListSBPLEnv<Point2D> e;
    Point2D p1(0, 0);
    Point2D p2(2, 1);
    Point2D p3(1, 4);
    Point2D p4(5, 5);

    e.addPoint(p1);
    e.addPoint(p4);
    e.addPoint(p3);
    e.addPoint(p2);

    e.setCost(p1, p2, 4);
    e.setCost(p1, p3, 6);
    e.setCost(p3, p4, 5);
    e.setCost(p2, p4, 15);

    e.setStartState(p1);
    e.setGoalState(p4);

    // Initialize the MDPConfig (what does this do exactly?)
    //MDPConfig c;
    //e.InitializeMDPCfg(&c);

    // Tests
    testPlanner(e);
    e.setCost(p2, p4, 1);
    testPlanner(e);
    e.removeLastPoints();
    testPlanner(e);
    e.writeToStream();
    return 0;
}

