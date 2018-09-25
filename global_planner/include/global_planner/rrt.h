//
// Created by alan on 4/15/18.
//

#ifndef MY_PLANNER_RTT_H
#define MY_PLANNER_RTT_H

#include<vector>
#include<global_planner/traceback.h>
#include <global_planner/planner_core.h>
#include <cmath>

namespace my_planner {

    struct _point{
        int parent;
        double pos_x;
        double pos_y;
    };

    typedef struct _point point;

    class RrtPath : public Traceback {
    public:
        RrtPath(PotentialCalculator *p_calc, bool use_auto_path) : Traceback(p_calc,use_auto_path) {}

        bool getPath(float *potential, unsigned char *costs, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> > &path);

        inline double distance(point x, point y) {
            return sqrt(pow(fabs(x.pos_x - y.pos_x), 2) + pow(fabs(x.pos_y - y.pos_y), 2));
        }

        int get_nearest(std::vector<point> &seeds, point seed) {
            int result = 0;
            double minimun = POT_HIGH;
            for (int i = 0; i < seeds.size(); i++) {
                double dis = distance(seeds[i], seed);
                if (dis < minimun) {
                    result = i;
                    minimun = dis;
                }
            }
            return result;
        }

    private:
        int ns_;
        double step;
    };

} //end namespace my_planner

#endif //MY_PLANNER_RTT_H
