//
// Created by alan on 4/15/18.
//

#include <global_planner/rrt.h>
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <random>
#include <iostream>
#include <ctime>

namespace my_planner {

    bool RrtPath::getPath(float* potential, unsigned char* costs, double start_x,
                          double start_y, double end_x, double end_y, std::vector<std::pair<float, float> >& path) {

        ns_ = xs_ * ys_;
        std::fill(potential, potential + ns_, POT_HIGH);
        lethal_cost_ = 253;
        step = 4.0;
        std::vector<point> seeds;
        point end_index;
        end_index.pos_x = end_x;
        end_index.pos_y = end_y;
        point current;
        current.pos_x = start_x;
        current.pos_y = start_y;
        current.parent = -1;
        seeds.push_back(current);
        int cycles = 0;

        std::default_random_engine generator(time(NULL));
        while (cycles < 5000) {
            cycles++;

            std::uniform_int_distribution<int> random_gen_y(1800, 2030);
            std::uniform_int_distribution<int> random_gen_x(1890, 2180);
            int ran_x = random_gen_x(generator);
            int ran_y = random_gen_y(generator);
            current.pos_x = ran_x;
            current.pos_y = ran_y;
            int nearest = get_nearest(seeds, current);
            if (distance(seeds[nearest], current) < step) {
                cycles--;
                continue;
            }
            else {
                double alpha = 1.0, beta = 1.0;
                if (!use_auto_path)
                    beta = 0.0;
                double delta_x_end = end_index.pos_x - seeds[nearest].pos_x;
                double delta_y_end = end_index.pos_y - seeds[nearest].pos_y;
                double delta_x = current.pos_x - seeds[nearest].pos_x;
                double delta_y = current.pos_y - seeds[nearest].pos_y;
                double rate = sqrt(pow(step, 2) / (pow(fabs(delta_x), 2) + pow(fabs(delta_y), 2)));
                double rate_end = sqrt(pow(step, 2) / (pow(fabs(delta_x_end), 2) + pow(fabs(delta_y_end), 2)));
                double increment_x = seeds[nearest].pos_x + delta_x*rate*alpha + delta_x_end*rate_end*beta;
                double increment_y = seeds[nearest].pos_y + delta_y*rate*alpha + delta_y_end*rate_end*beta;
                if (costs[getIndex(increment_x, increment_y)] > 0) {
                    if (use_auto_path) {
                        double _increment_x = seeds[nearest].pos_x + delta_x * rate * alpha;
                        double _increment_y = seeds[nearest].pos_y + delta_y * rate * alpha;
                        if (costs[getIndex(_increment_x, _increment_y)] > 0)
                            continue;
                        else {
                            increment_x = _increment_x;
                            increment_y = _increment_y;
                        }
                    }
                    else
                        continue;
                }
                current.pos_x = increment_x;
                current.pos_y = increment_y;
                current.parent = nearest;
                seeds.push_back(current);

                potential[getIndex(current.pos_x, current.pos_y)] = 255;
                if (distance(end_index, current) < step)
                    break;
            }
        }
        end_index.parent = seeds.size() - 1;

        seeds.push_back(end_index);
        std::pair<float, float> now;
        while (current.parent != -1) {
            now.first = current.pos_x;
            now.second = current.pos_y;
            path.push_back(now);
            current = seeds[current.parent];
        }

        return true;
    }

} //end namespace my_planner
