#ifndef MATH_TOOL_HPP_
#define MATH_TOOL_HPP_

#include <cmath>
#include <iostream>
#include <vector>
#include <random>

namespace pure_pursuit_controller
{
using path_type = std::pair<std::pair<double, double>, double>;
class MathTool {
    public:
        /**
         * @brief Get the index of min distance between robot pose and path
         * 
         * @param current_pose robot current pose
         * @param path_vector path point vector
         * @return index(int)
        */
        int GetMinDisIndex(path_type current_pose, 
                std::vector<path_type> path_vector);
                
        /**
         * @brief Get distance between pos1 and pos2
         * 
         * @param pos1 pos1 pose
         * @param pos2 pos2 pose
         * @return distance between pos1 and pos2
        */
        double GetDistance(path_type pos1, path_type pos2);

        /**
         * @brief Construction of math tool
        */
        MathTool() {};
        ~MathTool() {};
};
}

#endif // MATH_TOOL_HPP_