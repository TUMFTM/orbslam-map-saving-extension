//
// Created by sebastiano on 8/24/16.
//

#ifndef ORB_SLAM2_FAILURE_H
#define ORB_SLAM2_FAILURE_H

#include <stdexcept>

namespace ORB_SLAM2
{
class Failure : public std::runtime_error
{
    // Inherit constructors
    using std::runtime_error::runtime_error;
};
}


#endif //ORB_SLAM2_FAILURE_H
