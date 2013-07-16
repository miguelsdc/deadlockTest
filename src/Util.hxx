/*
 * Copyright (c) 2013 Miguel Sarabia del Castillo
 * Imperial College London
 *
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#ifndef UTIL_HXX
#define UTIL_HXX

#include "Util.h"


namespace Util
{

inline double getRandom(double maxValue)
{
    double basic = static_cast<double>( rand() ) / RAND_MAX;
    return basic * maxValue;
}

inline Util::Goal::Ptr makeGoal(unsigned int countdown)
{
    Util::Goal::Ptr result ( new Util::Goal() );
    result->countdown = countdown;

    return result;
}

template<typename T>
inline bool actionSucceeded(actionlib::SimpleActionClient<T>& c)
{
    const actionlib::SimpleClientGoalState state = c.getState();
    return state == actionlib::SimpleClientGoalState::SUCCEEDED;
}

template<typename T>
inline bool actionOngoing(actionlib::SimpleActionClient<T>& c)
{
    const actionlib::SimpleClientGoalState state = c.getState();
    return (state == actionlib::SimpleClientGoalState::PENDING ||
            state == actionlib::SimpleClientGoalState::ACTIVE );
}

}

#endif // UTIL_HXX
