/*
 * Copyright (c) 2012 Miguel Sarabia del Castillo
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


//ROS includes
#include <ros/ros.h>

//Include project classes
#include "Util.h"

//We need boost::foreach to initialise the random seed different for every node
#include <boost/foreach.hpp>
#define foreach_         BOOST_FOREACH
#define foreach_r_       BOOST_REVERSE_FOREACH


namespace Constants
{

//Topics
const std::string actionTopic = "deadlock";
const unsigned int maxValue = 25;
const double maxActionDuration = 0.5; // in seconds

} //End of constants namespace


int main(int argc, char* argv[])
{
    //Init ROS
    ros::init(argc, argv, "bugTest");
    ros::NodeHandle nh;

    //Initialise random seed with fully qualified name of node
    {
        unsigned int seed = ros::Time::now().toNSec();

        const std::string& name = ros::this_node::getName();
        foreach_ (const char& c, name){
            seed += static_cast<unsigned int>(c);
        }

        srand(seed);
    }

    Util::Client action_client(nh, Constants::actionTopic, false);
    while( ros::ok() )
    {
        //Generate random instruction
        Util::Goal::Ptr goal = Util::makeGoal(
                    Util::getRandom(Constants::maxValue));

        action_client.sendGoal(*goal);

        //Wait for a random amount of time before cancelling the instruction
        action_client.waitForResult(
                    ros::Duration(
                        Util::getRandom( Constants::maxActionDuration)) );

        if ( Util::actionOngoing(action_client ) )
        {
            ROS_WARN_STREAM( "Action did not complete in time" );
            action_client.cancelGoal();
        }

        ros::spinOnce();
    }

    return 0;
}

