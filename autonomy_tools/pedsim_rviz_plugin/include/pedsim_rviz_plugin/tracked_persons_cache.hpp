/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013-2015, Timm Linder, Social Robotics Lab, University of Freiburg
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef PEDSIM_RVIZ_PLUGIN__TRACKED_PERSONS_CACHE_HPP_
#define PEDSIM_RVIZ_PLUGIN__TRACKED_PERSONS_CACHE_HPP_

#ifndef Q_MOC_RUN
#include <map>
#include <memory>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <pedsim_msgs/msg/tracked_persons.hpp>
#endif

#include <pedsim_rviz_plugin/additional_topic_subscriber.hpp>

namespace pedsim_rviz_plugin
{
    typedef unsigned int track_id;

    /// Data structure for storing information about individual person tracks
    struct CachedTrackedPerson
    {
        Ogre::Vector3 center;
        geometry_msgs::msg::PoseWithCovariance pose;
        geometry_msgs::msg::TwistWithCovariance twist;
        bool isOccluded;
    };

    /// Subscribes to a TrackedPersons topic and caches all TrackedPersons of the current cycle.
    class TrackedPersonsCache {
    public:
        typedef std::map<track_id, std::shared_ptr<CachedTrackedPerson> > CachedTrackedPersonsMap;

        ~TrackedPersonsCache();

        void initialize(
            rviz_common::Display* display,
            rviz_common::DisplayContext* context,
            rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node);

        void reset();

        const std::shared_ptr<CachedTrackedPerson> lookup(track_id trackId);

        const CachedTrackedPersonsMap& getMap() {
            return m_cachedTrackedPersons;
        }

    private:
        void processTrackedPersonsMessage(pedsim_msgs::msg::TrackedPersons::ConstSharedPtr msg);

        AdditionalTopicSubscriber<pedsim_msgs::msg::TrackedPersons>* m_tracked_person_subscriber{nullptr};
        rviz_common::Display* m_display;
        rviz_common::DisplayContext* m_context;

        CachedTrackedPersonsMap m_cachedTrackedPersons;
    };

}

#endif  // PEDSIM_RVIZ_PLUGIN__TRACKED_PERSONS_CACHE_HPP_
