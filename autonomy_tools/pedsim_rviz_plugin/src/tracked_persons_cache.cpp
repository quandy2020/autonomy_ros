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

#include "pedsim_rviz_plugin/tracked_persons_cache.hpp"

#ifndef Q_MOC_RUN
#include <sstream>
#endif

namespace pedsim_rviz_plugin
{

TrackedPersonsCache::~TrackedPersonsCache()
{
    m_cachedTrackedPersons.clear();
    delete m_tracked_person_subscriber;
    m_tracked_person_subscriber = nullptr;
}

void TrackedPersonsCache::initialize(
    rviz_common::Display* display,
    rviz_common::DisplayContext* context,
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node)
{
    m_display = display;
    m_context = context;

    m_tracked_person_subscriber = new AdditionalTopicSubscriber<pedsim_msgs::msg::TrackedPersons>(
        "Tracked persons topic", display, context, rviz_ros_node,
        std::bind(&TrackedPersonsCache::processTrackedPersonsMessage, this, std::placeholders::_1));
}

void TrackedPersonsCache::reset()
{
    m_cachedTrackedPersons.clear();
}

const std::shared_ptr<CachedTrackedPerson> TrackedPersonsCache::lookup(track_id trackId)
{
    CachedTrackedPersonsMap::const_iterator entry = m_cachedTrackedPersons.find(trackId);
    if (entry == m_cachedTrackedPersons.end()) {
        return std::shared_ptr<CachedTrackedPerson>();
    }
    return entry->second;
}

void TrackedPersonsCache::processTrackedPersonsMessage(
    pedsim_msgs::msg::TrackedPersons::ConstSharedPtr msg)
{
    Ogre::Vector3 frameOrigin;
    Ogre::Quaternion frameOrientation;
    m_context->getFrameManager()->getTransform(msg->header, frameOrigin, frameOrientation);
    Ogre::Matrix4 transform(frameOrientation);
    transform.setTrans(frameOrigin);

    m_cachedTrackedPersons.clear();
    for (const auto& trackedPerson : msg->tracks) {
        m_cachedTrackedPersons[trackedPerson.track_id] = std::shared_ptr<CachedTrackedPerson>(
            new CachedTrackedPerson);
        CachedTrackedPerson& cachedTrackedPerson = *m_cachedTrackedPersons[trackedPerson.track_id];

        const geometry_msgs::msg::Point& position = trackedPerson.pose.pose.position;
        cachedTrackedPerson.center = transform * Ogre::Vector3(position.x, position.y, position.z);
        cachedTrackedPerson.pose = trackedPerson.pose;
        cachedTrackedPerson.twist = trackedPerson.twist;
        cachedTrackedPerson.isOccluded = trackedPerson.is_occluded;
    }

    std::stringstream ss;
    ss << msg->tracks.size() << " track(s)";
    m_display->setStatusStd(rviz_common::properties::StatusProperty::Ok, "Tracks", ss.str());
}

}  // namespace pedsim_rviz_plugin
