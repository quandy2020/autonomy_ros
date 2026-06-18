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

#ifndef PEDSIM_RVIZ_PLUGIN__TRACKED_GROUPS_DISPLAY_HPP_
#define PEDSIM_RVIZ_PLUGIN__TRACKED_GROUPS_DISPLAY_HPP_

#ifndef Q_MOC_RUN
#include <map>
#include <memory>
#include <boost/circular_buffer.hpp>
#include <pedsim_msgs/msg/tracked_groups.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <pedsim_rviz_plugin/person_display_common.hpp>
#include <pedsim_rviz_plugin/scene_node_ptr.hpp>
#include <pedsim_rviz_plugin/tracked_persons_cache.hpp>
#endif

namespace pedsim_rviz_plugin
{
    typedef unsigned int group_id;

    struct GroupAffiliationHistoryEntry
    {
        group_id groupId;
        std::shared_ptr<rviz_rendering::Shape> shape;
        bool wasOccluded, wasSinglePersonGroup;
    };

    typedef circular_buffer<std::shared_ptr<GroupAffiliationHistoryEntry> > GroupAffiliationHistory;

    class TrackedGroupsDisplay: public PersonDisplayCommon<pedsim_msgs::msg::TrackedGroups>
    {
    Q_OBJECT
    public:
        TrackedGroupsDisplay() {};
        virtual ~TrackedGroupsDisplay();

        virtual void onInitialize();
        virtual void update(float wall_dt, float ros_dt);

    protected:
        virtual void reset();

        virtual rviz_common::DisplayContext* getContext() {
            return context_;
        }

        virtual void personVisualStyleChanged() override {
            personVisualTypeChanged();
        }

    private:
        struct GroupVisual {
            vector<std::shared_ptr<rviz_rendering::Shape> > groupAssignmentCircles;
            vector<std::shared_ptr<PersonVisual> > personVisuals;
            vector<SceneNodePtr> personVisualSceneNodes;
            vector<std::shared_ptr<rviz_rendering::BillboardLine> > connectionLines;
            std::shared_ptr<TextNode> idText;
            group_id groupId;
            geometry_msgs::msg::Point groupCenter;
            size_t personCount;
        };

        void processMessage(pedsim_msgs::msg::TrackedGroups::ConstSharedPtr msg) override;

        void updateGroupVisualStyles(std::shared_ptr<GroupVisual>& groupVisual);
        void updateHistoryStyles();
        bool isGroupHidden(group_id groupId);

        SceneNodePtr m_groupAffiliationHistorySceneNode, m_groupsSceneNode;

        std::string m_realFixedFrame;

        rviz_common::properties::StringProperty* m_excluded_group_ids_property;
        rviz_common::properties::StringProperty* m_included_group_ids_property;

        rviz_common::properties::BoolProperty* m_render_intragroup_connections_property;
        rviz_common::properties::BoolProperty* m_render_ids_property;
        rviz_common::properties::BoolProperty* m_render_history_property;
        rviz_common::properties::BoolProperty* m_single_person_groups_in_constant_color_property;
        rviz_common::properties::BoolProperty* m_hide_ids_of_single_person_groups_property;

        rviz_common::properties::IntProperty*   m_history_length_property;

        rviz_common::properties::FloatProperty* m_occlusion_alpha_property;
        rviz_common::properties::FloatProperty* m_group_id_offset;

        vector<std::shared_ptr<GroupVisual> > m_groupVisuals;

        map<track_id, group_id> m_groupAffiliations;
        GroupAffiliationHistory m_groupAffiliationHistory;
        set<group_id> m_excludedGroupIDs, m_includedGroupIDs;

        Ogre::Matrix4 m_frameTransform;
        TrackedPersonsCache m_trackedPersonsCache;

    private Q_SLOTS:
        void personVisualTypeChanged();
        virtual void stylesChanged();
    };

}

#endif  // PEDSIM_RVIZ_PLUGIN__TRACKED_GROUPS_DISPLAY_HPP_
