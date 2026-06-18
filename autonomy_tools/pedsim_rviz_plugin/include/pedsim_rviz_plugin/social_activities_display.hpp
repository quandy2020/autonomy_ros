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

#ifndef PEDSIM_RVIZ_PLUGIN__SOCIAL_ACTIVITIES_DISPLAY_HPP_
#define PEDSIM_RVIZ_PLUGIN__SOCIAL_ACTIVITIES_DISPLAY_HPP_

#ifndef Q_MOC_RUN
#include <map>
#include <vector>
#include <memory>
#include <boost/circular_buffer.hpp>
#include <pedsim_msgs/msg/social_activities.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <pedsim_rviz_plugin/person_display_common.hpp>
#include <pedsim_rviz_plugin/scene_node_ptr.hpp>
#include <pedsim_rviz_plugin/tracked_persons_cache.hpp>
#endif

namespace pedsim_rviz_plugin
{
    typedef std::string activity_type;

    class SocialActivitiesDisplay: public PersonDisplayCommon<pedsim_msgs::msg::SocialActivities>
    {
    Q_OBJECT
    public:
        struct ActivityWithConfidence {
            activity_type type;
            float confidence;
        };

        SocialActivitiesDisplay() {};
        virtual ~SocialActivitiesDisplay();

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
        struct SocialActivityVisual {
            vector<std::shared_ptr<rviz_rendering::Shape> > socialActivityAssignmentCircles;
            vector<std::shared_ptr<rviz_rendering::BillboardLine> > connectionLines;
            vector<std::shared_ptr<TextNode> > typeTexts;
            vector<track_id> trackIds;
            activity_type activityType;
            float confidence;
            geometry_msgs::msg::Point socialActivityCenter;
            size_t personCount;
            size_t declutteringOffset;
        };

        void processMessage(pedsim_msgs::msg::SocialActivities::ConstSharedPtr msg) override;

        void updateSocialActivityVisualStyles(std::shared_ptr<SocialActivityVisual>& groupVisual);
        bool isActivityTypeHidden(activity_type activityType);
        Ogre::ColourValue getActivityColor(activity_type activityType, float confidence);

        SceneNodePtr m_socialActivitiesSceneNode;

        rviz_common::properties::StringProperty* m_excluded_activity_types_property;
        rviz_common::properties::StringProperty* m_included_activity_types_property;

        rviz_common::properties::BoolProperty* m_render_intraactivity_connections_property;
        rviz_common::properties::BoolProperty* m_render_activity_types_property;
        rviz_common::properties::BoolProperty* m_activity_type_per_track_property;
        rviz_common::properties::BoolProperty* m_render_confidences_property;
        rviz_common::properties::BoolProperty* m_render_circles_property;
        rviz_common::properties::BoolProperty* m_hide_with_no_activity_property;

        rviz_common::properties::FloatProperty* m_occlusion_alpha_property;
        rviz_common::properties::FloatProperty* m_min_confidence_property;
        rviz_common::properties::FloatProperty* m_circle_radius_property;
        rviz_common::properties::FloatProperty* m_circle_alpha_property;
        rviz_common::properties::FloatProperty* m_line_width_property;
        rviz_common::properties::FloatProperty* m_activity_type_offset;

        rviz_common::properties::Property* m_activity_colors;
        rviz_common::properties::ColorProperty* m_activity_color_none;
        rviz_common::properties::ColorProperty* m_activity_color_unknown;
        rviz_common::properties::ColorProperty* m_activity_color_shopping;
        rviz_common::properties::ColorProperty* m_activity_color_standing;
        rviz_common::properties::ColorProperty* m_activity_color_individual_moving;
        rviz_common::properties::ColorProperty* m_activity_color_waiting_in_queue;
        rviz_common::properties::ColorProperty* m_activity_color_looking_at_information_screen;
        rviz_common::properties::ColorProperty* m_activity_color_looking_at_kiosk;
        rviz_common::properties::ColorProperty* m_activity_color_group_assembling;
        rviz_common::properties::ColorProperty* m_activity_color_group_moving;
        rviz_common::properties::ColorProperty* m_activity_color_flow;
        rviz_common::properties::ColorProperty* m_activity_color_antiflow;
        rviz_common::properties::ColorProperty* m_activity_color_waiting_for_others;
        rviz_common::properties::ColorProperty* m_activity_color_looking_for_help;

        struct PersonVisualContainer {
            std::shared_ptr<PersonVisual> personVisual;
            SceneNodePtr sceneNode;
            track_id trackId;
        };

        vector<std::shared_ptr<SocialActivityVisual> > m_socialActivityVisuals;
        map<track_id, PersonVisualContainer > m_personVisualMap;

        map<track_id, ActivityWithConfidence> m_highestConfidenceActivityPerTrack;
        map<track_id, vector<ActivityWithConfidence> > m_allActivitiesPerTrack;

        set<activity_type> m_excludedActivityTypes, m_includedActivityTypes;

        Ogre::Matrix4 m_frameTransform;
        TrackedPersonsCache m_trackedPersonsCache;

    private Q_SLOTS:
        void personVisualTypeChanged();
        virtual void stylesChanged();
    };

}

#endif  // PEDSIM_RVIZ_PLUGIN__SOCIAL_ACTIVITIES_DISPLAY_HPP_
