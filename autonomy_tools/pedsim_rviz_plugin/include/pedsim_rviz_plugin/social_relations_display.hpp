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

#ifndef PEDSIM_RVIZ_PLUGIN__SOCIAL_RELATIONS_DISPLAY_HPP_
#define PEDSIM_RVIZ_PLUGIN__SOCIAL_RELATIONS_DISPLAY_HPP_

#ifndef Q_MOC_RUN
#include <memory>
#include <pedsim_msgs/msg/social_relations.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <pedsim_rviz_plugin/person_display_common.hpp>
#include <pedsim_rviz_plugin/scene_node_ptr.hpp>
#include <pedsim_rviz_plugin/tracked_persons_cache.hpp>
#endif

namespace pedsim_rviz_plugin
{
    class SocialRelationsDisplay: public PersonDisplayCommon<pedsim_msgs::msg::SocialRelations>
    {
    Q_OBJECT
    public:
        SocialRelationsDisplay() {};
        virtual ~SocialRelationsDisplay();

        virtual void onInitialize();

    protected:
        virtual void reset();

        virtual rviz_common::DisplayContext* getContext() {
            return context_;
        }

    private:
        struct RelationVisual {
            std::string type;
            double relationStrength;
            std::shared_ptr<rviz_rendering::BillboardLine> relationLine;
            std::shared_ptr<TextNode> relationText;
            track_id trackId1, trackId2;
        };

        void processMessage(pedsim_msgs::msg::SocialRelations::ConstSharedPtr msg) override;

        void updateRelationVisualStyles(std::shared_ptr<RelationVisual>& relationVisual);

        SceneNodePtr m_socialRelationsSceneNode;

        rviz_common::properties::StringProperty* m_relation_type_filter_property;

        rviz_common::properties::BoolProperty* m_render_positive_person_relations_property;
        rviz_common::properties::BoolProperty* m_render_negative_person_relations_property;

        rviz_common::properties::FloatProperty* m_positive_person_relation_threshold;
        rviz_common::properties::ColorProperty* m_positive_person_relations_color;
        rviz_common::properties::ColorProperty* m_negative_person_relations_color;

        vector<std::shared_ptr<RelationVisual> > m_relationVisuals;
        TrackedPersonsCache m_trackedPersonsCache;

    private Q_SLOTS:
        virtual void stylesChanged();
    };

}

#endif  // PEDSIM_RVIZ_PLUGIN__SOCIAL_RELATIONS_DISPLAY_HPP_
