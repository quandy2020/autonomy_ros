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

#include "pedsim_rviz_plugin/social_relations_display.hpp"

#ifndef Q_MOC_RUN
#include <iomanip>
#include <sstream>
#endif

namespace pedsim_rviz_plugin
{

void SocialRelationsDisplay::onInitialize()
{
    m_trackedPersonsCache.initialize(this, context_, context_->getRosNodeAbstraction());
    PersonDisplayCommon::onInitialize();

    m_relation_type_filter_property = new rviz_common::properties::StringProperty( "Relation type filter", "", "Type of social relations to display (see \"type\" field in message). No wildcards allowed. Leave empty to allow any type of relation.", this, SLOT(stylesChanged()));

    m_positive_person_relation_threshold = new rviz_common::properties::FloatProperty( "Positive relation threshold", 0.5, "Above which probability threshold a social relation between tracks is considered as positive", this, SLOT(stylesChanged()));

    m_positive_person_relations_color = new rviz_common::properties::ColorProperty( "Positive relation color", QColor(0,255,0), "Color for positive track relations", this, SLOT(stylesChanged()));
    m_negative_person_relations_color = new rviz_common::properties::ColorProperty( "Negative relation color", QColor(255,0,0), "Color for negative track relations", this, SLOT(stylesChanged()));

    m_render_positive_person_relations_property = new rviz_common::properties::BoolProperty( "Render positive relations", true,  "Render positive person relations", this, SLOT(stylesChanged()));
    m_render_negative_person_relations_property = new rviz_common::properties::BoolProperty( "Render negative relations", false, "Render negative person relations", this, SLOT(stylesChanged()));

    m_socialRelationsSceneNode = create_child_scene_node_ptr(context_->getSceneManager(), scene_node_);
}

SocialRelationsDisplay::~SocialRelationsDisplay()
{
}

void SocialRelationsDisplay::reset()
{
    PersonDisplayCommon::reset();
    m_trackedPersonsCache.reset();
    m_relationVisuals.clear();
}

void SocialRelationsDisplay::processMessage(pedsim_msgs::msg::SocialRelations::ConstSharedPtr msg)
{
    if(!preprocessMessage(msg)) return;

    m_relationVisuals.clear();
    m_socialRelationsSceneNode->removeAndDestroyAllChildren();

    for (const auto& socialRelation : msg->elements)
    {
        std::shared_ptr<CachedTrackedPerson> personTrack1 = m_trackedPersonsCache.lookup(socialRelation.track1_id);
        std::shared_ptr<CachedTrackedPerson> personTrack2 = m_trackedPersonsCache.lookup(socialRelation.track2_id);

        if(!personTrack1 || !personTrack2) continue;

        std::shared_ptr<RelationVisual> relationVisual = std::shared_ptr<RelationVisual>(new RelationVisual);
        relationVisual->type = socialRelation.type;
        relationVisual->relationStrength = socialRelation.strength;
        m_relationVisuals.push_back(relationVisual);

        const Ogre::Vector3 verticalShift(0,0, 1.0 + m_commonProperties->z_offset->getFloat()), textShift(0,0, 0.3);
        const Ogre::Vector3& position1 = verticalShift + personTrack1->center;
        const Ogre::Vector3& position2 = verticalShift + personTrack2->center;
        const Ogre::Vector3& centerPosition = (position1 + position2) / 2.0;

        std::shared_ptr<rviz_rendering::BillboardLine> relationLine(
            new rviz_rendering::BillboardLine(context_->getSceneManager(), m_socialRelationsSceneNode.get()));
        relationLine->setMaxPointsPerLine(2);
        relationLine->addPoint(position1);
        relationLine->addPoint(position2);
        relationVisual->relationLine = relationLine;

        stringstream ss;
        std::shared_ptr<TextNode> relationText(new TextNode(context_->getSceneManager(), m_socialRelationsSceneNode.get()));
        ss.str(""); ss << std::fixed << std::setprecision(0) << socialRelation.strength * 100 << "%";
        relationText->setCaption(ss.str());
        relationText->setPosition(centerPosition + textShift);
        relationText->showOnTop();
        relationVisual->relationText = relationText;

        relationVisual->trackId1 = socialRelation.track1_id;
        relationVisual->trackId2 = socialRelation.track2_id;

        updateRelationVisualStyles(relationVisual);
    }
}

void SocialRelationsDisplay::stylesChanged()
{
    for (auto& relationVisual : m_relationVisuals) {
        updateRelationVisualStyles(relationVisual);
    }
}

void SocialRelationsDisplay::updateRelationVisualStyles(std::shared_ptr<RelationVisual>& relationVisual)
{
    std::string typeFilter = m_relation_type_filter_property->getStdString();
    bool validRelationType = relationVisual->type.find(typeFilter) != std::string::npos;
    bool hideRelation = !validRelationType || isPersonHidden(relationVisual->trackId1) || isPersonHidden(relationVisual->trackId2);

    bool isPositiveRelation = relationVisual->relationStrength > m_positive_person_relation_threshold->getFloat();

    Ogre::ColourValue relationColor = isPositiveRelation ? m_positive_person_relations_color->getOgreColor() : m_negative_person_relations_color->getOgreColor();
    relationColor.a *= m_commonProperties->alpha->getFloat();
    if(hideRelation) relationColor.a = 0;

    if(isPositiveRelation && !m_render_positive_person_relations_property->getBool()) relationColor.a = 0;
    if(!isPositiveRelation && !m_render_negative_person_relations_property->getBool()) relationColor.a = 0;

    relationVisual->relationLine->setLineWidth(0.03 * (isPositiveRelation ? 1.0 : 0.3));
    relationVisual->relationLine->setColor(relationColor.r, relationColor.g, relationColor.b, relationColor.a);

    relationVisual->relationText->setCharacterHeight(0.15 * m_commonProperties->font_scale->getFloat());
    relationVisual->relationText->setColor(relationColor);
}

}  // namespace pedsim_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pedsim_rviz_plugin::SocialRelationsDisplay, rviz_common::Display)
