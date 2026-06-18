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

#include "pedsim_rviz_plugin/tracked_groups_display.hpp"

#ifndef Q_MOC_RUN
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#endif

namespace pedsim_rviz_plugin
{

void TrackedGroupsDisplay::onInitialize()
{
    m_realFixedFrame = "map";

    m_trackedPersonsCache.initialize(this, context_, context_->getRosNodeAbstraction());
    PersonDisplayCommon::onInitialize();

    m_excluded_group_ids_property = new rviz_common::properties::StringProperty( "Excluded group IDs", "", "Comma-separated list of group IDs whose group visualization should be hidden", this, SLOT(stylesChanged()) );
    m_included_group_ids_property = new rviz_common::properties::StringProperty( "Included group IDs", "", "Comma-separated list of group IDs whose group visualization should be visible, overrides excluded", this, SLOT(stylesChanged()) );

    m_render_intragroup_connections_property = new rviz_common::properties::BoolProperty( "Connect group members", true, "Connect all members of a group by lines", this, SLOT(stylesChanged()));
    m_render_ids_property = new rviz_common::properties::BoolProperty( "Render group IDs", true, "Render group IDs as text", this, SLOT(stylesChanged()));
    m_render_history_property = new rviz_common::properties::BoolProperty( "Render history", false, "Render group affiliation history", this, SLOT(stylesChanged()));

    m_single_person_groups_in_constant_color_property  = new rviz_common::properties::BoolProperty( "Single-person groups in constant color", true, "Render single-person groups in constant color", this, SLOT(stylesChanged()));
    m_hide_ids_of_single_person_groups_property = new rviz_common::properties::BoolProperty( "Hide IDs of single-person groups", false, "Hide IDs of single-person groups", m_render_ids_property, SLOT(stylesChanged()), this);

    m_history_length_property = new rviz_common::properties::IntProperty( "Global history size", 1000, "Global number of group affiliation history entries to display.", this, SLOT(stylesChanged()));
    m_history_length_property->setMin( 1 );
    m_history_length_property->setMax( 10000000 );

    m_occlusion_alpha_property = new rviz_common::properties::FloatProperty( "Occlusion alpha", 0.5, "Alpha multiplier for history of occluded tracks", this, SLOT(stylesChanged()) );
    m_occlusion_alpha_property->setMin( 0.0 );

    m_group_id_offset = new rviz_common::properties::FloatProperty( "Group ID Z offset", 2.0, "Offset in z position (height) of the group ID text", this, SLOT(stylesChanged()) );

    m_groupAffiliationHistorySceneNode = create_child_scene_node_ptr(context_->getSceneManager(), scene_node_);
    m_groupsSceneNode = create_child_scene_node_ptr(context_->getSceneManager(), scene_node_);
}

TrackedGroupsDisplay::~TrackedGroupsDisplay()
{
}

void TrackedGroupsDisplay::reset()
{
    PersonDisplayCommon::reset();
    m_trackedPersonsCache.reset();
    m_groupVisuals.clear();
    m_groupAffiliationHistory.clear();
    m_groupAffiliations.clear();
}

void TrackedGroupsDisplay::update(float wall_dt, float ros_dt)
{
    (void) wall_dt;
    (void) ros_dt;

    Ogre::Vector3 mapFramePosition;
    Ogre::Quaternion mapFrameOrientation;
    rclcpp::Clock clock(RCL_ROS_TIME);
    getContext()->getFrameManager()->getTransform(
        m_realFixedFrame, clock.now(), mapFramePosition, mapFrameOrientation);
    m_groupAffiliationHistorySceneNode->setPosition(mapFramePosition);
    m_groupAffiliationHistorySceneNode->setOrientation(mapFrameOrientation);
}

bool TrackedGroupsDisplay::isGroupHidden(group_id groupId) {
    bool isIncluded = m_includedGroupIDs.find(groupId) != m_includedGroupIDs.end();
    if(isIncluded) return false;
    if(!m_includedGroupIDs.empty()) return true;

    return m_excludedGroupIDs.find(groupId) != m_excludedGroupIDs.end();
}

void TrackedGroupsDisplay::stylesChanged()
{
     m_excludedGroupIDs.clear();
     {
         string groupIDString = m_excluded_group_ids_property->getStdString();
         char_separator<char> separator(",");
         tokenizer< char_separator<char> > tokens(groupIDString, separator);
         for (const string& token : tokens) {
             try { m_excludedGroupIDs.insert(lexical_cast<track_id>(token)); }
             catch(bad_lexical_cast &) {}
         }
     }
     m_includedGroupIDs.clear();
     {
        string groupIDString = m_included_group_ids_property->getStdString();
        char_separator<char> separator(",");
        tokenizer< char_separator<char> > tokens(groupIDString, separator);
        for (const string& token : tokens) {
            try { m_includedGroupIDs.insert(lexical_cast<track_id>(token)); }
            catch(bad_lexical_cast &) {}
        }
    }

    for (auto& groupVisual : m_groupVisuals) {
        updateGroupVisualStyles(groupVisual);
    }

    m_groupAffiliationHistory.rset_capacity(m_history_length_property->getInt());
    updateHistoryStyles();
}

void TrackedGroupsDisplay::personVisualTypeChanged()
{
    Ogre::SceneManager* scene_manager = context_->getSceneManager();
    for (auto& groupVisual : m_groupVisuals) {
        for (auto& personVisual : groupVisual->personVisuals) {
            Ogre::SceneNode* parentSceneNode = personVisual->getParentSceneNode();
            personVisual.reset();
            destroyStalePersonVisualNodes(scene_manager, parentSceneNode);
            createPersonVisualIfRequired(parentSceneNode, personVisual);
        }
    }
    stylesChanged();
}

void TrackedGroupsDisplay::updateGroupVisualStyles(std::shared_ptr<GroupVisual>& groupVisual)
{
    bool hideGroup = isGroupHidden(groupVisual->groupId);

    Ogre::ColourValue groupColor = m_commonProperties->constant_color->getOgreColor();

    if(groupVisual->personCount > 1 || !m_single_person_groups_in_constant_color_property->getBool()) {
        groupColor = getColorFromId(groupVisual->groupId);
    }

    groupColor.a *= m_commonProperties->alpha->getFloat();
    if(hideGroup) groupColor.a = 0;

    for (auto& groupAssignmentCircle : groupVisual->groupAssignmentCircles) {
        groupAssignmentCircle->setColor(groupColor.r, groupColor.g, groupColor.b, groupColor.a);
    }

    for (auto& personVisual : groupVisual->personVisuals) {
        if(personVisual) {
            applyCommonStyles(personVisual);
            personVisual->setColor(groupColor);
        }
    }

    double connectionLineVisibilityAlpha = m_render_intragroup_connections_property->getBool() ? 1.0 : 0.0;
    for (auto& connectionLine : groupVisual->connectionLines) {
        connectionLine->setColor(groupColor.r, groupColor.g, groupColor.b, groupColor.a * connectionLineVisibilityAlpha);
        connectionLine->setLineWidth(0.05);
    }

    Ogre::ColourValue fontColor = m_commonProperties->font_color_style->getOptionInt() == FONT_COLOR_CONSTANT ? m_commonProperties->constant_font_color->getOgreColor() : groupColor;
    fontColor.a = m_commonProperties->alpha->getFloat();
    if(hideGroup) fontColor.a = 0;
    bool groupIdVisible = groupVisual->personCount > 1 ? true : !m_hide_ids_of_single_person_groups_property->getBool();
    groupVisual->idText->setVisible(m_render_ids_property->getBool() && groupIdVisible);
    groupVisual->idText->setCharacterHeight(0.23 * m_commonProperties->font_scale->getFloat());
    groupVisual->idText->setColor(fontColor);
    groupVisual->idText->setPosition(m_frameTransform * Ogre::Vector3(
        groupVisual->groupCenter.x,
        groupVisual->groupCenter.y,
        groupVisual->groupCenter.z + m_group_id_offset->getFloat() + m_commonProperties->z_offset->getFloat()));
}

void TrackedGroupsDisplay::updateHistoryStyles()
{
    const Ogre::Quaternion shapeQuaternion( Ogre::Degree(90), Ogre::Vector3(1,0,0) );
    for (auto& groupAffiliationHistoryEntry : m_groupAffiliationHistory) {
        bool hideGroup = isGroupHidden(groupAffiliationHistoryEntry->groupId);
        const double historyShapeDiameter = 0.1;

        Ogre::ColourValue historyColor = m_commonProperties->constant_color->getOgreColor();

        if(!groupAffiliationHistoryEntry->wasSinglePersonGroup || !m_single_person_groups_in_constant_color_property->getBool()) {
            historyColor = getColorFromId(groupAffiliationHistoryEntry->groupId);
        }

        historyColor.a = m_commonProperties->alpha->getFloat();

        if(groupAffiliationHistoryEntry->wasOccluded) historyColor.a *= m_occlusion_alpha_property->getFloat();
        if(hideGroup) historyColor.a = 0;
        if(!m_render_history_property->getBool()) historyColor.a = 0;

        groupAffiliationHistoryEntry->shape->setColor(historyColor);
        groupAffiliationHistoryEntry->shape->setScale(shapeQuaternion * Ogre::Vector3(historyShapeDiameter, historyShapeDiameter, 0.05));
    }
}

void TrackedGroupsDisplay::processMessage(pedsim_msgs::msg::TrackedGroups::ConstSharedPtr msg)
{
    if(!preprocessMessage(msg)) return;

    Ogre::Vector3 mapFramePosition;
    Ogre::Quaternion mapFrameOrientation;
    getContext()->getFrameManager()->getTransform(
        m_realFixedFrame, msg->header.stamp, mapFramePosition, mapFrameOrientation);
    Ogre::Matrix4 mapFrameTransform(mapFrameOrientation);
    mapFrameTransform.setTrans(mapFramePosition);

    m_frameTransform = Ogre::Matrix4(m_frameOrientation);
    m_frameTransform.setTrans(m_framePosition);

    const Ogre::Quaternion shapeQuaternion( Ogre::Degree(90), Ogre::Vector3(1,0,0) );
    stringstream ss;

    m_groupVisuals.clear();
    m_groupsSceneNode->removeAndDestroyAllChildren();

    unsigned int numTracksWithUnknownPosition = 0;
    m_groupAffiliations.clear();

    for (const auto& trackedGroup : msg->groups)
    {
        std::shared_ptr<GroupVisual> groupVisual = std::shared_ptr<GroupVisual>(new GroupVisual);
        groupVisual->groupId = trackedGroup.group_id;
        groupVisual->personCount = trackedGroup.track_ids.size();
        m_groupVisuals.push_back(groupVisual);

        for(size_t trackIndex = 0; trackIndex < trackedGroup.track_ids.size(); trackIndex++)
        {
            const track_id trackId = trackedGroup.track_ids[trackIndex];
            std::shared_ptr<CachedTrackedPerson> trackedPerson = m_trackedPersonsCache.lookup(trackId);

            if(!trackedPerson) {
                numTracksWithUnknownPosition++;
            }
            else
            {
                Ogre::Vector3 trackCenterAtGroundPlane(
                    trackedPerson->center.x, trackedPerson->center.y,
                    m_commonProperties->z_offset->getFloat());

                m_groupAffiliations[trackId] = trackedGroup.group_id;

                const double groupAssignmentCircleHeight = 0;
                const double groupAssignmentCircleDiameter = 0.9;
                std::shared_ptr<rviz_rendering::Shape> groupAssignmentCircle = std::shared_ptr<rviz_rendering::Shape>(
                    new rviz_rendering::Shape(
                        rviz_rendering::Shape::Cylinder, context_->getSceneManager(),
                        m_groupsSceneNode.get()));

                groupAssignmentCircle->setScale(shapeQuaternion * Ogre::Vector3(
                    groupAssignmentCircleDiameter, groupAssignmentCircleDiameter,
                    groupAssignmentCircleHeight));
                groupAssignmentCircle->setOrientation(shapeQuaternion);

                Ogre::Vector3 groupAssignmentCirclePos = trackCenterAtGroundPlane + Ogre::Vector3(0, 0, -0.5*groupAssignmentCircleHeight - 0.01);
                groupAssignmentCircle->setPosition(groupAssignmentCirclePos);

                groupVisual->groupAssignmentCircles.push_back(groupAssignmentCircle);

                SceneNodePtr sceneNode = create_child_scene_node_ptr(context_->getSceneManager(), scene_node_);
                groupVisual->personVisualSceneNodes.push_back(sceneNode);

                std::shared_ptr<PersonVisual> personVisual;
                createPersonVisualIfRequired(sceneNode.get(), personVisual);
                groupVisual->personVisuals.push_back(personVisual);

                const double personHeight = personVisual ? personVisual->getHeight() : 0;
                const Ogre::Matrix3 covXYZinTargetFrame = covarianceXYZIntoTargetFrame(trackedPerson->pose);
                setPoseOrientation(sceneNode.get(), trackedPerson->pose, covXYZinTargetFrame, personHeight);

                for(size_t otherTrackIndex = trackIndex + 1; otherTrackIndex < trackedGroup.track_ids.size(); otherTrackIndex++)
                {
                    const track_id otherTrackId = trackedGroup.track_ids[otherTrackIndex];
                    std::shared_ptr<CachedTrackedPerson> otherTrackedPerson = m_trackedPersonsCache.lookup(otherTrackId);

                    if(otherTrackedPerson) {
                        const Ogre::Vector3 verticalShift(0,0, 0.5 + m_commonProperties->z_offset->getFloat());
                        const Ogre::Vector3& position1 = verticalShift + trackedPerson->center;
                        const Ogre::Vector3& position2 = verticalShift + otherTrackedPerson->center;

                        std::shared_ptr<rviz_rendering::BillboardLine> connectionLine(
                            new rviz_rendering::BillboardLine(context_->getSceneManager(), m_groupsSceneNode.get()));
                        connectionLine->setMaxPointsPerLine(2);
                        connectionLine->addPoint(position1);
                        connectionLine->addPoint(position2);
                        groupVisual->connectionLines.push_back(connectionLine);
                    }
                }

                std::shared_ptr<GroupAffiliationHistoryEntry> newHistoryEntry(new GroupAffiliationHistoryEntry);
                newHistoryEntry->shape = std::shared_ptr<rviz_rendering::Shape>(
                    new rviz_rendering::Shape(
                        rviz_rendering::Shape::Cylinder, context_->getSceneManager(),
                        m_groupAffiliationHistorySceneNode.get()));
                newHistoryEntry->shape->setPosition(mapFrameTransform.inverse() * trackCenterAtGroundPlane);
                newHistoryEntry->shape->setOrientation(shapeQuaternion);
                newHistoryEntry->wasOccluded = trackedPerson->isOccluded;
                newHistoryEntry->wasSinglePersonGroup = trackedGroup.track_ids.size() <= 1;
                newHistoryEntry->groupId = trackedGroup.group_id;
                m_groupAffiliationHistory.push_back(newHistoryEntry);
            }
        }

        const geometry_msgs::msg::Point& groupCenter = trackedGroup.center_of_gravity.pose.position;
        groupVisual->groupCenter = groupCenter;

        std::shared_ptr<TextNode> idText(new TextNode(context_->getSceneManager(), m_groupsSceneNode.get()));
        ss.str(""); ss << "group " << trackedGroup.group_id;
        idText->setCaption(ss.str());
        idText->showOnTop();
        groupVisual->idText = idText;

        updateGroupVisualStyles(groupVisual);
        updateHistoryStyles();
    }

    ss.str("");
    ss << msg->groups.size() << " group(s)";
    setStatusStd(rviz_common::properties::StatusProperty::Ok, "Groups", ss.str());

    ss.str("");
    ss << numTracksWithUnknownPosition << " track(s) with unknown position";
    setStatusStd(
        0 == numTracksWithUnknownPosition ? rviz_common::properties::StatusProperty::Ok :
        rviz_common::properties::StatusProperty::Warn,
        "Track-to-group assignment", ss.str());
}

}  // namespace pedsim_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pedsim_rviz_plugin::TrackedGroupsDisplay, rviz_common::Display)
