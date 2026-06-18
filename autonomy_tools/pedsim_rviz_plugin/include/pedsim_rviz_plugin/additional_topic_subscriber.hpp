/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013-2015, Timm Linder, Social Robotics Lab, University of Freiburg
*  Copyright (c) 2012, Willow Garage, Inc.
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
#ifndef PEDSIM_RVIZ_PLUGIN__ADDITIONAL_TOPIC_SUBSCRIBER_HPP_
#define PEDSIM_RVIZ_PLUGIN__ADDITIONAL_TOPIC_SUBSCRIBER_HPP_

#ifndef Q_MOC_RUN

#include <functional>
#include <memory>
#include <string>

#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#endif

namespace pedsim_rviz_plugin
{

/** @brief Helper superclass for AdditionalTopicSubscriber, needed because
 * Qt's moc and c++ templates don't work nicely together.  Not
 * intended to be used directly. */
class _AdditionalTopicSubscriber: public QObject
{
Q_OBJECT
public:
    void initialize(
        rviz_common::Display* display,
        rviz_common::DisplayContext* context,
        rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node)
    {
        m_display = display;
        m_context = context;
        m_rviz_ros_node = rviz_ros_node;

        QObject::connect(display, SIGNAL(changed()), this, SLOT(displayEnableChanged()));
        QObject::connect(
            context->getFrameManager(),
            SIGNAL(fixedFrameChanged()),
            this,
            SLOT(fixedFrameChanged()));

        additional_topic_property_ = new rviz_common::properties::RosTopicProperty(
            "Additional topic", "", "", "", display, SLOT(updateTopic()));
        additional_topic_property_->initialize(rviz_ros_node);
    }

protected Q_SLOTS:
    virtual void updateTopic() = 0;
    virtual void displayEnableChanged() = 0;
    virtual void fixedFrameChanged() = 0;

protected:
    rviz_common::properties::RosTopicProperty* additional_topic_property_;
    rviz_common::Display* m_display;
    rviz_common::DisplayContext* m_context;
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr m_rviz_ros_node;
};

/** @brief Display helper using a tf2_ros::MessageFilter, templated on the ROS message type. */
template<class MessageType>
class AdditionalTopicSubscriber: public _AdditionalTopicSubscriber
{
public:
    typedef AdditionalTopicSubscriber<MessageType> ATSClass;

    AdditionalTopicSubscriber(
        const QString& propertyName,
        rviz_common::Display* display,
        rviz_common::DisplayContext* context,
        rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
        const std::function<void(typename MessageType::ConstSharedPtr)>& messageCallback)
    : m_messagesReceived(0),
      m_enabled(false),
      m_messageCallback(messageCallback),
      m_qos_profile(10)
    {
        _AdditionalTopicSubscriber::initialize(display, context, rviz_ros_node);

        additional_topic_property_->setName(propertyName);
        QString message_type = QString::fromStdString(rosidl_generator_traits::name<MessageType>());
        additional_topic_property_->setMessageType(message_type);
        additional_topic_property_->setDescription(message_type + " topic to subscribe to.");

        setEnabled(m_display->isEnabled());
        updateTopic();
        fixedFrameChanged();
    }

    virtual ~AdditionalTopicSubscriber()
    {
        unsubscribe();
    }

    virtual void reset()
    {
        if (tf_filter_) {
            tf_filter_->clear();
        }
        m_messagesReceived = 0;
    }

    void setEnabled(bool enabled)
    {
        m_enabled = enabled;
        if (enabled) {
            subscribe();
        } else {
            unsubscribe();
        }
    }

protected:
    virtual void updateTopic() override
    {
        unsubscribe();
        reset();
        subscribe();
        m_context->queueRender();
    }

    virtual void displayEnableChanged() override
    {
        setEnabled(m_display->isEnabled());
    }

    virtual void fixedFrameChanged() override
    {
        if (tf_filter_) {
            tf_filter_->setTargetFrame(m_context->getFixedFrame().toStdString());
        }
        reset();
    }

    virtual void subscribe()
    {
        if (!m_display->isEnabled()) {
            return;
        }

        if (additional_topic_property_->isEmpty()) {
            return;
        }

        try {
            auto node_interface = m_rviz_ros_node.lock();
            if (!node_interface) {
                return;
            }
            rclcpp::Node::SharedPtr node = node_interface->get_raw_node();
            subscription_ = std::make_shared<message_filters::Subscriber<MessageType>>(
                node,
                additional_topic_property_->getTopicStd(),
                m_qos_profile.get_rmw_qos_profile());
            tf_filter_ = std::make_shared<
                tf2_ros::MessageFilter<MessageType, rviz_common::transformation::FrameTransformer>>(
                *m_context->getFrameManager()->getTransformer(),
                m_context->getFixedFrame().toStdString(),
                10,
                node);
            tf_filter_->connectInput(*subscription_);
            tf_filter_->registerCallback(
                std::bind(
                    &AdditionalTopicSubscriber<MessageType>::incomingMessage, this,
                    std::placeholders::_1));
            m_display->setStatus(
                rviz_common::properties::StatusProperty::Ok,
                additional_topic_property_->getName(),
                "OK");
        } catch (rclcpp::exceptions::InvalidTopicNameError& e) {
            m_display->setStatus(
                rviz_common::properties::StatusProperty::Error,
                additional_topic_property_->getName(),
                QString("Error subscribing: ") + e.what());
        }
    }

    virtual void unsubscribe()
    {
        tf_filter_.reset();
        subscription_.reset();
    }

    void incomingMessage(typename MessageType::ConstSharedPtr msg)
    {
        if (!msg) {
            return;
        }

        ++m_messagesReceived;
        m_display->setStatus(
            rviz_common::properties::StatusProperty::Ok,
            additional_topic_property_->getName(),
            QString::number(m_messagesReceived) + " messages received");

        m_messageCallback(msg);
    }

private:
    std::shared_ptr<message_filters::Subscriber<MessageType>> subscription_;
    std::shared_ptr<
        tf2_ros::MessageFilter<MessageType, rviz_common::transformation::FrameTransformer>>
    tf_filter_;
    uint32_t m_messagesReceived;
    bool m_enabled;
    const std::function<void(typename MessageType::ConstSharedPtr)> m_messageCallback;
    rclcpp::QoS m_qos_profile;
};

}  // namespace pedsim_rviz_plugin

#endif  // PEDSIM_RVIZ_PLUGIN__ADDITIONAL_TOPIC_SUBSCRIBER_HPP_
