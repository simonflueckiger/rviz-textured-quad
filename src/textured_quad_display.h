#ifndef SRC_TEXTURED_QUAD_DISPLAY_H
#define SRC_TEXTURED_QUAD_DISPLAY_H

#include <message_filters/subscriber.h>
#include <OgreMaterial.h>
#include <OgreManualObject.h>
#include <OgreQuaternion.h>
#include <OgreVector3.h>
#include <rviz/display.h>
#include <rviz/display_context.h>
#include <rviz/image/ros_image_texture.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <tf/message_filter.h>

#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "rviz_textured_quad/TexturedQuad.h"
#include "rviz_textured_quad/TexturedQuadArray.h"

class TexturedQuadDisplay;

class TexturedQuad {
public:
    TexturedQuad(
        TexturedQuadDisplay* owner,
        rviz::DisplayContext*
        context, Ogre::SceneNode* parent_node);
    virtual ~TexturedQuad();

    const std::string &ns() const;

    int id() const;

    void setMessage(const rviz_textured_quad::TexturedQuad::ConstPtr &msg);

    void updateFrameLocked();

    void setPosition(const Ogre::Vector3& position);

    void setOrientation(const Ogre::Quaternion& orientation);

    const Ogre::Vector3& getPosition() const;

    const Ogre::Quaternion& getOrientation() const;

protected:
    bool transform(
        const rviz_textured_quad::TexturedQuad::ConstPtr& msg,
        Ogre::Vector3& pos,
        Ogre::Quaternion& orient,
        Ogre::Vector3& scale);

    void onNewMessage(
        const rviz_textured_quad::TexturedQuad::ConstPtr& old_msg,
        const rviz_textured_quad::TexturedQuad::ConstPtr& new_msg);

    void createQuad(const rviz_textured_quad::TexturedQuad::ConstPtr& msg);
    void updateQuad(
            const rviz_textured_quad::TexturedQuad::ConstPtr& old_msg,
            const rviz_textured_quad::TexturedQuad::ConstPtr& new_msg);
    void destroyQuad();

    void createBorder(const rviz_textured_quad::TexturedQuad::ConstPtr& msg);
    void updateBorder(
            const rviz_textured_quad::TexturedQuad::ConstPtr& old_msg,
            const rviz_textured_quad::TexturedQuad::ConstPtr& new_msg);
    void destroyBorder();

    TexturedQuadDisplay* owner_;
    rviz::DisplayContext* context_;
    Ogre::SceneNode* scene_node_;
    rviz_textured_quad::TexturedQuad::ConstPtr msg_;

    Ogre::ManualObject* quad_object_;
    Ogre::MaterialPtr quad_mat_;
    std::string quad_mat_name_;
    rviz::ROSImageTexture* quad_texture_;

    Ogre::ManualObject* border_object_;
    Ogre::MaterialPtr border_mat_;
    std::string border_mat_name_;

    static uint32_t count_;
};

typedef boost::shared_ptr<TexturedQuad> TexturedQuadPtr;

class TexturedQuadDisplay : public rviz::Display {
Q_OBJECT

public:
    TexturedQuadDisplay();
    virtual ~TexturedQuadDisplay();

    virtual void onInitialize();

    virtual void update(float wall_dt, float ros_dt);

    virtual void fixedFrameChanged();
    virtual void reset();

    virtual void setTopic(const QString &topic, const QString &datatype);

protected:
    virtual void onEnable();
    virtual void onDisable();

    virtual void subscribe();
    virtual void unsubscribe();

    void incomingTexturedQuadArray(
            const rviz_textured_quad::TexturedQuadArray::ConstPtr& array);

    void incomingTexturedQuad(
            const rviz_textured_quad::TexturedQuad::ConstPtr& one);
    void failedTexturedQuad(
            const ros::MessageEvent<rviz_textured_quad::TexturedQuad>& event,
            tf::FilterFailureReason reason);

    void processMessage(const rviz_textured_quad::TexturedQuad::ConstPtr& message);

    void addTexturedQuad(
            const rviz_textured_quad::TexturedQuad::ConstPtr& message);
    void deleteTexturedQuad(
            const rviz_textured_quad::TexturedQuad::ConstPtr& message);
    void deleteAllTexturedQuads(
            const rviz_textured_quad::TexturedQuad::ConstPtr& message);

    void setTexturedQuadStatus(
            int id, rviz::StatusLevel level, const std::string& text);
    void deleteTexturedQuadStatus(int id);

    void clearTexturedQuads();

private Q_SLOTS:
    void updateQueueSize();
    void updateTopic();

private:
    rviz::RosTopicProperty* topic_property_;
    rviz::IntProperty* queue_size_property_;

    ros::Subscriber array_sub_;
    message_filters::Subscriber<rviz_textured_quad::TexturedQuad> sub_;
    tf::MessageFilter<rviz_textured_quad::TexturedQuad>* tf_filter_;

    typedef std::vector<rviz_textured_quad::TexturedQuad::ConstPtr> MessageQueue;
    MessageQueue msg_q_;
    boost::mutex msg_q_mutex_;

    typedef std::pair<std::string, int> TexturedQuadId;
    typedef std::map<TexturedQuadId, TexturedQuadPtr> TexturedQuadMap;
    TexturedQuadMap textured_quads_;

    typedef std::set<TexturedQuadPtr> TexturedQuadSet;
    TexturedQuadSet frame_locked_textured_quads_;

    friend TexturedQuad;
};

#endif  // SRC_TEXTURED_QUAD_DISPLAY_H
