#include "./textured_quad_display.h"

#include <cv_bridge/cv_bridge.h>
#include <OgreSceneManager.h>
#include <rviz/frame_manager.h>
#include <rviz/uniform_string_stream.h>
#include <shape_msgs/MeshTriangle.h>

#include <algorithm>
#include <string>
#include <vector>

// TexturedQuad

uint32_t TexturedQuad::count_ = 0;

TexturedQuad::TexturedQuad(
    TexturedQuadDisplay* owner,
    rviz::DisplayContext* context,
    Ogre::SceneNode* parent_node) :
    owner_(owner),
    context_(context),
    scene_node_(parent_node->createChildSceneNode()),
    quad_object_(NULL),
    quad_texture_(NULL),
    border_object_(NULL) {
}

TexturedQuad::~TexturedQuad() {
    context_->getSceneManager()->destroySceneNode(scene_node_);
    destroyQuad();
    destroyBorder();
}

const std::string &TexturedQuad::ns() const {
    return msg_->ns;
}

int TexturedQuad::id() const {
    return msg_->id;
}

void TexturedQuad::setMessage(
        const rviz_textured_quad::TexturedQuad::ConstPtr &message) {
    rviz_textured_quad::TexturedQuad::ConstPtr old = msg_;
    msg_ = message;
    onNewMessage(old, message);
}

void TexturedQuad::updateFrameLocked() {
    onNewMessage(msg_, msg_);
}

bool TexturedQuad::transform(
        const rviz_textured_quad::TexturedQuad::ConstPtr& message,
        Ogre::Vector3& pos,
        Ogre::Quaternion& orient,
        Ogre::Vector3& scale) {
    ros::Time stamp = message->header.stamp;
    if (message->frame_locked) {
        stamp = ros::Time();
    }

    if (!context_->getFrameManager()->transform(
            message->header.frame_id, stamp, message->pose, pos, orient)) {
        std::string error;
        context_->getFrameManager()->transformHasProblems(
                message->header.frame_id, message->header.stamp, error);
        if (owner_) {
            owner_->setTexturedQuadStatus(
                    id(), rviz::StatusProperty::Error, error);
        }
        return false;
  }

  scale = Ogre::Vector3(message->scale.x, message->scale.y, message->scale.z);

  return true;
}

void TexturedQuad::setPosition(const Ogre::Vector3& position) {
    scene_node_->setPosition(position);
}

void TexturedQuad::setOrientation(const Ogre::Quaternion& orientation) {
    scene_node_->setOrientation(orientation);
}

const Ogre::Vector3& TexturedQuad::getPosition() const {
    return scene_node_->getPosition();
}

const Ogre::Quaternion& TexturedQuad::getOrientation() const {
    return scene_node_->getOrientation();
}

void TexturedQuad::onNewMessage(
        const rviz_textured_quad::TexturedQuad::ConstPtr& old_msg,
        const rviz_textured_quad::TexturedQuad::ConstPtr& new_msg) {
    // tf
    Ogre::Vector3 pos, scale;
    Ogre::Quaternion orient;
    if (!transform(new_msg, pos, orient, scale)) {
        ROS_WARN("unable to transform texture quad message");
        scene_node_->setVisible(false);
        return;
    }
    bool zero = (
        new_msg->scale.x *
        new_msg->scale.y *
        new_msg->scale.z == 0.0f);
    if (owner_ && zero) {
        owner_->setTexturedQuadStatus(
           id(), rviz::StatusProperty::Warn, "Scale of 0 in one of x/y/z");
    }
    setPosition(pos);
    setOrientation(orient);
    scene_node_->setScale(scale);

    // quad
    if (!quad_object_) {
        createQuad(new_msg);
    }
    updateQuad(old_msg, new_msg);

    // border
    if (!border_object_ && new_msg->border_size != 0) {
        createBorder(new_msg);
    } else if (border_object_ && new_msg->border_size == 0) {
        destroyBorder();
    }
    if (border_object_) {
        updateBorder(old_msg, new_msg);
    }
}

void TexturedQuad::createQuad(
        const rviz_textured_quad::TexturedQuad::ConstPtr& msg) {
    rviz::UniformStringStream ss;
    ss << "TextureQuad Quad " << count_++;
    quad_object_ = context_->getSceneManager()->createManualObject(
            ss.str());
    scene_node_->attachObject(quad_object_);

    ss << " Material";
    quad_mat_name_ = ss.str();
    // TODO(me): own resource manager not "rviz"?
    quad_mat_ = Ogre::MaterialManager::getSingleton().create(
            quad_mat_name_, "rviz");
    quad_mat_->setReceiveShadows(false);
    quad_mat_->setCullingMode(Ogre::CULL_NONE);
    Ogre::Technique* tech = quad_mat_->getTechnique(0);
    Ogre::Pass* pass = tech->getPass(0);
    tech->setLightingEnabled(false);
    if (msg->alpha != 0) {
        // decal
        pass->setAlphaRejectSettings(Ogre::CMPF_GREATER, msg->alpha);
        tech->setDepthWriteEnabled(true);
    } else {
        // translucent overlay
        tech->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        tech->setDepthWriteEnabled(false);
    }
    pass->createTextureUnitState();

    quad_texture_ = new rviz::ROSImageTexture();
}

void TexturedQuad::updateQuad(
            const rviz_textured_quad::TexturedQuad::ConstPtr& old_msg,
            const rviz_textured_quad::TexturedQuad::ConstPtr& new_msg) {
    const size_t num_points = 4;

    if (old_msg) {
        // update existing
        quad_object_->beginUpdate(0);
    } else {
        quad_object_->clear();
        quad_object_->estimateVertexCount(num_points * 2);  // both sides
        quad_object_->begin(
                quad_mat_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);
    }

    const float hw = new_msg->width/2.0;
    const float hh = new_msg->height/2.0;
    std::vector<geometry_msgs::Point> points(4);
    points[0].x = -hw;
    points[0].y = hh;
    points[0].z = 0;

    points[1].x = hw;
    points[1].y = hh;
    points[1].z = 0;

    points[2].x = hw;
    points[2].y = -hh;
    points[2].z = 0;

    points[3].x = -hw;
    points[3].y = -hh;
    points[3].z = 0;

    std::vector<shape_msgs::MeshTriangle> triangles(2);
    triangles[0].vertex_indices[0] = 0;
    triangles[0].vertex_indices[1] = 1;
    triangles[0].vertex_indices[2] = 3;
    triangles[1].vertex_indices[0] = 1;
    triangles[1].vertex_indices[1] = 2;
    triangles[1].vertex_indices[2] = 3;

    std::vector<Ogre::Vector2> tex_coords(4);
    tex_coords[0].x = 0.0;
    tex_coords[0].y = 1.0;
    tex_coords[1].x = 1.0;
    tex_coords[1].y = 1.0;
    tex_coords[2].x = 1.0;
    tex_coords[2].y = 0.0;
    tex_coords[3].x = 0.0;
    tex_coords[3].y = 0.0;

    for (size_t i = 0; i < triangles.size(); i++) {
        // front-face + back-face triangles
        for (int side = 0; side < 2; side++) {
            // order
            Ogre::Vector3 v[3];
            Ogre::Vector2 tc[3];
            for (size_t c = 0; c < 3; c++) {
                // order of corners if side == 1
                size_t corner = side ? 2 - c : c;
                v[corner] = Ogre::Vector3(
                    points[triangles[i].vertex_indices[corner]].x,
                    points[triangles[i].vertex_indices[corner]].y,
                    points[triangles[i].vertex_indices[corner]].z);
                tc[corner] = Ogre::Vector2(
                    tex_coords[triangles[i].vertex_indices[corner]].x,
                    tex_coords[triangles[i].vertex_indices[corner]].y);
            }
            Ogre::Vector3 normal = (v[1] - v[0]).crossProduct(v[2] - v[0]);
            normal.normalise();

            // add
            for (size_t c = 0; c < 3; c++) {
                quad_object_->position(v[c]);
                quad_object_->normal(normal);
                quad_object_->textureCoord(tc[c]);
            }
        }
    }

    quad_object_->end();

    // texture
    // TODO(me): better way?
    sensor_msgs::Image::Ptr image(new sensor_msgs::Image(new_msg->image));
    quad_texture_->addMessage(image);
    quad_texture_->update();
    Ogre::Pass* pass = quad_mat_->getTechnique(0)->getPass(0);
    Ogre::TextureUnitState* tex_state = pass->getTextureUnitState(0);
    tex_state->setTextureName(quad_texture_->getTexture()->getName());
}

void TexturedQuad::destroyQuad() {
    if (quad_object_ != NULL) {
        context_->getSceneManager()->destroyManualObject(quad_object_);
        quad_object_ = NULL;
    }
    if (!quad_mat_.isNull()) {
        quad_mat_->unload();
        Ogre::MaterialManager::getSingleton().remove(quad_mat_->getName());
        quad_mat_.setNull();
    }
    if (quad_texture_ != NULL) {
        delete quad_texture_;
        quad_texture_ = NULL;
    }
}

void TexturedQuad::createBorder(
        const rviz_textured_quad::TexturedQuad::ConstPtr& msg) {
    rviz::UniformStringStream ss;
    ss << "TextureQuad Border " << count_++;  // append
    border_object_ = context_->getSceneManager()->createManualObject(
            ss.str());
    scene_node_->attachObject(border_object_);

    ss << " Material";  // append
    border_mat_name_ = ss.str();
    // TODO(me): own resource manager not "rviz"?
    border_mat_ = Ogre::MaterialManager::getSingleton().create(
            border_mat_name_, "rviz");
    border_mat_->setReceiveShadows(false);
    border_mat_->setCullingMode(Ogre::CULL_ANTICLOCKWISE);
    Ogre::Technique* tech = border_mat_->getTechnique(0);
    Ogre::Pass* pass = tech->getPass(0);
    pass->setSelfIllumination(
            msg->border_color.r, msg->border_color.g, msg->border_color.b);
    pass->setLightingEnabled(true);
}

void TexturedQuad::updateBorder(
            const rviz_textured_quad::TexturedQuad::ConstPtr& old_msg,
            const rviz_textured_quad::TexturedQuad::ConstPtr& new_msg) {
    const size_t num_points = 16;

    if (old_msg) {
        // update existing
        border_object_->beginUpdate(0);
    } else {
        border_object_->clear();
        border_object_->estimateVertexCount(num_points * 2);  // both sides
        border_object_->begin(
                border_mat_name_,
                Ogre::RenderOperation::OT_TRIANGLE_LIST);
    }

    const float hw = new_msg->width/2.0;
    const float hh = new_msg->height/2.0;
    const float b = new_msg->border_size;

    Ogre::Vector3 points[] = {
            // left
            Ogre::Vector3(-hw, hh, 0),
            Ogre::Vector3(-hw + b, hh, 0),
            Ogre::Vector3(-hw + b, -hh, 0),
            Ogre::Vector3(-hw, -hh, 0),

            // top
            Ogre::Vector3(-hw + b, hh - b, 0),
            Ogre::Vector3(-hw + b, hh, 0),
            Ogre::Vector3(hw - b, hh, 0),
            Ogre::Vector3(hw - b, hh - b, 0),

            // right
            Ogre::Vector3(hw - b, hh, 0),
            Ogre::Vector3(hw, hh, 0),
            Ogre::Vector3(hw, -hh, 0),
            Ogre::Vector3(hw - b, -hh, 0),

            // bottom
            Ogre::Vector3(-hw + b, -hh, 0),
            Ogre::Vector3(-hw + b, -hh + b, 0),
            Ogre::Vector3(hw - b, -hh + b, 0),
            Ogre::Vector3(hw - b, -hh, 0),
    };

    int triangles[4 * 2 * 3] = {
            // left
            0, 1, 2,
            0, 2, 3,

            // top
            4, 5, 6,
            4, 6, 7,

            // right
            8, 9, 10,
            8, 10, 11,

            // bottom
            12, 13, 14,
            12, 14, 15,
    };


    for (size_t i = 0; i < 4 * 2 * 3; i += 3) {
        // front-face + back-face triangles
        for (int side = 0; side !=2; side += 1) {
            Ogre::Vector3 v[3];
            v[0] = points[triangles[i]];
            v[1] = points[triangles[i + (side == 0 ? 1 : 2)]];
            v[2] = points[triangles[i + (side == 0 ? 2 : 1)]];
            Ogre::Vector3 normal = (v[1] - v[0]).crossProduct(v[2] - v[0]);
            normal.normalise();
            for (size_t c = 0; c != 3; c++) {
                border_object_->position(v[c]);
                border_object_->normal(normal);
            }
        }
    }

    border_object_->end();
}

void TexturedQuad::destroyBorder() {
    if (!border_mat_.isNull()) {
        border_mat_->unload();
        Ogre::MaterialManager::getSingleton().remove(quad_mat_->getName());
        border_mat_.setNull();
    }
    if (border_object_ != NULL) {
        context_->getSceneManager()->destroyManualObject(border_object_);
        border_object_ = NULL;
    }
}

// TexturedQuadDisplay

TexturedQuadDisplay::TexturedQuadDisplay() :
    Display(),
    tf_filter_(NULL) {
    topic_property_ = new rviz::RosTopicProperty(
        "Topic",
        "textured_quad",
        QString::fromStdString(
                ros::message_traits::datatype<rviz_textured_quad::TexturedQuad>()),
        "rviz_textured_quad/TexturedQuad topic to subscribe to. <topic>_array will "
        "also automatically be subscribed with type "
        "rviz_textured_quad/TexturedQuadArray.",
        this,
        SLOT(updateTopic()));

    queue_size_property_ = new rviz::IntProperty(
        "Queue Size",
        100,
        "Set the size of the incoming rviz_textured_quad/TexturedQuad message "
        "queue.",
        this, SLOT(updateQueueSize()));
    queue_size_property_->setMin(0);
}

TexturedQuadDisplay::~TexturedQuadDisplay() {
    if (!initialized()) {
        return;
    }
    unsubscribe();
    clearTexturedQuads();
    if (tf_filter_ != NULL) {
        delete tf_filter_;
    }
}

void TexturedQuadDisplay::onInitialize() {
    tf_filter_ = new tf::MessageFilter<rviz_textured_quad::TexturedQuad>(
        *context_->getTFClient(),
        fixed_frame_.toStdString(),
        queue_size_property_->getInt(),
        update_nh_);

    tf_filter_->connectInput(sub_);
    tf_filter_->registerCallback(
            boost::bind(
                    &TexturedQuadDisplay::incomingTexturedQuad, this, _1));
    tf_filter_->registerFailureCallback(
            boost::bind(
                    &TexturedQuadDisplay::failedTexturedQuad, this, _1, _2));
}

void TexturedQuadDisplay::update(float wall_dt, float ros_dt) {
    MessageQueue msg_q;
    {
        boost::mutex::scoped_lock lock(msg_q_mutex_);
        msg_q.swap(msg_q_);
    }

    if (!msg_q.empty()) {
        MessageQueue::iterator msg_it = msg_q.begin();
        MessageQueue::iterator msg_end = msg_q.end();
        for (; msg_it != msg_end; ++msg_it) {
            rviz_textured_quad::TexturedQuad::ConstPtr& texture_quad = *msg_it;
            processMessage(texture_quad);
      }
    }

    {
        TexturedQuadSet::iterator it = frame_locked_textured_quads_.begin();
        TexturedQuadSet::iterator end = frame_locked_textured_quads_.end();
        for (; it != end; ++it) {
          TexturedQuadPtr texture_quad = *it;
          texture_quad->updateFrameLocked();
        }
    }
}

void TexturedQuadDisplay::fixedFrameChanged() {
    tf_filter_->setTargetFrame(fixed_frame_.toStdString());
    clearTexturedQuads();
}

void TexturedQuadDisplay::reset() {
    Display::reset();
    clearTexturedQuads();
}

void TexturedQuadDisplay::setTopic(
        const QString &topic, const QString &datatype) {
    topic_property_->setString(topic);
}

void TexturedQuadDisplay::onEnable() {
    subscribe();
}

void TexturedQuadDisplay::onDisable() {
    unsubscribe();
}

void TexturedQuadDisplay::subscribe() {
    std::string topic = topic_property_->getTopicStd();
    if (!topic.empty()) {
        array_sub_.shutdown();
        sub_.unsubscribe();
        try {
          sub_.subscribe(update_nh_, topic, queue_size_property_->getInt());
          array_sub_ = update_nh_.subscribe(
              topic + "_array",
              queue_size_property_->getInt(),
              &TexturedQuadDisplay::incomingTexturedQuadArray,
              this);
          setStatus(rviz::StatusProperty::Ok, "Topic", "OK");
        }
        catch( ros::Exception& e ) {
            setStatus(
                rviz::StatusProperty::Error,
                "Topic",
                QString("Error subscribing: ") + e.what());
        }
    }
}

void TexturedQuadDisplay::unsubscribe() {
    sub_.unsubscribe();
    array_sub_.shutdown();
}

void TexturedQuadDisplay::incomingTexturedQuadArray(
        const rviz_textured_quad::TexturedQuadArray::ConstPtr& array) {
    std::vector<rviz_textured_quad::TexturedQuad>::const_iterator it = (
            array->textured_quads.begin());
    std::vector<rviz_textured_quad::TexturedQuad>::const_iterator end = (
            array->textured_quads.end());
    for (; it != end; ++it) {
        rviz_textured_quad::TexturedQuad::Ptr textured_quad(
            new rviz_textured_quad::TexturedQuad(*it));
        tf_filter_->add(textured_quad);
    }
}

void TexturedQuadDisplay::incomingTexturedQuad(
        const rviz_textured_quad::TexturedQuad::ConstPtr& one) {
    boost::mutex::scoped_lock lock(msg_q_mutex_);
    msg_q_.push_back(one);
}

void TexturedQuadDisplay::failedTexturedQuad(
        const ros::MessageEvent<rviz_textured_quad::TexturedQuad>& event,
        tf::FilterFailureReason reason) {
    rviz_textured_quad::TexturedQuad::ConstPtr texture_quad = (
            event.getConstMessage());
    std::string authority = event.getPublisherName();
    std::string error = context_->getFrameManager()->discoverFailureReason(
        texture_quad->header.frame_id,
        texture_quad->header.stamp,
        authority,
        reason);
    setTexturedQuadStatus(
            texture_quad->id, rviz::StatusProperty::Error, error);
}

void TexturedQuadDisplay::processMessage(
        const rviz_textured_quad::TexturedQuad::ConstPtr& message) {
    switch (message->action) {
        case rviz_textured_quad::TexturedQuad::ADD:
            addTexturedQuad(message);
            break;
        case rviz_textured_quad::TexturedQuad::DELETE:
            deleteTexturedQuad(message);
            break;
        case rviz_textured_quad::TexturedQuad::DELETEALL:
            deleteAllTexturedQuads(message);
            break;
        default: {
            ROS_WARN_STREAM("Invalid action=" << message->action << ".");
        }
    }
}

void TexturedQuadDisplay::addTexturedQuad(
    const rviz_textured_quad::TexturedQuad::ConstPtr& message) {
    TexturedQuadPtr texture_quad;
    bool create = true;
    TexturedQuadId id(message->ns, message->id);
    TexturedQuadMap::iterator it = textured_quads_.find(id);
    if (it != textured_quads_.end()) {
        texture_quad = it->second;
        create = false;
    }
    if (create) {
        texture_quad.reset(new TexturedQuad(this, context_, scene_node_));
    }
    textured_quads_.insert(std::make_pair(id, texture_quad));
    texture_quad->setMessage(message);
    if (message->frame_locked) {
        frame_locked_textured_quads_.insert(texture_quad);
    }
    context_->queueRender();
}

void TexturedQuadDisplay::deleteTexturedQuad(
    const rviz_textured_quad::TexturedQuad::ConstPtr& message) {
    TexturedQuadId id(message->ns, message->id);
    TexturedQuadMap::iterator it = textured_quads_.find(id);
    if (it == textured_quads_.end()) {
        return;
    }
    TexturedQuadSet::iterator it_fl = frame_locked_textured_quads_.find(
            it->second);
    if (it_fl != frame_locked_textured_quads_.end()) {
        frame_locked_textured_quads_.erase(it_fl);
    }
    textured_quads_.erase(it);
}

void TexturedQuadDisplay::deleteAllTexturedQuads(
    const rviz_textured_quad::TexturedQuad::ConstPtr& message) {
    for (TexturedQuadMap::iterator it = textured_quads_.begin();
         it != textured_quads_.end();) {
        if (it->second->ns() != message->ns) {
            it++;
            continue;
        }
        TexturedQuadSet::iterator it_fl = frame_locked_textured_quads_.find(
                it->second);
        if (it_fl != frame_locked_textured_quads_.end()) {
            frame_locked_textured_quads_.erase(it_fl);
        }
        textured_quads_.erase(it++);
    }
}

void TexturedQuadDisplay::updateQueueSize() {
    tf_filter_->setQueueSize((uint32_t) queue_size_property_->getInt());
}

void TexturedQuadDisplay::updateTopic() {
    unsubscribe();
    subscribe();
}

void TexturedQuadDisplay::setTexturedQuadStatus(
        int id, rviz::StatusLevel level, const std::string& text) {
    std::stringstream ss;
    ss << id;
    std::string textured_quad_name = ss.str();
    setStatusStd(level, textured_quad_name, text);
}

void TexturedQuadDisplay::deleteTexturedQuadStatus(int id) {
    std::stringstream ss;
    ss << id;
    std::string textured_quad_name = ss.str();
    deleteStatusStd(textured_quad_name);
}

void TexturedQuadDisplay::clearTexturedQuads() {
    textured_quads_.clear();
    frame_locked_textured_quads_.clear();
    tf_filter_->clear();
}

#include <pluginlib/class_list_macros.h> // NOLINT

PLUGINLIB_EXPORT_CLASS(TexturedQuadDisplay, rviz::Display)
