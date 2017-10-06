uint8 ADD=0
uint8 DELETE=2
uint8 DELETEALL=3

std_msgs/Header header          # header for time/frame information
string ns                       # namespace to place this object in, used in conjunction with id to create a unique name for the object
int32 id                        # object ID useful in conjunction with the namespace for manipulating and deleting the object later
int32 action                    # 0 add/modify an object, 2 deletes an object, 3 deletes all objects
bool frame_locked               # whether quad should be frame-locked, i.e. re-transformed into its frame every timestep
sensor_msgs/Image image         # texture
geometry_msgs/Pose pose         # pose of the center of the quad to be displayed
geometry_msgs/Vector3 scale     # scale of the object 1,1,1 means default (usually 1 meter square)
float32 width                   # width of the quad (usually meters) 
float32 height                  # height of the quad (usually meters)
uint8 alpha                     # discard pixels w/ alpha > than this value or 0 for translucent "overlay"
float32 border_size             # border size (usually meters) or 0 for no border
std_msgs/ColorRGBA border_color # border color (alpha ignored)
