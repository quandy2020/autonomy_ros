# PersonMaskWithHeader.msg
# 人体mask数据消息，包含语义分割数据

std_msgs/Header header

# 图像尺寸
int32 width
int32 height

# 语义分割mask - 将所有人体实例合并后的语义分割图
# 每个像素值0表示人体区域，1表示背景
uint8[] semantic_mask

# 被语义分割mask过滤后的非人体区域depth图像
# 已经resize到与semantic_mask相同尺寸(width x height)
# 人体区域的depth值被设置为0，保留背景区域的depth信息
# 深度值以毫米为单位存储，使用uint16格式
uint16[] filtered_depth_image