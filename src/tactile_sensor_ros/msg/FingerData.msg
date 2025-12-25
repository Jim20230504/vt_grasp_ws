# 对应 fingerDataPack 中的单个传感器数据
int32 sensor_index
int32[] channel_cap_data      # 原始电容通道数值
float32[] nf                  # 法向力 (Normal Force)
float32[] tf                  # 切向力 (Tangential Force)
int32[] tf_dir                # 切向力方向
int32[] s_prox_cap_data       # 接近(自电容)
int32[] m_prox_cap_data       # 接近(互电容)