# this message contains a UBX-NAV-DOP (0x01 0x04) record dilution of precision
std_msgs/Header header # Header timestamp should be acquisition time

uint32 itow           # ms - GPS Time of week of the navigation epoch
uint32 g_dop          # 0.01 - Geometric DOP 
uint32 p_dop          # 0.01 - Position DOP 
uint32 t_dop          # 0.01 - Time DOP 
uint32 v_dop          # 0.01 - Vertical DOP 
uint32 h_dop          # 0.01 - Horizontal DOP 
uint32 n_dop          # 0.01 - Northing DOP 
uint32 e_dop          # 0.01 - Easting DOP 