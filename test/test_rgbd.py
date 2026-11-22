import numpy as np
from sensor_msgs.msg import Image
from runa_sim.rgbd_bridge import depth_mm


def test_depth_units_invalid_and_big_endian_padding():
    msg=Image()
    msg.height,msg.width,msg.step=1,4,20
    msg.encoding='32FC1'
    msg.is_bigendian=1
    msg.data=np.array([1.25,np.nan,np.inf,-1,999],dtype='>f4').tobytes()
    out=depth_mm(msg)
    assert out.encoding=='16UC1' and out.step==8 and out.is_bigendian==0
    assert np.frombuffer(out.data,dtype='<u2').tolist()==[1250,0,0,0]
