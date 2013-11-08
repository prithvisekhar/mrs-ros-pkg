Commands run 
xacro to urdf :~/gcatkin_ws/src/telerobcar-ros-pkg/mrs_description/robots$ rosrun xacro xacro.py A2M.xacro > a2m.urdf
urdf to collada:~/gcatkin_ws/src/telerobcar-ros-pkg/mrs_description/robots$ rosrun collada_urdf urdf_to_collada a2m.urdf a2m.dae


--- From History---- 
//Install HDF viewer and mayavi to view the data and visualise it 
 1717  rosrun xacro xacro.py A2M_kinematic.xacro > arm_kinematic1.urdf
 1718  rosrun collada_urdf urdf_to_collada arm_kinematic1.urdf arm_kinematic1.dae
 1719  rosrun moveit_ikfast round_collada_numbers.py arm_kinematic1.dae arm_k1.dae 5 //Install the movit_ikfast package from source 
 
 
 
 //using OpenRave
 1720  openrave-robot.py arm_k1.dae --info links
 1725  openrave.py --database linkstatistics --robot=a2m_k1.robot.xml --show
 1726  openrave.py --database linkstatistics --robot=a2m_k1.robot.xml
 1728  openrave.py --database kinematicreachability --robot=a2m_k1.robot.xml 
 1729  openrave.py --database kinematicreachability --robot=a2m_k1.robot.xml --show
 1730  openrave.py --database inversekinematics --robot=a2m_k1.robot.xml 
 1731  openrave.py --database kinematicreachability --robot=a2m_k1.robot.xml --xyzdelta=0.01 --showscale=10
 1732  openrave.py --database kinematicreachability --robot=a2m_k1.robot.xml --xyzdelta=0.01 --showscale=10 --show
 1733  openrave.py --database kinematicreachability --robot=a2m_k1.robot.xml --show
 1734  openrave.py --database kinematicreachability --robot=a2m_k1.robot.xml --xyzdelta=0.01 --showscale=2
 1735  openrave.py --database kinematicreachability --robot=a2m_k1.robot.xml --xyzdelta=0.01 --showscale=2 --show
 1736  openrave.py --database kinematicreachability --robot=a2m_k1.robot.xml --xyzdelta=0.01 --showscale=1.5 --show
 1737  openrave.py --database kinematicreachability --robot=a2m_k1.robot.xml --xyzdelta=0.01 --showscale=5 --show
 1738  openrave.py --database kinematicreachability --robot=a2m_k1.robot.xml --xyzdelta=0.01 --showscale=10 --show

 
 

