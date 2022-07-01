import rospy
import rospkg
import numpy as np
import random as rnd
import math
import matplotlib.pyplot as plt
import matplotlib.patches as mpatche
# generate random integer values
from random import randint
from random import random
# seed random number generator

from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle

rospack = rospkg.RosPack()
rospack.list()
rospy.init_node('sarc_map', anonymous=True)
rate = rospy.Rate(10) # 10hz
quant = 60 ## this quantity is this number times 3, since each tree type will have this many replicas
dronesquant = 5 ## number of drones to spawn, above 5 pay atention in the spawn circle radius
spawnCircleRadius = 0.75


def sarc_map():
  startText = "<?xml version='1.0' ?>\n\
    <?xml-model href='http://sdformat.org/schemas/root.xsd' schematypens='http://www.w3.org/2001/XMLSchema'?>\n\
    <sdf version='1.5'>\n\
      <world name='default'>\n\
        <plugin name='mrs_gazebo_static_transform_republisher_plugin' filename='libMRSGazeboStaticTransformRepublisher.so'/>\n\
        <spherical_coordinates>\n\
          <surface_model>EARTH_WGS84</surface_model>\n\
          <latitude_deg>47.397743</latitude_deg>\n\
          <longitude_deg>8.545594</longitude_deg>\n\
          <elevation>0.0</elevation>\n\
          <heading_deg>0</heading_deg>\n\
        </spherical_coordinates>\n\
                                \n\
        <physics name='default_physics' default='0' type='ode'>\n\
          <gravity>0 0 -9.8066</gravity>\n\
          <ode>\n\
            <solver>\n\
              <type>quick</type>\n\
              <iters>10</iters>\n\
              <sor>1.3</sor>\n\
              <use_dynamic_moi_rescaling>0</use_dynamic_moi_rescaling>\n\
            </solver>\n\
            <constraints>\n\
              <cfm>0</cfm>\n\
              <erp>0.2</erp>\n\
              <contact_max_correcting_vel>1000</contact_max_correcting_vel>\n\
              <contact_surface_layer>0.001</contact_surface_layer>\n\
            </constraints>\n\
          </ode>\n\
          <max_step_size>0.004</max_step_size>\n\
          <real_time_factor>1</real_time_factor>\n\
          <real_time_update_rate>250</real_time_update_rate>\n\
          <magnetic_field>6.0e-06 2.3e-05 -4.2e-05</magnetic_field>\n\
        </physics>\n\
                  \n\
        <scene>\n\
          <shadows>false</shadows>\n\
          <sky>\n\
            <clouds/>\n\
          </sky>\n\
        </scene>\n\
                \n\
        <light name='sun' type='directional'>\n\
          <pose frame=''>0 0 1000 0.4 0.2 0</pose>\n\
          <diffuse>1 1 1 1</diffuse>\n\
          <specular>0.6 0.6 0.6 1</specular>\n\
          <direction>0.1 0.1 -0.9</direction>\n\
          <attenuation>\n\
            <range>20</range>\n\
            <constant>0.5</constant>\n\
            <linear>0.01</linear>\n\
            <quadratic>0.001</quadratic>\n\
          </attenuation>\n\
          <cast_shadows>1</cast_shadows>\n\
        </light>\n\
                \n\
        <model name='ground_plane'>\n\
          <static>true</static>\n\
          <link name='link'>\n\
            <collision name='collision'>\n\
              <pose>0 0 0 0 0 0</pose>\n\
              <geometry>\n\
                <plane>\n\
                  <normal>0 0 1</normal>\n\
                  <size>250 250</size>\n\
                </plane>\n\
              </geometry>\n\
              <surface>\n\
                <friction>\n\
                  <ode>\n\
                    <mu>1</mu>\n\
                    <mu2>1</mu2>\n\
                  </ode>\n\
                </friction>\n\
              </surface>\n\
            </collision>\n\
            <visual name='grass'>\n\
              <pose>0 0 0 0 0 0</pose>\n\
              <cast_shadows>false</cast_shadows>\n\
              <geometry>\n\
                <mesh>\n\
                  <uri>model://SARcgrass/grass_plane.dae</uri>\n\
                  <scale>1.0 1.0 1.0</scale>\n\
                </mesh>\n\
              </geometry>\n\
            </visual>\n\
          </link>\n\
        </model>\n\
                \n\
        <model name='the_void'>\n\
          <static>1</static>\n\
          <link name='link'>\n\
            <pose frame=''>0 0 0.1 0 -0 0</pose>\n\
            <visual name='the_void'>\n\
              <pose frame=''>0 0 2 0 -0 0</pose>\n\
              <geometry>\n\
                <sphere>\n\
                  <radius>0.25</radius>\n\
                </sphere>\n\
              </geometry>\n\
              <material>\n\
                <script>\n\
                  <uri>file://media/materials/scripts/Gazebo.material</uri>\n\
                  <name>Gazebo/Black</name>\n\
                </script>\n\
              </material>\n\
            </visual>\n\
            <self_collide>0</self_collide>\n\
            <enable_wind>0</enable_wind>\n\
            <kinematic>0</kinematic>\n\
          </link>\n\
          <pose frame=''>-1000 -1000 0 0 0 0</pose>\n\
        </model>\n"

  endText = "\t\t<gui>\n\
        <camera name='camera'>\n\
          <pose>-60 -100 30 0 0.4 0.89</pose>\n\
        </camera>\n\
      </gui>\n\
    \n\
        <plugin name='mrs_gazebo_rviz_cam_synchronizer' filename='libMRSGazeboRvizCameraSynchronizer.so' >\n\
          <target_frame_id>gazebo_user_camera</target_frame_id>\n\
          <world_origin_frame_id>uav1/gps_origin</world_origin_frame_id>\n\
          <frame_to_follow>uav1</frame_to_follow>\n\
        </plugin>\n\
    \n\
    </world>\n\
  </sdf>\n"

  someX, someY = 125, 125
  #someX, someY = 50, 50
  fig,ax = plt.subplots()
  currentAxis = plt.gca()
  currentAxis.add_patch(Rectangle((someX - 0.1, someY - 0.1), 0.2, 0.2,
                        alpha=1, facecolor='none'))

  # x-axis values
  x1 = [0] * quant
  # y-axis values
  y1 = [0] * quant

  # x-axis values
  x2 = [0] * quant
  # y-axis values
  y2 = [0] * quant

  # x-axis values
  x3 = [0] * quant
  # y-axis values
  y3 = [0] * quant

  xorigin = randint(-someX, someX)
  while xorigin >  someX - someX/5 or xorigin < -someX + someX/5:
    xorigin = randint(-someX, someX)
    # print('origin in x recalculated: %d' % xorigin)
  yorigin = randint(-someY, someY)
  while yorigin > someY - someY/5 or yorigin < -someY + someY/5:
    yorigin = randint(-someY, someY)
    # print('origin in y recalculated: %d' % yorigin)

  #xorigin = yorigin = 0
  #xorigin = yorigin = 25

  i = 0

  f = open(rospack.get_path('sarc_environment') + "/worlds/SARC.world", "w")
  f.write(startText)

  count = 0

  def insertFireTree(x, y, count):
      model = "\t\t\t<model name='SARcfireTree" + str(count) + "'>\n\
          <include>\n\
            <pose>" + str(x) + " " + str(y) + " 0 0 0 0" + "</pose>\n\
            <uri>model://SARcfireTree</uri>\n\
          </include>\n\
        </model>\n"

      count += 1
      return model

  def insertCloseTree(x, y, count):
      model = "\t\t\t<model name='SARccloseTree" + str(count) + "'>\n\
          <include>\n\
            <pose>" + str(x) + " " + str(y) + " 0 0 0 0" + "</pose>\n\
            <uri>model://SARccloseTree</uri>\n\
          </include>\n\
        </model>\n"

      count += 1
      return model

  def insertTree(x, y, count):
      model = "\t\t\t<model name='SARctree" + str(count) + "'>\n\
          <include>\n\
            <pose>" + str(x) + " " + str(y) + " 0 0 0 0" + "</pose>\n\
            <uri>model://SARctree</uri>\n\
          </include>\n\
        </model>\n"
      
      count += 1
      return model

  def insertLandArea(x, y):
      model = "\t\t\t<model name='SARclandArea'>\n\
          <include>\n\
            <pose>" + str(x) + " " + str(y) + " 0.02 0 0 0" + "</pose>\n\
            <uri>model://SARclandArea</uri>\n\
          </include>\n\
        </model>\n"

      return model

  def insertRedArea(x, y, count):
      model = "\t\t\t<model name='SARC_Red" + str(count) + "'>\n\
          <include>\n\
            <pose>" + str(x) + " " + str(y) + " 0.08 0 0 0" + "</pose>\n\
            <uri>model://SARC_Red</uri>\n\
          </include>\n\
        </model>\n"

      return model

  def insertYellowArea(x, y, count):
      model = "\t\t\t<model name='SARC_Yellow" + str(count) + "'>\n\
          <include>\n\
            <pose>" + str(x) + " " + str(y) + " 0.05 0 0 0" + "</pose>\n\
            <uri>model://SARC_Yellow</uri>\n\
          </include>\n\
        </model>\n"

      return model

  def insertGreenArea(x, y, count):
      model = "\t\t\t<model name='SARC_Green" + str(count) + "'>\n\
          <include>\n\
            <pose>" + str(x) + " " + str(y) + " 0.02 0 0 0" + "</pose>\n\
            <uri>model://SARC_Green</uri>\n\
          </include>\n\
        </model>\n"

      return model

  def insertKc(x, y, yaw):
      model = "\t\t\t<model name='SARckc'>\n\
          <include>\n\
            <pose>" + str(x) + " " + str(y) + " 100 0 0 " + str(yaw) + "</pose>\n\
            <uri>model://SARckc</uri>\n\
          </include>\n\
        </model>\n"

      return model

  # generate fire area
  for i in range(quant):
      radius1 = random()
      radius1 = 0 + (radius1 * (someX/5 - 0))
      angle = math.radians(randint(0, 360))

      xfire = (math.cos(angle) * radius1) + xorigin
      while xfire < -someX or xfire > someX:
          radius1 = random()
          radius1 = 0 + (radius1 * (someX/5 - 0))
          angle = math.radians(randint(0, 360))
          xfire = (math.cos(angle) * radius1) + xorigin
          # print('x recalculated: %d' % xfire)
      else:
          x1[i] = xfire
      # print(x1[i])

      yfire = (math.sin(angle) * radius1) + yorigin
      while yfire < -someY or yfire > someY:
          radius1 = random()
          radius1 = 0 + (radius1 * (someY/5 - 0))
          angle = math.radians(randint(0, 360))
          yfire = (math.sin(angle) * radius1) + yorigin
          # print('y recalculated: %d' % yfire)
      else:
          y1[i] = yfire
      f.write(insertFireTree(xfire, yfire, i))
      #f.write(insertRedArea(xfire, yfire, i))
      # print(y1[i])

  # generate close area
  for i in range(quant):
      radius2 = random()
      radius2 = someX/5  + (radius2 * (someX/3  - someX/5))
      angle = math.radians(randint(0, 360))

      xclose = (math.cos(angle) * radius2) + xorigin
      while xclose < -someX or xclose > someX:
          radius2 = random()
          radius2 = someX/5  + (radius2 * (someX/3  - someX/5))
          angle = math.radians(randint(0, 360))
          xclose = (math.cos(angle) * radius2) + xorigin
          # print('x recalculated: %d' % xclose)
      else:
          x2[i] = xclose
      # print(x2[i])

      yclose = (math.sin(angle) * radius2) + yorigin
      while yclose < -someY or yclose > someY:
          radius2 = random()
          radius2 = someY/5  + (radius2 * (someY/3  - someY/5))
          angle = math.radians(randint(0, 360))
          yclose = (math.sin(angle) * radius2) + yorigin
          # print('y recalculated: %d' % yclose)
      else:
          y2[i] = yclose 
      f.write(insertCloseTree(xclose, yclose, i))
      #f.write(insertYellowArea(xclose, yclose, i))
      # print(y2[i])

  # generate not affected area
  for i in range(quant):
      radius3 = random()
      radius3 = someY/3  + (radius3 * (someY/2  - someY/3))
      angle = math.radians(randint(0, 360))

      xtree = (math.cos(angle) * radius3) + xorigin
      while xtree < -someX or xtree > someX:
          radius3 = random()
          radius3 = someY/3  + (radius3 * (someY/2  - someY/3))
          angle = math.radians(randint(0, 360))
          xtree = (math.cos(angle) * radius3) + xorigin
          # print('x recalculated: %d' % xtree)
      else:
          x3[i] = xtree
      # print(x3[i])

      ytree = (math.sin(angle) * radius3) + yorigin
      while ytree < -someY or ytree > someY:
          radius3 = random()
          radius3 = someY/3  + (radius3 * (someY/2  - someY/3))
          angle = math.radians(randint(0, 360))
          ytree = (math.sin(angle) * radius3) + yorigin
          # print('y recalculated: %d' % ytree)
      else:
          y3[i] = ytree
      f.write(insertTree(xtree, ytree, i))
      #f.write(insertGreenArea(xtree, ytree, i))
      # print(y3[i])
      
  def checkSafeX(i):
    if i < someX - someX/5 and i > -someX + someX/5:
      return False
    if i > someX - 10 and i < -someX + 10:
      return False
    if i < 0 and xorigin < 0:
      return False
    if i > 0 and xorigin > 0:
      return False
    else:
      return True

  def checkSafeY(i):
    if i < someY - someY/5 and i > -someY + someY/5:
      return False
    if i > someY - 10 and i < -someY + 10:
      return False
    if i < 0 and yorigin < 0:
      return False
    if i > 0 and yorigin > 0:
      return False
    else:
      return True

  # generate land area
  x4 = randint(-someX, someX)
  while checkSafeX(x4) == False:
    x4 = randint(-someX, someX)
    # print('land area in x recalculated: %d' % x4)
  y4 = randint(-someY, someY)
  while checkSafeY(y4) == False:
    y4 = randint(-someY, someY)
    # print('land area in y recalculated: %d' % y4)

  #x4 = 15
  #y4 = 25

  f.write(insertLandArea(x4, y4))

  # generate kc path
  x5 = [0] * 2
  y5 = [0] * 2

  if xorigin > 0:
    x5[0] = someX
  else:
    x5[0] = -someX

  if yorigin > 0:
    y5[0] = -someY
  else:
    y5[0] = someY
  
  if x5[0] > 0 and 0 > y5[0]:
    yaw = 0.785398
  elif x5[0] > 0 and 0 < y5[0]:
    yaw = 2.35619
  elif x5[0] < 0 and 0 < y5[0]:
    yaw = -2.35619
  else:
    yaw = -0.785398

  if x4 > 0:
    x5[1] = someX
  else:
    x5[1] = -someX

  if y4 > 0:
    y5[1] = -someY
  else:
    y5[1] = someY
  f.write(insertKc(x5[0], y5[0], yaw))

  def insertUAV(x, xkc, y, ykc, count):
    model = "uav_name:\n\
    id: " + str(count + 1) + "\n\
    x: "+ str(xkc + x) + "\n\
    y: "+ str(ykc + y) +"\n\
    z: 98.6\n\
    heading: 0"
    return model
  
  xdrone = [0] * dronesquant
  ydrone = [0] * dronesquant
  angleradiusdrones = [0] * dronesquant
  eachangle = 360/dronesquant

  for i in range(dronesquant):
      #print('Drone {}'.format(i))
      angleradiusdrones[i] = math.radians(i * eachangle)
      #print(angleradiusdrones[i])
      xdrone[i] = (math.cos(angleradiusdrones[i]) * spawnCircleRadius)
      #print(xdrone[i])
      ydrone[i] = (math.sin(angleradiusdrones[i]) * spawnCircleRadius)
      #print(ydrone[i])
      d = open(rospack.get_path('sarc_environment') + "/start/pos/pos" + str(i + 1) + ".yaml", "w")
      d.write(insertUAV(xdrone[i], x5[0], ydrone[i], y5[0],  i))


  f.write(endText)

  '''print('Origin: %d, %d' % (xorigin, yorigin))
  print('Safe: %d, %d' % (x4, y4))

  # plotting points as a scatter plot
  plt.scatter(x1, y1, label= "dot", color= "red",
              marker= "o", s=30)
  plt.scatter(x2, y2, label= "dot", color= "yellow",
              marker= "o", s=30)
  plt.scatter(x3, y3, label= "dot", color= "green",
              marker= "o", s=30)
  plt.scatter(x4, y4, label= "dot", color= "black",
              marker= "o", s=30)
  plt.plot(x5, y5, color="blue")

  # x-axis label
  plt.xlabel('x - axis')
  # frequency label
  plt.ylabel('y - axis')
  # plot title
  plt.title('My scatter plot!')
  # showing legend
  plt.legend()

  # function to show the plot
  plt.show()'''

  pass

if __name__ == '__main__':
    try:
        rnd.seed(1)

        sarc_map()
    except rospy.ROSInterruptException:
        pass