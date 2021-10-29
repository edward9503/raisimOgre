/*-------------------------------------------------------------------------
This source file is a part of OGRE
(Object-oriented Graphics Rendering Engine)
For the latest info, see http://www.ogre3d.org/


Copyright (c) 2000-2013 Torus Knot Software Ltd
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:


The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.


THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE
-------------------------------------------------------------------------*/

#include <raisim/OgreVis.hpp>
#include "raisimBasicImguiPanel.hpp"
#include "helper.hpp"

void setupCallback() {
  auto vis = raisim::OgreVis::get();

  /// light
  vis->getLight()->setDiffuseColour(1, 1, 1);
  vis->getLight()->setCastShadows(true);
  Ogre::Vector3 lightdir(1., 1, -1);
  lightdir.normalise();
  vis->getLight()->setDirection(lightdir);
  vis->setContactVisObjectSize(0.04, 0.3);

  /// load  textures
  vis->addResourceDirectory(vis->getResourceDir() + "/material/checkerboard");
  vis->loadMaterialFile("checkerboard.material");

  /// shadow setting
  vis->getSceneManager()->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE);
  vis->getSceneManager()->setShadowTextureSettings(2048, 3);

  // beyond this distance, shadow disappears
  vis->getSceneManager()->setShadowFarDistance(10);
  // size of contact points and contact forces
  vis->setContactVisObjectSize(0.01, 0.4);
  // speed of camera motion in freelook mode
  vis->getCameraMan()->setTopSpeed(5);
}

int main(int argc, char **argv) {
  raisim::World::setActivationKey(raisim::loadResource("activation.raisim"));

  /// create raisim world
  raisim::World world;
  world.setTimeStep(0.0001);
  auto vis = raisim::OgreVis::get();
  world.setERP(0,0);

  /// these method must be called before initApp
  vis->setWorld(&world);
  vis->setWindowSize(1800, 1200);
  vis->setImguiSetupCallback(imguiSetupCallback);
  vis->setImguiRenderCallback(imguiRenderCallBack);
  vis->setSetUpCallback(setupCallback);
  vis->setAntiAliasing(8);
  raisim::gui::manualStepping = true;

  /// starts visualizer thread
  vis->initApp();

  /// create raisim objects
  auto ground = world.addGround();

  std::vector<raisim::ArticulatedSystem*> anymal;
  std::vector<std::vector<raisim::GraphicObject>*> anymal_visual;

  /// create visualizer objects
  vis->createGraphicalObject(ground, 20, "floor", "checkerboard_green");  

  /// create raisim objects
  auto sphere1 = world.addSphere(0.1, 1);
  auto sphere2 = world.addSphere(0.1, 1);
  anymal.push_back(world.addArticulatedSystem(raisim::loadResource("anymalC_kinova/urdf/anymal_kinova.urdf")));
  sphere1->setPosition(0, 0, 5);
  sphere2->setPosition(0.5, 0, 3);

  /// create heightmap
  // raisim::TerrainProperties terrainProperties;
  // terrainProperties.frequency = 0.2;
  // terrainProperties.zScale = 3.0;
  // terrainProperties.xSize = 20.0;
  // terrainProperties.ySize = 20.0;
  // terrainProperties.xSamples = 50;
  // terrainProperties.ySamples = 50;
  // terrainProperties.fractalOctaves = 3;
  // terrainProperties.fractalLacunarity = 2.0;
  // terrainProperties.fractalGain = 0.25;

  /// comment one of the following two heightmap methods

  /// using terrain properties
  // auto hm = world.addHeightMap(0.0, 0.0, terrainProperties);

  /// using raisim text file
//  auto hm = world.addHeightMap(raisim::loadResource("heightMap/heightMapExample.txt"), 0, 0);

  /// create visualizer objects
  // vis->createGraphicalObject(hm, "terrain", "default");
  vis->createGraphicalObject(sphere1, "sphere1", "gravel");
  vis->createGraphicalObject(sphere2, "sphere2", "default");

  /// ANYmal joint PD controller
  Eigen::VectorXd jointNominalConfig(31), jointVelocityTarget(30);
  Eigen::VectorXd jointState(30), jointVel(30), jointForce(30), jointPgain(30), jointDgain(30);
  jointPgain.setZero();
  jointDgain.setZero();
  jointVelocityTarget.setZero();
  jointPgain.tail(24).setConstant(200.0);
  jointDgain.tail(24).setConstant(10.0);

  jointNominalConfig << 0, 0, 0, 0, 0, 0, 0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8, 
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  jointVel.setZero();

  anymal_visual.push_back(vis->createGraphicalObject(anymal.back(), "ANYmal_kinova"));  

  anymal.back()->setGeneralizedCoordinate({0, 0, 3.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4,
                                    -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8,
                                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  Eigen::Vector4d quat; quat <<1., 0., 0., 0.;
  // hm->setPosition(0.1, 0.2, -0.3);

  /// lambda function for the controller
  double time=0.;
  auto controller = [vis,
                     &time,
                     &world,
                     anymal]() {
    time += world.getTimeStep();
    raisim::Vec<3> camera_position;
    raisim::Mat<3, 3> camera_orientation;
    Eigen::Matrix3f camera_oriEigen;

    anymal.back()->getFramePosition("face_front_to_wide_angle_camera_front_camera", camera_position);
    anymal.back()->getFrameOrientation("face_front_to_wide_angle_camera_front_camera", camera_orientation);
    for (int i=0; i<3; i++) {
      for (int j=0; j<3; j++) {
        camera_oriEigen(i,j) = camera_orientation(i,j);
      }
    }
    Eigen::Matrix3f preR_camera;
    preR_camera << 0, 0, -1,
                   -1, 0, 0,
                   0, 1, 0;
    Eigen::Quaternionf camera_quatEigen(camera_oriEigen*preR_camera);
    camera_quatEigen.normalize();
    // vis->getCameraMan()->setQuatPos(Ogre::Real(camera_quatEigen.w()), Ogre::Real(camera_quatEigen.x()), 
    //                                         Ogre::Real(camera_quatEigen.y()), Ogre::Real(camera_quatEigen.z()), 
    //                                         Ogre::Vector3(camera_position(0), camera_position(1), camera_position(2)));
    std::cout<<"w: "<<camera_quatEigen.w()<<"x: "<<camera_quatEigen.x()<<"y: "<<camera_quatEigen.y()<<"z: "<<camera_quatEigen.z() << "\n";
    // Camera->setPosition(1,1,1);
    // mCamera->setOrientation(Ogre::Quaternion{1.,0.,0.,0.});
    // vis->getCameraMan()->setYawPitchDist(Ogre::Radian(0), -Ogre::Radian(M_PI_4), 2);
    // vis->getCameraMan()->setCamera(mCamera);
    // RSINFO(Camera->getPosition);
    // vis->getCameraMan()->setCamera(Camera);
    RSINFO(camera_position);
    RSINFO(camera_oriEigen);
    RSINFO(camera_orientation);
    // RSINFO(anymal.back()-> getFrameIdxByName("face_front_to_depth_camera_front_camera"));

    Eigen::VectorXd jointNominalConfig(31), jointVelocityTarget(30);
    jointVelocityTarget.setZero();
    jointNominalConfig << 0, 0, 0, 0, 0, 0, 0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8, 
                          1.57 * sin(1.8*M_PI*time/10), 0, 1.57 * sin(1.8*M_PI*time/10), 0, 1.57 * sin(1.8*M_PI*time/10), 0, 3 * sin(1.8*M_PI*time), 0, 0, 3 * sin(1.8*M_PI*time), 0, 0;  
                          // 3 * sin(1.8*M_PI*time)  
    anymal.back()->setPdTarget(jointNominalConfig, jointVelocityTarget);
  };

  vis->setControlCallback(controller); 
  raisim::Vec<3> camera_position;
  anymal.back()->getFramePosition("face_front_to_wide_angle_camera_front_camera", camera_position); 

  vis->select(anymal_visual.back()->at(0));
  // vis->getCameraMan()->setYawPitchDist(Ogre::Radian(M_PI_2), -Ogre::Radian(0), 10);  
  vis->getCameraMan()->setQuatPos(Ogre::Real(0.5), Ogre::Real(0.5), Ogre::Real(-0.5), Ogre::Real(-0.5), 
                                          Ogre::Vector3(camera_position(0), camera_position(1), camera_position(2)));
  anymal.back()->setGeneralizedForce(Eigen::VectorXd::Zero(anymal.back()->getDOF()));
  anymal.back()->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  anymal.back()->setPdGains(jointPgain, jointDgain);
  anymal.back()->setPdTarget(jointNominalConfig, jointVelocityTarget);
  
  RSINFO(camera_position);
  RSINFO(anymal.back()->getDOF());
  anymal.back()->printOutMovableJointNamesInOrder();
  

  /// run the app
  vis->run();

  // for (int i=0; i< 1000; i++) {
  //   camera_vis->getCameraMan()->setYawPitchDist(Ogre::Radian(i*0.01), Ogre::Radian(-1.), 12);
  //   // vis->renderOneFrame();
  //   camera_vis->renderOneFrame();
  // }  

  /// terminate
  vis->closeApp();

  return 0;
}
