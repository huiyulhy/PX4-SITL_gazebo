/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @Author: Karthik Srivatsan
 * @Version: 1.0
 * @Date: June 2022
 *
 * @Brief: this plugin models the lift and drag of an aircraft from
 * as a single body, using static coefficients, aerodynamic damping 
 * coefficients, aerodynamic derivatives and control derivatives. 
 * It takes in a specified number of control surfaces and control
 * derivatives are defined/specified with respect to the deflection
 * of individual control surfaces. Coefficients for this plugin can be 
 * obtained using Athena Vortex Lattice (AVL) by Mark Drela 
 * https://web.mit.edu/drela/Public/web/avl/
 * The sign conventions used in this plugin are therefore written
 * in a way to be compatible with AVL. 
 * Force equations are computed in the body, while
 * moment equations are computed in the stability frame.
 *
 */

#include <algorithm>
#include <string>

#include "common.h"
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "liftdrag_plugin/advanced_liftdrag_plugin.h"

#include "Force.pb.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(AdvancedAdvancedLiftDragPlugin)

/////////////////////////////////////////////////
AdvancedLiftDragPlugin::AdvancedLiftDragPlugin() : CD0(0.01), Cem0(0.0), rho(1.2041), M(15),
CLa(1.0), CYa(0.0), Cella(0.0), Cema(0.0), Cena(0.0),
CLb(0.0), CYb(0.0), Cellb(0.0), Cemb(0.0), Cenb(0.0),
num_ctrl_surfaces(0), ctrl_surface_direction(0),
CD_ctrl(0), CY_ctrl(0), CL_ctrl(0), Cell_ctrl(0), Cem_ctrl(0), Cen_ctrl(0),
CDp(0), CYp(0), CLp(0), Cellp(0), Cemp(0), Cenp(0),
CDq(0), CYq(0), CLq(0), Cellq(0), Cemq(0), Cenq(0),
CDr(0), CYr(0), CLr(0), Cellr(0), Cemr(0), Cenr(0)
{
  this->ref_pt = ignition::math::Vector3d(0, 0, 0);
  this->forward = ignition::math::Vector3d(1, 0, 0);
  this->upward = ignition::math::Vector3d(0, 0, 1);
  this->wind_vel_ = ignition::math::Vector3d(0.0, 0.0, 0.0);
  this->area = 1.0;
  this->alpha = 0.0;
  this->beta = 0.0;
  this->velocityStall = 0.0;
  this->CL0 = 0.0;

  // 90 deg stall
  this->alphaStall = 0.5*M_PI;

  this->radialSymmetry = false;
  this->CemaStall = 0.0;

}

/////////////////////////////////////////////////
AdvancedLiftDragPlugin::~AdvancedLiftDragPlugin()
{
}

/////////////////////////////////////////////////
void AdvancedLiftDragPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "AdvancedLiftDragPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "AdvancedLiftDragPlugin _sdf pointer is NULL");
  this->model = _model;
  this->sdf = _sdf;

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "AdvancedLiftDragPlugin world pointer is NULL");

#if GAZEBO_MAJOR_VERSION >= 9
  this->physics = this->world->Physics();
  this->last_pub_time = this->world->SimTime();
#else
  this->physics = this->world->GetPhysicsEngine();
  this->last_pub_time = this->world->GetSimTime();
#endif
  GZ_ASSERT(this->physics, "AdvancedLiftDragPlugin physics pointer is NULL");

  GZ_ASSERT(_sdf, "AdvancedLiftDragPlugin _sdf pointer is NULL");

  if (_sdf->HasElement("radial_symmetry"))
    this->radialSymmetry = _sdf->Get<bool>("radial_symmetry");

  if (_sdf->HasElement("CL0"))
    this->CL0 = _sdf->Get<double>("CL0");

  if (_sdf->HasElement("CD0"))
    this->CD0 = _sdf->Get<double>("CD0");

  if (_sdf->HasElement("Cem0"))
    this->Cem0 = _sdf->Get<double>("Cem0");

  if (_sdf->HasElement("CLa"))
    this->CLa = _sdf->Get<double>("CLa");

  if (_sdf->HasElement("CYa"))
    this->CYa = _sdf->Get<double>("CYa");

  if (_sdf->HasElement("Cella"))
    this->Cella = _sdf->Get<double>("Cella");

  if (_sdf->HasElement("Cema"))
    this->Cema = _sdf->Get<double>("Cema");

  if (_sdf->HasElement("Cena"))
    this->Cena = _sdf->Get<double>("Cena");

  if (_sdf->HasElement("CLb"))
    this->CLb = _sdf->Get<double>("CLb");

  if (_sdf->HasElement("CYb"))
    this->CYb = _sdf->Get<double>("CYb");

  if (_sdf->HasElement("Cellb"))
    this->Cellb = _sdf->Get<double>("Cellb");

  if (_sdf->HasElement("Cemb"))
    this->Cemb = _sdf->Get<double>("Cemb");

  if (_sdf->HasElement("Cenb"))
    this->Cenb = _sdf->Get<double>("Cenb");

  if (_sdf->HasElement("alpha_stall"))
    this->alphaStall = _sdf->Get<double>("alpha_stall");

  if (_sdf->HasElement("Cema_stall"))
    this->CemaStall = _sdf->Get<double>("Cema_stall");

  if (_sdf->HasElement("ref_pt"))
    this->ref_pt = _sdf->Get<ignition::math::Vector3d>("ref_pt");

  if (_sdf->HasElement("CYb"))
    this->CYb = _sdf->Get<double>("CYb");

  if (_sdf->HasElement("Cellb"))
    this->Cellb = _sdf->Get<double>("Cellb");

  if (_sdf->HasElement("Cenb"))
    this->Cenb = _sdf->Get<double>("Cenb");

  // Determine the body rate derivatives.
  //This CDp is not parasitic drag: that is CD0.
  if (_sdf->HasElement("CDp"))
    this->CDp = _sdf->Get<double>("CDp");

  if (_sdf->HasElement("CYp"))
    this->CYp = _sdf->Get<double>("CYp");

  if (_sdf->HasElement("CLp"))
    this->CLp = _sdf->Get<double>("CLp");

  if (_sdf->HasElement("Cellp"))
    this->Cellp = _sdf->Get<double>("Cellp");

  if (_sdf->HasElement("Cemp"))
    this->Cemp = _sdf->Get<double>("Cemp");

  if (_sdf->HasElement("Cenp"))
    this->Cenp = _sdf->Get<double>("Cenp");



  if (_sdf->HasElement("CDq"))
    this->CDq = _sdf->Get<double>("CDq");

  if (_sdf->HasElement("CYq"))
    this->CYq = _sdf->Get<double>("CYq");

  if (_sdf->HasElement("CLq"))
    this->CLq = _sdf->Get<double>("CLq");

  if (_sdf->HasElement("Cellq"))
    this->Cellq = _sdf->Get<double>("Cellq");

  if (_sdf->HasElement("Cemq"))
    this->Cemq = _sdf->Get<double>("Cemq");

  if (_sdf->HasElement("Cenq"))
    this->Cenq = _sdf->Get<double>("Cenq");



    if (_sdf->HasElement("CDr"))
    this->CDr = _sdf->Get<double>("CDr");

  if (_sdf->HasElement("CYr"))
    this->CYr = _sdf->Get<double>("CYr");

  if (_sdf->HasElement("CLr"))
    this->CLr = _sdf->Get<double>("CLr");

  if (_sdf->HasElement("Cellr"))
    this->Cellr = _sdf->Get<double>("Cellr");

  if (_sdf->HasElement("Cemr"))
    this->Cemr = _sdf->Get<double>("Cemr");

  if (_sdf->HasElement("Cenr"))
    this->Cenr = _sdf->Get<double>("Cenr");


  if (_sdf->HasElement("num_ctrl_surfaces")){
    this->num_ctrl_surfaces = _sdf->Get<int>("num_ctrl_surfaces");
  }

  while( _sdf->HasElement("control_surface") )
  {
    gzdbg << "Control surface \n";
    sdf::ElementPtr curr_ctrl_surface = _sdf->GetElement("control_surface");
    gzdbg << ((curr_ctrl_surface->GetElement("name"))) << "\n";
    std::string ctrl_surface_name = ((curr_ctrl_surface->GetElement("name"))->GetValue())->GetAsString();


    this->controlJoints.push_back(this->model->GetJoint(ctrl_surface_name));
    this->ctrl_surface_direction.push_back(std::stod(((curr_ctrl_surface->GetElement("direction"))->GetValue())->GetAsString()));
    this->CD_ctrl.push_back(std::stod(((curr_ctrl_surface->GetElement("CD_ctrl"))->GetValue())->GetAsString()));
    this->CY_ctrl.push_back(std::stod(((curr_ctrl_surface->GetElement("CY_ctrl"))->GetValue())->GetAsString()));
    this->CL_ctrl.push_back(std::stod(((curr_ctrl_surface->GetElement("CL_ctrl"))->GetValue())->GetAsString()));
    this->Cell_ctrl.push_back(std::stod(((curr_ctrl_surface->GetElement("Cell_ctrl"))->GetValue())->GetAsString()));
    this->Cem_ctrl.push_back(std::stod(((curr_ctrl_surface->GetElement("Cem_ctrl"))->GetValue())->GetAsString()));
    this->Cen_ctrl.push_back(std::stod(((curr_ctrl_surface->GetElement("Cen_ctrl"))->GetValue())->GetAsString()));

    _sdf->RemoveChild(curr_ctrl_surface);

  }

  //Add aspect ratio
  if (_sdf->HasElement("AR"))
    this->AR = _sdf->Get<double>("AR");

  //Add efficiency
  if (_sdf->HasElement("eff"))
    this->eff = _sdf->Get<double>("eff");

  // forward (-drag) direction in link frame
  if (_sdf->HasElement("forward"))
    this->forward = _sdf->Get<ignition::math::Vector3d>("forward");
  this->forward.Normalize();

  // upward (+lift) direction in link frame
  if (_sdf->HasElement("upward"))
    this->upward = _sdf->Get<ignition::math::Vector3d>("upward");
  this->upward.Normalize();

  if (_sdf->HasElement("area"))
    this->area = _sdf->Get<double>("area");

  if (_sdf->HasElement("air_density"))
    this->rho = _sdf->Get<double>("air_density");

  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    GZ_ASSERT(elem, "Element link_name doesn't exist!");
    std::string linkName = elem->Get<std::string>();
    this->link = this->model->GetLink(linkName);
    GZ_ASSERT(this->link, "Link was NULL");

    if (!this->link)
    {
      gzerr << "Link with name[" << linkName << "] not found. "
        << "The AdvancedLiftDragPlugin will not generate forces\n";
    }
    else
    {
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&AdvancedLiftDragPlugin::OnUpdate, this));
    }
  }

  if (_sdf->HasElement("robotNamespace"))
  {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_advanced_liftdrag_plugin] Please specify a robotNamespace.\n";
  }
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  if (_sdf->HasElement("topic_name")) {
      const auto lift_force_topic = this->sdf->Get<std::string>("topic_name");
      lift_force_pub_ = node_handle_->Advertise<physics_msgs::msgs::Force>("~/" + lift_force_topic);
      gzdbg << "Publishing to ~/" << lift_force_topic << std::endl;
  }

  if (_sdf->HasElement("windSubTopic")){
    this->wind_sub_topic_ = _sdf->Get<std::string>("windSubTopic");
    wind_sub_ = node_handle_->Subscribe("~/" + wind_sub_topic_, &AdvancedLiftDragPlugin::WindVelocityCallback, this);
  }

}

/////////////////////////////////////////////////
void AdvancedLiftDragPlugin::OnUpdate()
{
  GZ_ASSERT(this->link, "Link was NULL");
  // get linear velocity at ref_pt in inertial frame
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d vel = this->link->WorldLinearVel(this->ref_pt) - wind_vel_;
  const common::Time current_time = this->world->SimTime();
#else
  ignition::math::Vector3d vel = ignitionFromGazeboMath(this->link->GetWorldLinearVel(this->ref_pt)) - wind_vel_;
  const common::Time current_time = this->world->GetSimTime();
#endif
  const double dt = (current_time - this->last_pub_time).Double();

  if (vel.Length() <= 0.01)
    return;

  // pose of body
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d pose = this->link->WorldPose();
#else
  ignition::math::Pose3d pose = ignitionFromGazeboMath(this->link->GetWorldPose());
#endif

  //define body and stability frame
  ignition::math::Vector3d bodyX = pose.Rot().RotateVector(this->forward);
  ignition::math::Vector3d bodyZ = -1*(pose.Rot().RotateVector(this->upward));
  ignition::math::Vector3d bodyY =  bodyZ.Cross(bodyX);

  ignition::math::Vector3d velInLDPlane = vel - vel.Dot(bodyY)*bodyY;

  ignition::math::Vector3d stabilityX = velInLDPlane;
  stabilityX.Normalize();
  ignition::math::Vector3d stabilityY = bodyY;
  ignition::math::Vector3d stabilityZ = stabilityX.Cross(stabilityY);

  double span = std::sqrt(this->area*this->AR);
  double mac = this->area/span;

  ignition::math::Vector3d body_rates = this->link->RelativeAngularVel();
  double speed = vel.Length();
  double p = body_rates.X()*span/(2*speed);
  double q = -1*body_rates.Y()*mac/(2*speed);
  double r = -1*body_rates.Z()*span/(2*speed);

  if (bodyX.Dot(vel) <= 0.0){
    // Only calculate lift or drag if the wind relative velocity is in the same direction
    return;
  }

  const double minRatio = -1.0;
  const double maxRatio = 1.0;

  //compute alpha
  double stabXHoriz = stabilityX.Dot(bodyX);
  double stabXVert = stabilityX.Dot(bodyZ);
  this->alpha = atan2(stabXVert,stabXHoriz);

  double sinAlpha = sin(this->alpha);
  double cosAlpha = cos(this->alpha);


  // normalize to within +/-90 deg
  while (fabs(this->alpha) > 0.5 * M_PI)
    this->alpha = this->alpha > 0 ? this->alpha - M_PI
                                  : this->alpha + M_PI;

  //compute sideslip angle
  double velSW = vel.Dot(bodyY);
  double velFW = vel.Dot(bodyX);
  this->beta = (atan2(velSW,velFW));


  // compute dynamic pressure
  double speedInLDPlane = velInLDPlane.Length();
  double dyn_pres = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;

  // compute CL at ref_pt, check for stall
  double CL;

   // Use sigmoid blending function
 double sigma = (1+exp(-1*this->M*(this->alpha-this->alphaStall))+exp(this->M*(this->alpha+this->alphaStall)))/((1+exp(-1*this->M*(this->alpha-this->alphaStall)))*(1+exp(this->M*(this->alpha+this->alphaStall)))); //blending function
CL = (1-sigma)*(this->CL0 + this->CLa*this->alpha) + sigma*(2*(this->alpha/abs(this->alpha))*pow(sinAlpha,2.0)*cosAlpha);
// add rate terms
CL = CL + this->CLp*p + this->CLq*q + this->CLr*r;
gzdbg << "Current Zero Deflection CL:" << CL << "\n";
gzdbg << "control surface: " << this->sdf->Get<std::string>("control_joint_name") << "\n";
gzdbg << "alpha: " << this->alpha << "\n";
gzdbg << "beta: " << this->beta << "\n";
gzdbg << "sigma: " << sigma << "\n";

double CL_ctrl_tot = 0;
double CD_ctrl_tot = 0;
double CY_ctrl_tot = 0;
double Cell_ctrl_tot = 0;
double Cem_ctrl_tot = 0;
double Cen_ctrl_tot = 0;
for(int i = 0; i < this->num_ctrl_surfaces; i++){
  #if GAZEBO_MAJOR_VERSION >= 9
    double controlAngle = this->controlJoints[i]->Position(0) * 180/M_PI;
  #else
    double controlAngle = this->controlJoints[i]->GetAngle(0).Radian() * 180/M_PI;
  #endif

  /* AVL's amd Gazebo's direction of "positive" deflection may be different.
  As such, there is functionality in this plugin to flip the direction of
  "positive" deflection in the Jinja file. Future users, keep an eye on this. */
  CL_ctrl_tot += controlAngle*this->CL_ctrl[i]*this->ctrl_surface_direction[i];
  CD_ctrl_tot += controlAngle*this->CD_ctrl[i]*this->ctrl_surface_direction[i];
  CY_ctrl_tot += controlAngle*this->CY_ctrl[i]*this->ctrl_surface_direction[i];
  Cell_ctrl_tot += controlAngle*this->Cell_ctrl[i]*this->ctrl_surface_direction[i];
  Cem_ctrl_tot += controlAngle*this->Cem_ctrl[i]*this->ctrl_surface_direction[i];
  Cen_ctrl_tot += controlAngle*this->Cen_ctrl[i]*this->ctrl_surface_direction[i];
}

  // AVL outputs a "CZ_elev", but the Z axis is down. This plugin
  // uses CL_elev, which is the negative of CZ_elev
  CL = CL+CL_ctrl_tot;
  gzdbg << "Current CL:" << CL << "\n";

  // compute lift force at ref_pt
  ignition::math::Vector3d lift = CL * dyn_pres * this->area * (-1 * stabilityZ);

  // compute CD at ref_pt, check for stall
  double CD;

    //add in quadratic model and a high-angle-of-attack (Newtonian) model

    /* The post stall model used below has two parts. The first part, the
    (estimated) flat plate drag, comes from the data in Ostowari and Naik,
    Post-Stall Wind Tunnel Data for NACA 44XX Series Airfoil Sections.
    https://www.nrel.gov/docs/legosti/old/2559.pdf
    The second part (1-2cos(alpha)) is fitted using data from Stringer et al,
    A new 360° airfoil model for predicting airfoil thrust potential in
    vertical-axis wind turbine designs.
    https://aip.scitation.org/doi/pdf/10.1063/1.5011207
    */

    /*To estimate the flat plate coefficient of drag, sigmoid function was fit
    to the data in Ostowari and Naik. The sigmoid function was chosen because 
    the CD would initially increase quickly with aspect
    ratio, but that rate of increase would slow down as AR goes to infinity.
    The form used was:
    CD_FP = 2/(1+exp(k1+k2*AR)).
    */
  double CD_fp_k1 = -0.224;
  double CD_fp_k2 = -0.115;
  double CD_fp = 2/(1+exp(CD_fp_k1+CD_fp_k2*(std::max(this->AR,1/this->AR))));
  CD = (1-sigma)*(this->CD0 + (pow(CL,2))/(M_PI*this->AR*this->eff))+sigma*abs(CD_fp*(1-2*cosAlpha));
  // add rate terms
  CD = CD + this->CDp*p + this->CDq*q + this->CDr * r;
  gzdbg << "Current Efficiency:" << this->eff << "\n";
  gzdbg << "Current Lift-Induced CD:" << (pow(CL,2))/(M_PI*this->AR*this->eff) << "\n";
  gzdbg << "Current CD, no deflection:" << CD << "\n";
  //add in control surface terms
  CD = CD + CD_ctrl_tot;
  gzdbg << "Current CD:" << CD << "\n";
  // drag at ref_pt
  ignition::math::Vector3d drag = CD * dyn_pres * this->area * (-1*stabilityX);

  double cy = this->CYb * this->beta + CY_ctrl_tot;
    // add rate terms
  cy = cy + this->CYp*p + this->CYq*q + this->CYr * r;
  gzdbg << "Current CY:" << cy << "\n";

  ignition::math::Vector3d sideforce = cy * dyn_pres * this->area * stabilityY;

  // compute Cem at ref_pt, check for stall
  double Cem;
  if (this->alpha > this->alphaStall)
  {
    Cem = this->Cem0 + (this->Cema * this->alphaStall +
          this->CemaStall * (this->alpha - this->alphaStall));
    // make sure Cem is still greater than 0
    Cem = std::max(0.0, Cem);
  }
  else if (this->alpha < -this->alphaStall)
  {
    Cem = this->Cem0 + (-this->Cema * this->alphaStall +
          this->CemaStall * (this->alpha + this->alphaStall));
    // make sure Cem is still less than 0
    Cem = std::min(0.0, Cem);
  }
  else
  Cem = this->Cem0 + this->Cema * this->alpha;
  // add rate terms
  Cem = Cem + this->Cemp*p + this->Cemq*q + this->Cemr * r;

  // Take into account the effect of control surface deflection angle to Cm
  Cem += Cem_ctrl_tot;
  gzdbg << "Current Cm:" << Cem << "\n";

  double cell = this->Cellb * this-> beta + Cell_ctrl_tot;
    // add rate terms
  cell = cell + this->Cellp*p + this->Cellq*q + this->Cellr * r;

  double cen = this->Cenb * this->beta + Cen_ctrl_tot;
    // add rate terms
  cen = cen + this->Cenp*p + this->Cenq*q + this->Cenr * r;
  gzdbg << "Current Cl:" << cell << "\n";
  gzdbg << "Current Cn:" << cen << "\n\n";

  // compute moment (torque) at ref_pt
  ignition::math::Vector3d moment = (Cem * dyn_pres * this->area * mac * bodyY) + (cell * dyn_pres * this->area * span * bodyX) + (cen * dyn_pres * this->area * span * bodyZ);

  // force about cg in inertial frame
  ignition::math::Vector3d force = lift + drag + sideforce;

    gzdbg << "force: " << force << "\n";
    gzdbg << "moment: " << moment << "\n";

  // Correct for nan or inf
  force.Correct();
  this->ref_pt.Correct();
  moment.Correct();

  // apply forces at cg (with torques for position shift)
  this->link->AddForceAtRelativePosition(force, this->ref_pt);
  this->link->AddTorque(moment);

  auto relative_center = this->link->RelativePose().Pos() + this->ref_pt;

  // Publish force and center of pressure for potential visual plugin.
  // - dt is used to control the rate at which the force is published
  // - it only gets published if 'topic_name' is defined in the sdf
  if (dt > 1.0 / 10 && this->sdf->HasElement("topic_name")) {
      msgs::Vector3d* force_center_msg = new msgs::Vector3d;
      force_center_msg->set_x(relative_center.X());
      force_center_msg->set_y(relative_center.Y());
      force_center_msg->set_z(relative_center.Z());

      msgs::Vector3d* force_vector_msg = new msgs::Vector3d;
      force_vector_msg->set_x(force.X());
      force_vector_msg->set_y(force.Y());
      force_vector_msg->set_z(force.Z());

      physics_msgs::msgs::Force force_msg;
      force_msg.set_allocated_center(force_center_msg);
      force_msg.set_allocated_force(force_vector_msg);

      lift_force_pub_->Publish(force_msg);
      this->last_pub_time = current_time;
  }
}

void AdvancedLiftDragPlugin::WindVelocityCallback(const boost::shared_ptr<const physics_msgs::msgs::Wind> &msg) {
  wind_vel_ = ignition::math::Vector3d(msg->velocity().x(),
            msg->velocity().y(),
            msg->velocity().z());
}
