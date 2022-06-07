
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

using hr_clock = std::chrono::high_resolution_clock;
using drake::geometry::GeometrySet;
using drake::geometry::QueryObject;
using drake::geometry::SceneGraph;
using drake::geometry::SignedDistancePair;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::Body;
using drake::multibody::BodyIndex;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Context;

DEFINE_double(target_realtime_rate, 1.0,
  "Rate at which to run the simulation, relative to realtime");
DEFINE_double(simulation_time, 10,
  "How long to simulate the pendulum");
DEFINE_double(max_time_step, 1.0e-3,
  "Simulation time step used for integrator.");
DEFINE_string(urdf, "", "Name of urdf to load");

int main(int argc, char *argv[]) {
  // Running main function
  DRAKE_DEMAND(FLAGS_simulation_time > 0);

  // create builder
  drake::systems::DiagramBuilder<double> builder;
  // create scene graph
  auto [plant, scene_graph] =
    AddMultibodyPlantSceneGraph(&builder, FLAGS_max_time_step);
  scene_graph.set_name("scene_graph");
  drake::geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);

  // define geometry source
  // const char *kModelPath = "./lcl.urdf";
  const char *kModelPath = "./iCubGenova04_drake_obj.urdf";
  const std::string robot_root_link_in_urdf{"root_link"};

  auto parser = Parser(&plant, &scene_graph);
  parser.package_map().PopulateFromEnvironment("./urdfs");
  // ModelInstanceIndex model = Parser(&plant, &scene_graph).AddModelFromFile(kModelPath, "lcl-robot");
  ModelInstanceIndex model = parser.AddModelFromFile(kModelPath, "lcl-robot");
  auto &child_frame = plant.GetFrameByName(robot_root_link_in_urdf, model);
  plant.WeldFrames(plant.world_frame(), child_frame);

  // add initial and final locations
  Eigen::Vector3d cart0{0.15, 0.1, 0.45};
  Eigen::Vector3d cartf{-0.05, 0.35, 0.3};

  plant.Finalize();
  auto robot_dof_ = plant.num_positions(model);

  // connect the visualiser and make the context
  // drake::geometry::ConnectDrakeVisualizer(&builder, scene_graph);

  std::unique_ptr<drake::systems::Diagram<double>> diagram_ = builder.Build();
  std::unique_ptr<drake::systems::Context<double>> context_ =
    diagram_->CreateDefaultContext();
  // steps different from the python code
  diagram_->SetDefaultContext(context_.get());
  auto scene_context = scene_graph.AllocateContext();
  auto output = diagram_->AllocateOutput();

  // create a simulator for visualisation
  std::unique_ptr<drake::systems::Simulator<double>> simulator_ =
    std::make_unique<drake::systems::Simulator<double>>(*diagram_);
  simulator_->Initialize();
  drake::systems::Context<double> *plant_context =
    &diagram_->GetMutableSubsystemContext(plant, &simulator_->get_mutable_context());

  plant.SetPositions(plant_context, model,
                   Eigen::VectorXd::Zero(robot_dof_));
  simulator_->get_system().Publish(simulator_->get_context());

  return 0;
}
