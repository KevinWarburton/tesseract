#include <console_bridge/console.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_scene_graph/srdf_model.h>
#include <iostream>

using namespace tesseract_scene_graph;

std::string toString(const SceneGraph::Path& path)
{
  std::stringstream ss;
  ss << path;
  return ss.str();
}

std::string toString(bool b) { return b ? "true" : "false"; }

int main(int /*argc*/, char** /*argv*/)
{
  // Get the srdf file path
  std::string srdf_file = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf";

  // Create scene graph
  SceneGraph g;

  g.setName("kuka_lbr_iiwa_14_r820");

  Link base_link("base_link");
  Link link_1("link_1");
  Link link_2("link_2");
  Link link_3("link_3");
  Link link_4("link_4");
  Link link_5("link_5");
  Link link_6("link_6");
  Link link_7("link_7");
  Link tool0("tool0");

  g.addLink(base_link);
  g.addLink(link_1);
  g.addLink(link_2);
  g.addLink(link_3);
  g.addLink(link_4);
  g.addLink(link_5);
  g.addLink(link_6);
  g.addLink(link_7);
  g.addLink(tool0);

  Joint base_joint("base_joint");
  base_joint.parent_link_name = "base_link";
  base_joint.child_link_name = "link_1";
  base_joint.type = JointType::FIXED;
  g.addJoint(base_joint);

  Joint joint_1("joint_1");
  joint_1.parent_link_name = "link_1";
  joint_1.child_link_name = "link_2";
  joint_1.type = JointType::REVOLUTE;
  g.addJoint(joint_1);

  Joint joint_2("joint_2");
  joint_2.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_2.parent_link_name = "link_2";
  joint_2.child_link_name = "link_3";
  joint_2.type = JointType::REVOLUTE;
  g.addJoint(joint_2);

  Joint joint_3("joint_3");
  joint_3.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_3.parent_link_name = "link_3";
  joint_3.child_link_name = "link_4";
  joint_3.type = JointType::REVOLUTE;
  g.addJoint(joint_3);

  Joint joint_4("joint_4");
  joint_4.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_4.parent_link_name = "link_4";
  joint_4.child_link_name = "link_5";
  joint_4.type = JointType::REVOLUTE;
  g.addJoint(joint_4);

  Joint joint_5("joint_5");
  joint_5.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_5.parent_link_name = "link_5";
  joint_5.child_link_name = "link_6";
  joint_5.type = JointType::REVOLUTE;
  g.addJoint(joint_5);

  Joint joint_6("joint_6");
  joint_6.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_6.parent_link_name = "link_6";
  joint_6.child_link_name = "link_7";
  joint_6.type = JointType::REVOLUTE;
  g.addJoint(joint_6);

  Joint joint_tool0("joint_tool0");
  joint_tool0.parent_link_name = "link_7";
  joint_tool0.child_link_name = "tool0";
  joint_tool0.type = JointType::FIXED;
  g.addJoint(joint_tool0);

  // Parse the srdf
  SRDFModel srdf;
  bool success = srdf.initFile(g, srdf_file);
  CONSOLE_BRIDGE_logInform("SRDF loaded: %s", toString(success).c_str());

  // Add allowed collision matrix to scene graph
  g.addAllowedCollision("link_1", "link_2", "adjacent");

  processSRDFAllowedCollisions(g, srdf);

  // Get info about allowed collision matrix
  AllowedCollisionMatrix::ConstPtr acm = g.getAllowedCollisionMatrix();
  const AllowedCollisionEntries& acm_entries = acm->getAllAllowedCollisions();
  CONSOLE_BRIDGE_logInform("ACM Number of entries: %d", acm_entries.size());
}
