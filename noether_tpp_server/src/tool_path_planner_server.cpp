#include <rclcpp/rclcpp.hpp>
#include <noether_gui/widgets/configurable_tpp_pipeline_widget.h>
#include <noether_tpp/core/tool_path_planner_pipeline.h>
#include <noether_tpp/core/types.h>
#include <pcl/io/vtk_lib_io.h>
#include <yaml-cpp/yaml.h>
#include <QApplication>

#include "noether_tpp_server/srv/plan_tool_path.hpp"

namespace noether_tpp_server
{

namespace
{
YAML::Node toYaml(const std::vector<noether::ToolPaths>& paths)
{
  YAML::Node root;
  for (const auto& tps : paths)
  {
    YAML::Node tps_node;
    for (const auto& tp : tps)
    {
      YAML::Node tp_node;
      for (const auto& seg : tp)
      {
        YAML::Node seg_node;
        for (const auto& wp : seg)
        {
          Eigen::Quaterniond q(wp.linear());
          YAML::Node wp_node;
          wp_node["x"] = wp.translation().x();
          wp_node["y"] = wp.translation().y();
          wp_node["z"] = wp.translation().z();
          wp_node["qx"] = q.x();
          wp_node["qy"] = q.y();
          wp_node["qz"] = q.z();
          wp_node["qw"] = q.w();
          seg_node.push_back(wp_node);
        }
        tp_node.push_back(seg_node);
      }
      tps_node.push_back(tp_node);
    }
    root.push_back(tps_node);
  }
  return root;
}
}  // namespace

class ToolPathPlannerServer : public rclcpp::Node
{
public:
  ToolPathPlannerServer() : Node("tool_path_planner_server")
  {
    using namespace std::placeholders;
    service_ = this->create_service<noether_tpp_server::srv::PlanToolPath>(
        "plan_tool_path",
        std::bind(&ToolPathPlannerServer::callback, this, _1, _2));
  }

private:
  void callback(const std::shared_ptr<noether_tpp_server::srv::PlanToolPath::Request> req,
                std::shared_ptr<noether_tpp_server::srv::PlanToolPath::Response> res)
  {
    try
    {
      // Load mesh
      pcl::PolygonMesh mesh;
      if (pcl::io::loadPolygonFile(req->mesh_file, mesh) < 1)
        throw std::runtime_error("Failed to load mesh");

      // Build pipeline
      boost_plugin_loader::PluginLoader loader;
      loader.search_libraries.insert(NOETHER_GUI_PLUGINS);
      loader.search_libraries_env = NOETHER_GUI_PLUGIN_LIBS_ENV;
      loader.search_paths_env = NOETHER_GUI_PLUGIN_PATHS_ENV;
      noether::ConfigurableTPPPipelineWidget widget(std::move(loader));
      widget.configure(YAML::LoadFile(req->config_file));
      noether::ToolPathPlannerPipeline pipeline = widget.createPipeline();

      // Run pipeline
      std::vector<pcl::PolygonMesh> meshes = pipeline.mesh_modifier->modify(mesh);
      std::vector<noether::ToolPaths> paths;
      paths.reserve(meshes.size());
      for (const auto& m : meshes)
      {
        noether::ToolPaths tp = pipeline.planner->plan(m);
        paths.push_back(pipeline.tool_path_modifier->modify(tp));
      }

      YAML::Node yaml = toYaml(paths);
      std::ofstream ofh(req->output_file);
      if (!ofh)
        throw std::runtime_error("Failed to open output file");
      ofh << yaml;
      res->yaml = YAML::Dump(yaml);
      res->success = true;
    }
    catch (const std::exception& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Planning failed: %s", ex.what());
      res->success = false;
      res->yaml = "";
    }
  }

  rclcpp::Service<noether_tpp_server::srv::PlanToolPath>::SharedPtr service_;
};

}  // namespace noether_tpp_server

int main(int argc, char** argv)
{
  QApplication app(argc, argv);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<noether_tpp_server::ToolPathPlannerServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
